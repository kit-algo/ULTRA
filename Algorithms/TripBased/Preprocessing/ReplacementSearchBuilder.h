#pragma once

#include "../../../DataStructures/TripBased/DelayUpdateData.h"
#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/Timer.h"

#include "ReplacementShortcutSearch.h"

namespace TripBased {

struct ShortcutInfo {
    ShortcutInfo(const Vertex from = noVertex, const Vertex to = noVertex, const int travelTime = INFTY) :
        from(from),
        to(to),
        travelTime(travelTime) {
    }

    inline void applyVertexPermutation(const Permutation& permutation) noexcept {
        from = permutation.permutate(from);
        to = permutation.permutate(to);
    }

    inline bool operator<(const ShortcutInfo& other) noexcept {
        return std::tie(from, to) < std::tie(other.from, other.to);
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(from, to, travelTime);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(from, to, travelTime);
    }

    Vertex from;
    Vertex to;
    int travelTime;
};

class ReplacementSearchBuilder {
public:
    ReplacementSearchBuilder(const DelayUpdateData& data, const ThreadPinning& threadPinning, const int targetSlack) :
        data(data),
        threadPinning(threadPinning),
        delayedTrips(data.tripData.numberOfTrips()),
        originSearchTime(0),
        targetSearchTime(0) {
        shortcutGraph.addVertices(data.tripData.numberOfStopEvents());
        const ReplacementShortcutSearch search(data, targetSlack);
        for (size_t i = 0; i < threadPinning.numberOfThreads; i++) {
            searches.emplace_back(search);
        }
    }

    inline void run(const DelayInfo& delayInfo, const bool replaceDelayedTargets) noexcept {
        clear();
        if (delayInfo.getDelayedEvents().empty()) return;
        Timer timer;
        if (replaceDelayedTargets && !delayInfo.getHighlyDelayedEvents().empty()) {
            timer.restart();
            runReplacementShortcutSearch<true>(delayInfo.getHighlyDelayedEvents());
            targetSearchTime = timer.elapsedMicroseconds();
        }
        timer.restart();
        runReplacementShortcutSearch<false>(delayInfo.getDelayedEvents());
        originSearchTime = timer.elapsedMicroseconds();
        prepareShortcuts();
    }

    inline const std::vector<ShortcutInfo>& getShortcuts() const noexcept {
        return shortcuts;
    }

    inline double getOriginSearchTime() const noexcept {
        return originSearchTime;
    }

    inline double getTargetSearchTime() const noexcept {
        return targetSearchTime;
    }

private:
    inline void clear() noexcept {
        shortcutGraph.clearEdges();
        originSearchTime = 0;
        targetSearchTime = 0;
    }

    template<bool FOR_TARGETS>
    inline void runReplacementShortcutSearch(const std::vector<StopEventId>& delayedEvents) noexcept {
        computeDelayedTrips(delayedEvents);

        omp_set_num_threads(std::min(threadPinning.numberOfThreads, delayedTrips.size()));
        #pragma omp parallel
        {
            threadPinning.pinThread();
            const int threadNum = omp_get_thread_num();
            searches[threadNum].resetData();

            #pragma omp for schedule(dynamic)
            for (size_t i = 0; i < delayedTrips.size(); i++) {
                const TripId trip = delayedTrips.getKeys()[i];
                if constexpr (FOR_TARGETS) {
                    searches[threadNum].runForDelayedTargets(delayedTrips[trip]);
                } else {
                    searches[threadNum].runForDelayedOrigins(delayedTrips[trip]);
                }
            }

            for (const StopEventId origin : searches[threadNum].getShortcutOrigins()) {
                for (const ReplacementShortcutSearch::ShortcutData& shortcut : searches[threadNum].getShortcutsFrom(origin)) {
                    #pragma omp critical
                    {
                        if (!shortcutGraph.hasEdge(Vertex(origin), Vertex(shortcut.destination))) {
                            shortcutGraph.addEdge(Vertex(origin), Vertex(shortcut.destination)).set(TravelTime, shortcut.travelTime);
                        }
                    }
                }
            }
        }
    }

    inline void computeDelayedTrips(const std::vector<StopEventId>& delayedEvents) noexcept {
        delayedTrips.clear();
        for (const StopEventId event : delayedEvents) {
            const TripId trip = data.tripData.tripOfStopEvent[event];
            if (!delayedTrips.contains(trip)) {
                delayedTrips.insert(trip);
            }
            delayedTrips[trip].emplace_back(event);
        }
    }

    inline void prepareShortcuts() noexcept {
        shortcutGraph.sortEdges(ToVertex);
        shortcuts.clear();
        shortcuts.reserve(shortcutGraph.numEdges());
        for (const Vertex from : shortcutGraph.vertices()) {
            for (const Edge edge : shortcutGraph.edgesFrom(from)) {
                shortcuts.emplace_back(from, shortcutGraph.get(ToVertex, edge), shortcutGraph.get(TravelTime, edge));
            }
        }
    }

    const DelayUpdateData& data;
    const ThreadPinning& threadPinning;
    std::vector<ReplacementShortcutSearch> searches;

    IndexedMap<std::vector<StopEventId>, false, TripId> delayedTrips;

    DynamicTransferGraph shortcutGraph;
    std::vector<ShortcutInfo> shortcuts;
    double originSearchTime;
    double targetSearchTime;
};

}
