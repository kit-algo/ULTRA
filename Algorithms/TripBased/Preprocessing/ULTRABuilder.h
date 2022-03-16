#pragma once

#include <algorithm>

#include "../../../DataStructures/TripBased/Data.h"
#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/Timer.h"
#include "../../../Helpers/Console/Progress.h"

#include "ShortcutSearch.h"

namespace TripBased {

template<bool DEBUG = false, bool IGNORE_ISOLATED_CANDIDATES = false>
class ULTRABuilder {

public:
    inline static constexpr bool Debug = DEBUG;
    inline static constexpr bool IgnoreIsolatedCandidates = IGNORE_ISOLATED_CANDIDATES;
    using Type = ULTRABuilder<Debug, IgnoreIsolatedCandidates>;
    using ShortcutSearchType = ShortcutSearch<Debug, IgnoreIsolatedCandidates>;

public:
    ULTRABuilder(const Data& data) :
        data(data) {
        stopEventGraph.addVertices(data.numberOfStopEvents());
    }

    void computeShortcuts(const ThreadPinning& threadPinning, const int witnessTransferLimit = 15 * 60, const int minDepartureTime = -never, const int maxDepartureTime = never, const bool verbose = true) noexcept {
        if (verbose) std::cout << "Computing shortcuts with " << threadPinning.numberOfThreads << " threads." << std::endl;

        std::vector<Shortcut> shortcuts;

        Progress progress(data.numberOfStops(), verbose);
        omp_set_num_threads(threadPinning.numberOfThreads);
        #pragma omp parallel
        {
            threadPinning.pinThread();

            ShortcutSearchType shortcutSearch(data, witnessTransferLimit);

            #pragma omp for schedule(dynamic)
            for (size_t i = 0; i < data.numberOfStops(); i++) {
                shortcutSearch.run(StopId(i), minDepartureTime, maxDepartureTime);
                progress++;
            }

            #pragma omp critical
            {
                const std::vector<Shortcut>& localShortcuts = shortcutSearch.getShortcuts();
                for (const Shortcut& shortcut : localShortcuts) {
                    shortcuts.emplace_back(shortcut);
                }
            }
        }

        if (!shortcuts.empty()) {
            std::sort(shortcuts.begin(), shortcuts.end(), [](const Shortcut& a, const Shortcut& b){
                return (a.origin < b.origin) || ((a.origin == b.origin) && (a.destination < b.destination));
            });
            stopEventGraph.addEdge(Vertex(shortcuts[0].origin), Vertex(shortcuts[0].destination)).set(TravelTime, shortcuts[0].walkingDistance);
            for (size_t i = 1; i < shortcuts.size(); i++) {
                if ((shortcuts[i].origin == shortcuts[i - 1].origin) && (shortcuts[i].destination == shortcuts[i - 1].destination)) continue;
                stopEventGraph.addEdge(Vertex(shortcuts[i].origin), Vertex(shortcuts[i].destination)).set(TravelTime, shortcuts[i].walkingDistance);
            }
            stopEventGraph.sortEdges(ToVertex);
        }

        progress.finished();
    }

    inline const DynamicTransferGraph& getStopEventGraph() const noexcept {
        return stopEventGraph;
    }

    inline DynamicTransferGraph& getStopEventGraph() noexcept {
        return stopEventGraph;
    }

private:
    const Data& data;
    DynamicTransferGraph stopEventGraph;

};

}
