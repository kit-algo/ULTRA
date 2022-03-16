#pragma once

#include <algorithm>

#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/Timer.h"
#include "../../../Helpers/Console/Progress.h"

#include "ShortcutSearch.h"

namespace RAPTOR::ULTRA {

template<bool DEBUG = false, bool COUNT_OPTIMAL_CANDIDATES = false, bool IGNORE_ISOLATED_CANDIDATES = false>
class Builder {

public:
    inline static constexpr bool Debug = DEBUG;
    inline static constexpr bool CountOptimalCandidates = COUNT_OPTIMAL_CANDIDATES;
    inline static constexpr bool IgnoreIsolatedCandidates = IGNORE_ISOLATED_CANDIDATES;
    using Type = Builder<Debug, CountOptimalCandidates, IgnoreIsolatedCandidates>;

public:
    Builder(const Data& data) :
        data(data) {
        shortcutGraph.addVertices(data.numberOfStops());
        for (const Vertex vertex : shortcutGraph.vertices()) {
            shortcutGraph.set(Coordinates, vertex, data.transferGraph.get(Coordinates, vertex));
        }
    }

    void computeShortcuts(const ThreadPinning& threadPinning, const int witnessTransferLimit = 15 * 60, const int minDepartureTime = -never, const int maxDepartureTime = never, const bool verbose = true) noexcept {
        if (verbose) std::cout << "Computing shortcuts with " << threadPinning.numberOfThreads << " threads." << std::endl;

        size_t optimalCandidates = 0;
        Progress progress(data.numberOfStops(), verbose);
        omp_set_num_threads(threadPinning.numberOfThreads);
        #pragma omp parallel
        {
            threadPinning.pinThread();

            DynamicTransferGraph localShortcutGraph = shortcutGraph;
            ShortcutSearch<Debug, CountOptimalCandidates, IgnoreIsolatedCandidates> shortcutSearch(data, localShortcutGraph, witnessTransferLimit);

            #pragma omp for schedule(dynamic)
            for (size_t i = 0; i < data.numberOfStops(); i++) {
                shortcutSearch.run(StopId(i), minDepartureTime, maxDepartureTime);
                progress++;
            }

            #pragma omp critical
            {
                if constexpr (CountOptimalCandidates) {
                    optimalCandidates += shortcutSearch.getNumberOfOptimalCandidates();
                }
                for (const Vertex from : shortcutGraph.vertices()) {
                    for (const Edge edge : localShortcutGraph.edgesFrom(from)) {
                        const Vertex to = localShortcutGraph.get(ToVertex, edge);
                        if (!shortcutGraph.hasEdge(from, to)) {
                            shortcutGraph.addEdge(from, to).set(TravelTime, localShortcutGraph.get(TravelTime, edge));
                        } else {
                            AssertMsg(shortcutGraph.get(TravelTime, shortcutGraph.findEdge(from, to)) == localShortcutGraph.get(TravelTime, edge), "Edge from " << from << " to " << to << " has inconclusive travel time (" << shortcutGraph.get(TravelTime, shortcutGraph.findEdge(from, to)) << ", " << localShortcutGraph.get(TravelTime, edge) << ")");
                        }
                    }
                }
            }
        }
        progress.finished();
        if constexpr (CountOptimalCandidates) {
            std::cout << "#Optimal candidates: " << String::prettyInt(optimalCandidates) << std::endl;
        } else {
            suppressUnusedParameterWarning(optimalCandidates);
        }
    }

    inline const DynamicTransferGraph& getShortcutGraph() const noexcept {
        return shortcutGraph;
    }

    inline DynamicTransferGraph& getShortcutGraph() noexcept {
        return shortcutGraph;
    }

private:
    const Data& data;
    DynamicTransferGraph shortcutGraph;
};

}
