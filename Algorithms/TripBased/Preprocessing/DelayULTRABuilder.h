#pragma once

#include <algorithm>

#include "../../../DataStructures/TripBased/Data.h"
#include "../../../DataStructures/TripBased/Shortcut.h"
#include "../../../DataStructures/TripBased/ShortcutCollection.h"
#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/Timer.h"
#include "../../../Helpers/Console/Progress.h"

#include "DelayShortcutSearch.h"

namespace TripBased {

template<bool DEBUG = false>
class DelayULTRABuilder {

public:
    inline static constexpr bool Debug = DEBUG;
    using Type = DelayULTRABuilder<Debug>;

public:
    DelayULTRABuilder(const Data& data) :
        data(data),
        shortcuts(data.numberOfStopEvents()) {
    }

    void computeShortcuts(const ThreadPinning& threadPinning, const int arrivalDelayBuffer, const int departureDelayBuffer, const size_t memoryLimit = 2048, const int minDepartureTime = -never, const int maxDepartureTime = never, const bool verbose = true) noexcept {
        if (verbose) std::cout << "Computing shortcuts with " << threadPinning.numberOfThreads << " threads." << std::endl;

        Progress progress(data.numberOfStops(), verbose);
        omp_set_num_threads(threadPinning.numberOfThreads);
        std::vector<ShortcutCollection> threadShortcuts(threadPinning.numberOfThreads, ShortcutCollection(data.numberOfStopEvents()));
        #pragma omp parallel
        {
            threadPinning.pinThread();
            const size_t threadNum = omp_get_thread_num();

            DelayShortcutSearch<Debug> shortcutSearch(data, arrivalDelayBuffer, departureDelayBuffer, threadShortcuts[threadNum]);

            #pragma omp for schedule(dynamic)
            for (size_t i = 0; i < data.numberOfStops(); i++) {
                shortcutSearch.run(StopId(i), minDepartureTime, maxDepartureTime);
                if (threadShortcuts[threadNum].memoryUsageInBytes() > static_cast<long long>(memoryLimit * 1024 * 1024)) {
                    #pragma omp critical
                    {
                        shortcuts.merge(threadShortcuts[threadNum]);
                    }
                    threadShortcuts[threadNum].clear();
                }
                progress++;
            }
        }

        for (size_t i = 1; i < threadPinning.numberOfThreads; i *= 2) {
            #pragma omp parallel
            {
                threadPinning.pinThread();
                const size_t threadNum = omp_get_thread_num();
                if ((threadNum % (2*i)) == 0 && threadNum + i < threadPinning.numberOfThreads) {
                    threadShortcuts[threadNum].merge(threadShortcuts[threadNum + i]);
                }
            }
        }

        shortcuts.merge(threadShortcuts[0]);
        stopEventGraph = shortcuts.getGraph();
        progress.finished();
    }


    inline const DynamicDelayGraph& getStopEventGraph() const noexcept {
        return stopEventGraph;
    }

    inline DynamicDelayGraph& getStopEventGraph() noexcept {
        return stopEventGraph;
    }

private:
    const Data& data;
    ShortcutCollection shortcuts;
    DynamicDelayGraph stopEventGraph;
};

}
