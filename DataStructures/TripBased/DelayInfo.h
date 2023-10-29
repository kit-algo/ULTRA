#pragma once

#include <random>
#include <vector>

#include "Delay.h"
#include "DelayData.h"

#include "../../Helpers/Vector/Permutation.h"

namespace TripBased {

class DelayInfo {
public:
    DelayInfo(const DelayData& delayData) :
        delayData(delayData),
        delayedEvents(delayData.data.numberOfStopEvents()) {
    }

    inline void update(const std::vector<DelayUpdate>& updates) noexcept {
        delayedEvents.clear();
        highlyDelayedEvents.clear();
        for (const DelayUpdate& update : updates) {
            StopEventId event = update.firstEvent;
            for (size_t i = 0; i < update.arrivalDelay.size(); i++, event++) {
                delayedEvents.insert(event);
            }
        }
        for (const StopEventId event : delayedEvents) {
            if (delayData.arrivalDelay[event] > delayData.maxArrivalDelay || delayData.departureDelay[event] > delayData.maxDepartureDelay)
                highlyDelayedEvents.emplace_back(event);
        }
    }

    inline void applyPermutation(const Permutation& permutation) noexcept {
        delayedEvents.applyPermutation(permutation);
        permutation.mapPermutation(highlyDelayedEvents);
    }

    inline const std::vector<StopEventId>& getDelayedEvents() const noexcept {
        return delayedEvents.getValues();
    }

    inline const std::vector<StopEventId>& getHighlyDelayedEvents() const noexcept {
        return highlyDelayedEvents;
    }

private:
    const DelayData& delayData;
    IndexedSet<false, StopEventId> delayedEvents;
    std::vector<StopEventId> highlyDelayedEvents;
};

}
