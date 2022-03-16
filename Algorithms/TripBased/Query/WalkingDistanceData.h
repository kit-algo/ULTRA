#pragma once

#include "../../../DataStructures/TripBased/Data.h"

namespace TripBased {

class WalkingDistanceData {

public:
    WalkingDistanceData(const Data& data) :
        data(data),
        labels(data.numberOfStopEvents(), INFTY) {
    }

public:
    inline void clear() noexcept {
        std::fill(labels.begin(), labels.end(), INFTY);
    }

    inline int operator()(const StopEventId stopEvent) const noexcept {
        AssertMsg(stopEvent < labels.size(), "StopEvent " << stopEvent << " is out of bounds!");
        return labels[stopEvent];
    }

    inline StopEventId getScanEnd(const StopEventId stopEvent, const StopEventId tripEnd, const int walkingDistance) const noexcept {
        for (StopEventId event = stopEvent; event < tripEnd; event++) {
            if (labels[event] <= walkingDistance) return event;
        }
        return tripEnd;
    }

    inline void update(const StopEventId stopEvent, const StopEventId tripEnd, const StopEventId routeEnd, const StopIndex tripLength, const int walkingDistance) noexcept {
        StopEventId currentStart = stopEvent;
        StopEventId currentEnd = tripEnd;
        for (; currentStart < routeEnd; currentStart += tripLength, currentEnd += tripLength) {
            for (StopEventId event = currentStart; event < currentEnd; event++) {
                if (labels[event] <= walkingDistance) break;
                labels[event] = walkingDistance;
            }
        }
    }

private:
    const Data& data;

    std::vector<int> labels;

};

}
