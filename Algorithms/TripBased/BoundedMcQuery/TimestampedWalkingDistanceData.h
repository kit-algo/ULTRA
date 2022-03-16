#pragma once

#include "../../../DataStructures/TripBased/Data.h"

namespace TripBased {

class TimestampedWalkingDistanceData {

public:
    TimestampedWalkingDistanceData(const Data& data) :
        data(data),
        labels(data.numberOfStopEvents(), INFTY),
        timestamps(data.numberOfStopEvents(), 0),
        timestamp(0) {
    }

public:
    inline void clear() noexcept {
        timestamp++;
    }

    inline int operator()(const StopEventId stopEvent) noexcept {
        AssertMsg(stopEvent < labels.size(), "StopEvent " << stopEvent << " is out of bounds!");
        return getLabel(stopEvent);
    }

    inline StopEventId getScanEnd(const StopEventId stopEvent, const StopEventId tripEnd, const int walkingDistance) noexcept {
        for (StopEventId event = stopEvent; event < tripEnd; event++) {
            if (getLabel(event) <= walkingDistance) return event;
        }
        return tripEnd;
    }

    inline void update(const StopEventId stopEvent, const StopEventId tripEnd, const StopEventId routeEnd, const StopIndex tripLength, const int walkingDistance) noexcept {
        StopEventId currentStart = stopEvent;
        StopEventId currentEnd = tripEnd;
        for (; currentStart < routeEnd; currentStart += tripLength, currentEnd += tripLength) {
            for (StopEventId event = currentStart; event < currentEnd; event++) {
                int& label = getLabel(event);
                if (label <= walkingDistance) break;
                label = walkingDistance;
            }
        }
    }

private:
    inline int& getLabel(const StopEventId stopEvent) noexcept {
        if (timestamps[stopEvent] != timestamp) {
            labels[stopEvent] = INFTY;
            timestamps[stopEvent] = timestamp;
        }
        return labels[stopEvent];
    }

    const Data& data;

    std::vector<int> labels;
    std::vector<int> timestamps;
    int timestamp;

};

}
