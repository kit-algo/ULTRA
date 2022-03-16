#pragma once

#include "StopEvent.h"

#include "../../../Helpers/Types.h"

namespace RAPTOR {

class TripIterator {

public:
    TripIterator(const u_int32_t tripSize, const StopId* const stops, const StopEvent* const firstTrip, const StopIndex stopIndex, const StopEvent* const trip) :
        tripSize(tripSize),
        stops(stops),
        firstTrip(firstTrip),
        stopIndex(stopIndex),
        trip(trip) {
    }

    inline StopIndex getStopIndex() const noexcept {
        return stopIndex;
    }

    inline void setStopIndex(const StopIndex newStopIndex) noexcept {
        AssertMsg(newStopIndex < tripSize, "Stop index " << newStopIndex << " is out of bounds, because the trip has only " << tripSize << " stops!");
        stopIndex = newStopIndex;
    }

    inline bool hasFurtherStops() const noexcept {
        return (stopIndex + 1) < tripSize;
    }

    inline void nextStop() noexcept {
        AssertMsg(hasFurtherStops(), "There is no next stop (number of stops: " << tripSize << ")!");
        ++stopIndex;
    }

    inline bool advanceStop() noexcept {
        if (!hasFurtherStops()) return false;
        ++stopIndex;
        return true;
    }

    inline bool hasEarlierTrip() const noexcept {
        return trip > firstTrip;
    }

    inline bool hasEarlierTrip(const StopEvent* otherTrip) const noexcept {
        return otherTrip > firstTrip;
    }

    inline bool hasEarlierTrip(const int arrivalTime) const noexcept {
        return hasEarlierTrip() && (previousDepartureTime() >= arrivalTime);
    }

    inline bool hasEarlierTrip(const StopEvent* otherTrip, const int arrivalTime) const noexcept {
        return hasEarlierTrip(otherTrip) && (previousDepartureTime(otherTrip) >= arrivalTime);
    }

    inline void previousTrip() noexcept {
        AssertMsg(hasEarlierTrip(), "There is no earlier trip!");
        trip = trip - tripSize;
    }

    inline const StopEvent* previousTrip(const StopEvent* otherTrip) const noexcept {
        AssertMsg(hasEarlierTrip(otherTrip), "There is no earlier trip!");
        return otherTrip - tripSize;
    }

    inline bool decreaseTrip() noexcept {
        if (!hasEarlierTrip()) return false;
        trip = trip - tripSize;
        return true;
    }

    inline int arrivalTime() const noexcept {
        return (trip + stopIndex)->arrivalTime;
    }

    inline int arrivalTime(const StopEvent* otherTrip) const noexcept {
        return (otherTrip + stopIndex)->arrivalTime;
    }

    inline int departureTime() const noexcept {
        return (trip + stopIndex)->departureTime;
    }

    inline int departureTime(const StopEvent* otherTrip) const noexcept {
        return (otherTrip + stopIndex)->departureTime;
    }

    inline int previousArrivalTime() const noexcept {
        AssertMsg(hasEarlierTrip(), "There is no earlier trip!");
        return (trip + stopIndex - tripSize)->arrivalTime;
    }

    inline int previousDepartureTime() const noexcept {
        AssertMsg(hasEarlierTrip(), "There is no earlier trip!");
        return (trip + stopIndex - tripSize)->departureTime;
    }

    inline int previousDepartureTime(const StopEvent* otherTrip) const noexcept {
        AssertMsg(hasEarlierTrip(otherTrip), "There is no earlier trip!");
        return (otherTrip + stopIndex - tripSize)->departureTime;
    }

    inline StopId stop() const noexcept {
        return stops[stopIndex];
    }

    inline StopId stop(const StopIndex index) const noexcept {
        return stops[index];
    }

    inline const StopEvent* stopEvent() const noexcept {
        return trip + stopIndex;
    }

    inline const StopEvent* stopEvent(const StopEvent* otherTrip) const noexcept {
        return otherTrip + stopIndex;
    }

    inline const StopEvent* stopEvent(const StopIndex index) const noexcept {
        return trip + index;
    }

    inline const StopEvent* getCurrentTrip() const noexcept {
        return trip;
    }

    inline size_t getCurrentTripNumber() const noexcept {
        return (trip - firstTrip) / tripSize;
    }

    inline const StopEvent* getPreviousTrip() const noexcept {
        if (hasEarlierTrip()) {
            return trip - tripSize;
        } else {
            return nullptr;
        }
    }

private:
    const u_int32_t tripSize;
    const StopId* const stops;
    const StopEvent* const firstTrip;

    StopIndex stopIndex;
    const StopEvent* trip;

};

}
