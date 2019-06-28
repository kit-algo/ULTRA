/**********************************************************************************

 Copyright (c) 2019 Jonas Sauer, Tobias Zündorf

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
 modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************************/

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

    inline bool hasEarlierTrip(const int arrivalTime) const noexcept {
        return hasEarlierTrip() && (previousDepartureTime() >= arrivalTime);
    }

    inline void previousTrip() noexcept {
        AssertMsg(hasEarlierTrip(), "There is no earlier trip!");
        trip = trip - tripSize;
    }

    inline bool decreaseTrip() noexcept {
        if (!hasEarlierTrip()) return false;
        trip = trip - tripSize;
        return true;
    }

    inline int arrivalTime() const noexcept {
        return (trip + stopIndex)->arrivalTime;
    }

    inline int departureTime() const noexcept {
        return (trip + stopIndex)->departureTime;
    }

    inline int previousArrivalTime() const noexcept {
        AssertMsg(hasEarlierTrip(), "There is no earlier trip!");
        return (trip + stopIndex - tripSize)->arrivalTime;
    }

    inline int previousDepartureTime() const noexcept {
        AssertMsg(hasEarlierTrip(), "There is no earlier trip!");
        return (trip + stopIndex - tripSize)->departureTime;
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
