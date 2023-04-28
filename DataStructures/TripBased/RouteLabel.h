#pragma once

#include <algorithm>
#include <vector>

#include "Data.h"

namespace TripBased {

struct RouteLabel {
    RouteLabel(const Data& data, const RouteId route) :
        numberOfTrips(data.numberOfTripsInRoute(route)) {
        const size_t numberOfStops = data.numberOfStopsInRoute(route);
        departureTimes.resize((numberOfStops - 1) * numberOfTrips);
        TripId trip = data.firstTripOfRoute[route];
        for (size_t i = 0; i < numberOfTrips; i++, trip++) {
            for (StopIndex stopIndex(0); stopIndex + 1 < numberOfStops; stopIndex++) {
                const StopEventId stopEvent = data.getStopEventId(trip, stopIndex);
                departureTimes[(stopIndex * numberOfTrips) + i] = data.departureTime(stopEvent);
            }
        }
    }

    inline StopIndex end() const noexcept {
        return StopIndex(departureTimes.size() / numberOfTrips);
    }

    inline bool findEarliestTrip(const StopIndex stopIndex, const int departureTime, TripId& tripIndex) const noexcept {
        if (tripIndex >= numberOfTrips) {
            return findEarliestTripBinary(stopIndex, departureTime, tripIndex);
        } else {
            return findEarliestTripLinear(stopIndex, departureTime, tripIndex);
        }
    }

    inline bool findEarliestTripBinary(const StopIndex stopIndex, const int departureTime, TripId& tripIndex) const noexcept {
        const u_int32_t labelIndex = stopIndex * numberOfTrips;
        tripIndex = std::lower_bound(TripId(0), TripId(numberOfTrips), departureTime, [&](const TripId trip, const int time) {
            return departureTimes[labelIndex + trip] < time;
        });
        return tripIndex < numberOfTrips;
    }

    inline bool findEarliestTripLinear(const StopIndex stopIndex, const int departureTime, TripId& tripIndex) const noexcept {
        const u_int32_t labelIndex = stopIndex * numberOfTrips;
        if (departureTimes[labelIndex + tripIndex - 1] < departureTime) return false;
        tripIndex--;
        while ((tripIndex > 0) && (departureTimes[labelIndex + tripIndex - 1] >= departureTime)) {
            tripIndex--;
        }
        return true;
    }

    u_int32_t numberOfTrips;
    std::vector<int> departureTimes;
};

}
