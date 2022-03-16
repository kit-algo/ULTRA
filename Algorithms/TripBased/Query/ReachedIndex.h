#pragma once

#include <algorithm>

#include "../../../DataStructures/TripBased/Data.h"

namespace TripBased {

class ReachedIndex {

public:
    ReachedIndex(const Data& data) :
        data(data),
        labels(data.numberOfTrips(), -1),
        defaultLabels(data.numberOfTrips(), -1) {
        for (const TripId trip : data.trips()) {
            if (data.numberOfStopsInTrip(trip) > 255) warning("Trip ", trip, " has ", data.numberOfStopsInTrip(trip), " stops!");
            defaultLabels[trip] = data.numberOfStopsInTrip(trip);
        }
    }

public:
    inline void clear() noexcept {
        labels = defaultLabels;
    }

    inline void clear(const RouteId route) noexcept {
        const TripId start = data.firstTripOfRoute[route];
        const TripId end = data.firstTripOfRoute[route + 1];
        std::copy_n(defaultLabels.begin() + start, end - start, labels.begin() + start);
    }

    inline StopIndex operator()(const TripId trip) const noexcept {
        AssertMsg(trip < labels.size(), "Trip " << trip << " is out of bounds!");
        return StopIndex(labels[trip]);
    }

    inline bool alreadyReached(const TripId trip, const u_int8_t index) const noexcept {
        return labels[trip] <= index;
    }

    inline void update(const TripId trip, const StopIndex index) noexcept {
        AssertMsg(trip < labels.size(), "Trip " << trip << " is out of bounds!");
        const TripId routeEnd = data.firstTripOfRoute[data.routeOfTrip[trip] + 1];
        for (TripId i = trip; i < routeEnd; i++) {
            if (labels[i] <= index) break;
            labels[i] = index;
        }
    }

    inline void updateRaw(const TripId trip, const TripId tripEnd, const StopIndex index) noexcept {
        AssertMsg(trip < labels.size(), "Trip " << trip << " is out of bounds!");
        AssertMsg(tripEnd <= data.firstTripOfRoute[data.routeOfTrip[trip] + 1], "Trip end" << tripEnd << " is out of bounds!");
        std::fill(labels.begin() + trip, labels.begin() + tripEnd, index);
    }

private:
    const Data& data;

    std::vector<u_int8_t> labels;

    std::vector<u_int8_t> defaultLabels;

};

}
