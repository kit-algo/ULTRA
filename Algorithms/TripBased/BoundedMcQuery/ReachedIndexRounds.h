#pragma once

#include "../../../DataStructures/TripBased/Data.h"

namespace TripBased {

class ReachedIndexRounds {

public:
    ReachedIndexRounds(const Data& data) :
        data(data),
        defaultLabels(data.numberOfTrips(), -1),
        currentRound(0) {
        for (const TripId trip : data.trips()) {
            if (data.numberOfStopsInTrip(trip) > 255) warning("Trip ", trip, " has ", data.numberOfStopsInTrip(trip), " stops!");
            defaultLabels[trip] = data.numberOfStopsInTrip(trip);
        }
    }

public:
    inline void clear() noexcept {
        labels.resize(1);
        labels[0] = defaultLabels;
        currentRound = 0;
    }

    inline void startNewRound() noexcept {
        currentRound = labels.size();
        labels.emplace_back(labels.back());
    }

    inline void startNewRound(const size_t round) noexcept {
        while (round >= labels.size()) {
            labels.emplace_back(labels.back());
        }
        currentRound = round;
    }

    inline StopIndex operator()(const TripId trip) const noexcept {
        AssertMsg(trip < labels[currentRound].size(), "Trip " << trip << " is out of bounds!");
        return StopIndex(labels[currentRound][trip]);
    }

    inline StopIndex operator()(const TripId trip, const size_t round) const noexcept {
        const size_t trueRound = std::min(round, labels.size() - 1);
        AssertMsg(trip < labels[trueRound].size(), "Trip " << trip << " is out of bounds!");
        return StopIndex(labels[trueRound][trip]);
    }

    inline bool alreadyReached(const TripId trip, const u_int8_t index) const noexcept {
        return labels[currentRound][trip] <= index;
    }

    inline void update(const TripId trip, const StopIndex index) noexcept {
        AssertMsg(trip < labels[currentRound].size(), "Trip " << trip << " is out of bounds!");
        const TripId routeEnd = data.firstTripOfRoute[data.routeOfTrip[trip] + 1];
        for (TripId i = trip; i < routeEnd; i++) {
            if (labels[currentRound][i] <= index) break;
            labels[currentRound][i] = index;
        }
    }

    inline void updateCopyForward(const TripId trip, const StopIndex index) noexcept {
        AssertMsg(trip < labels[currentRound].size(), "Trip " << trip << " is out of bounds!");
        const TripId routeEnd = data.firstTripOfRoute[data.routeOfTrip[trip] + 1];
        for (TripId i = trip; i < routeEnd; i++) {
            if (labels[currentRound][i] <= index) break;
            labels[currentRound][i] = index;
            for (size_t round = currentRound + 1; round < labels.size(); round++) {
                if (labels[round][i] <= index) break;
                labels[round][i] = index;
            }
        }
    }

private:
    const Data& data;

    std::vector<std::vector<u_int8_t>> labels;

    std::vector<u_int8_t> defaultLabels;

    size_t currentRound;

};

}
