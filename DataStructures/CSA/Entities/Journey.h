#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/Types.h"

namespace CSA {

class JourneyLeg {

public:
    JourneyLeg(const Vertex from = noVertex, const Vertex to = noVertex, const int departureTime = never, const int arrivalTime = never, const Edge transferId = noEdge) :
        from(from),
        to(to),
        departureTime(departureTime),
        arrivalTime(arrivalTime),
        usesTrip(false),
        transferId(transferId) {
    }

    JourneyLeg(const Vertex from, const Vertex to, const int departureTime, const int arrivalTime, const TripId tripId) :
        from(from),
        to(to),
        departureTime(departureTime),
        arrivalTime(arrivalTime),
        usesTrip(true),
        tripId(tripId) {
    }

    inline friend std::ostream& operator<<(std::ostream& out, const JourneyLeg& leg) noexcept {
        return out << "from: " << leg.from << ", to: " << leg.to << ", dep-Time: " << leg.departureTime << ", arr-Time: " << leg.arrivalTime << (leg.usesTrip ? ", trip: " : ", transfer: ") << leg.tripId;
    }

public:
    Vertex from;
    Vertex to;
    int departureTime;
    int arrivalTime;
    bool usesTrip;
    union {
        TripId tripId;
        Edge transferId;
    };

};

using Journey = std::vector<JourneyLeg>;

inline std::vector<Vertex> journeyToPath(const Journey& journey) noexcept {
    std::vector<Vertex> path;
    for (const JourneyLeg& leg : journey) {
        path.emplace_back(leg.from);
    }
    path.emplace_back(journey.back().to);
    return path;
}

}
