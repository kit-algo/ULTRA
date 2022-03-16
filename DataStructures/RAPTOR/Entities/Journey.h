#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/Types.h"

namespace RAPTOR {

class JourneyLeg {

public:
    JourneyLeg(const Vertex from = noVertex, const Vertex to = noVertex, const int departureTime = never, const int arrivalTime = never, const bool usesRoute = false, const RouteId routeId = noRouteId) :
        from(from),
        to(to),
        departureTime(departureTime),
        arrivalTime(arrivalTime),
        usesRoute(usesRoute),
        routeId(routeId) {
    }

    JourneyLeg(const Vertex from, const Vertex to, const int departureTime, const int arrivalTime, const Edge edge) :
        from(from),
        to(to),
        departureTime(departureTime),
        arrivalTime(arrivalTime),
        usesRoute(false),
        transferId(edge) {
    }

    inline int transferTime() const noexcept {
        return usesRoute ? 0 : arrivalTime - departureTime;
    }

    inline friend std::ostream& operator<<(std::ostream& out, const JourneyLeg& leg) noexcept {
        return out << "from: " << leg.from << ", to: " << leg.to << ", dep-Time: " << leg.departureTime << ", arr-Time: " << leg.arrivalTime << (leg.usesRoute ? ", route: " : ", transfer: ") << leg.routeId;
    }

public:
    Vertex from;
    Vertex to;
    int departureTime;
    int arrivalTime;
    bool usesRoute;
    union {
        RouteId routeId;
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

inline int totalTransferTime(const Journey& journey) noexcept {
    int transferTime = 0;
    for (const JourneyLeg& leg : journey) {
        transferTime += leg.transferTime();
    }
    return transferTime;
}

inline int intermediateTransferTime(const Journey& journey) noexcept {
    int transferTime = 0;
    for (size_t i = 1; i < journey.size() - 1; i++) {
        transferTime += journey[i].transferTime();
    }
    return transferTime;
}

inline int initialTransferTime(const Journey& journey) noexcept {
    if (journey.empty()) return 0;
    int transferTime = journey[0].transferTime();
    if (journey.size() > 1) {
        transferTime += journey.back().transferTime();
    }
    return transferTime;
}

inline size_t countTrips(const Journey& journey) noexcept {
    size_t numTrips = 0;
    for (const JourneyLeg& leg : journey) {
        if (leg.usesRoute && leg.routeId != noRouteId) numTrips++;
    }
    return numTrips;
}

}
