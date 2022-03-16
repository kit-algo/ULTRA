#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/Types.h"
#include "../../../Helpers/IO/Serialization.h"

namespace RAPTOR {

class RouteSegment {

public:
    RouteSegment(const RouteId routeId = noRouteId, const StopIndex stopIndex = noStopIndex) :
        routeId(routeId),
        stopIndex(stopIndex) {
    }
    RouteSegment(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    friend std::ostream& operator<<(std::ostream& out, const RouteSegment& r) {
        return out << "RouteSegment{" << r.routeId << ", " << r.stopIndex << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(routeId, stopIndex);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(routeId, stopIndex);
    }

    inline bool operator<(const RouteSegment& other) const noexcept {
        return routeId < other.routeId || (routeId == other.routeId && stopIndex < other.stopIndex);
    }

public:
    RouteId routeId;
    StopIndex stopIndex;

};

}
