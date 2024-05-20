#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/Types.h"
#include "../../../Helpers/IO/Serialization.h"

namespace RAPTOR {

#ifdef USE_SMALLER_ROUTE_SEG
class RouteSegment {

public:
    RouteSegment(const RouteId routeId = noRouteId, const StopIndex stopIndex = noStopIndex) :
        value((routeId << 8) | stopIndex) {
            Ensure(routeId == noRouteId || routeId < (1<<24), "Route " << routeId << " out of bounds");
            Ensure(stopIndex == noStopIndex || stopIndex < (1<<8), "StopIndex " << stopIndex << " out of bounds");
    }
    RouteSegment(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline RouteId getRouteId() const noexcept {
        return static_cast<RouteId>(value >> 8);
    }

    inline StopIndex getStopIndex() const noexcept {
        return static_cast<StopIndex>(value & ((1<<8)-1));
    }

    friend std::ostream& operator<<(std::ostream& out, const RouteSegment& r) {
        return out << "[COMPACT] RouteSegment{" << r.getRouteId() << ", " << r.getStopIndex() << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(value);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(value);
    }

    inline bool operator<(const RouteSegment& other) const noexcept {
        return value < other.value;
    }

public:
    uint32_t value;

};
#else
class RouteSegment {

public:
    RouteSegment(const RouteId routeId = noRouteId, const StopIndex stopIndex = noStopIndex) :
        routeId(routeId),
        stopIndex(stopIndex) {
    }
    RouteSegment(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline RouteId getRouteId() const noexcept {
        return routeId;
    }

    inline StopIndex getStopIndex() const noexcept {
        return stopIndex;
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

    // made it private in order to enforce using the getRouteId() / getStopIndex() method
private:
    RouteId routeId;
    StopIndex stopIndex;

};
#endif
}

