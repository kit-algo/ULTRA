#pragma once

#include <limits>
#include <vector>
#include <string>

#include "TaggedInteger.h"

using Vertex = TaggedInteger<0, u_int32_t, -u_int32_t(1)>;
constexpr Vertex noVertex(Vertex::InvalidValue);

using Edge = TaggedInteger<1, u_int32_t, -u_int32_t(1)>;
constexpr Edge noEdge(Edge::InvalidValue);

using StopId = DependentTaggedInteger<2, Vertex>;
constexpr StopId noStop(StopId::InvalidValue);

using RouteId = TaggedInteger<3, u_int32_t, -u_int32_t(1)>;
constexpr RouteId noRouteId(RouteId::InvalidValue);

using TripId = TaggedInteger<4, u_int32_t, -u_int32_t(1)>;
constexpr TripId noTripId(TripId::InvalidValue);

using StopIndex = TaggedInteger<5, u_int32_t, -u_int32_t(1)>;
constexpr StopIndex noStopIndex(StopIndex::InvalidValue);

using ConnectionId = TaggedInteger<6, u_int32_t, -u_int32_t(1)>;
constexpr ConnectionId noConnection(ConnectionId::InvalidValue);

using StopEventId = TaggedInteger<8, u_int32_t, -u_int32_t(1)>;
constexpr StopEventId noStopEvent(StopEventId::InvalidValue);

inline constexpr int intMax = std::numeric_limits<int>::max();
inline constexpr double doubleMax = std::numeric_limits<double>::max();
inline constexpr int INFTY = std::numeric_limits<int>::max() / 2;
inline constexpr int never = INFTY;

inline constexpr int EARTH_RADIUS_IN_CENTIMETRE = 637813700;

inline constexpr int FORWARD = 0;
inline constexpr int BACKWARD = 1;

struct NO_OPERATION {
    template<typename... ARGS>
    constexpr inline bool operator() (ARGS...) const noexcept {return false;}
};

NO_OPERATION NoOperation;
