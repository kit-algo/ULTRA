#pragma once

#include "../../Helpers/Types.h"

namespace TripBased {

struct Shortcut {
    Shortcut(const StopEventId origin, const StopEventId destination, const int walkingDistance = 0) :
        origin(origin),
        destination(destination),
        walkingDistance(walkingDistance) {
    }

    StopEventId origin;
    StopEventId destination;
    int walkingDistance;
};

}
