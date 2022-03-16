#pragma once

#include "../../../Helpers/Types.h"

namespace RAPTOR {

struct Shortcut {
    Shortcut(const StopId origin, const StopId destination, const int travelTime = 0) :
        origin(origin),
        destination(destination),
        travelTime(travelTime) {
    }

    StopId origin;
    StopId destination;
    int travelTime;
};

}
