#pragma once

#include <algorithm>
#include "../../Helpers/Types.h"

struct DistanceBounds {
    DistanceBounds() : lowerBound(INFTY), upperBound(0) { }

    inline void update(const int distance) noexcept {
        lowerBound = std::min(lowerBound, distance);
        upperBound = std::max(upperBound, distance);
    }

    int lowerBound;
    int upperBound;
};
