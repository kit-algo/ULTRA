#pragma once

#include "../../../Helpers/Meta.h"
#include "../../../Helpers/Types.h"

namespace RAPTOR {

struct SeparatedEarliestArrivalTime {
    SeparatedEarliestArrivalTime() : arrivalTimeByRoute(never), arrivalTimeByTransfer(never) {}

    inline void setArrivalTimeByRoute(const int time) noexcept {
        arrivalTimeByRoute = time;
    }

    inline void setArrivalTimeByTransfer(const int time) noexcept {
        arrivalTimeByTransfer = time;
    }

    inline int getArrivalTimeByRoute() const noexcept {
        return arrivalTimeByRoute;
    }

    inline int getArrivalTimeByTransfer() const noexcept {
        return arrivalTimeByTransfer;
    }

    inline int getArrivalTime() const noexcept {
        return std::min(arrivalTimeByRoute, arrivalTimeByTransfer);
    }

    int arrivalTimeByRoute;
    int arrivalTimeByTransfer;
};

struct CombinedEarliestArrivalTime {
    CombinedEarliestArrivalTime() : arrivalTime(never) {}

    inline void setArrivalTimeByRoute(const int time) noexcept {
        arrivalTime = time;
    }

    inline void setArrivalTimeByTransfer(const int time) noexcept {
        arrivalTime = time;
    }

    inline int getArrivalTimeByRoute() const noexcept {
        return arrivalTime;
    }

    inline int getArrivalTimeByTransfer() const noexcept {
        return arrivalTime;
    }

    inline int getArrivalTime() const noexcept {
        return arrivalTime;
    }

    int arrivalTime;
};

template<bool SEPARATE>
using EarliestArrivalTime = Meta::IF<SEPARATE, SeparatedEarliestArrivalTime, CombinedEarliestArrivalTime>;
}
