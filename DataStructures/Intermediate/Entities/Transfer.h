#pragma once

static_assert(false, "Deprecated!");

#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/IO/Serialization.h"

namespace Intermediate {

class Transfer {

public:
    Transfer(const StopId fromStopId = noStop, const StopId toStopId = noStop, const int minTransferTime = 0) :
        fromStopId(fromStopId),
        toStopId(toStopId),
        minTransferTime(minTransferTime) {
    }
    Transfer(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline bool operator<(const Transfer& t) const noexcept {
        return (fromStopId < t.fromStopId) || ((fromStopId == t.fromStopId) && (
               (toStopId < t.toStopId)));
    }

    friend std::ostream& operator<<(std::ostream& out, const Transfer& t) {
        return out << "Transfer{" << t.fromStopId << ", " << t.toStopId << ", " << t.minTransferTime << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(fromStopId, toStopId, minTransferTime);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(fromStopId, toStopId, minTransferTime);
    }

public:
    StopId fromStopId{noStop};
    StopId toStopId{noStop};
    int minTransferTime{0};

};

}
