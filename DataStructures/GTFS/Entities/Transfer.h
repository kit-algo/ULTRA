#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/IO/Serialization.h"

namespace GTFS {

class Transfer {

public:
    Transfer(const std::string& fromStopId = "", const std::string& toStopId = "", const int minTransferTime = 0) :
        fromStopId(fromStopId),
        toStopId(toStopId),
        minTransferTime(minTransferTime) {
    }
    Transfer(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline bool validate() noexcept {
        return (!fromStopId.empty()) && (!toStopId.empty()) && (minTransferTime >= 0);
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
    std::string fromStopId{""};
    std::string toStopId{""};
    int minTransferTime{0};

};

}
