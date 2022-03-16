#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/String/String.h"
#include "../../../Helpers/IO/Serialization.h"

namespace GTFS {

class Frequency {

public:
    Frequency(const std::string& tripId = "", const int startTime = -1, const int endTime = -2, const int headwaySecs = 0, const bool exactTimes = true) :
        tripId(tripId),
        startTime(startTime),
        endTime(endTime),
        headwaySecs(headwaySecs),
        exactTimes(exactTimes) {
    }
    Frequency(const std::string& tripId, const std::string& startTime, const std::string& endTime, const int headwaySecs = 0, const bool exactTimes = true) :
        tripId(tripId),
        startTime(String::parseSeconds(startTime)),
        endTime(String::parseSeconds(endTime)),
        headwaySecs(headwaySecs),
        exactTimes(exactTimes) {
    }
    Frequency(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline bool validate() noexcept {
        return (!tripId.empty()) && (startTime <= endTime) && (headwaySecs > 0);
    }

    friend std::ostream& operator<<(std::ostream& out, const Frequency& f) {
        return out << "Frequency{" << f.tripId << ", " << f.startTime << ", " << f.endTime << ", " << f.headwaySecs << ", " << f.exactTimes << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(tripId, startTime, endTime, headwaySecs, exactTimes);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(tripId, startTime, endTime, headwaySecs, exactTimes);
    }

public:
    std::string tripId{""};
    int startTime{-1};
    int endTime{-2};
    int headwaySecs{0};
    bool exactTimes{true};

};

}
