#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/IO/Serialization.h"

namespace GTFS {

class StopTime {

public:
    StopTime(const std::string& tripId = "", const int arrivalTime = -1, const int departureTime = -2, const std::string& stopId = "", const int stopSequence = -1) :
        tripId(tripId),
        arrivalTime(arrivalTime),
        departureTime(departureTime),
        stopId(stopId),
        stopSequence(stopSequence) {
    }
    StopTime(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline bool validate() noexcept {
        return (!tripId.empty()) && (!stopId.empty()) && (arrivalTime <= departureTime);
    }

    inline bool operator<(const StopTime& s) const noexcept {
        return (tripId < s.tripId) || ((tripId == s.tripId) && (
               (stopSequence < s.stopSequence) || ((stopSequence == s.stopSequence) && (
               (arrivalTime < s.arrivalTime) || ((arrivalTime == s.arrivalTime) && (
               (departureTime < s.departureTime)))))));
    }

    friend std::ostream& operator<<(std::ostream& out, const StopTime& s) {
        return out << "StopTime{" << s.tripId << ", " << s.arrivalTime  << ", " << s.departureTime << ", " << s.stopId  << ", " << s.stopSequence << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(tripId, arrivalTime, departureTime, stopId, stopSequence);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(tripId, arrivalTime, departureTime, stopId, stopSequence);
    }

public:
    std::string tripId{""};
    int arrivalTime{-1};
    int departureTime{-2};
    std::string stopId{""};
    int stopSequence{-1};

};

}
