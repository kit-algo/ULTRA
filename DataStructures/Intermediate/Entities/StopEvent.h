#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/IO/Serialization.h"
#include "../../../Helpers/Types.h"
#include "../../../Helpers/Vector/Permutation.h"

namespace Intermediate {

class StopEvent {

public:
    static const std::string CSV_HEADER;

public:
    StopEvent(const StopId stopId = noStop, const int arrivalTime = -1, const int departureTime = -2) :
        stopId(stopId),
        arrivalTime(arrivalTime),
        departureTime(departureTime) {
    }
    StopEvent(const StopEvent& se, const int timeOffset) :
        stopId(se.stopId),
        arrivalTime(se.arrivalTime + timeOffset),
        departureTime(se.departureTime + timeOffset) {
    }
    StopEvent(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline bool operator<(const StopEvent& s) const noexcept {
        return (arrivalTime < s.arrivalTime) || ((arrivalTime == s.arrivalTime) && (
               (departureTime < s.departureTime)));
    }

    friend std::ostream& operator<<(std::ostream& out, const StopEvent& s) {
        return out << "StopEvent{" << s.stopId << ", " << s.arrivalTime  << ", " << s.departureTime << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(stopId, arrivalTime, departureTime);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(stopId, arrivalTime, departureTime);
    }

    inline std::ostream& toCSV(std::ostream& out) const {
        return out << size_t(stopId) << "," << arrivalTime << "," << departureTime;
    }

    inline std::string toCSV() const {
        std::stringstream ss;
        toCSV(ss);
        return ss.str();
    }

    inline bool matches(const StopEvent& other) const noexcept {
        return stopId == other.stopId && departureTime == other.departureTime;
    }

    inline void update(const StopEvent& other) noexcept {
        AssertMsg(matches(other), "Updating a stop event with one that does not match!");
        stopId = other.stopId;
        departureTime = other.departureTime;
        arrivalTime = std::max(arrivalTime, other.arrivalTime);
    }

    inline void applyStopPermutation(const Permutation& permutation) noexcept {
        stopId = permutation.permutate(stopId);
    }

public:
    StopId stopId{noStop};
    int arrivalTime{-1};
    int departureTime{-2};

};

const std::string StopEvent::CSV_HEADER = "stop_id,arr_time,dep_time";

inline bool less(const std::vector<StopEvent>& a, const std::vector<StopEvent>& b) noexcept {
    const size_t size = std::min(a.size(), b.size());
    for (size_t i = 0; i < size; i++) {
        if (a[i].stopId == b[i].stopId) continue;
        return a[i].stopId < b[i].stopId;
    }
    return a.size() < b.size();
}

inline bool equals(const std::vector<StopEvent>& a, const std::vector<StopEvent>& b) noexcept {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); i++) {
        if (a[i].stopId != b[i].stopId) return false;
    }
    return true;
}

}
