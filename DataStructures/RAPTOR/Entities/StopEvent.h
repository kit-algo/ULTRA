#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/IO/Serialization.h"
#include "../../Intermediate/Entities/StopEvent.h"

namespace RAPTOR {

class StopEvent {

public:
    StopEvent(const int arrivalTime = -1, const int departureTime = -2) :
        arrivalTime(arrivalTime),
        departureTime(departureTime) {
    }
    StopEvent(const Intermediate::StopEvent& s) :
        arrivalTime(s.arrivalTime),
        departureTime(s.departureTime) {
    }
    StopEvent(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline StopEvent reverseStopEvent() const noexcept {
        return StopEvent(-departureTime, -arrivalTime);
    }

    friend std::ostream& operator<<(std::ostream& out, const StopEvent& s) {
        return out << "StopEvent{" << s.arrivalTime  << ", " << s.departureTime << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(arrivalTime, departureTime);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(arrivalTime, departureTime);
    }

public:
    int arrivalTime{-1};
    int departureTime{-2};

};

}
