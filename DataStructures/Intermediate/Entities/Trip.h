#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "StopEvent.h"

#include "../../../Helpers/IO/Serialization.h"
#include "../../../Helpers/Types.h"

namespace Intermediate {

class Trip {

public:
    static const std::string CSV_HEADER;

public:
    Trip(const std::string& tripName = "", const std::string& routeName = "", const int type = -1) :
        tripName(tripName),
        routeName(routeName),
        type(type) {
    }
    template<typename TRIP_TYPE>
    Trip(const TRIP_TYPE& t) :
        tripName(t.tripName),
        routeName(t.routeName),
        type(t.type) {
    }
    Trip(const Trip& t, const int timeOffset) :
        tripName(t.tripName),
        routeName(t.routeName),
        type(t.type) {
        for (const StopEvent se : t.stopEvents) {
            stopEvents.emplace_back(se, timeOffset);
        }
    }
    Trip(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline bool operator<(const Trip& t) const noexcept {
        if (less(stopEvents, t.stopEvents)) return true;
        if (!equals(stopEvents, t.stopEvents)) return false;
        for (size_t i = 0; i < stopEvents.size(); i++) {
            if (stopEvents[i].departureTime < t.stopEvents[i].departureTime) return true;
            if (stopEvents[i].departureTime > t.stopEvents[i].departureTime) return false;
            if (stopEvents[i].arrivalTime > t.stopEvents[i].arrivalTime) return true;
            if (stopEvents[i].arrivalTime < t.stopEvents[i].arrivalTime) return false;
        }
        return false;
    }

    inline bool operator==(const Trip& t) const noexcept {
        if (!equals(stopEvents, t.stopEvents)) return false;
        for (size_t i = 0; i < stopEvents.size(); i++) {
            if (stopEvents[i].departureTime != t.stopEvents[i].departureTime) return false;
            if (stopEvents[i].arrivalTime != t.stopEvents[i].arrivalTime) return false;
        }
        return true;
    }

    inline bool dominates(const Trip& t) const noexcept {
        if (!equals(stopEvents, t.stopEvents)) return false;
        for (size_t i = 0; i < stopEvents.size(); i++) {
            if (stopEvents[i].departureTime < t.stopEvents[i].departureTime) return false;
            if (stopEvents[i].arrivalTime > t.stopEvents[i].arrivalTime) return false;
        }
        return true;
    }

    friend std::ostream& operator<<(std::ostream& out, const Trip& t) {
        return out << "Trip{" << t.routeName << ", " << t.tripName  << ", " << t.type  << ", " << t.stopEvents.size() << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(stopEvents, tripName, routeName, type);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(stopEvents, tripName, routeName, type);
    }

    inline std::ostream& toCSV(std::ostream& out) const {
        return out << "\"" << tripName << "\",\"" << routeName << "\"," << type;
    }

    inline std::string toCSV() const {
        std::stringstream ss;
        toCSV(ss);
        return ss.str();
    }

    inline bool matches(const Trip& other) const noexcept {
        if (stopEvents.size() != other.stopEvents.size()) return false;
        for (size_t i = 0; i < stopEvents.size(); i++) {
            if (!stopEvents[i].matches(other.stopEvents[i])) return false;
        }
        return true;
    }

    inline void update(const Trip& other) noexcept {
        AssertMsg(matches(other), "Updating a trip with one that does not match!");
        for (size_t i = 0; i < stopEvents.size(); i++) {
            stopEvents[i].update(other.stopEvents[i]);
        }
        tripName = other.tripName;
        routeName = other.routeName;
        type = other.type;
    }

    inline void shiftInTime(const int amount) noexcept {
        for (StopEvent& stopEvent : stopEvents) {
            stopEvent.departureTime += amount;
            stopEvent.arrivalTime += amount;
        }
    }

    inline void applyStopPermutation(const Permutation& permutation) noexcept {
        for (StopEvent& stopEvent : stopEvents) {
            stopEvent.applyStopPermutation(permutation);
        }
    }

public:
    std::vector<StopEvent> stopEvents{};
    std::string tripName{""};
    std::string routeName{""};
    int type{-1};

};

const std::string Trip::CSV_HEADER = "name,route,vehicle";

inline bool isFiFo(const Trip& a, const Trip& b) noexcept {
    AssertMsg(a.stopEvents.size() == b.stopEvents.size(), "FiFO property can only be tested for trips of equal size!");
    for (size_t i = 0; i < a.stopEvents.size(); i++) {
        if (a.stopEvents[i].arrivalTime > b.stopEvents[i].arrivalTime) return false;
        if (a.stopEvents[i].departureTime > b.stopEvents[i].departureTime) return false;
    }
    return true;
}

inline bool isOffset(const Trip& a, const Trip& b) noexcept {
    AssertMsg(a.stopEvents.size() == b.stopEvents.size(), "Shifted property can only be tested for trips of equal size!");
    if (a.stopEvents.size() == 0) return true;
    int ofset = a.stopEvents.front().departureTime - b.stopEvents.front().departureTime;
    if (a.stopEvents.back().arrivalTime - b.stopEvents.back().arrivalTime != ofset) return false;
    for (size_t i = 1; i < a.stopEvents.size() - 1; i++) {
        if (a.stopEvents[i].arrivalTime - b.stopEvents[i].arrivalTime != ofset) return false;
        if (a.stopEvents[i].departureTime - b.stopEvents[i].departureTime != ofset) return false;
    }
    return true;
}

}
