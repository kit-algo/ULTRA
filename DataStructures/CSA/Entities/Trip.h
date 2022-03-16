#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../Intermediate/Entities/Trip.h"

#include "../../../Helpers/IO/Serialization.h"

namespace CSA {

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
    Trip(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }
    friend std::ostream& operator<<(std::ostream& out, const Trip& t) {
        return out << "Trip{" << t.routeName << ", " << t.tripName  << ", " << t.type << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(tripName, routeName, type);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(tripName, routeName, type);
    }

public:
    std::string tripName{""};
    std::string routeName{""};
    int type{-1};

};

const std::string Trip::CSV_HEADER = "name,vehicle";

}
