#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../Geometry/Point.h"
#include "../../GTFS/Entities/Stop.h"
#include "../../../Helpers/IO/Serialization.h"
#include "../../../Helpers/String/String.h"

namespace Intermediate {

class Stop {

public:
    static const std::string CSV_HEADER;

public:
    Stop(const std::string& name = "", const Geometry::Point& coordinates = Geometry::Point(), const int minTransferTime = 0) :
        name(name),
        coordinates(coordinates),
        minTransferTime(minTransferTime) {
    }
    Stop(const GTFS::Stop& s) :
        name(s.name),
        coordinates(s.coordinates),
        minTransferTime(0) {
    }
    template<typename STOP_TYPE>
    Stop(const STOP_TYPE& s) :
        name(s.name),
        coordinates(s.coordinates),
        minTransferTime(s.minTransferTime) {
    }
    Stop(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    friend std::ostream& operator<<(std::ostream& out, const Stop& s) {
        return out << "Stop{" << s.name  << ", " << s.coordinates  << ", " << s.minTransferTime << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(name, coordinates, minTransferTime);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(name, coordinates, minTransferTime);
    }

    inline std::ostream& toCSV(std::ostream& out) const {
        return out << coordinates.longitude << "," << coordinates.latitude << ",\"" << name << "\"," << minTransferTime;
    }

    inline std::string toCSV() const {
        std::stringstream ss;
        toCSV(ss);
        return ss.str();
    }

    inline bool matches(const Stop& other) const noexcept {
        return Geometry::geoDistanceInCM(coordinates, other.coordinates) < 300;
    }

    inline void update(const Stop& other) noexcept {
        AssertMsg(matches(other), "Updating a stop with one that does not match!");
        coordinates = other.coordinates;
        name = other.name;
        minTransferTime = std::max(minTransferTime, other.minTransferTime);
    }

public:
    std::string name{""};
    Geometry::Point coordinates{};
    int minTransferTime{0};

};

const std::string Stop::CSV_HEADER = "lon,lat,name,change_time";

}
