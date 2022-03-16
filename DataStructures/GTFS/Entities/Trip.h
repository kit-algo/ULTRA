#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/IO/Serialization.h"

namespace GTFS {

class Trip {

public:
    Trip(const std::string& routeId = "", const std::string& serviceId = "", const std::string& tripId = "", const std::string& name = "") :
        routeId(routeId),
        serviceId(serviceId),
        tripId(tripId),
        name(name) {
    }
    Trip(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline bool validate() noexcept {
        if (name.empty()) name = "NOT_NAMED";
        return (!routeId.empty()) && (!tripId.empty()) && (!tripId.empty());
    }

    friend std::ostream& operator<<(std::ostream& out, const Trip& t) {
        return out << "Trip{" << t.routeId << ", " << t.serviceId << ", " << t.tripId << ", " << t.name << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(routeId, serviceId, tripId, name);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(routeId, serviceId, tripId, name);
    }

public:
    std::string routeId{""};
    std::string serviceId{""};
    std::string tripId{""};
    std::string name{""};

};

}
