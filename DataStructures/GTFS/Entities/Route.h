#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "Vehicle.h"

#include "../../../Helpers/IO/Serialization.h"

namespace GTFS {

class Route {

public:
    Route(const std::string& routeId = "", const std::string& agencyId = "", const std::string& name = "", const int type = -1, const std::string& routeColor = "FFFFFF", const std::string& textColor = "000000") :
        routeId(routeId),
        agencyId(agencyId),
        name(name),
        type(type),
        routeColor(routeColor),
        textColor(textColor) {
    }
    Route(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline bool validate() noexcept {
        if (name.empty()) name = "NOT_NAMED";
        if (!String::isColor(routeColor)) routeColor = "FFFFFF";
        if (!String::isColor(textColor)) textColor = "000000";
        return !routeId.empty();
    }

    friend std::ostream& operator<<(std::ostream& out, const Route& r) {
        return out << "Route{" << r.routeId << ", " << r.agencyId << ", " << r.name << ", " << r.type << ", " << r.routeColor << ", " << r.textColor << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(routeId, agencyId, name, type, routeColor, textColor);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(routeId, agencyId, name, type, routeColor, textColor);
    }

public:
    std::string routeId{""};
    std::string agencyId{""};
    std::string name{""};
    int type{-1};
    std::string routeColor{"FFFFFF"};
    std::string textColor{"000000"};

};

}
