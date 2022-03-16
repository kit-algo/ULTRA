#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/IO/Serialization.h"

namespace RAPTOR {

class Route {

public:
    Route(const std::string& name = "", const int type = -1) :
        name(name),
        type(type) {
    }
    Route(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    friend std::ostream& operator<<(std::ostream& out, const Route& r) {
        return out << "Route{" << r.name << ", " << r.type << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(name, type);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(name, type);
    }

public:
    std::string name{""};
    int type{-1};

};

}
