#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/IO/Serialization.h"

namespace GTFS {

class Agency {

public:
    Agency(const std::string& agencyId = "",const std::string& name = "", const std::string& timezone = "") :
        agencyId(agencyId),
        name(name),
        timezone(timezone) {
    }
    Agency(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline bool validate() noexcept {
        if (name.empty()) name = "NOT_NAMED";
        return !agencyId.empty();
    }

    friend std::ostream& operator<<(std::ostream& out, const Agency& a) {
        return out << "Agency{" << a.agencyId << ", " << a.name << ", " << a.timezone << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(agencyId, name, timezone);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(agencyId, name, timezone);
    }

public:
    std::string agencyId{""};
    std::string name{""};
    std::string timezone{""};

};

}
