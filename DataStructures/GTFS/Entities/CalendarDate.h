#pragma once

#include <fstream>
#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/Calendar.h"
#include "../../../Helpers/IO/Serialization.h"

namespace GTFS {

class CalendarDate {

public:
    CalendarDate(const std::string& serviceId = "", const int date = -1, const bool operates = true) :
        serviceId(serviceId),
        date(date),
        operates(operates) {
    }
    CalendarDate(const std::string& serviceId, const std::string& date, const bool operates = true) :
        serviceId(serviceId),
        date(stringToDay(date)),
        operates(operates) {
    }
    CalendarDate(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline bool validate() noexcept {
        return !serviceId.empty();
    }

    friend std::ostream& operator<<(std::ostream& out, const CalendarDate& c) {
        return out << "CalendarDate{" << c.serviceId << ", " << c.date << ", " << c.operates << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(serviceId, date, operates);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(serviceId, date, operates);
    }

public:
    std::string serviceId{""};
    int date{-1};
    bool operates{true};

};

}
