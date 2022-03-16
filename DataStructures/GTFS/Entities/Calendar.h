#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <array>

#include "../../../Helpers/Calendar.h"
#include "../../../Helpers/IO/Serialization.h"

namespace GTFS {

class Calendar {

public:
    Calendar(const std::string& serviceId = "", const std::array<bool, 7>& operatesOnWeekday = {false, false, false, false, false, false, false}, const int startDate = -1, const int endDate = -2) :
        serviceId(serviceId),
        operatesOnWeekday(operatesOnWeekday),
        startDate(startDate),
        endDate(endDate) {
    }
    Calendar(const std::string& serviceId, const std::array<bool, 7>& operatesOnWeekday, const std::string& startDate, const std::string& endDate) :
        serviceId(serviceId),
        operatesOnWeekday(operatesOnWeekday),
        startDate(stringToDay(startDate)),
        endDate(stringToDay(endDate)) {
    }
    Calendar(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline bool validate() noexcept {
        return (!serviceId.empty()) && (startDate <= endDate);
    }

    friend std::ostream& operator<<(std::ostream& out, const Calendar& c) {
        out << "Calendar{" << c.serviceId << ", ";
        for (const int day : week) out << c.operatesOnWeekday[day] << ", ";
        return out << c.startDate << ", " << c.endDate << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(serviceId, operatesOnWeekday, startDate, endDate);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(serviceId, operatesOnWeekday, startDate, endDate);
    }

public:
    std::string serviceId{""};
    std::array<bool, 7> operatesOnWeekday{{false, false, false, false, false, false, false}};
    int startDate{-1};
    int endDate{-2};

};

}
