#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "String/String.h"
#include "Ranges/Range.h"

const std::string dayName[7]{"sunday", "monday", "tuesday", "wednesday", "thursday", "friday", "saturday"};

const Range<int> week{0, 7};

inline int stringToDay(const std::string& time) {
    if (time.size() != 8) error("The string " + time + " is not in the format YYYYMMDD");
    int year = String::lexicalCast<int>(time.substr(0, 4)) - 1900;
    int month = String::lexicalCast<int>(time.substr(4, 2)) - 1;
    int day = String::lexicalCast<int>(time.substr(6, 2));
    std::tm t = {0,0,12,day,month,year, 0, 0, 0, 0, 0};
    time_t seconds = std::mktime(&t);
    return (seconds < 0) ? (seconds / (60 * 60 * 24)) - 1 : (seconds / (60 * 60 * 24));
}

inline std::string dayToString(const int day) {
    time_t seconds = (((time_t)day * 24) + 12) * 60 * 60;
    std::tm t = *std::localtime(&seconds);
    std::string result = std::to_string(t.tm_mday);
    while (result.length() < 2) result = "0" + result;
    result = std::to_string(t.tm_mon + 1) + result;
    while (result.length() < 4) result = "0" + result;
    result = std::to_string(t.tm_year + 1900) + result;
    while (result.length() < 8) result = "0" + result;
    return result;
}

inline int weekday(const int day) {
    time_t seconds = (((time_t)day * 24) + 12) * 60 * 60;
    std::tm t = *std::localtime(&seconds);
    return t.tm_wday;
}
