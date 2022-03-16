#pragma once

#include <iostream>
#include <vector>
#include <string>

namespace GTFS {

namespace Type {
    constexpr int Tram = 0;
    constexpr int Subway = 1;
    constexpr int Rail = 2;
    constexpr int Bus = 3;
    constexpr int Ferry = 4;
    constexpr int CableCar = 5;
    constexpr int Gondola = 6;
    constexpr int Funicular = 7;
    constexpr int Undefined = -1;
}

const std::string TypeNames[] = {"Tram", "Subway", "Rail", "Bus", "Ferry", "Cable Car", "Gondola", "Funicular"};
const std::vector<int> Types = {3, 2, 4, 5, 6, 7, 1, 0};

inline std::string typeName(const int type) noexcept {
    if (type < 0 || type > 7) return "Undefined";
    return TypeNames[type];
}

}
