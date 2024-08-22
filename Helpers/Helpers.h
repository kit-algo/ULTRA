#pragma once

#include <set>
#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <ctime>
#include <thread>
#include <chrono>

#include <strings.h>
#include <string.h>

#include "HighlightText.h"

inline double degreesToRadians(const double radius) {
    return (radius / 180.0) * M_PI;
}

inline double radiansToDegrees(const double radius) {
    return (radius / M_PI) * 180;
}

template<typename T>
inline T floor(const T value, const T unit) noexcept {
    return std::floor((value - unit) / unit) * unit;
}

template<typename T>
inline T ceil(const T value, const T unit) noexcept {
    return std::ceil((value + unit) / unit) * unit;
}

template<typename T>
inline void sort(std::vector<T>& data) noexcept {
    std::sort(data.begin(), data.end());
}

template<typename T, typename LESS>
inline void sort(std::vector<T>& data, const LESS& less) noexcept {
    std::sort(data.begin(), data.end(), less);
}

template<typename T>
inline void suppressUnusedParameterWarning(const T&) noexcept {}

inline int branchlessConditional(const bool predicate, const int ifTrue, const int ifFalse) noexcept {
    int result = 0;
    __asm__ __volatile__(
        "mov    %[ifTrue], %[result]\n\t"
        "test   %[predicate], %[predicate]\n\t"
        "cmovz  %[ifFalse], %[result]\n\t"
        : [result] "=&r" (result)
        : [predicate] "r" (predicate), [ifTrue] "r" (ifTrue), [ifFalse] "r" (ifFalse)
        : "cc");
    return result;
}
