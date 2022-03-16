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

using alias = std::vector<std::string>;

inline double degreesToRadians(const double radius) {
    return (radius / 180.0) * M_PI;
}

inline double radiansToDegrees(const double radius) {
    return (radius / M_PI) * 180;
}

inline int mostSignificantBit(int i) {
    return __builtin_clz(i);
}

inline long long mostSignificantBit(long long i) {
    return __builtin_clzll(i);
}

inline int leastSignificantBit(int i) {
    return ffs(i);
}

inline long int leastSignificantBit(long int i) {
    return ffsl(i);
}

inline long long int leastSignificantBit(long long int i) {
    return ffsll(i);
}

inline void sleep(int milliSeconds) {
    std::chrono::milliseconds timespan(milliSeconds);
    std::this_thread::sleep_for(timespan);
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
