/**********************************************************************************

 Copyright (c) 2019 Tobias Zündorf

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
 modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************************/

#pragma once

inline double degreesToRadians(const double radius) {
    return (radius / 180.0) * M_PI;
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
