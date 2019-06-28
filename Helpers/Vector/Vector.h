/**********************************************************************************

 Copyright (c) 2019 Jonas Sauer, Tobias Zündorf

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

#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <limits>

#include "../Meta.h"
#include "../Helpers.h"
#include "../Assert.h"

namespace Vector {

    template<typename T>
    inline std::vector<T> id(const size_t size) {
        std::vector<T> result;
        result.reserve(size);
        while (result.size() < size) {
            result.emplace_back(T(result.size()));
        }
        return result;
    }

    template<typename T>
    inline size_t count(const std::vector<T>& container, const T& element) {
        size_t c = 0;
        for (size_t i = 0; i < container.size(); i++) {
            if (container[i] == element) c++;
        }
        return c;
    }

    template<typename T>
    inline std::vector<T> reverse(std::vector<T>&& a) {
        std::vector<T> result(a);
        std::reverse(result.begin(), result.end());
        return result;
    }

    template<typename T>
    inline std::vector<T>& reverse(std::vector<T>& a) {
        std::reverse(a.begin(), a.end());
        return a;
    }

    template<typename T, typename U, typename COMPARE>
    inline size_t lowerBound(const std::vector<T>& vec, const U& val, const COMPARE& compare) noexcept {
        AssertMsg(isSorted(vec), "Vector is not sorted!");
        return std::lower_bound(vec.begin(), vec.end(), val, compare) - vec.begin();
    }

    template<typename T>
    inline long long byteSize(const std::vector<T>& vec) {
        return sizeof(T) * vec.size();
    }

    template<>
    inline long long byteSize(const std::vector<bool>& vec) {
        return vec.size();
    }

    template<typename T>
    inline long long byteSize(const std::vector<std::vector<T>>& vec) {
        long long size = 0;
        for (const std::vector<T>& e : vec) size += sizeof(std::vector<T>) + byteSize(e);
        return size;
    }

    template<typename T, typename LESS>
    inline T max(const std::vector<T>& vec, const LESS& less) {
        AssertMsg(!vec.empty(), "Vector is empty!");
        T result = vec.front();
        for (const T& element : vec) {
            if (less(result, element)) result = element;
        }
        return result;
    }

    template<typename T, typename LESS>
    inline T min(const std::vector<T>& vec, const LESS& less) {
        AssertMsg(!vec.empty(), "Vector is empty!");
        T result = vec.front();
        for (const T& element : vec) {
            if (less(element, result)) result = element;
        }
        return result;
    }

    template<typename T>
    inline T max(const std::vector<T>& vec) {
        return max(vec, [](const T& a, const T& b){return a < b;});
    }

    template<typename T>
    inline T min(const std::vector<T>& vec) {
        return min(vec, [](const T& a, const T& b){return a < b;});
    }

    template<typename T>
    inline T sum(const std::vector<T>& vec) {
        T result = 0;
        for (const T& element : vec) {
            result += element;
        }
        return result;
    }

    template<typename T, typename EVALUATE>
    inline T sum(const std::vector<T>& vec, const EVALUATE& evaluate) {
        T result = 0;
        for (const T& element : vec) {
            result += evaluate(element);
        }
        return result;
    }

    template<typename T, typename U, typename EVALUATE>
    inline T sum(const std::vector<U>& vec, const EVALUATE& evaluate) {
        T result = 0;
        for (const U& element : vec) {
            result += evaluate(element);
        }
        return result;
    }

    template<typename T>
    inline void fill(std::vector<T>& vector, const T& value = T()) noexcept {
        std::fill(vector.begin(), vector.end(), value);
    }

    template<typename T, typename U, typename = std::enable_if_t<!Meta::Equals<T, U>()>>
    inline void assign(std::vector<T>& to, const std::vector<U>& from) noexcept {
        to.clear();
        to.reserve(from.size());
        for (const U& u : from) {
            to.push_back(u);
        }
    }

    template<typename T, typename U>
    inline void assign(std::vector<std::vector<T>>& to, const std::vector<std::vector<U>>& from) noexcept {
        to.clear();
        to.resize(from.size());
        for (size_t i = 0; i < from.size(); i++) {
            assign(to[i], from[i]);
        }
    }

    template<typename T>
    inline void assign(std::vector<std::vector<T>>& to, const std::vector<std::vector<T>>& from) noexcept {
        to = from;
    }

    template<typename T>
    inline void assign(std::vector<T>& to, const std::vector<T>& from) noexcept {
        to = from;
    }

    template<typename T>
    inline void assign(std::vector<T>& to, std::vector<T>&& from) noexcept {
        to = std::move(from);
    }

    inline std::vector<uint8_t> packBool(const std::vector<bool>& vector) noexcept {
        std::vector<uint8_t> result;
        result.resize(std::ceil(vector.size() / 8.0), 0);
        size_t vectorIndex = 0;
        size_t resultIndex = 0;
        while (vectorIndex + 7 < vector.size()) {
            uint8_t& resultValue = result[resultIndex];
            for (size_t i = 0; i < 8; i++) {
                resultValue = (resultValue << 1) | static_cast<uint8_t>(vector[vectorIndex]);
                vectorIndex++;
            }
            resultIndex++;
        }
        if (resultIndex == result.size()) return result;
        uint8_t& resultValue = result[resultIndex];
        for (size_t i = 0; i < 8; i++) {
            resultValue = (resultValue << 1) | static_cast<uint8_t>((vectorIndex < vector.size()) ? (vector[vectorIndex]) : (false));
            vectorIndex++;
        }
        return result;
    }

    inline constexpr uint8_t bitMask = (1 << 7);

    inline std::vector<bool> unpackBool(const std::vector<uint8_t>& vector) noexcept {
        std::vector<bool> result;
        result.reserve(vector.size() * 8);
        for (size_t i = 0; i < vector.size(); i++) {
            uint8_t vectorValue = vector[i];
            for (size_t j = 0; j < 8; j++) {
                result.emplace_back(vectorValue & bitMask);
                vectorValue = vectorValue << 1;
            }
        }
        return result;
    }

}

template<typename T>
inline std::vector<T> operator+(const T& a, const std::vector<T>& b) {
    std::vector<T> result;
    for (const T& t : b) {
        result.emplace_back(a + t);
    }
    return result;
}

template<typename T>
inline std::vector<T> operator+(std::vector<T>&& a, const std::vector<T>& b) {
    std::vector<T> result;
    result.swap(a);
    result.reserve(result.size() + b.size());
    result.insert(result.end(), b.begin(), b.end());
    return result;
}

template<typename T>
inline std::vector<T> operator+(const std::vector<T>& a, const std::vector<T>& b) {
    std::vector<T> result;
    result.reserve(a.size() + b.size());
    result.insert(result.end(), a.begin(), a.end());
    result.insert(result.end(), b.begin(), b.end());
    return result;
}

template<typename T>
inline std::vector<T>& operator+=(std::vector<T>& a, const std::vector<T>& b) {
    a.reserve(a.size() + b.size());
    a.insert(a.end(), b.begin(), b.end());
    return a;
}
//
//namespace std {
//
//    template<typename T>
//    inline std::ostream& operator<<(std::ostream& out, const std::vector<T>& a) {
//        for (size_t i = 0; i < a.size(); i++) {
//            out << a[i] << std::endl;
//        }
//        return out;
//    }
//
//}
