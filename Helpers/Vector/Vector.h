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

    template<typename T, typename COUNT_IF, typename = decltype(std::declval<COUNT_IF>()(std::declval<T>()))>
    inline size_t count(const std::vector<T>& container, const COUNT_IF& countIf) {
        size_t c = 0;
        for (size_t i = 0; i < container.size(); i++) {
            if (countIf(container[i])) c++;
        }
        return c;
    }

    template<typename T>
    inline size_t indexOf(const std::vector<T>& vec, const T& elem) {
        for (size_t i = 0; i < vec.size(); i++) {
            if (vec[i] == elem) return i;
        }
        return static_cast<size_t>(-1);
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

    template<typename T>
    inline T& remove(T& array, size_t index) {
        Assert(index >= 0);
        Assert(index < array.size());
        array[index] = array.back();
        array.pop_back();
        return array;
    }

    template<typename T, typename FUNCTION>
    inline void removeIf(std::vector<T>& container, const FUNCTION& condition) noexcept {
        container.erase(std::remove_if(container.begin(), container.end(), condition), container.end());
    }

    template<typename T, typename FUNCTION>
    inline void removeIf(std::vector<T>& container, const size_t beginIndex, const FUNCTION& condition) noexcept {
        AssertMsg(beginIndex < container.size(), "Index " << beginIndex << " is out of bounds! (" << container.size() << ")");
        container.erase(std::remove_if(container.begin() + beginIndex, container.end(), condition), container.end());
    }

    template<typename T, typename FUNCTION>
    inline void removeIf(std::vector<T>& container, const size_t beginIndex, const size_t endIndex, const FUNCTION& condition) noexcept {
        AssertMsg(beginIndex < container.size(), "Index " << beginIndex << " is out of bounds! (" << container.size() << ")");
        AssertMsg(endIndex <= container.size(), "Index " << endIndex << " is out of bounds! (" << container.size() << ")");
        container.erase(std::remove_if(container.begin() + beginIndex, container.begin() + endIndex, condition), container.begin() + endIndex);
    }

    template<typename T>
    inline bool contains(const std::vector<T>& container, const T& element) {
        for (size_t i = 0; i < container.size(); i++) {
            if (container[i] == element) {
                return true;
            }
        }
        return false;
    }

    template<typename T, bool ALLOW_DUPLICATES = false>
    inline void insertSorted(std::vector<T>& container, const T& element) {
        size_t insertionIndex = container.size();
        while ((insertionIndex > 0) && container[insertionIndex - 1] >= element) {
            insertionIndex--;
        }
        if (insertionIndex == container.size()) {
            container.emplace_back(element);
        } else {
            if constexpr (!ALLOW_DUPLICATES) {
                if (container[insertionIndex] == element) return;
            }
            container.emplace_back(container.back());
            for (size_t i = container.size() - 2; i > insertionIndex; i-- ) {
                container[i] = container[i - 1];
            }
            container[insertionIndex] = element;
        }
    }

    template<typename T>
    inline bool equals(const std::vector<T>& a, const std::vector<T>& b) {
        if (a.size() != b.size()) return false;
        for (size_t i = 0; i < a.size(); i++) {
            if (!(a[i] == b[i])) return false;
        }
        return true;
    }

    template<typename T>
    inline bool equals(const std::vector<std::vector<T>>& a, const std::vector<std::vector<T>>& b) {
        if (a.size() != b.size()) return false;
        for (size_t i = 0; i < a.size(); i++) {
            if (!equals(a[i], b[i])) return false;
        }
        return true;
    }

    template<typename T, typename LESS>
    inline bool isSorted(const std::vector<T>& vec, const LESS& less) {
        for (size_t i = 0; i + 1 < vec.size(); i++) {
            if (less(vec[i + 1], vec[i])) return false;
        }
        return true;
    }

    template<typename T>
    inline bool isSorted(const std::vector<T>& vec) {
        return isSorted(vec, [](const T& a, const T& b){return a < b;});
    }

    template<typename T>
    inline std::vector<T> sortedIntersection(const std::vector<T>& a, const std::vector<T>& b) noexcept {
        std::vector<T> result;
        AssertMsg(isSorted(a), "First vector is not sorted!");
        AssertMsg(isSorted(b), "Second vector is not sorted!");
        std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(result));
        return result;
    }

    template<typename T>
    inline std::vector<T> sortedUnion(const std::vector<T>& a, const std::vector<T>& b) noexcept {
        std::vector<T> result;
        AssertMsg(isSorted(a), "First vector is not sorted!");
        AssertMsg(isSorted(b), "Second vector is not sorted!");
        std::set_union(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(result));
        return result;
    }

    template<typename T, typename U, typename COMPARE>
    inline size_t lowerBound(const std::vector<T>& vec, const U& val, const COMPARE& compare) noexcept {
        AssertMsg(isSorted(vec), "Vector is not sorted!");
        return std::lower_bound(vec.begin(), vec.end(), val, compare) - vec.begin();
    }

    template<typename T, typename U, typename COMPARE>
    inline size_t upperBound(const std::vector<T>& vec, const U& val, const COMPARE& compare) noexcept {
        AssertMsg(isSorted(vec), "Vector is not sorted!");
        return std::upper_bound(vec.begin(), vec.end(), val, compare) - vec.begin();
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

    template<typename T>
    inline long long memoryUsageInBytes(const std::vector<T>& vec) {
        return sizeof(std::vector<T>) + (sizeof(T) * vec.capacity());
    }

    template<>
    inline long long memoryUsageInBytes(const std::vector<bool>& vec) {
        return vec.capacity() /  8;
    }

    template<typename T>
    inline long long memoryUsageInBytes(const std::vector<std::vector<T>>& vec) {
        long long size = sizeof(std::vector<std::vector<T>>);
        for (const std::vector<T>& e : vec) size += memoryUsageInBytes(e);
        return size + ((vec.capacity() - vec.size()) * sizeof(std::vector<T>));
    }

    template<typename T, typename LESS>
    inline T max(const std::vector<T>& vec, const LESS& less) {
        Assert(!vec.empty());
        T result = vec.front();
        for (const T& element : vec) {
            if (less(result, element)) result = element;
        }
        return result;
    }

    template<typename T, typename LESS>
    inline T min(const std::vector<T>& vec, const LESS& less) {
        Assert(!vec.empty());
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

    template<typename T, typename LESS>
    inline std::array<T, 2> twoSmallestValues(const std::vector<T>& vec, const LESS& less) {
        Assert(!vec.empty());
        std::array<T, 2> result{std::numeric_limits<T>::max(), std::numeric_limits<T>::max()};
        for (const T& element : vec) {
            if (less(element, result[1])) {
                if (less(element, result[0])) {
                    result[1] = result[0];
                    result[0] = element;
                } else {
                    result[1] = element;
                }
            }
        }
        return result;
    }

    template<typename T>
    inline std::array<T, 2> twoSmallestValues(const std::vector<T>& vec) {
        return twoSmallestValues(vec, [](const T& a, const T& b){return a < b;});
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
    inline double mean(const std::vector<T>& vec) {
        long double sum = 0;
        for (const T& element : vec) {
            sum += element;
        }
        return static_cast<double>(sum / vec.size());
    }

    template<typename T, typename EVALUATE>
    inline double mean(const std::vector<T>& vec, const EVALUATE& evaluate) {
        long double sum = 0;
        for (const T& element : vec) {
            sum += evaluate(element);
        }
        return static_cast<double>(sum / vec.size());
    }

    template<typename T, typename EVALUATE>
    inline double percentile(const std::vector<T>& sortedData, const double p, const EVALUATE& evaluate) noexcept {
        AssertMsg(!sortedData.empty(), "Percentile is not defined for empty data sets!");
        AssertMsg(p >= 0, "Percentile cannot be negative!");
        AssertMsg(p <= 1, "Percentile cannot be greater than one!");
        if (sortedData.size() == 1) return evaluate(sortedData.front());
        const double index = (sortedData.size() - 1) * p;
        const size_t lowerIndex = index;
        const size_t higherIndex = lowerIndex + 1;
        if (higherIndex == sortedData.size()) return evaluate(sortedData.back());
        const double lambda = higherIndex - index;
        return (lambda * evaluate(sortedData[lowerIndex])) + ((1 - lambda) * evaluate(sortedData[higherIndex]));
    }

    template<typename T>
    inline double percentile(const std::vector<T>& sortedData, const double p) noexcept {
        return percentile(sortedData, p, [&](const T& element) {
            return element;
        });
    }

    template<typename T, typename EVALUATE>
    inline double median(const std::vector<T>& sortedData, const EVALUATE& evaluate) noexcept {
        return percentile(sortedData, 0.5, evaluate);
    }

    template<typename T>
    inline double median(const std::vector<T>& sortedData) noexcept {
        return percentile(sortedData, 0.5);
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

    template<typename T, typename STREAM = std::ostream>
    inline void printConcise(const std::vector<T>& vector, STREAM& out = std::cout, const std::string& separator = ", ") noexcept {
        for (size_t i = 0; i < vector.size(); i++) {
            out << vector[i];
            if (i != vector.size() - 1) out << separator;
        }
    }

    template<typename T, typename TO_STRING, typename STREAM = std::ostream>
    inline void printConciseMapped(const std::vector<T>& vector, const TO_STRING& toString, STREAM& out = std::cout, const std::string& separator = ", ") noexcept {
        for (size_t i = 0; i < vector.size(); i++) {
            out << toString(vector[i]);
            if (i != vector.size() - 1) out << separator;
        }
    }

    inline double difference(const std::vector<bool>& firstVector, const std::vector<bool>& secondVector) noexcept {
        AssertMsg(firstVector.size() == secondVector.size(), "Vectors have different sizes!");
        if (firstVector.size() != secondVector.size()) return -1;
        size_t common = 1;
        size_t different = 0;
        for (size_t i = 0; i < firstVector.size(); i++) {
            if (firstVector[i] && secondVector[i]) {
                common++;
            } else if (firstVector[i] != secondVector[i]) {
                different++;
            }
        }
        return different / (double)common;
    }

    template<typename T>
    inline std::vector<T> flatten(const std::vector<std::vector<T>>& vector) noexcept {
        std::vector<T> result;
        for (const std::vector<T>& e : vector) {
            result += e;
        }
        return result;
    }

    template<typename T, typename F>
    inline std::vector<decltype(std::declval<F>()(std::declval<T>()))> map(const std::vector<T>& vector, const F& function) noexcept {
        std::vector<decltype(std::declval<F>()(std::declval<T>()))> result;
        for (const T& e : vector) {
            result.emplace_back(function(e));
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

template<typename T>
inline std::vector<T>& operator+=(std::vector<T>& a, std::vector<T>&& b) {
    a.reserve(a.size() + b.size());
    std::move(b.begin(), b.end(), std::back_inserter(a));
    b.clear();
    return a;
}

namespace std {

    template<typename T>
    inline std::ostream& operator<<(std::ostream& out, const std::vector<T>& a) {
        for (size_t i = 0; i < a.size(); i++) {
            out << a[i] << std::endl;
        }
        return out;
    }

}
