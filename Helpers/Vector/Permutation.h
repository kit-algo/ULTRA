#pragma once

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <random>

#include "Vector.h"

#include "../Meta.h"
#include "../Assert.h"
#include "../ConstructorTags.h"
#include "../IO/Serialization.h"

/*
 * Permutation = maps old IDs to new IDs
 * Order       = maps new IDs to old IDs
 */

#ifdef _GLIBCXX_DEBUG
namespace std {
    inline void swap(std::vector<bool>::reference a, std::vector<bool>::reference b) noexcept {
        const bool temp = a;
        a = b;
        b = temp;
    }
}
#endif

class IdMapping : public std::vector<size_t> {

protected:
    IdMapping(const size_t n = 0) : std::vector<size_t>(n) {}
    IdMapping(const std::vector<size_t>& data) : std::vector<size_t>(data) {}
    IdMapping(std::vector<size_t>&& data) : std::vector<size_t>(std::move(data)) {}
    IdMapping(const std::string& fileName) {this->deserialize(fileName);}
    IdMapping(IO::Deserialization& deserialize) {this->deserialize(deserialize);}

    template<typename T, typename = std::enable_if_t<!Meta::Equals<T, size_t>()>>
    IdMapping(const std::vector<T>& data) {
        reserve(data.size());
        for (const size_t i : data) {
            emplace_back(i);
        }
    }

    IdMapping(const Construct::IdTag, const size_t n) :
        std::vector<size_t>() {
        reserve(n);
        for (size_t i = 0; i < n; i++) {
            emplace_back(i);
        }
    }

    IdMapping(const Construct::RandomTag, const size_t n, const int seed = 42) :
        IdMapping(Construct::Id, n) {
        std::mt19937 randomGenerator(seed);
        std::uniform_int_distribution<> distribution(0, n - 1);
        for (size_t i = 0; i < n; i++) {
            std::swap((*this)[i], (*this)[distribution(randomGenerator)]);
        }
    }

    template<typename T>
    IdMapping(const Construct::SortTag, const std::vector<T>& vector) :
        IdMapping(Construct::Id, vector.size()) {
        std::sort(begin(), end(), [&](const size_t& a, const size_t& b){return vector[a] < vector[b];});
    }

    template<typename T, typename COMPARE>
    IdMapping(const Construct::SortTag, const std::vector<T>& vector, const COMPARE& comp) :
        IdMapping(Construct::Id, vector.size()) {
        std::sort(begin(), end(), [&](const size_t& a, const size_t& b){return comp(vector[a], vector[b]);});
    }

    template<typename T>
    IdMapping(const Construct::InvertTag, const T& original) :
        std::vector<size_t>(original.size()) {
        AssertMsg(original.isValid(), "The original is not valid!");
        for (size_t i = 0; i < size(); i++) {
            (*this)[original[i]] = i;
        }
    }

    template<typename T>
    IdMapping(const Construct::InvertTag, T&& original) :
        std::vector<size_t>(std::move(original)) {
        AssertMsg(isValid(), "The original is not valid!");
        std::vector<bool> seen(size(), false);
        for (size_t i = 0; i < size(); i++) {
            if (seen[i]) continue;
            size_t value = i;
            size_t index = (*this)[value];
            while (!seen[index]) {
                seen[index] = true;
                (*this)[i] = (*this)[index];
                (*this)[index] = value;
                value = index;
                index = (*this)[i];
            }
        }
    }

    inline std::vector<size_t>& vector() noexcept {
        return *this;
    }

    inline const std::vector<size_t>& vector() const noexcept {
        return *this;
    }

public:
    inline bool isValid() const noexcept {
        std::vector<bool> seen(size(), false);
        for (const size_t entry : (*this)) {
            if (entry >= size()) return false;
            if (seen[entry]) return false;
            seen[entry] = true;
        }
        return true;
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(vector());
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(vector());
    }

    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, vector());
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, vector());
    }

};

class Order;

class Permutation : public IdMapping {

public:
    Permutation(const size_t n = 0) : IdMapping(n) {}
    explicit Permutation(const std::vector<size_t>& data) : IdMapping(data) {}
    explicit Permutation(std::vector<size_t>&& data) : IdMapping(std::move(data)) {}
    Permutation(const std::string& fileName) {this->deserialize(fileName);}
    Permutation(IO::Deserialization& deserialize) {this->deserialize(deserialize);}

    template<typename T, typename = std::enable_if_t<!Meta::Equals<T, size_t>()>>
    explicit Permutation(const std::vector<T>& data) : IdMapping(data) {}

    Permutation(const Construct::IdTag, const size_t n) :
        IdMapping(Construct::Id, n) {
    }

    Permutation(const Construct::RandomTag, const size_t n, const int seed = 42) :
        IdMapping(Construct::Random, n, seed) {
    }

    Permutation(const Construct::InvertTag, const Order& order) :
        IdMapping(Construct::Invert, order) {
    }

    Permutation(const Construct::InvertTag, Order&& order) :
        IdMapping(Construct::Invert, std::move(order)) {
    }

public:
    template<typename T>
    inline std::vector<T> getPermuted(const std::vector<T>& vector) const noexcept {
        AssertMsg(vector.size() == size(), "Cannot permute a vector of size " << vector.size() << " with a permutation of size " << size() << "!");
        AssertMsg(isValid(), "The permutation is not valid!");
        std::vector<T> result(size());
        for (size_t i = 0; i < size(); i++) {
            result[(*this)[i]] = vector[i];
        }
        return result;
    }

    template<typename T>
    inline T permutate(const T element) const noexcept {
        return T((*this)[element]);
    }

    template<typename T>
    inline void permutate(std::vector<T>& vector) const noexcept {
        std::vector<T> result = getPermuted(vector);
        vector.swap(result);
    }

    template<typename T>
    inline void mapPermutation(std::vector<T>& vector) const noexcept {
        AssertMsg(isValid(), "The permutation is not valid!");
        mapPermutationImplementation(vector);
    }

    inline Permutation splitAt(const size_t limit) const noexcept;

    inline Permutation extend(const size_t newSize) const noexcept {
        AssertMsg(newSize >= size(), "newSize is smaller than size!");
        Permutation result = *this;
        for (size_t i = size(); i < newSize; i++) {
            result.emplace_back(i);
        }
        return result;
    }

protected:
    template<typename T>
    inline void mapPermutationImplementation(std::vector<T>& vector) const noexcept {
        for (size_t i = 0; i < vector.size(); i++) {
            const size_t element = vector[i];
            if (element < size()) {
                vector[i] = T((*this)[element]);
            }
        }
    }

    template<typename T>
    inline void mapPermutationImplementation(std::vector<std::vector<T>>& vectors) const noexcept {
        for (std::vector<T>& vector : vectors) {
            mapPermutationImplementation(vector);
        }
    }

};

class Order : public IdMapping {

public:
    Order(const size_t n = 0) : IdMapping(n) {}
    explicit Order(const std::vector<size_t>& data) : IdMapping(data) {}
    explicit Order(std::vector<size_t>&& data) : IdMapping(std::move(data)) {}
    Order(const std::string& fileName) {this->deserialize(fileName);}
    Order(IO::Deserialization& deserialize) {this->deserialize(deserialize);}

    template<typename T, typename = std::enable_if_t<!Meta::Equals<T, size_t>()>>
    explicit Order(const std::vector<T>& data) : IdMapping(data) {}

    Order(const Construct::IdTag, const size_t n) :
        IdMapping(Construct::Id, n) {
    }

    Order(const Construct::RandomTag, const size_t n, const int seed = 42) :
        IdMapping(Construct::Random, n, seed) {
    }

    template<typename T>
    Order(const Construct::SortTag, const std::vector<T>& vector) :
        IdMapping(Construct::Sort, vector) {
    }

    template<typename T, typename COMPARE>
    Order(const Construct::SortTag, const std::vector<T>& vector, const COMPARE& comp) :
        IdMapping(Construct::Sort, vector, comp) {
    }

    Order(const Construct::InvertTag, const Permutation& permutation) :
        IdMapping(Construct::Invert, permutation) {
    }

    Order(const Construct::InvertTag, Permutation&& permutation) :
        IdMapping(Construct::Invert, std::move(permutation)) {
    }

    Order(const Construct::FromTextFileTag, const std::string& fileName) {
        std::ifstream file(fileName);
        IO::checkStream(file, fileName);
        std::string line;
        while (std::getline(file, line)) {
            emplace_back(String::lexicalCast<size_t>(line));
        }
    }

public:
    template<typename T>
    inline std::vector<T> getOrdered(const std::vector<T>& vector) const noexcept {
        std::vector<T> result;
        result.reserve(size());
        for (size_t i = 0; i < size(); i++) {
            result.emplace_back(vector[(*this)[i]]);
        }
        return result;
    }

    template<typename T>
    inline void order(std::vector<T>& vector) const noexcept {
        std::vector<T> result = getOrdered(vector);
        vector.swap(result);
    }

    inline Order splitAt(const size_t limit) const noexcept {
        std::vector<size_t> left;
        std::vector<size_t> right;
        for (const size_t i : *this) {
            if (i < limit) {
                left.emplace_back(i);
            } else {
                right.emplace_back(i);
            }
        }
        return Order(left + right);
    }

    inline Order extend(const size_t newSize) const noexcept {
        AssertMsg(newSize >= size(), "newSize is smaller than size!");
        Order result = *this;
        for (size_t i = size(); i < newSize; i++) {
            result.emplace_back(i);
        }
        return result;
    }
};

inline Permutation Permutation::splitAt(const size_t limit) const noexcept {
    return Permutation(Construct::Invert, Order(Construct::Invert, *this).splitAt(limit));
}
