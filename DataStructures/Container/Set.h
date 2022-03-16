#pragma once

#include <set>
#include <limits>
#include <bitset>

#include "../../Helpers/Types.h"
#include "../../Helpers/Helpers.h"
#include "../../Helpers/ConstructorTags.h"
#include "../../Helpers/IO/Serialization.h"
#include "../../Helpers/Vector/Vector.h"
#include "../../Helpers/Vector/Permutation.h"

template<typename VALUE>
class Set : public std::set<VALUE> {

public:
    using Value = VALUE;
    using Type = Set<Value>;

private:
    using Super = std::set<Value>;

public:
    inline bool contains(const Value& value) const noexcept {
        return Super::count(value) == 1;
    }

};

template<bool RESIZEABLE = false, typename VALUE_TYPE = size_t>
class IndexedSet {

public:
    static constexpr bool Resizeable = RESIZEABLE;
    using ValueType = VALUE_TYPE;
    using Type = IndexedSet<Resizeable, ValueType>;

    inline static constexpr size_t NotContained = std::numeric_limits<size_t>::max();
    using Iterator = typename std::vector<ValueType>::const_iterator;

public:
    IndexedSet(const size_t initialCapacity = 0) :
        indices(initialCapacity, NotContained) {
    }

    IndexedSet(const size_t initialCapacity, const std::vector<ValueType>& values) :
        indices(initialCapacity, NotContained) {
        for (const ValueType value : values) {
            insert(value);
        }
    }

    IndexedSet(const std::vector<ValueType>& values) :
        IndexedSet(ValueType(Vector::max(values) + 1), values) {
    }

    IndexedSet(const Construct::CompleteTag, const size_t size) :
        indices(Vector::id<size_t>(size)),
        values(Vector::id<ValueType>(size)) {
    }

    IndexedSet(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    template<typename T>
    inline operator std::vector<T>() const noexcept {
        std::vector<T> result;
        result.reserve(values.size());
        for (const ValueType value : values) {
            result.emplace_back(value);
        }
        return result;
    }

    inline const std::vector<ValueType>& getValues() const noexcept {
        return values;
    }

    inline void sortValues() noexcept {
        sort(values);
    }

    inline Iterator begin() const noexcept {
        return values.begin();
    }

    inline Iterator end() const noexcept {
        return values.end();
    }

    inline size_t size() const noexcept {
        return values.size();
    }

    inline bool empty() const noexcept {
        return values.empty();
    }

    inline size_t capacity() const noexcept {
        return indices.size();
    }

    inline ValueType operator[](const size_t i) const noexcept {
        AssertMsg(i < size(), "Index " << i << " is out of range!");
        return values[i];
    }

    inline ValueType front() const noexcept {
        AssertMsg(!empty(), "The set is empty!");
        return values.front();
    }

    inline ValueType back() const noexcept {
        AssertMsg(!empty(), "The set is empty!");
        return values.back();
    }

    inline bool contains(const ValueType value) const noexcept {
        AssertMsg(value < capacity(), "Value " << value << " is out of range!");
        return indices[value] != NotContained;
    }

    inline bool contains(const ValueType value) noexcept {
        if (Resizeable) {
            if ((size_t)value >= capacity()) indices.resize(value + 1, NotContained);
        } else {
            AssertMsg((size_t)value < capacity(), "Value " << value << " is out of range!");
        }
        return indices[value] != NotContained;
    }

    inline bool insert(const ValueType value) noexcept {
        if (contains(value)) return false;
        indices[value] = values.size();
        values.emplace_back(value);
        return true;
    }

    template<typename TYPE>
    inline void insert(const std::vector<TYPE>& range) noexcept {
        for (const ValueType id : range) {
            insert(id);
        }
    }

    inline bool remove(const ValueType value) noexcept {
        if (!contains(value)) return false;
        values[indices[value]] = values.back();
        indices[values.back()] = indices[value];
        indices[value] = NotContained;
        values.pop_back();
        return true;
    }

    inline void sort() noexcept {
        std::sort(values.begin(), values.end());
        for (size_t i = 0; i < values.size(); i++) {
            indices[values[i]] = i;
        }
    }

    inline void clear() noexcept {
        for (const ValueType value : values) {
            indices[value] = NotContained;
        }
        values.clear();
    }

    inline void applyPermutation(const Permutation& permutation) noexcept {
        permutation.permutate(indices);
        permutation.mapPermutation(values);
    }

    inline bool operator==(const Type& other) const noexcept {
        if (size() != other.size()) return false;
        for (const ValueType id : other) {
            if (!contains(id)) return false;
        }
        return true;
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(indices, values);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(indices, values);
    }

    inline long long byteSize() const noexcept {
        long long result = Vector::byteSize(indices);
        result += Vector::byteSize(values);
        return result;
    }

private:
    std::vector<size_t> indices;
    std::vector<ValueType> values;

};

template<size_t CAPACITY = 32, typename VALUE_TYPE = size_t>
class SmallSet {

public:
    inline constexpr static size_t Capacity = CAPACITY;
    using ValueType = VALUE_TYPE;
    using Type = SmallSet<Capacity, ValueType>;

    class Iterator {
    public:
        Iterator(const std::bitset<Capacity>* data, const size_t i) : data(data), i(i) {}
        inline bool operator!=(const Iterator& other) const noexcept {return i != other.i;}
        inline ValueType operator*() const noexcept {return ValueType(i);}
        inline Iterator& operator++() noexcept {do {++i;} while (i < Capacity && !(*data)[i]); return *this;}
        inline Iterator& operator+=(const size_t n) noexcept {for (size_t j = 0; j < n; j++) ++(*this); return *this;}
        inline Iterator operator+(const size_t n) const noexcept {return Iterator(*this) += n;}
        inline ValueType operator[](const size_t n) const noexcept {return *(*this + n);}
    private:
        const std::bitset<Capacity>* data;
        size_t i;
    };

public:
    SmallSet() {}
    SmallSet(const std::vector<ValueType>& values) {
        for (const ValueType value : values) {
            insert(value);
        }
    }

    inline std::vector<ValueType> getValues() const noexcept {
        std::vector<ValueType> values;
        for (const ValueType value : *this) {
            values.emplace_back(value);
        }
        return values;
    }

    inline Iterator begin() const noexcept {
        return ++Iterator(&data, -1);
    }

    inline Iterator end() const noexcept {
        return Iterator(&data, Capacity);
    }

    inline size_t size() const noexcept {
        return data.count();
    }

    inline bool empty() const noexcept {
        return data.none();
    }

    inline size_t capacity() const noexcept {
        return Capacity;
    }

    inline ValueType operator[](const size_t i) const noexcept {
        AssertMsg(i < size(), "Index " << i << " is out of range!");
        return begin()[i];
    }

    inline bool contains(const ValueType value) noexcept {
        AssertMsg(value < capacity(), "Value " << value << " is out of range!");
        return data[value];
    }

    inline bool insert(const ValueType value) noexcept {
        if (contains(value)) return false;
        data[value] = true;
        return true;
    }

    inline bool remove(const ValueType value) noexcept {
        if (!contains(value)) return false;
        data[value] = false;
        return true;
    }

    inline void clear() noexcept {
        data.reset();
    }

private:
    std::bitset<Capacity> data;

};
