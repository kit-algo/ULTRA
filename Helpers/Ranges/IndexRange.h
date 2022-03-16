#pragma once

#include <vector>

#include "../Assert.h"

template<typename INDEX, typename RANGE>
class IndexRange {

public:
    using Index = INDEX;
    using Range = RANGE;
    using Type = IndexRange<Index, Range>;

public:
    class Iterator {
    public:
        Iterator(const Index* const index, const Range* const range, const size_t i) : index(index), range(range), i(i) {}
        inline bool operator!=(const Iterator& other) const noexcept {return i != other.i;}
        inline auto operator*() const noexcept {return (*index)[(*range)[i]];}
        inline Iterator& operator++() noexcept {++i; return *this;}
        inline Iterator& operator+=(const size_t n) noexcept {i += n; return *this;}
        inline Iterator operator+(const size_t n) const noexcept {return Iterator(index, range, i + n);}
        inline auto operator[](const size_t n) const noexcept {return (*index)[(*range)[i + n]];}
    private:
        const Index* index;
        const Range* range;
        size_t i;
    };

    IndexRange() :
        index(nullptr),
        range(nullptr),
        beginIndex(0),
        endIndex(0) {
    }

    IndexRange(const Index& index, const Range& range, const size_t beginIndex, const size_t endIndex) :
        index(&index),
        range(&range),
        beginIndex(beginIndex),
        endIndex(endIndex) {
    }

    IndexRange(const Index& index, const Range& range, const size_t beginIndex = 0) :
        index(&index),
        range(&range),
        beginIndex(beginIndex),
        endIndex(range.size()) {
    }

    IndexRange(const Index&&, const Range&, const size_t = 0, const size_t = 0) = delete;
    IndexRange(const Index&, const Range&&, const size_t = 0, const size_t = 0) = delete;

    inline Iterator begin() const noexcept {
        return Iterator(index, range, beginIndex);
    }

    inline Iterator end() const noexcept {
        return Iterator(index, range, endIndex);
    }

    inline bool empty() const noexcept {
        return endIndex <= beginIndex;
    }

    inline size_t size() const noexcept {
        return endIndex - beginIndex;
    }

    inline auto operator[](const size_t i) const noexcept {
        AssertMsg(i < size(), "Index " << i << " is out of range!");
        return (*index)[(*range)[beginIndex + i]];
    }

    inline auto front() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        return (*index)[(*range)[beginIndex]];
    }

    inline auto back() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        return (*index)[(*range)[endIndex - 1]];
    }

private:
    const Index* index;
    const Range* range;
    size_t beginIndex;
    size_t endIndex;

};
