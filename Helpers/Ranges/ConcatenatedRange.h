#pragma once

#include <vector>

#include "../Assert.h"

template<typename RANGE, typename ELEMENT>
class ConcatenatedRange {

public:
    using Range = RANGE;
    using Element = ELEMENT;
    using Type = ConcatenatedRange<Range, Element>;

public:
    class Iterator {
    public:
        Iterator(const ConcatenatedRange* const concatenatedRange, const size_t i) : concatenatedRange(concatenatedRange), i(i) {}
        inline bool operator!=(const Iterator& other) const noexcept {return i != other.i;}
        inline Element operator*() const noexcept {return (*concatenatedRange)[i];}
        inline Iterator& operator++() noexcept {++i; return *this;}
        inline Iterator& operator+=(const size_t n) noexcept {i += n; return *this;}
        inline Iterator operator+(const size_t n) const noexcept {return Iterator(concatenatedRange, i + n);}
        inline Element operator[](const size_t n) const noexcept {return (*concatenatedRange)[i + n];}
    private:
        const ConcatenatedRange* concatenatedRange;
        size_t i;
    };

    ConcatenatedRange() :
        offset(0) {
    }

    ConcatenatedRange(const Range& firstRange, const Range& secondRange, const Element offset) :
        firstRange(firstRange),
        secondRange(secondRange),
        offset(offset) {
    }

    inline Iterator begin() const noexcept {
        return Iterator(this, 0);
    }

    inline Iterator end() const noexcept {
        return Iterator(this, size());
    }

    inline bool empty() const noexcept {
        return firstRange.empty() && secondRange.empty();
    }

    inline size_t size() const noexcept {
        return firstRange.size() + secondRange.size();
    }

    inline Element operator[](const size_t i) const noexcept {
        AssertMsg(i < size(), "Index " << i << " is out of range!");
        if (i < firstRange.size()) {
            return firstRange[i];
        } else {
            return secondRange[i - firstRange.size()] + offset;
        }
    }
    inline Element front() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        return operator[](0);
    }

    inline Element back() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        return operator[](size() - 1);
    }

private:
    Range firstRange;
    Range secondRange;
    Element offset;

};
