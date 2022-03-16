#pragma once

#include <vector>

#include "../Assert.h"

template<typename RANGE>
class ReverseRange {

public:
    using Range = RANGE;
    using Type = ReverseRange<Range>;

public:
    class Iterator {
    public:
        Iterator(const Range* const range, const size_t i) : range(range), i(i) {}
        inline bool operator!=(const Iterator& other) const noexcept {return i != other.i;}
        inline auto operator*() const noexcept {return (*range)[i];}
        inline Iterator& operator++() noexcept {--i; return *this;}
        inline Iterator& operator+=(const size_t n) noexcept {i -= n; return *this;}
        inline Iterator operator+(const size_t n) const noexcept {return Iterator(range, i - n);}
        inline auto operator[](const size_t n) const noexcept {return (*range)[i - n];}
    private:
        const Range* range;
        size_t i;
    };

    ReverseRange() :
        range(nullptr) {
    }

    ReverseRange(const Range& range) :
        range(&range) {
    }

    ReverseRange(const Range&&) = delete;

    inline Iterator begin() const noexcept {
        return Iterator(range, size() - 1);
    }

    inline Iterator end() const noexcept {
        return Iterator(range, static_cast<size_t>(-1));
    }

    inline bool empty() const noexcept {
        return (range) ? range->empty() : true;
    }

    inline size_t size() const noexcept {
        return (range) ? range->size() : 0;
    }

    inline auto operator[](const size_t i) const noexcept {
        AssertMsg(i < size(), "Index " << i << " is out of range!");
        return (*range)[range->size() - 1 - i];
    }

    inline auto front() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        return range->back();
    }

    inline auto back() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        return range->front();
    }

private:
    const Range* range;

};

template<typename RANGE>
inline ReverseRange<RANGE> descending(const RANGE& range) {
    return ReverseRange<RANGE>(range);
}

template<typename RANGE>
inline ReverseRange<RANGE> descending(const RANGE&&) = delete;
