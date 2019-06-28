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

#include <tuple>
#include <vector>
#include <utility>

#include "../Assert.h"

template<typename RANGE_A, typename RANGE_B>
class SimultaneousRange {

public:
    using RangeA = RANGE_A;
    using RangeB = RANGE_B;
    using Type = SimultaneousRange<RangeA, RangeB>;

    using IteratorA = decltype(std::declval<const RangeA*>()->begin());
    using IteratorB = decltype(std::declval<const RangeB*>()->begin());
    using Element = decltype(std::make_tuple(*(std::declval<IteratorA>()), *(std::declval<IteratorB>())));

public:
    class Iterator {
    public:
        Iterator(const IteratorA& a, const IteratorB& b) : a(a), b(b) {}
        inline bool operator!=(const Iterator& other) const noexcept {return a != other.a;}
        inline Element operator*() const noexcept {return std::make_tuple(*a, *b);}
        inline Iterator& operator++() noexcept {++a; ++b; return *this;}
        inline Iterator& operator+=(const size_t n) noexcept {a += n; b += n; return *this;}
        inline Iterator operator+(const size_t n) const noexcept {return Iterator(a + n, b + n);}
        inline Element operator[](const size_t n) const noexcept {return std::make_tuple(a[n], b[n]);}
    private:
        IteratorA a;
        IteratorB b;
    };

    SimultaneousRange() :
        rangeA(nullptr),
        rangeB(nullptr) {
    }

    SimultaneousRange(const RangeA& rangeA, const RangeB& rangeB) :
        rangeA(&rangeA),
        rangeB(&rangeB) {
        AssertMsg(rangeA.size() == rangeB.size(), "Both ranges must have the same size (" << rangeA.size() << ", " << rangeB.size() << ")!");
    }

    SimultaneousRange(const RangeA&&, const RangeB&) = delete;
    SimultaneousRange(const RangeA&, const RangeB&&) = delete;

    inline Iterator begin() const noexcept {
        return (rangeA) ? Iterator(rangeA->begin(), rangeB->begin()) : Iterator(fallbackRange.begin(), RangeB().begin());
    }

    inline Iterator end() const noexcept {
        return (rangeA) ? Iterator(rangeA->end(), rangeB->end()) : Iterator(fallbackRange.end(), RangeB().end());
    }

    inline bool empty() const noexcept {
        return (rangeA) ? rangeA->empty() : true;
    }

    inline size_t size() const noexcept {
        return (rangeA) ? rangeA->size() : 0;
    }

    inline Element operator[](const size_t i) const noexcept {
        AssertMsg(i < size(), "Index " << i << " is out of range!");
        return std::make_tuple((*rangeA)[i], (*rangeB)[i]);
    }
    inline Element front() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        return std::make_tuple(rangeA->front(), rangeB->front());
    }

    inline Element back() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        return std::make_tuple(rangeA->back(), rangeB->back());
    }

private:
    const RangeA fallbackRange;
    const RangeA* rangeA;
    const RangeB* rangeB;

};
