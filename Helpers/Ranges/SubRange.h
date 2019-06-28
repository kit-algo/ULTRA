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

#include <vector>

#include "../Assert.h"

template<typename RANGE>
class SubRange {

public:
    using Range = RANGE;
    using Type = SubRange<Range>;

public:
    class Iterator {
    public:
        Iterator(const Range* const range, const size_t i) : range(range), i(i) {}
        inline bool operator!=(const Iterator& other) const noexcept {return i != other.i;}
        inline auto operator*() const noexcept {return (*range)[i];}
        inline Iterator& operator++() noexcept {++i; return *this;}
        inline Iterator& operator+=(const size_t n) noexcept {i += n; return *this;}
        inline Iterator operator+(const size_t n) const noexcept {return Iterator(range, i + n);}
        inline auto operator[](const size_t n) const noexcept {return (*range)[i + n];}
    private:
        const Range* range;
        size_t i;
    };

    SubRange() :
        range(nullptr),
        beginIndex(0),
        endIndex(0) {
    }

    SubRange(const Range& range, const size_t beginIndex = 0, const size_t endIndex = 0) :
        range(&range),
        beginIndex(beginIndex),
        endIndex(endIndex) {
    }

    template<typename INDEX_TYPE_A, typename INDEX_TYPE_B>
    SubRange(const Range& range, const std::vector<INDEX_TYPE_A>& indices, const INDEX_TYPE_B index) :
        range(&range),
        beginIndex(indices[index]),
        endIndex(indices[index + 1]) {
    }

    SubRange(const Range&&, const size_t = 0, const size_t = 0) = delete;

    template<typename INDEX_TYPE_A, typename INDEX_TYPE_B>
    SubRange(const Range&& range, const std::vector<INDEX_TYPE_A>&, const INDEX_TYPE_B) = delete;

    inline Iterator begin() const noexcept {
        return Iterator(range, beginIndex);
    }

    inline Iterator end() const noexcept {
        return Iterator(range, endIndex);
    }

    inline bool empty() const noexcept {
        return endIndex <= beginIndex;
    }

    inline size_t size() const noexcept {
        return endIndex - beginIndex;
    }

    inline auto operator[](const size_t i) const noexcept {
        AssertMsg(i < size(), "Index " << i << " is out of range!");
        return (*range)[beginIndex + i];
    }

    inline auto front() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        return (*range)[beginIndex];
    }

    inline auto back() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        return (*range)[endIndex - 1];
    }

private:
    const Range* range;
    size_t beginIndex;
    size_t endIndex;

};
