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
#include "../Vector/Vector.h"

template<typename ELEMENT>
class SparseRange {

public:
    using Element = ELEMENT;
    using Type = SparseRange<Element>;

public:
    class Iterator {
    public:
        Iterator(const std::vector<bool>* const flag, const Element i) : flag(flag), i(i) {}
        inline bool operator!=(const Iterator& other) const noexcept {return i != other.i;}
        inline Element operator*() const noexcept {return i;}
        inline Iterator& operator++() noexcept {do {++i;} while (i < flag->size() && !(*flag)[i]); return *this;}
        inline Iterator& operator+=(const size_t n) noexcept {for (size_t j = 0; j < n; j++) ++(*this); return *this;}
        inline Iterator operator+(const size_t n) const noexcept {return Iterator(*this) += n;}
        inline Element operator[](const size_t n) const noexcept {return *(*this + n);}
    private:
        const std::vector<bool>* flag;
        Element i;
    };

    SparseRange() :
        flag(nullptr),
        beginIndex(0),
        endIndex(0) {
    }

    SparseRange(const std::vector<bool>& flag) :
        flag(&flag),
        beginIndex(0),
        endIndex(flag.size()) {
        while (beginIndex < endIndex && !flag[beginIndex]) beginIndex++;
        while (beginIndex < endIndex && !flag[endIndex - 1]) endIndex--;
    }

    SparseRange(const std::vector<bool>&&) = delete;

    inline Iterator begin() const noexcept {
        return Iterator(flag, beginIndex);
    }

    inline Iterator end() const noexcept {
        return Iterator(flag, (flag) ? Element(flag->size()) : beginIndex);
    }

    inline bool empty() const noexcept {
        return endIndex <= beginIndex;
    }

    inline size_t size() const noexcept {
        return (flag) ? Vector::count(*flag, true) : 0;
    }

    inline Element operator[](const size_t i) const noexcept {
        AssertMsg(i < size(), "Index " << i << " is out of range!");
        return begin()[i];
    }

    inline Element front() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        return beginIndex;
    }

    inline Element back() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        return endIndex - Element(1);
    }

private:
    const std::vector<bool>* flag;
    Element beginIndex;
    Element endIndex;

};
