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

template<typename ELEMENT>
class Range {

public:
    using Element = ELEMENT;
    using Type = Range<Element>;

public:
    class Iterator {
    public:
        Iterator(const Element i) : i(i) {}
        inline bool operator!=(const Iterator& other) const noexcept  {return i != other.i;}
        inline Element operator*() const noexcept  {return i;}
        inline Iterator& operator++() noexcept  {++i; return *this;}
        inline Iterator& operator+=(const size_t n) noexcept  {i += Element(n); return *this;}
        inline Iterator operator+(const size_t n) const noexcept {return Iterator(i + Element(n));}
        inline Element operator[](const size_t n) const noexcept {return i + Element(n);}
    private:
        Element i;
    };

    Range(const Element beginElement = Element(0), const Element endElement = Element(0)) :
        beginElement(beginElement),
        endElement(endElement) {
    }

    template<typename T>
    Range(const std::vector<T>& vector) :
        beginElement(0),
        endElement(Element(vector.size())) {
    }

    inline Iterator begin() const noexcept  {
        return Iterator(beginElement);
    }

    inline Iterator end() const noexcept  {
        return Iterator(endElement);
    }

    inline bool empty() const noexcept {
        return endElement <= beginElement;
    }

    inline size_t size() const noexcept  {
        return endElement - beginElement;
    }

    inline Element operator[](const size_t i) const noexcept  {
        AssertMsg(i < size(), "Index " << i << " is out of range!");
        return Element(beginElement + i);
    }

    inline Element front() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        return beginElement;
    }

    inline Element back() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        return endElement - Element(1);
    }

private:
    Element beginElement;
    Element endElement;

};

template<typename T>
inline Range<T> range(const T begin, const T end) noexcept {
    return Range<T>(begin, end);
}

template<typename T>
inline Range<T> range(const T end) noexcept {
    return Range<T>(T(0), end);
}

template<typename CONTAINER>
inline Range<size_t> indices(const CONTAINER& container) noexcept {
    return Range<size_t>(0, container.size());
}
