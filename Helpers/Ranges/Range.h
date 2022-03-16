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
