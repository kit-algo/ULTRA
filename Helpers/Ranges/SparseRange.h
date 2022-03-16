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
