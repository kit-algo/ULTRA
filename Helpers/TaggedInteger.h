#pragma once

#include <sstream>
#include <iostream>

#include "Assert.h"

#include "Meta.h"
#include "String/String.h"

template<int TAG, typename VALUE_TYPE, VALUE_TYPE INVALID, VALUE_TYPE DEFAULT = INVALID, typename... ADDITIONAL_CASTS>
class TaggedInteger {

public:
    using ValueType = VALUE_TYPE;
    constexpr static ValueType InvalidValue = INVALID;
    constexpr static ValueType DefaultValue = DEFAULT;
    using AdditionalCasts = Meta::List<ADDITIONAL_CASTS...>;
    using Type = TaggedInteger<TAG, ValueType, InvalidValue, DefaultValue, ADDITIONAL_CASTS...>;

    using iterator_category = std::random_access_iterator_tag;
    using value_type = ValueType;
    using difference_type = ValueType;
    using pointer = Type;
    using reference = ValueType&;

public:
    constexpr TaggedInteger() : internalValue(DefaultValue) {}

    constexpr explicit TaggedInteger(const ValueType& value) : internalValue(value) {}

    constexpr inline operator const ValueType&() const noexcept {return internalValue;}

    template<typename T, typename = typename std::enable_if_t<Meta::Contains<T, AdditionalCasts>()>>
    constexpr inline operator T() const noexcept {return T(internalValue);}

    constexpr inline const ValueType& value() const noexcept {return internalValue;}
    constexpr inline const Type& operator*() const noexcept {return *this;}

    constexpr static Type Invalid() {return TaggedInteger(InvalidValue);}

    inline bool isValid()  const noexcept {return internalValue != InvalidValue;}
    inline bool isInvalid() const noexcept {return internalValue == InvalidValue;}
    inline void invalidate() noexcept {internalValue = InvalidValue;}

    inline bool operator<(const Type& other)  const noexcept {return internalValue < other.internalValue;}
    inline bool operator>(const Type& other)  const noexcept {return internalValue > other.internalValue;}
    inline bool operator<=(const Type& other) const noexcept {return internalValue <= other.internalValue;}
    inline bool operator>=(const Type& other) const noexcept {return internalValue >= other.internalValue;}
    inline bool operator==(const Type& other) const noexcept {return internalValue == other.internalValue;}
    inline bool operator!=(const Type& other) const noexcept {return internalValue != other.internalValue;}

    inline Type& operator+=(const Type& other) noexcept {
        AssertMsg(isValid(), "Cannot add something to an Invalid value.");
        AssertMsg(other.isValid(), "Cannot add an Invalid value to something.");
        internalValue += other.internalValue;
        return *this;
    }
    inline Type& operator+=(const ValueType& other) noexcept {
        AssertMsg(isValid(), "Cannot add something to an Invalid value.");
        internalValue += other;
        return *this;
    }

    inline Type& operator-=(const Type& other) noexcept {
        AssertMsg(isValid(), "Cannot subtract from an Invalid value.");
        AssertMsg(other.isValid(), "Cannot subtract an Invalid value");
        internalValue -= other.internalValue;
        return *this;
    }
    inline Type& operator-=(const ValueType& other) noexcept {
        AssertMsg(isValid(), "Cannot subtract from an Invalid value.");
        internalValue -= other;
        return *this;
    }

    inline Type& operator*=(const Type& other) noexcept {
        AssertMsg(isValid(), "Cannot multiply something with an Invalid value.");
        AssertMsg(other.isValid(), "Cannot multiply an Invalid value with something.");
        internalValue *= other.internalValue;
        return *this;
    }
    inline Type& operator*=(const ValueType& other) noexcept {
        AssertMsg(isValid(), "Cannot multiply something with an Invalid value.");
        internalValue *= other;
        return *this;
    }

    inline Type& operator/=(const Type& other) noexcept {
        AssertMsg(isValid(), "Cannot divide an Invalid value.");
        AssertMsg(other.isValid(), "Cannot divide something by an Invalid value.");
        internalValue /= other.internalValue;
        return *this;
    }
    inline Type& operator/=(const ValueType& other) noexcept {
        AssertMsg(isValid(), "Cannot divide an Invalid value.");
        internalValue /= other;
        return *this;
    }

    inline Type operator+(const Type& other) const noexcept {
        AssertMsg(isValid(), "Cannot add something to an Invalid value.");
        AssertMsg(other.isValid(), "Cannot add an Invalid value to something.");
        return Type(internalValue + other.internalValue);
    }

    inline Type operator-() const noexcept {
        AssertMsg(isValid(), "Cannot subtract from an Invalid value.");
        return Type(-internalValue);
    }

    inline Type operator-(const Type& other) const noexcept {
        AssertMsg(isValid(), "Cannot subtract from an Invalid value.");
        AssertMsg(other.isValid(), "Cannot subtract an Invalid value");
        return Type(internalValue - other.internalValue);
    }

    inline Type operator*(const Type& other) const noexcept {
        AssertMsg(isValid(), "Cannot multiply something with an Invalid value.");
        AssertMsg(other.isValid(), "Cannot multiply an Invalid value with something.");
        return Type(internalValue * other.internalValue);
    }

    inline Type operator/(const Type& other) const noexcept {
        AssertMsg(isValid(), "Cannot divide an Invalid value.");
        AssertMsg(other.isValid(), "Cannot divide something by an Invalid value.");
        return Type(internalValue / other.internalValue);
    }

    inline Type& operator++() noexcept {
        AssertMsg(isValid(), "Cannot increment an Invalid value.");
        internalValue++;
        return *this;
    }

    inline Type operator++(int) noexcept {
        AssertMsg(isValid(), "Cannot increment an Invalid value.");
        internalValue++;
        return Type(internalValue - 1);
    }

    inline Type& operator--() noexcept {
        AssertMsg(isValid(), "Cannot decrement an Invalid value.");
        internalValue--;
        return *this;
    }

    inline Type operator--(int) noexcept {
        AssertMsg(isValid(), "Cannot decrement an Invalid value.");
        internalValue--;
        return Type(internalValue + 1);
    }

    inline friend std::istream& operator>>(std::istream& in, Type& type) noexcept {
        in >> type.internalValue;
        return in;
    }

    inline friend std::ostream& operator<<(std::ostream& out, const Type& type) noexcept {
        if (type.isValid()) {
            return out << String::prettyInt(type.internalValue);
        } else {
            return out << "Invalid";
        }
    }

    inline friend std::string operator+(const std::string& string, const Type& type) noexcept {
        std::stringstream result;
        result << string << type;
        return result.str();
    }

    inline friend std::string operator+(const char string[], const Type& type) noexcept {
        std::stringstream result;
        result << string << type;
        return result.str();
    }

private:
    ValueType internalValue;

};

template<int TAG, typename DEPENDENCE>
using DependentTaggedInteger = TaggedInteger<TAG, typename DEPENDENCE::ValueType, DEPENDENCE::InvalidValue, DEPENDENCE::DefaultValue, DEPENDENCE>;
