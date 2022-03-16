#pragma once

#include "Attribute.h"
#include "AttributeNames.h"

#include "../../Helpers/Meta.h"

template<typename ATTRIBUTES, typename INDEX = size_t>
class AttributeHandle {

public:
    using Attributes = ATTRIBUTES;
    using Index = INDEX;
    using Type = AttributeHandle<Attributes, Index>;
    inline constexpr static size_t NumberOfAttributes = Attributes::NumberOfAttributes;

    template<AttributeNameType ATTRIBUTE_NAME>
    using AttributeType = typename Attributes::template AttributeType<ATTRIBUTE_NAME>;

    template<AttributeNameType ATTRIBUTE_NAME>
    using AttributeReferenceType = typename Attributes::template AttributeReferenceType<ATTRIBUTE_NAME>;

    template<AttributeNameType ATTRIBUTE_NAME>
    using AttributeConstReferenceType = typename Attributes::template AttributeConstReferenceType<ATTRIBUTE_NAME>;

    template<AttributeNameType ATTRIBUTE_NAME>
    inline constexpr static bool HasAttribute(const AttributeNameWrapper<ATTRIBUTE_NAME>) noexcept {return Attributes::HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>());}

    inline constexpr static std::array<AttributeNameType, ATTRIBUTES::NumberOfAttributes> AttributeNames = Attributes::AttributeNames;

public:
    AttributeHandle(Attributes& attributes, const Index index) :
        index(index),
        attributes(attributes) {
    }

    constexpr inline operator Index() const noexcept {return index;}

    template<AttributeNameType ATTRIBUTE_NAME>
    inline const AttributeReferenceType<ATTRIBUTE_NAME> operator[](const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName) const noexcept {
        return attributes[attributeName][index];
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline AttributeReferenceType<ATTRIBUTE_NAME> operator[](const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName) noexcept {
        return attributes[attributeName][index];
    }

    template<AttributeNameType ATTRIBUTE_NAME>
    inline const AttributeReferenceType<ATTRIBUTE_NAME> get(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName) const noexcept {
        return attributes[attributeName][index];
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline AttributeReferenceType<ATTRIBUTE_NAME> get(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName) noexcept {
        return attributes[attributeName][index];
    }

    template<AttributeNameType ATTRIBUTE_NAME, typename TYPE>
    inline Type& set(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName, const TYPE& value) noexcept {
        attributes.set(attributeName, index, value);
        return *this;
    }
    template<typename RECORD>
    inline Type& set(const RECORD& record) noexcept {
        attributes.set(index, record);
        return *this;
    }

private:
    const Index index;
    Attributes& attributes;

};
