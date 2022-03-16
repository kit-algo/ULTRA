#pragma once

#include <array>
#include <string>
#include <vector>
#include <utility>

#include "Attribute.h"
#include "AttributeNames.h"
#include "AttributeHandle.h"
#include "AttributeRecord.h"

#include "../../Helpers/Assert.h"
#include "../../Helpers/Meta.h"
#include "../../Helpers/IO/Serialization.h"
#include "../../Helpers/Vector/Vector.h"
#include "../../Helpers/String/String.h"
using Meta::List;

// Introduction of Attributes: A class which, given a list of Attributes, holds one std::vector of equal size for every attribute in the list
template<typename ATTRIBUTE_LIST>
class Attributes;

template<>
class Attributes<List<>> {

public:
    using Type = Attributes<List<>>;
    inline constexpr static size_t NumberOfAttributes = 0;

    template<AttributeNameType ATTRIBUTE_NAME>
    using AttributeType = typename Meta::FindAttributeType<ATTRIBUTE_NAME, List<>>;

    template<AttributeNameType ATTRIBUTE_NAME>
    inline constexpr static bool HasAttribute(const AttributeNameWrapper<ATTRIBUTE_NAME>) noexcept {return false;}

public:
    Attributes() : attributesSize(0) {}
    Attributes(const size_t n) : attributesSize(n) {}
    template<typename DEFAULT_VALUE_HEAD, typename... DEFAULT_VALUE_TAIL>
    Attributes(const size_t n, const DEFAULT_VALUE_HEAD&, const DEFAULT_VALUE_TAIL&...) : attributesSize(n) {}
    Attributes(const Type& other) : attributesSize(other.attributesSize) {}
    Attributes(Type&& other) : attributesSize(other.attributesSize) {}
    inline Type& operator=(const Type& other) noexcept {attributesSize = other.attributesSize; return *this;}
    inline Type& operator=(Type&& other) noexcept {attributesSize = other.attributesSize; return *this;}

    template<typename OTHER, typename... NAME_CHANGES>
    Attributes(const OTHER& other, const NAME_CHANGES...) : attributesSize(other.attributesSize) {}
    template<typename OTHER, typename... NAME_CHANGES>
    Attributes(OTHER&& other, const NAME_CHANGES...) : attributesSize(other.attributesSize) {}
    template<typename OTHER>
    inline Type& operator=(const OTHER& other) noexcept {return assign(other);}
    template<typename OTHER>
    inline Type& operator=(OTHER&& other) noexcept {return assign(std::move(other));}

    template<typename OTHER, typename... NAME_CHANGES>
    inline Type& assign(const Attributes<OTHER>& other, const NAME_CHANGES...) noexcept {
        attributesSize = other.attributesSize;
        return *this;
    }
    template<typename OTHER, typename... NAME_CHANGES>
    inline Type& assign(Attributes<OTHER>&& other, const NAME_CHANGES...) noexcept {
        attributesSize = other.attributesSize;
        return *this;
    }

    inline long long byteSize() const noexcept {return sizeof(size_t);}
    inline long long memoryUsageInBytes() const noexcept {return sizeof(size_t);}
    inline size_t size() const noexcept {return attributesSize;}
    inline bool hasSize(const size_t n) const noexcept {return attributesSize == n;}
    inline bool empty() const noexcept {return attributesSize == 0;}
    inline void reserve(size_t) noexcept {}
    inline void clear() noexcept {attributesSize = 0;}
    inline void popBack() noexcept {attributesSize--;}

    inline void resize(const size_t n) noexcept {attributesSize = n;}
    template<typename RECORD>
    inline void resize(const size_t n, const RECORD&) noexcept {attributesSize = n;}
    template<AttributeNameType HEAD_NAME, typename HEAD_TYPE, typename... DEFAULT_VALUE_TAIL>
    inline void resize(const size_t n, const AttributeValueWrapper<HEAD_NAME, HEAD_TYPE>&, const DEFAULT_VALUE_TAIL&...) noexcept {attributesSize = n;}

    inline void emplaceBack() noexcept {attributesSize++;}
    template<typename RECORD>
    inline void emplaceBack(const RECORD&) noexcept {attributesSize++;}
    template<AttributeNameType HEAD_NAME, typename HEAD_TYPE, typename... DEFAULT_VALUE_TAIL>
    inline void emplaceBack(const AttributeValueWrapper<HEAD_NAME, HEAD_TYPE>&, const DEFAULT_VALUE_TAIL&...) noexcept {attributesSize++;}

    template<typename FUNCTION>
    inline void forEach(FUNCTION&) noexcept {}
    template<typename FUNCTION>
    inline void forEach(const FUNCTION&) const noexcept {}

    template<typename RECORD>
    inline void set(const size_t, const RECORD&) noexcept {}
    inline void setToDefault(const size_t) noexcept {}

    inline void serialize(IO::Serialization& serialize) const noexcept {serialize(attributesSize);}
    inline void deserialize(IO::Deserialization& deserialize) noexcept {deserialize(attributesSize);}

    inline void serialize(const std::string& fileNameBase, const std::string& separator = ".") const noexcept {IO::serialize(fileNameBase + separator + "attributesSize", attributesSize);}
    inline void deserialize(const std::string& fileNameBase, const std::string& separator = ".") noexcept {IO::deserialize(fileNameBase + separator + "attributesSize", attributesSize);}
    inline void deserialize(const std::string&, const std::string&, const size_t size) noexcept {attributesSize = size;}

    inline void copy(const size_t, const size_t) noexcept {}
    inline void swap(const size_t, const size_t) noexcept {}
    inline void swap(Type&) noexcept {}
    inline void swap(Type&&) noexcept {}

protected:
    void serialize(const std::string&, const std::string&, const bool) const noexcept {}
    void deserialize(const std::string&, const std::string&, const bool) noexcept {}

protected:
    size_t attributesSize;

};

template<typename ATTRIBUTE, typename... ATTRIBUTE_LIST>
class Attributes<List<ATTRIBUTE, ATTRIBUTE_LIST...>> : public Attributes<List<ATTRIBUTE_LIST...>> {

public:
    template<typename T>
    friend class Attributes;

private:
    using Attribute = ATTRIBUTE;
    using Super = Attributes<List<ATTRIBUTE_LIST...>>;
    using ValueType = typename Attribute::Type;
    using AttributeList = List<ATTRIBUTE, ATTRIBUTE_LIST...>;
    inline constexpr static AttributeNameWrapper<Attribute::Name> ThisAttribute = AttributeNameWrapper<Attribute::Name>();

    template<AttributeNameType ATTRIBUTE_NAME>
    using SuperByName = Attributes<Meta::FindAttributeList<ATTRIBUTE_NAME, AttributeList>>;

    template<typename T, typename = void>
    struct IsApplicable : Meta::False {};

    template<typename T>
    struct IsApplicable<T, decltype(std::declval<T>()(std::declval<std::vector<ValueType>&>()), void())> : Meta::True {};

    template<typename T, typename = void>
    struct IsApplicableWithName : Meta::False {};

    template<typename T>
    struct IsApplicableWithName<T, decltype(std::declval<T>()(std::declval<std::vector<ValueType>&>(), AttributeNameType()), void())> : Meta::True {};

public:
    using Type = Attributes<AttributeList>;
    inline constexpr static size_t NumberOfAttributes = Super::NumberOfAttributes + 1;

    using Handle = AttributeHandle<Type, size_t>;

    template<AttributeNameType ATTRIBUTE_NAME>
    using AttributeType = typename Meta::FindAttributeType<ATTRIBUTE_NAME, AttributeList>;

    template<AttributeNameType ATTRIBUTE_NAME>
    using AttributeReferenceType = typename std::vector<Meta::FindAttributeType<ATTRIBUTE_NAME, AttributeList>>::reference;

    template<AttributeNameType ATTRIBUTE_NAME>
    using AttributeConstReferenceType = typename std::vector<Meta::FindAttributeType<ATTRIBUTE_NAME, AttributeList>>::const_reference;

    template<AttributeNameType ATTRIBUTE_NAME>
    inline constexpr static bool HasAttribute(const AttributeNameWrapper<ATTRIBUTE_NAME>) noexcept {return Meta::ContainsAttribute<ATTRIBUTE_NAME, AttributeList>();}

    inline constexpr static std::array<AttributeNameType, sizeof...(ATTRIBUTE_LIST) + 1> AttributeNames = {ATTRIBUTE::Name, ATTRIBUTE_LIST::Name...};

    static_assert(!Meta::HasDuplicateAttribute<AttributeList>(), "It is not possible to use two attributes with the same name!");
    static_assert(!HasAttribute(Unknown), "'Unknown' cannot be used as an Attribute Name!");
    static_assert(!HasAttribute(AnyAttribute), "'AnyAttribute' cannot be used as an Attribute Name!");

public:
    Attributes() : Super(), defaultValue(ValueType()), values() {}
    Attributes(const size_t size) : Super(size), defaultValue(ValueType()), values(size, ValueType()) {}
    template<typename DEFAULT_VALUE_HEAD, typename... DEFAULT_VALUE_TAIL>
    Attributes(const size_t size, const DEFAULT_VALUE_HEAD& head, const DEFAULT_VALUE_TAIL&... tail) :
        Super(size, head, tail...),
        defaultValue(getAttributeValue(ThisAttribute, ValueType(), head, tail...)),
        values(size, defaultValue) {
    }
    Attributes(const Type& other) :
        Super(static_cast<const Super&>(other)),
        defaultValue(other.defaultValue),
        values(other.values) {
    }
    Attributes(Type&& other) :
        Super(std::move(static_cast<Super&&>(other))),
        defaultValue(std::move(other.defaultValue)),
        values(std::move(other.values)) {
        values.resize(this->attributesSize, defaultValue);
    }
    inline Type& operator=(const Type& other) noexcept {
        Super::operator=(static_cast<const Super&>(other));
        defaultValue = other.defaultValue;
        values = other.values;
        return *this;
    }
    inline Type& operator=(Type&& other) noexcept {
        Super::operator=(std::move(static_cast<Super&&>(other)));
        defaultValue = std::move(other.defaultValue);
        values.swap(other.values);
        values.resize(this->attributesSize, defaultValue);
        return *this;
    }

    template<typename OTHER, typename... NAME_CHANGES>
    Attributes(const Attributes<OTHER>& other, const NAME_CHANGES... nameChanges) :
        Super(other, nameChanges...) {
        if constexpr (Attributes<OTHER>::HasAttribute(getOldAttributeName(ThisAttribute, nameChanges...))) {
            defaultValue = other.getDefaultValue(getOldAttributeName(ThisAttribute, nameChanges...));
            Vector::assign(values, other[getOldAttributeName(ThisAttribute, nameChanges...)]);
        } else {
            defaultValue = ValueType();
            std::vector<ValueType>(other.size(), ValueType()).swap(values);
        }
    }
    template<typename OTHER, typename... NAME_CHANGES>
    Attributes(Attributes<OTHER>&& other, const NAME_CHANGES... nameChanges) :
        Super(std::move(other), nameChanges...) {
        if constexpr (Attributes<OTHER>::HasAttribute(getOldAttributeName(ThisAttribute, nameChanges...))) {
            defaultValue = std::move(other.getDefaultValue(getOldAttributeName(ThisAttribute, nameChanges...)));
            Vector::assign(values, std::move(other[getOldAttributeName(ThisAttribute, nameChanges...)]));
            values.resize(this->attributesSize, defaultValue);
        } else {
            defaultValue = ValueType();
            std::vector<ValueType>(this->attributesSize, ValueType()).swap(values);
        }
    }
    template<typename OTHER>
    inline Type& operator=(const Attributes<OTHER>& other) noexcept {
        return assign(other);
    }
    template<typename OTHER>
    inline Type& operator=(Attributes<OTHER>&& other) noexcept {
        return assign(std::move(other));
    }

    template<typename OTHER, typename... NAME_CHANGES>
    inline Type& assign(const Attributes<OTHER>& other, const NAME_CHANGES... nameChanges) noexcept {
        Super::assign(other, nameChanges...);
        if constexpr (Attributes<OTHER>::HasAttribute(getOldAttributeName(ThisAttribute, nameChanges...))) {
            defaultValue = other.getDefaultValue(getOldAttributeName(ThisAttribute, nameChanges...));
            Vector::assign(values, other[getOldAttributeName(ThisAttribute, nameChanges...)]);
        } else {
            defaultValue = ValueType();
            std::vector<ValueType>(other.size(), ValueType()).swap(values);
        }
        return *this;
    }
    template<typename OTHER, typename... NAME_CHANGES>
    inline Type& assign(Attributes<OTHER>&& other, const NAME_CHANGES... nameChanges) noexcept {
        Super::assign(std::move(other), nameChanges...);
        if constexpr (Attributes<OTHER>::HasAttribute(getOldAttributeName(ThisAttribute, nameChanges...))) {
            defaultValue = std::move(other.getDefaultValue(getOldAttributeName(ThisAttribute, nameChanges...)));
            Vector::assign(values, std::move(other[getOldAttributeName(ThisAttribute, nameChanges...)]));
            values.resize(this->attributesSize, defaultValue);
        } else {
            defaultValue = ValueType();
            std::vector<ValueType>(this->attributesSize, ValueType()).swap(values);
        }
        return *this;
    }

    inline long long byteSize() const noexcept {return Vector::byteSize(values) + Super::byteSize();}
    inline long long memoryUsageInBytes() const noexcept {return Vector::memoryUsageInBytes(values) + Super::memoryUsageInBytes();}
    inline size_t size() const noexcept {return values.size();}
    inline bool hasSize(const size_t size) const noexcept {return values.size() == size && Super::hasSize(size);}
    inline bool empty() const noexcept {return values.empty();}
    inline void reserve(const size_t size) noexcept {values.reserve(size); Super::reserve(size);}
    inline void clear() noexcept {std::vector<ValueType>().swap(values); Super::clear();}
    inline Handle front() noexcept {return Handle(*this, 0);}
    inline Handle back() noexcept {return Handle(*this, size() - 1);}
    inline void popBack() noexcept {
        AssertMsg(!empty(), "Cannot pop back on empty attributes!");
        values.pop_back();
        Super::popBack();
    }

    inline void resize(const size_t size) noexcept {values.resize(size, defaultValue); Super::resize(size);}
    template<typename RECORD>
    inline void resize(const size_t size, const RECORD& record) noexcept {
        if constexpr (RECORD::HasAttribute(ThisAttribute)) {
            values.resize(size, record[ThisAttribute]);
        } else {
            values.resize(size, defaultValue);
        }
        Super::resize(size, record);
    }
    template<AttributeNameType HEAD_NAME, typename HEAD_TYPE, typename... DEFAULT_VALUE_TAIL>
    inline void resize(const size_t size, const AttributeValueWrapper<HEAD_NAME, HEAD_TYPE>& head, const DEFAULT_VALUE_TAIL&... tail) noexcept {
        values.resize(size, getAttributeValue(ThisAttribute, defaultValue, head, tail...));
        Super::resize(size, head, tail...);
    }

    inline Handle emplaceBack() noexcept {
        values.emplace_back(defaultValue);
        Super::emplaceBack();
        return Handle(*this, size() - 1);
    }
    template<typename RECORD>
    inline Handle emplaceBack(const RECORD& record) noexcept {
        if constexpr (RECORD::HasAttribute(ThisAttribute)) {
            values.emplace_back(record[ThisAttribute]);
        } else {
            values.emplace_back(defaultValue);
        }
        Super::emplaceBack(record);
        return Handle(*this, size() - 1);
    }
    template<AttributeNameType HEAD_NAME, typename HEAD_TYPE, typename... DEFAULT_VALUE_TAIL>
    inline Handle emplaceBack(const AttributeValueWrapper<HEAD_NAME, HEAD_TYPE>& head, const DEFAULT_VALUE_TAIL&... tail) noexcept {
        values.emplace_back(getAttributeValue(ThisAttribute, defaultValue, head, tail...));
        Super::emplaceBack(head, tail...);
        return Handle(*this, size() - 1);
    }

    template<typename FUNCTION>
    inline void forEach(const FUNCTION& function) noexcept {
        if constexpr (IsApplicableWithName<FUNCTION>::Value) {
            function(values, ThisAttribute);
        } else if constexpr (IsApplicable<FUNCTION>::Value) {
            function(values);
        }
        Super::forEach(function);
    }
    template<typename FUNCTION>
    inline void forEach(const FUNCTION& function) const noexcept {
        if constexpr (IsApplicableWithName<FUNCTION>::Value) {
            function(values, ThisAttribute);
        } else if constexpr (IsApplicable<FUNCTION>::Value) {
            function(values);
        }
        Super::forEach(function);
    }

    inline Handle operator[](const size_t i) noexcept {
        AssertMsg(i < size(), "Invalid Index i = " << i << ", Attributes contains only " << size() << " values!");
        return Handle(*this, i);
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline const std::vector<AttributeType<ATTRIBUTE_NAME>>& operator[](const AttributeNameWrapper<ATTRIBUTE_NAME>) const noexcept {
        static_assert(HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()), "Current instantiation of Attributes<List<ATTRIBUTE, ATTRIBUTE_LIST...>> does not contain an Attribute named ATTRIBUTE_NAME");
        return SuperByName<ATTRIBUTE_NAME>::values;
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline std::vector<AttributeType<ATTRIBUTE_NAME>>& operator[](const AttributeNameWrapper<ATTRIBUTE_NAME>) noexcept {
        static_assert(HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()), "Current instantiation of Attributes<List<ATTRIBUTE, ATTRIBUTE_LIST...>> does not contain an Attribute named ATTRIBUTE_NAME");
        return SuperByName<ATTRIBUTE_NAME>::values;
    }

    inline Handle& get(const size_t i) noexcept {
        AssertMsg(i < size(), "Invalid Index i = " << i << ", Attributes contains only " << size() << " values!");
        return Handle(*this, i);
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline AttributeConstReferenceType<ATTRIBUTE_NAME> get(const AttributeNameWrapper<ATTRIBUTE_NAME>, const size_t i) const noexcept {
        static_assert(HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()), "Current instantiation of Attributes<List<ATTRIBUTE, ATTRIBUTE_LIST...>> does not contain an Attribute named ATTRIBUTE_NAME");
        AssertMsg(i < values.size(), "Invalid Index i = " << i << ", Attributes contains only " << values.size() << " values!");
        return SuperByName<ATTRIBUTE_NAME>::values[i];
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline AttributeReferenceType<ATTRIBUTE_NAME> get(const AttributeNameWrapper<ATTRIBUTE_NAME>, const size_t i) noexcept {
        static_assert(HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()), "Current instantiation of Attributes<List<ATTRIBUTE, ATTRIBUTE_LIST...>> does not contain an Attribute named ATTRIBUTE_NAME");
        AssertMsg(i < values.size(), "Invalid Index i = " << i << ", Attributes contains only " << values.size() << " values!");
        return SuperByName<ATTRIBUTE_NAME>::values[i];
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline const std::vector<AttributeType<ATTRIBUTE_NAME>>& get(const AttributeNameWrapper<ATTRIBUTE_NAME>) const noexcept {
        static_assert(HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()), "Current instantiation of Attributes<List<ATTRIBUTE, ATTRIBUTE_LIST...>> does not contain an Attribute named ATTRIBUTE_NAME");
        return SuperByName<ATTRIBUTE_NAME>::values;
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline std::vector<AttributeType<ATTRIBUTE_NAME>>& get(const AttributeNameWrapper<ATTRIBUTE_NAME>) noexcept {
        static_assert(HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()), "Current instantiation of Attributes<List<ATTRIBUTE, ATTRIBUTE_LIST...>> does not contain an Attribute named ATTRIBUTE_NAME");
        return SuperByName<ATTRIBUTE_NAME>::values;
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline const AttributeType<ATTRIBUTE_NAME>& getDefaultValue(const AttributeNameWrapper<ATTRIBUTE_NAME>) const noexcept {
        static_assert(HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()), "Current instantiation of Attributes<List<ATTRIBUTE, ATTRIBUTE_LIST...>> does not contain an Attribute named ATTRIBUTE_NAME");
        return SuperByName<ATTRIBUTE_NAME>::defaultValue;
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline AttributeType<ATTRIBUTE_NAME>& getDefaultValue(const AttributeNameWrapper<ATTRIBUTE_NAME>) noexcept {
        static_assert(HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()), "Current instantiation of Attributes<List<ATTRIBUTE, ATTRIBUTE_LIST...>> does not contain an Attribute named ATTRIBUTE_NAME");
        return SuperByName<ATTRIBUTE_NAME>::defaultValue;
    }

    template<typename RECORD>
    inline void set(const size_t i, const RECORD& record) noexcept {
        AssertMsg(i < values.size(), "Invalid Index i = " << i << ", Attributes contains only " << values.size() << " values!");
        if constexpr (RECORD::HasAttribute(ThisAttribute)) values[i] = record[ThisAttribute];
        Super::set(i, record);
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline void set(const AttributeNameWrapper<ATTRIBUTE_NAME>, const size_t i, const AttributeType<ATTRIBUTE_NAME>& value) noexcept {
        static_assert(HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()), "Current instantiation of Attributes<List<ATTRIBUTE, ATTRIBUTE_LIST...>> does not contain an Attribute named ATTRIBUTE_NAME");
        AssertMsg(i < values.size(), "Invalid Index i = " << i << ", Attributes contains only " << values.size() << " values!");
        SuperByName<ATTRIBUTE_NAME>::values[i] = value;
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline void set(const AttributeNameWrapper<ATTRIBUTE_NAME>, const std::vector<AttributeType<ATTRIBUTE_NAME>>& value) noexcept {
        static_assert(HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()), "Current instantiation of Attributes<List<ATTRIBUTE, ATTRIBUTE_LIST...>> does not contain an Attribute named ATTRIBUTE_NAME");
        AssertMsg(value.size() == size(), "Cannot set a vector of size " << value.size() << " to attributes of size " << size() << "!");
        SuperByName<ATTRIBUTE_NAME>::values = value;
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline void set(const AttributeNameWrapper<ATTRIBUTE_NAME>, std::vector<AttributeType<ATTRIBUTE_NAME>>&& value) noexcept {
        static_assert(HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()), "Current instantiation of Attributes<List<ATTRIBUTE, ATTRIBUTE_LIST...>> does not contain an Attribute named ATTRIBUTE_NAME");
        AssertMsg(value.size() == size(), "Cannot set a vector of size " << value.size() << " to attributes of size " << size() << "!");
        SuperByName<ATTRIBUTE_NAME>::values.swap(value);
    }
    inline void setToDefault(const size_t i) noexcept {
        AssertMsg(i < values.size(), "Invalid Index i = " << i << ", Attributes contains only " << values.size() << " values!");
        values[i] = defaultValue;
        Super::setToDefault(i);
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline void setDefaultValue(const AttributeNameWrapper<ATTRIBUTE_NAME>, const AttributeType<ATTRIBUTE_NAME>& value) noexcept {
        static_assert(HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()), "Current instantiation of Attributes<List<ATTRIBUTE, ATTRIBUTE_LIST...>> does not contain an Attribute named ATTRIBUTE_NAME");
        SuperByName<ATTRIBUTE_NAME>::defaultValue = value;
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(defaultValue, values);
        Super::serialize(serialize);
        AssertMsg(hasSize(size()), "Inconsistent attribute vector sizes!");
    }
    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(defaultValue, values);
        Super::deserialize(deserialize);
        AssertMsg(hasSize(size()), "Inconsistent attribute vector sizes!");
    }

    inline void serialize(const std::string& fileNameBase, const std::string& separator = ".") const noexcept {
        serialize(fileNameBase, separator, false);
    }
    inline void deserialize(const std::string& fileNameBase, const std::string& separator = ".") noexcept {
        deserialize(fileNameBase, separator, false);
    }
    inline void deserialize(const std::string& fileNameBase, const std::string& separator, const size_t size) noexcept {
        deserialize(fileNameBase, separator, false);
        AssertMsg(hasSize(size), "Inconsistent attribute vector sizes!");
    }

    inline void copy(const size_t from, const size_t to) noexcept {
        AssertMsg(from < values.size(), "Invalid Index from = " << from << ", Attributes contains only " << values.size() << " values!");
        AssertMsg(to < values.size(), "Invalid Index to = " << to << ", Attributes contains only " << values.size() << " values!");
        if (from == to) return;
        values[to] = values[from];
        Super::copy(from, to);
    }
    inline void swap(const size_t i, const size_t j) noexcept {
        AssertMsg((i >= 0) || (i < values.size()), "Invalid Index i = " << i << ", Attributes range from 0 to " << values.size() << "!");
        AssertMsg((j >= 0) || (j < values.size()), "Invalid Index j = " << j << ", Attributes range from 0 to " << values.size() << "!");
        std::swap(values[i], values[j]);
        Super::swap(i, j);
    }
    inline void swap(Type& other) noexcept {
        values.swap(other.values);
        std::swap(defaultValue, other.defaultValue);
        Super::swap(static_cast<Super&>(other));
    }
    inline void swap(Type&& other) noexcept {
        values.swap(other.values);
        std::swap(defaultValue, other.defaultValue);
        Super::swap(std::move(static_cast<Super&&>(other)));
    }

protected:
    inline void serialize(const std::string& fileNameBase, const std::string& separator, const bool) const noexcept {
        IO::serialize(fileNameBase + separator + String::firstToLower(Attribute::String), values);
        Super::serialize(fileNameBase, separator, false);
        AssertMsg(hasSize(size()), "Inconsistent attribute vector sizes!");
    }
    inline void deserialize(const std::string& fileNameBase, const std::string& separator, const bool) noexcept {
        IO::deserialize(fileNameBase + separator + String::firstToLower(Attribute::String), values);
        Super::deserialize(fileNameBase, separator, false);
        this->attributesSize = size();
        AssertMsg(hasSize(size()), "Inconsistent attribute vector sizes!");
    }

protected:
    ValueType defaultValue{ValueType()};
    std::vector<ValueType> values{};

};
