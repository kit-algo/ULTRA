#pragma once

#include <utility>
#include <typeinfo>
#include <type_traits>
#include <iostream>
#include <typeinfo>
#include <emmintrin.h>

#include <cstdlib>
#include <memory>
#include <cxxabi.h>

#include "String/String.h"

// Introducing namespace Meta, which shall contain all functions used for template meta programming

namespace Meta {

    // ID

    template<typename T>
    struct ID {using Type = T;};

    // TRUE & FALSE

    struct True {constexpr static bool Value = true;};

    struct False {constexpr static bool Value = false;};

    template<typename T>
    struct FalseIfInstantiated {constexpr static bool Value = false;};

    // EQUALS

    namespace Implementation {
        template<typename T, typename U>
        struct Equals : False {};

        template<typename T>
        struct Equals<T, T> : True {};
    }

    template<typename T, typename U>
    inline constexpr bool Equals() {return Implementation::Equals<T, U>::Value;}

    // IF THEN ELSE

    namespace Implementation {
        template<bool CONDITION, typename IF_TYPE, typename ELSE_TYPE>
        struct If {using Type = ELSE_TYPE;};

        template<typename IF_TYPE, typename ELSE_TYPE>
        struct If<true, IF_TYPE, ELSE_TYPE> {using Type = IF_TYPE;};
    }

    template<bool CONDITION, typename IF_TYPE, typename ELSE_TYPE>
    using IF = typename Implementation::If<CONDITION, IF_TYPE, ELSE_TYPE>::Type;

    // LIST

    template<typename... VALUES>
    struct List {
        using Type = List<VALUES...>;
        constexpr static size_t Size = sizeof...(VALUES);
    };

    // LIST HEAD

    namespace Implementation {
        template<typename LIST>
        struct Head;

        template<typename HEAD, typename... TAIL>
        struct Head<List<HEAD, TAIL...>> : ID<HEAD> {};
    }

    template<typename LIST>
    using Head = typename Implementation::Head<LIST>::Type;

    // LIST TAIL

    namespace Implementation {
        template<typename LIST>
        struct Tail;

        template<typename HEAD, typename... TAIL>
        struct Tail<List<HEAD, TAIL...>> : List<TAIL...> {};
    }

    template<typename LIST>
    using Tail = typename Implementation::Head<LIST>::Type;

    // LIST CONCAT

    namespace Implementation {
        template<typename LIST_A, typename LIST_B>
        struct Concat;

        template<typename... VALUES_A, typename... VALUES_B>
        struct Concat<List<VALUES_A...>, List<VALUES_B...>> : List<VALUES_A..., VALUES_B...> {};
    }

    template<typename LIST_A, typename LIST_B>
    using Concat = typename Implementation::Concat<LIST_A, LIST_B>::Type;

    // LIST CONTAINS

    namespace Implementation {
        template<typename T, typename LIST>
        struct Contains;

        template<typename T>
        struct Contains<T, List<>> : False {};

        template<typename T, typename HEAD, typename... TAIL>
        struct Contains<T, List<HEAD, TAIL...>> : IF<!Meta::Equals<T, HEAD>(),
            Contains<T, List<TAIL...>>,
            True> {
        };
    }

    template<typename T, typename LIST>
    inline constexpr bool Contains() {return Implementation::Contains<T, LIST>::Value;}

    // TO STRING (for Types)

    namespace Implementation {
        inline std::string type(const char* name) noexcept {
            int status = -4;
            std::unique_ptr<char, void(*)(void*)> res {
                abi::__cxa_demangle(name, NULL, NULL, &status),
                std::free
            };
            return (status==0) ? res.get() : name;
        }

        inline std::string type(const std::string& name) noexcept {
            return type(name.c_str());
        }

        inline std::string cleanType(const char* name) noexcept {
            std::string typeID = type(name);
            typeID = typeID.substr(9, typeID.size() - 10);
            typeID = String::replaceAll(typeID, "> ", ">");
            typeID = String::replaceAll(typeID, "::__debug::", "::");
            size_t i = String::firstIndexOf(typeID, ", std::allocator<");
            while (i < typeID.size()) {
                int parenthesisCount = 1;
                size_t j;
                for (j = i + 17; j < typeID.size(); j++) {
                    if (parenthesisCount == 0) break;
                    if (typeID[j] == '<') parenthesisCount++;
                    if (typeID[j] == '>') parenthesisCount--;
                }
                typeID = typeID.substr(0, i) + typeID.substr(j);
                i = String::firstIndexOf(typeID, ", std::allocator<");
            }
            return typeID;
        }

        inline std::string cleanType(const std::string& name) noexcept {
            return cleanType(name.c_str());
        }

        template<typename T>
        struct Type;
    }

    template<typename T>
    inline std::string type(T&&) noexcept {
        return Implementation::cleanType(typeid(ID<T&&>).name());
    }

    template<typename T>
    inline std::string type() noexcept {
        return Implementation::cleanType(typeid(ID<T>).name());
    }

    template<typename T>
    struct Type {Type(){Implementation::Type<T> type;}};

    // MAKE CONST

    namespace Implementation {
        template<typename T>
        struct MakeConst {using Type = const T;};

        template<typename T>
        struct MakeConst<T*> {using Type = const T*;};

        template<typename T>
        struct MakeConst<T* const> {using Type = const T* const;};

        template<typename T>
        struct MakeConst<T&> {using Type = const T&;};

        template<typename T>
        struct MakeConst<T&&> {using Type = const T&&;};
    }

    template<typename T>
    using MakeConst = typename Implementation::MakeConst<T>::Type;

    // IS CONST

    template<typename T>
    inline constexpr bool IsConst() {return Equals<T, MakeConst<T>>();}

    // REMOVE CONSTNESS

    namespace Implementation {
        template<typename T>
        struct RemoveConstness {using Type = T;};

        template<typename T>
        struct RemoveConstness<const T> {using Type = T;};

        template<typename T>
        struct RemoveConstness<const T*> {using Type = T*;};

        template<typename T>
        struct RemoveConstness<const T* const> {using Type = T*;};

        template<typename T>
        struct RemoveConstness<const T&> {using Type = T&;};

        template<typename T>
        struct RemoveConstness<const T&&> {using Type = T&&;};
    }

    template<typename T>
    using RemoveConstness = typename Implementation::RemoveConstness<T>::Type;

    // COPY CONSTNESS

    template<typename FROM, typename TO>
    using CopyConstness = IF<IsConst<FROM>(), MakeConst<TO>, TO>;

    // IS REFERENCE

    namespace Implementation {
        template<typename T>
        struct IsReference : False {};

        template<typename T>
        struct IsReference<T&> : True {};

        template<typename T>
        struct IsReference<T&&> : True {};
    }

    template<typename T>
    inline constexpr bool IsReference() {return Implementation::IsReference<T>::Value;}

    // REMOVE REFERENCE

    namespace Implementation {
        template<typename T>
        struct RemoveReference {using Type = T;};

        template<typename T>
        struct RemoveReference<T&> {using Type = T;};

        template<typename T>
        struct RemoveReference<T&&> {using Type = T;};
    }

    template<typename T>
    using RemoveReference = typename Implementation::RemoveReference<T>::Type;

    // IS POINTER

    namespace Implementation {
        template<typename T>
        struct IsPointer : False {};

        template<typename T>
        struct IsPointer<T*> : True {};
    }

    template<typename T>
    inline constexpr bool IsPointer() {return Implementation::IsPointer<RemoveConstness<T>>::Value;}

    // REMOVE POINTER

    namespace Implementation {
        template<typename T>
        struct RemovePointer {using Type = T;};

        template<typename T>
        struct RemovePointer<T*> {using Type = T;};

        template<typename T>
        struct RemovePointer<T* const> {using Type = T;};
    }

    template<typename T>
    using RemovePointer = typename Implementation::RemovePointer<T>::Type;

    // PLAIN TYPE

    template<typename T>
    using PlainType = IF<IsReference<T>(), RemoveReference<RemoveConstness<T>>, RemovePointer<RemoveConstness<T>>>;

    // IS MOVEABLE

    template<typename T>
    inline constexpr bool IsMovable() {return std::is_move_assignable<T>::value && std::is_move_constructible<T>::value;}

}
