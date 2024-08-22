#pragma once

#include <utility>
#include <typeinfo>
#include <type_traits>
#include <iostream>
#include <typeinfo>
#include <emmintrin.h>
#include <concepts>

#include <cstdlib>
#include <memory>
#include <cxxabi.h>

#include "String/String.h"

namespace Meta {

    // ID

    template<typename T>
    struct ID {using Type = T;};

    // TRUE & FALSE

    struct True {constexpr static bool Value = true;};

    struct False {constexpr static bool Value = false;};

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
        struct Contains<T, List<HEAD, TAIL...>> : std::conditional_t<!std::is_same_v<T, HEAD>,
            Contains<T, List<TAIL...>>,
            True> {
        };
    }

    template<typename T, typename LIST>
    concept Contains = Implementation::Contains<T, LIST>::Value;

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

    // PLAIN TYPE

    template<typename T>
    using PlainType = std::remove_pointer_t<std::remove_cvref_t<T>>;

    // DERIVED

    template<class T, class U>
    concept Derived = std::is_base_of<U, T>::value;
}
