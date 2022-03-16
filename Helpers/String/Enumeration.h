#pragma once

#include <iostream>
#include <string>
#include <sstream>

struct Sep {
    Sep(const std::string& sep = ", ") : sep(sep) {}
    std::string sep;
};
Sep newline("\n\r");
Sep clear("");
Sep sep(", ");

class Enumeration {

public:
    template<typename T>
    friend inline Enumeration& operator<<(Enumeration& out, const T& en);
    friend inline std::ostream& operator<<(std::ostream& out, const Enumeration& en);

public:
    Enumeration(const std::string& sep = "") : sep(sep) {}

    inline operator std::string() const noexcept {
        return state.str();
    }

    inline std::string str() const noexcept {
        return state.str();
    }

    inline bool empty() noexcept {
        return state.str().size() == 0;
    }

private:

    std::stringstream state;
    std::string sep;

};

inline std::ostream& operator<<(std::ostream& out, const Enumeration& en) {
    return out << en.state.str();
}

template<typename T>
inline Enumeration& operator<<(Enumeration& out, const T& t) {
    out.state << out.sep << t;
    out.sep = "";
    return out;
}

template<>
inline Enumeration& operator<<(Enumeration& out, const Sep& sep) {
    out.sep = sep.sep;
    return out;
}

template<>
inline Enumeration& operator<<(Enumeration& out, const Enumeration& en) {
    out.state << out.sep << en.state.str();
    out.sep = "";
    return out;
}
