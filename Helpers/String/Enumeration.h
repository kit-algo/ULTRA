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
