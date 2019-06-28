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
#include <vector>
#include <string>

#include "../../../Helpers/IO/Serialization.h"

namespace RAPTOR {

class Route {

public:
    Route(const std::string& name = "", const int type = -1) :
        name(name),
        type(type) {
    }
    Route(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    friend std::ostream& operator<<(std::ostream& out, const Route& r) {
        return out << "Route{" << r.name << ", " << r.type << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(name, type);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(name, type);
    }

public:
    std::string name{""};
    int type{-1};

};

}
