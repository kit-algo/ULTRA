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

#include "../../Intermediate/Entities/Trip.h"

#include "../../../Helpers/IO/Serialization.h"

namespace CSA {

class Trip {

public:
    static const std::string CSV_HEADER;

public:
    Trip(const std::string& tripName = "", const std::string& routeName = "", const int type = -1) :
        tripName(tripName),
        routeName(routeName),
        type(type) {
    }
    template<typename TRIP_TYPE>
    Trip(const TRIP_TYPE& t) :
        tripName(t.tripName),
        routeName(t.routeName),
        type(t.type) {
    }
    Trip(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }
    friend std::ostream& operator<<(std::ostream& out, const Trip& t) {
        return out << "Trip{" << t.routeName << ", " << t.tripName  << ", " << t.type << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(tripName, routeName, type);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(tripName, routeName, type);
    }

public:
    std::string tripName{""};
    std::string routeName{""};
    int type{-1};

};

const std::string Trip::CSV_HEADER = "name,vehicle";

}
