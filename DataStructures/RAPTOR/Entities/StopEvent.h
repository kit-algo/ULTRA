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
#include "../../Intermediate/Entities/StopEvent.h"

namespace RAPTOR {

class StopEvent {

public:
    StopEvent(const int arrivalTime = -1, const int departureTime = -2) :
        arrivalTime(arrivalTime),
        departureTime(departureTime) {
    }
    StopEvent(const Intermediate::StopEvent& s) :
        arrivalTime(s.arrivalTime),
        departureTime(s.departureTime) {
    }
    StopEvent(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline StopEvent reverseStopEvent() const noexcept {
        return StopEvent(-departureTime, -arrivalTime);
    }

    friend std::ostream& operator<<(std::ostream& out, const StopEvent& s) {
        return out << "StopEvent{" << s.arrivalTime  << ", " << s.departureTime << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(arrivalTime, departureTime);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(arrivalTime, departureTime);
    }

public:
    int arrivalTime{-1};
    int departureTime{-2};

};

}
