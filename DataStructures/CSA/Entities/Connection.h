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
#include <sstream>
#include <vector>
#include <string>

#include "../../../Helpers/Types.h"
#include "../../../Helpers/IO/Serialization.h"

namespace CSA {

class Connection {

public:
    static const std::string CSV_HEADER;

public:
    Connection(const StopId departureStopId = noStop, const StopId arrivalStopId = noStop, const int departureTime = -1, const int arrivalTime = 0, const TripId tripId = noTripId) :
        departureStopId(departureStopId),
        arrivalStopId(arrivalStopId),
        departureTime(departureTime),
        arrivalTime(arrivalTime),
        tripId(tripId) {
    }
    template<typename CONNECTION_TYPE>
    Connection(const CONNECTION_TYPE& c) :
        departureStopId(c.departureStopId),
        arrivalStopId(c.arrivalStopId),
        departureTime(c.departureTime),
        arrivalTime(c.arrivalTime),
        tripId(c.tripId) {
    }
    Connection(const Connection& c, const int timeOffset, const int tripOffset) :
        departureStopId(c.departureStopId),
        arrivalStopId(c.arrivalStopId),
        departureTime(c.departureTime + timeOffset),
        arrivalTime(c.arrivalTime + timeOffset),
        tripId(c.tripId + tripOffset) {
    }
    Connection(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    inline bool operator<(const Connection& c) const noexcept {
        return (departureTime < c.departureTime) || ((departureTime == c.departureTime) && (
               (arrivalTime < c.arrivalTime)));
    }

    inline int travelTime() const {
        return arrivalTime - departureTime;
    }

    inline bool isValid() const {
        return travelTime() > 0;
    }

    friend std::ostream& operator<<(std::ostream& out, const Connection& c) {
        return out << "Connection{" << c.departureStopId  << ", " << c.arrivalStopId  << ", " << c.departureTime  << ", " << c.arrivalTime  << ", " << c.tripId << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(departureStopId, arrivalStopId, departureTime, arrivalTime, tripId);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(departureStopId, arrivalStopId, departureTime, arrivalTime, tripId);
    }

    inline std::ostream& toCSV(std::ostream& out) const {
        return out << size_t(departureStopId) << "," << size_t(arrivalStopId) << "," << departureTime << "," << arrivalTime << "," << size_t(tripId);
    }

    inline std::string toCSV() const {
        std::stringstream ss;
        toCSV(ss);
        return ss.str();
    }

public:
    StopId departureStopId{noStop};
    StopId arrivalStopId{noStop};
    int departureTime{-1};
    int arrivalTime{0};
    TripId tripId{noTripId};

};

const std::string Connection::CSV_HEADER = "dep_stop,arr_stop,dep_time,arr_time,trip_id";

}
