/**********************************************************************************

 Copyright (c) 2019 Jonas Sauer

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

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>

#include "../../Helpers/Assert.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/Types.h"
#include "../../Helpers/Vector/Vector.h"
#include "../../DataStructures/CSA/Data.h"

namespace CSA {

template<typename DEBUGGER>
class CSA {

public:
    using Debugger = DEBUGGER;
    using Type = CSA<Debugger>;
    using TripFlag = ConnectionId;

private:
    struct ParentLabel {
        ParentLabel(const StopId parent = noStop, const bool reachedByTransfer = false, const TripId tripId = noTripId) :
            parent(parent),
            reachedByTransfer(reachedByTransfer),
            tripId(tripId) {
        }

        StopId parent;
        bool reachedByTransfer;
        union {
            TripId tripId;
            Edge transferId;
        };
    };

public:
    CSA(const Data& data, const Debugger& debuggerTemplate = Debugger()) :
        data(data),
        sourceStop(noStop),
        targetStop(noStop),
        tripReached(data.numberOfTrips(), TripFlag()),
        arrivalTime(data.numberOfStops(), never),
        parentLabel(data.numberOfStops()),
        debugger(debuggerTemplate) {
        AssertMsg(Vector::isSorted(data.connections), "Connections must be sorted in ascending order! ");
        debugger.initialize(data);
    }

    inline void run(const StopId source, const int departureTime, const StopId target = noStop) noexcept {
        debugger.start();
        debugger.startClear();
        AssertMsg(data.isStop(source), "Source stop " << source << " is not a valid stop!");
        clear();
        debugger.doneClear();

        debugger.startInitialization();
        sourceStop = source;
        targetStop = target;
        arrivalByTrip<true>(source, departureTime, noTripId);
        const ConnectionId firstConnection(Vector::lowerBound(data.connections, departureTime, [](const Connection& connection, const int time) {
            return connection.departureTime < time;
        }));
        debugger.doneInitialization();

        debugger.startConnectionScan();
        for (ConnectionId i = firstConnection; i < data.connections.size(); i++) {
            const Connection& connection = data.connections[i];
            if (target != noStop && connection.departureTime > getEarliestArrivalTime(target)) break;
            if (connectionIsReachable(connection)) {
                debugger.scanConnection(connection);
                useTrip(connection.tripId, i);
                arrivalByTrip(connection.arrivalStopId, connection.arrivalTime, connection.tripId);
            }
        }
        debugger.doneConnectionScan();
        debugger.done();
    }

    inline int getEarliestArrivalTime(const StopId stop) const noexcept {
        return arrivalTime[stop];
    }

    inline const Debugger& getDebugger() const noexcept {
        return debugger;
    }

private:
    inline void clear() {
        sourceStop = noStop;
        targetStop = noStop;
        Vector::fill(arrivalTime, never);
        Vector::fill(tripReached, TripFlag());
        Vector::fill(parentLabel, ParentLabel());
    }

    inline bool connectionIsReachable(const Connection& connection) const noexcept {
        return tripReached[connection.tripId] != TripFlag() || arrivalTime[connection.departureStopId] <= connection.departureTime;
    }

    inline void useTrip(const TripId trip, const ConnectionId connection) noexcept {
        if (tripReached[trip] == noConnection) {
            tripReached[trip] = connection;
        }
    }

    template<bool INITIAL_ARRIVAL = false>
    inline void arrivalByTrip(const StopId stop, const int time, const TripId trip) noexcept {
        if (arrivalTime[stop] <= time) return;
        debugger.updateStopByTrip(stop, time);
        arrivalTime[stop] = time;
        if constexpr (INITIAL_ARRIVAL) {
            parentLabel[stop].parent = noStop;
        } else {
            parentLabel[stop].parent = data.connections[tripReached[trip]].departureStopId;
        }
        parentLabel[stop].reachedByTransfer = false;
        parentLabel[stop].tripId = trip;

        for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
            debugger.relaxEdge(edge);
            const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
            const int newArrivalTime = time + data.transferGraph.get(TravelTime, edge);
            arrivalByTransfer(toStop, newArrivalTime, stop, edge);
        }
    }

    inline void arrivalByTransfer(const StopId stop, const int time, const StopId parent, const Edge edge) noexcept {
        if (arrivalTime[stop] <= time) return;
        debugger.updateStopByTransfer(stop, time);
        arrivalTime[stop] = time;
        parentLabel[stop].parent = parent;
        parentLabel[stop].reachedByTransfer = true;
        parentLabel[stop].transferId = edge;
    }

private:
    const Data& data;

    StopId sourceStop;
    StopId targetStop;

    std::vector<TripFlag> tripReached;
    std::vector<int> arrivalTime;
    std::vector<ParentLabel> parentLabel;

    Debugger debugger;
};
}
