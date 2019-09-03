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

#include "../CH/CH.h"
#include "../RAPTOR/InitialTransfers.h"

#include "../../Helpers/Assert.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/Types.h"
#include "../../Helpers/Vector/Vector.h"
#include "../../DataStructures/CSA/Data.h"

namespace CSA {

template<typename DEBUGGER>
class ULTRACSA {

public:
    using Debugger = DEBUGGER;
    using Type = ULTRACSA<Debugger>;
    using TripFlag = ConnectionId;

private:
    struct ParentLabel {
        ParentLabel(const Vertex parent = noVertex, const bool reachedByTransfer = false, const TripId tripId = noTripId) :
            parent(parent),
            reachedByTransfer(reachedByTransfer),
            tripId(tripId) {
        }

        Vertex parent;
        bool reachedByTransfer;
        union {
            TripId tripId;
            Edge transferId;
        };
    };

public:
    template<typename ATTRIBUTE>
    ULTRACSA(const Data& data, const CHGraph& forwardGraph, const CHGraph& backwardGraph, const ATTRIBUTE weight, const Debugger& debuggerTemplate = Debugger()) :
        data(data),
        initialTransfers(forwardGraph, backwardGraph, data.numberOfStops(), weight),
        sourceVertex(noVertex),
        targetVertex(noVertex),
        tripReached(data.numberOfTrips(), TripFlag()),
        arrivalTime(data.numberOfStops() + 1, never),
        parentLabel(data.numberOfStops() + 1),
        debugger(debuggerTemplate) {
        AssertMsg(Vector::isSorted(data.connections), "Connections must be sorted in ascending order!");
        debugger.initialize(data);
    }

    ULTRACSA(const Data& data, const CH::CH& chData, const Debugger& debuggerTemplate = Debugger()) :
        ULTRACSA(data, chData.forward, chData.backward, Weight, debuggerTemplate) {
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target = noVertex) noexcept {
        debugger.start();
        debugger.startClear();
        clear();
        debugger.doneClear();

        debugger.startInitialization();
        sourceVertex = source;
        targetVertex = target;
        if (target == noVertex) {
            targetStop = noStop;
        } else {
            targetStop = data.isStop(target) ? StopId(target) : StopId(data.numberOfStops());
        }
        if (data.isStop(source)) {
            arrivalByTrip<true>(StopId(source), departureTime, noTripId);
        }
        runInitialTransfers(departureTime);
        const ConnectionId firstConnection(Vector::lowerBound(data.connections, departureTime, [](const Connection& connection, const int time) {
            return connection.departureTime < time;
        }));
        debugger.doneInitialization();

        debugger.startConnectionScan();
        for (ConnectionId i = firstConnection; i < data.connections.size(); i++) {
            const Connection& connection = data.connections[i];
            if (target != noVertex && connection.departureTime > getEarliestArrivalTime(targetStop)) break;
            if (connectionIsReachable(connection)) {
                debugger.scanConnection(connection);
                useTrip(connection.tripId, i);
                arrivalByTrip(connection.arrivalStopId, connection.arrivalTime, connection.tripId);
            }
        }
        debugger.doneConnectionScan();
        debugger.done();
    }

    inline int getEarliestArrivalTime(const Vertex vertex) const noexcept {
        const StopId stop = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        return arrivalTime[stop];
    }

    inline bool reachable(const Vertex vertex) const noexcept {
        const StopId stop = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        return arrivalTime[stop] < never;
    }

    inline const Debugger& getDebugger() const noexcept {
        return debugger;
    }

private:
    inline void clear() {
        sourceVertex = noVertex;
        targetVertex = noVertex;
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

        if constexpr (!INITIAL_ARRIVAL) {
            for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
                debugger.relaxEdge(edge);
                const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
                const int newArrivalTime = time + data.transferGraph.get(TravelTime, edge);
                arrivalByTransfer(toStop, newArrivalTime, stop, edge);
            }

            if (initialTransfers.getBackwardDistance(stop) != INFTY) {
                debugger.relaxEdge(noEdge);
                const int newArrivalTime = time + initialTransfers.getBackwardDistance(stop);
                arrivalByTransfer(targetStop, newArrivalTime, stop, noEdge);
            }
        }
    }

    inline void runInitialTransfers(const int sourceDepartureTime) noexcept {
        initialTransfers.run(sourceVertex, targetVertex);
        for (const Vertex stop : initialTransfers.getForwardPOIs()) {
            AssertMsg(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
            AssertMsg(initialTransfers.getForwardDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
            debugger.relaxEdge(noEdge);
            const int newArrivalTime = sourceDepartureTime + initialTransfers.getForwardDistance(stop);
            arrivalByTransfer(StopId(stop), newArrivalTime, sourceVertex, noEdge);
        }
        if (initialTransfers.getDistance() != INFTY) {
            const int newArrivalTime = sourceDepartureTime + initialTransfers.getDistance();
            debugger.relaxEdge(noEdge);
            arrivalByTransfer(targetStop, newArrivalTime, sourceVertex, noEdge);
        }
    }

    inline void arrivalByTransfer(const StopId stop, const int time, const Vertex parent, const Edge edge) noexcept {
        if (arrivalTime[stop] <= time) return;
        debugger.updateStopByTransfer(stop, time);
        arrivalTime[stop] = time;
        parentLabel[stop].parent = parent;
        parentLabel[stop].reachedByTransfer = true;
        parentLabel[stop].transferId = edge;
    }

private:
    const Data& data;
    RAPTOR::BucketCHInitialTransfers initialTransfers;

    Vertex sourceVertex;
    Vertex targetVertex;
    StopId targetStop;

    std::vector<TripFlag> tripReached;
    std::vector<int> arrivalTime;
    std::vector<ParentLabel> parentLabel;

    Debugger debugger;

};
}
