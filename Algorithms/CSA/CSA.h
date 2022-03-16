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
        AssertMsg(Vector::isSorted(data.connections), "Connections must be sorted in ascending order!");
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
        arrivalTime[sourceStop] = departureTime;
        relaxEdges(sourceStop, departureTime);
        const ConnectionId firstConnection = firstReachableConnection(departureTime);
        debugger.doneInitialization();

        debugger.startConnectionScan();
        scanConnections(firstConnection, ConnectionId(data.connections.size()));
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

    inline ConnectionId firstReachableConnection(const int departureTime) const noexcept {
        return ConnectionId(Vector::lowerBound(data.connections, departureTime, [](const Connection& connection, const int time) {
            return connection.departureTime < time;
        }));
    }

    inline void scanConnections(const ConnectionId begin, const ConnectionId end) noexcept {
        for (ConnectionId i = begin; i < end; i++) {
            const Connection& connection = data.connections[i];
            if (targetStop != noStop && connection.departureTime > arrivalTime[targetStop]) break;
            if (connectionIsReachable(connection, i)) {
                debugger.scanConnection(connection);
                arrivalByTrip(connection.arrivalStopId, connection.arrivalTime, connection.tripId);
            }
        }
    }

    inline bool connectionIsReachableFromStop(const Connection& connection) const noexcept {
        return arrivalTime[connection.departureStopId] <= connection.departureTime - data.minTransferTime(connection.departureStopId);
    }

    inline bool connectionIsReachableFromTrip(const Connection& connection) const noexcept {
        return tripReached[connection.tripId] != TripFlag();
    }

    inline bool connectionIsReachable(const Connection& connection, const ConnectionId id) noexcept {
        if (connectionIsReachableFromTrip(connection)) return true;
        if (connectionIsReachableFromStop(connection)) {
            tripReached[connection.tripId] = id;
            return true;
        }
        return false;
    }

    inline void arrivalByTrip(const StopId stop, const int time, const TripId trip) noexcept {
        if (arrivalTime[stop] <= time) return;
        debugger.updateStopByTrip(stop, time);
        arrivalTime[stop] = time;
        parentLabel[stop].parent = data.connections[tripReached[trip]].departureStopId;
        parentLabel[stop].reachedByTransfer = false;
        parentLabel[stop].tripId = trip;
        relaxEdges(stop, time);
    }

    inline void relaxEdges(const StopId stop, const int time) noexcept {
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
