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
#include "../../DataStructures/Container/ExternalKHeap.h"

namespace CSA {

template<typename DEBUGGER>
class DijkstraCSA {

public:
    using Debugger = DEBUGGER;
    using Type = DijkstraCSA<Debugger>;
    using TripFlag = ConnectionId;

private:
    struct ParentLabel {
        ParentLabel(const Vertex parent = noVertex, const TripId tripId = noTripId) :
            parent(parent),
            tripId(tripId) {
        }

        Vertex parent;
        TripId tripId;
    };

    struct DijkstraLabel : public ExternalKHeapElement {
        DijkstraLabel() : arrivalTime(never), parent(noVertex) {}
        int arrivalTime;
        Vertex parent;
        inline bool hasSmallerKey(const DijkstraLabel* const other) const noexcept {
            return arrivalTime < other->arrivalTime;
        }
    };

public:
    template<typename ATTRIBUTE>
    DijkstraCSA(const Data& data, const CHGraph& forwardGraph, const CHGraph& backwardGraph, const ATTRIBUTE weight, const Debugger& debuggerTemplate = Debugger()) :
        data(data),
        initialTransfers(forwardGraph, backwardGraph, data.numberOfStops(), weight),
        sourceVertex(noVertex),
        sourceDepartureTime(never),
        targetVertex(noVertex),
        tripReached(data.numberOfTrips(), TripFlag()),
        arrivalTime(data.numberOfStops() + 1, never),
        parentLabel(data.numberOfStops() + 1),
        dijkstraLabels(data.transferGraph.numVertices()),
        debugger(debuggerTemplate) {
        AssertMsg(Vector::isSorted(data.connections), "Connections must be sorted in ascending order!");
        debugger.initialize(data);
    }

    DijkstraCSA(const Data& data, const CH::CH& chData, const Debugger& debuggerTemplate = Debugger()) :
        DijkstraCSA(data, chData.forward, chData.backward, Weight, debuggerTemplate) {
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target = noVertex) noexcept {
        debugger.start();
        debugger.startClear();
        clear();
        debugger.doneClear();

        debugger.startInitialization();
        sourceVertex = source;
        sourceDepartureTime = departureTime;
        targetVertex = target;
        if (target == noVertex) {
            targetStop = noStop;
        } else {
            targetStop = data.isStop(target) ? StopId(target) : StopId(data.numberOfStops());
        }
        if (data.isStop(source)) {
            arrivalTime[source] = departureTime;
        }
        runInitialTransfers();
        const ConnectionId firstConnection = firstReachableConnection(departureTime);
        debugger.doneInitialization();

        debugger.startConnectionScan();
        scanConnections(firstConnection, ConnectionId(data.connections.size()));
        debugger.doneConnectionScan();
        debugger.done();
    }

    inline int getEarliestArrivalTime(const Vertex vertex) const noexcept {
        const StopId stop = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        return arrivalTime[stop];
    }

    inline const Debugger& getDebugger() const noexcept {
        return debugger;
    }

private:
    inline void clear() {
        sourceVertex = noVertex;
        sourceDepartureTime = never;
        targetVertex = noVertex;
        targetStop = noStop;
        Vector::fill(arrivalTime, never);
        Vector::fill(tripReached, TripFlag());
        Vector::fill(dijkstraLabels, DijkstraLabel());
        queue.clear();
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
            runDijkstra(connection.departureTime);
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
        parentLabel[stop].tripId = trip;

        arrivalByEdge(stop, time, stop);
        if (initialTransfers.getBackwardDistance(stop) != INFTY) {
            debugger.relaxEdge(noEdge);
            const int newArrivalTime = time + initialTransfers.getBackwardDistance(stop);
            arrivalByTransfer(targetStop, newArrivalTime, stop);
        }
    }

    inline void runInitialTransfers() noexcept {
        initialTransfers.run(sourceVertex, targetVertex);
        for (const Vertex stop : initialTransfers.getForwardPOIs()) {
            AssertMsg(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
            AssertMsg(initialTransfers.getForwardDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
            debugger.relaxEdge(noEdge);
            const int newArrivalTime = sourceDepartureTime + initialTransfers.getForwardDistance(stop);
            arrivalByTransfer(StopId(stop), newArrivalTime, sourceVertex);
        }
        if (initialTransfers.getDistance() != INFTY) {
            const int newArrivalTime = sourceDepartureTime + initialTransfers.getDistance();
            debugger.relaxEdge(noEdge);
            arrivalByTransfer(targetStop, newArrivalTime, sourceVertex);
        }
    }

    inline void runDijkstra(const int nextDepartureTime) noexcept {
        while ((!queue.empty()) && (queue.min().arrivalTime <= nextDepartureTime)) {
            DijkstraLabel* const uLabel = queue.extractFront();
            const int time = uLabel->arrivalTime;
            if (targetStop != noStop && time > arrivalTime[targetStop]) break;
            const Vertex u = Vertex(uLabel - &(dijkstraLabels[0]));
            for (const Edge edge : data.transferGraph.edgesFrom(u)) {
                debugger.relaxEdge(edge);
                const Vertex v = data.transferGraph.get(ToVertex, edge);
                if (v == targetVertex || v == uLabel->parent) continue;
                const int newArrivalTime = time + data.transferGraph.get(TravelTime, edge);
                arrivalByEdge(v, newArrivalTime, uLabel->parent);
            }
            if (data.isStop(u)) {
                arrivalByTransfer(StopId(u), uLabel->arrivalTime, uLabel->parent);
            }
        }
    }

    inline void arrivalByEdge(const Vertex vertex, const int time, const Vertex parent) noexcept {
        if (dijkstraLabels[vertex].arrivalTime <= time) return;
        dijkstraLabels[vertex].arrivalTime = time;
        dijkstraLabels[vertex].parent = parent;
        queue.update(&dijkstraLabels[vertex]);
    }

    inline void arrivalByTransfer(const StopId stop, const int time, const Vertex parent) noexcept {
        if (arrivalTime[stop] <= time) return;
        debugger.updateStopByTransfer(stop, time);
        arrivalTime[stop] = time;
        parentLabel[stop].parent = parent;
        parentLabel[stop].tripId = noTripId;
    }

private:
    const Data& data;
    RAPTOR::CoreCHInitialTransfers initialTransfers;

    Vertex sourceVertex;
    int sourceDepartureTime;
    Vertex targetVertex;
    StopId targetStop;

    std::vector<TripFlag> tripReached;
    std::vector<int> arrivalTime;
    std::vector<ParentLabel> parentLabel;
    std::vector<DijkstraLabel> dijkstraLabels;
    ExternalKHeap<2, DijkstraLabel> queue;

    Debugger debugger;

};
}
