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
#include "../../DataStructures/CSA/Entities/Journey.h"
#include "Profiler.h"

namespace CSA {

template<typename PROFILER = NoProfiler>
class HLCSA {

public:
    using Profiler = PROFILER;
    using Type = HLCSA<Profiler>;
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
        TripId tripId;
    };

public:
    HLCSA(const Data& data, const TransferGraph& outHubGraph, const TransferGraph& inHubGraph, const Profiler& profilerTemplate = Profiler()) :
        data(data),
        outHubs(outHubGraph),
        inHubs(inHubGraph),
        transferDistanceToTarget(inHubs.numVertices(), INFTY),
        sourceVertex(noVertex),
        sourceDepartureTime(never),
        targetVertex(noVertex),
        lastTarget(Vertex(0)),
        tripReached(data.numberOfTrips(), TripFlag()),
        arrivalTime(inHubGraph.numVertices(), never),
        parentLabel(inHubGraph.numVertices()),
        profiler(profilerTemplate) {
        AssertMsg(Vector::isSorted(data.connections), "Connections must be sorted in ascending order!");
        profiler.registerPhases({PHASE_CLEAR, PHASE_INITIALIZATION, PHASE_CONNECTION_SCAN});
        profiler.registerMetrics({METRIC_CONNECTIONS, METRIC_EDGES, METRIC_STOPS_BY_TRIP});
        profiler.initialize();
        outHubs.sortEdges(TravelTime);
        inHubs.sortEdges(TravelTime);
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target = noVertex) noexcept {
        profiler.start();

        profiler.startPhase();
        clear();
        profiler.donePhase(PHASE_CLEAR);

        profiler.startPhase();
        sourceVertex = source;
        sourceDepartureTime = departureTime;
        targetVertex = target;

        arrivalTime[sourceVertex] = departureTime;
        runInitialTransfers();
        const ConnectionId firstConnection = firstReachableConnection(departureTime);
        profiler.donePhase(PHASE_INITIALIZATION);

        profiler.startPhase();
        scanConnections(firstConnection, ConnectionId(data.connections.size()));
        profiler.donePhase(PHASE_CONNECTION_SCAN);

        profiler.done();
    }

    inline bool reachable(const Vertex vertex) const noexcept {
        return arrivalTime[vertex] < never;
    }

    inline int getEarliestArrivalTime(const Vertex vertex) const noexcept {
        return arrivalTime[vertex];
    }

    inline Journey getJourney() noexcept {
        return getJourney(targetVertex);
    }

    inline Journey getJourney(Vertex vertex) noexcept {
        Journey journey;
        if (!reachable(vertex)) return journey;
        while (vertex != sourceVertex) {
            const ParentLabel& label = parentLabel[vertex];
            if (label.reachedByTransfer) {
                const int parentDepartureTime = (label.parent == sourceVertex) ? sourceDepartureTime : arrivalTime[label.parent];
                if (!journey.empty() && !journey.back().usesTrip) {
                    journey.back().from = label.parent;
                    journey.back().departureTime = parentDepartureTime;
                } else {
                    journey.emplace_back(label.parent, vertex, parentDepartureTime, arrivalTime[vertex]);
                }
            } else {
                journey.emplace_back(label.parent, vertex, data.connections[tripReached[label.tripId]].departureTime, arrivalTime[vertex], label.tripId);
            }
            vertex = label.parent;
        }
        Vector::reverse(journey);
        return journey;
    }

    inline std::vector<Vertex> getPath(const Vertex vertex) noexcept {
        return journeyToPath(getJourney(vertex));
    }

    inline std::vector<std::string> getRouteDescription(const Vertex vertex) noexcept {
        return data.journeyToText(getJourney(vertex));
    }

    inline const Profiler& getProfiler() const noexcept {
        return profiler;
    }

private:
    inline void clear() {
        sourceVertex = noVertex;
        sourceDepartureTime = never;
        targetVertex = noVertex;
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
            if (targetVertex != noVertex && connection.departureTime > arrivalTime[targetVertex]) break;
            if (connectionIsReachable(connection, i)) {
                profiler.countMetric(METRIC_CONNECTIONS);
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
        scanInHubs(connection.departureStopId);
        if (connectionIsReachableFromStop(connection)) {
            tripReached[connection.tripId] = id;
            return true;
        }
        return false;
    }

    inline void arrivalByTrip(const StopId stop, const int time, const TripId trip) noexcept {
        if (arrivalTime[stop] <= time) return;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        arrivalTime[stop] = time;
        parentLabel[stop].parent = data.connections[tripReached[trip]].departureStopId;
        parentLabel[stop].reachedByTransfer = false;
        parentLabel[stop].tripId = trip;
        scanOutHubs(stop);
    }

    inline void runInitialTransfers() noexcept {
        transferDistanceToTarget[lastTarget] = INFTY;
        for (const Edge edge : inHubs.edgesFrom(lastTarget)) {
            transferDistanceToTarget[inHubs.get(ToVertex, edge)] = INFTY;
        }
        transferDistanceToTarget[targetVertex] = 0;
        for (const Edge edge : inHubs.edgesFrom(targetVertex)) {
            transferDistanceToTarget[inHubs.get(ToVertex, edge)] = inHubs.get(TravelTime, edge);
        }
        lastTarget = targetVertex;
        scanOutHubs(sourceVertex);
    }

    inline void scanOutHubs(const Vertex from) noexcept {
        for (const Edge edge : outHubs.edgesFrom(from)) {
            profiler.countMetric(METRIC_EDGES);
            const Vertex hub = outHubs.get(ToVertex, edge);
            const int newArrivalTime = arrivalTime[from] + outHubs.get(TravelTime, edge);
            if (newArrivalTime >= arrivalTime[targetVertex]) break;
            arrivalByTransfer(hub, newArrivalTime, from);
            if (transferDistanceToTarget[hub] != INFTY) {
                profiler.countMetric(METRIC_EDGES);
                arrivalByTransfer(targetVertex, newArrivalTime + transferDistanceToTarget[hub], from);
            }
        }
    }

    inline void scanInHubs(const Vertex to) noexcept {
        for (const Edge edge : inHubs.edgesFrom(to)) {
            profiler.countMetric(METRIC_EDGES);
            const Vertex hub = inHubs.get(ToVertex, edge);
            const int walkingTime = inHubs.get(TravelTime, edge);
            if (sourceDepartureTime + walkingTime >= arrivalTime[to]) break;
            const int newArrivalTime = arrivalTime[hub] + walkingTime;
            arrivalByTransfer(to, newArrivalTime, hub);
        }
    }

    inline void arrivalByTransfer(const Vertex vertex, const int time, const Vertex parent) noexcept {
        if (arrivalTime[vertex] <= time) return;
        arrivalTime[vertex] = time;
        parentLabel[vertex].parent = parent;
        parentLabel[vertex].reachedByTransfer = true;
    }

private:
    const Data& data;
    TransferGraph outHubs;
    TransferGraph inHubs;

    std::vector<int> transferDistanceToTarget;

    Vertex sourceVertex;
    int sourceDepartureTime;
    Vertex targetVertex;
    Vertex lastTarget;

    std::vector<TripFlag> tripReached;
    std::vector<int> arrivalTime;
    std::vector<ParentLabel> parentLabel;

    Profiler profiler;

};
}
