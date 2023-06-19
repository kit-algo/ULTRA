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
#include "../../DataStructures/CSA/Entities/Journey.h"
#include "../../DataStructures/Container/ExternalKHeap.h"
#include "Profiler.h"

namespace CSA {

template<typename INITIAL_TRANSFER_GRAPH, bool PATH_RETRIEVAL = true, typename PROFILER = NoProfiler>
class OneToAllDijkstraCSA {

public:
    using InitialTransferGraph = INITIAL_TRANSFER_GRAPH;
    constexpr static bool PathRetrieval = PATH_RETRIEVAL;
    using Profiler = PROFILER;
    using Type = OneToAllDijkstraCSA<InitialTransferGraph, PathRetrieval, Profiler>;
    using TripFlag = Meta::IF<PathRetrieval, ConnectionId, bool>;

private:
    struct DijkstraLabel : public ExternalKHeapElement {
        DijkstraLabel(int* const arrivalTime) :
            ExternalKHeapElement(),
            arrivalTime(arrivalTime) {
        }

        inline int getArrivalTime() const noexcept {
            return *arrivalTime;
        }

        inline bool hasSmallerKey(const DijkstraLabel* other) const noexcept {return getArrivalTime() < other->getArrivalTime();}

        int* const arrivalTime;
    };

public:
    template<typename ATTRIBUTE>
    OneToAllDijkstraCSA(const Data& data, const InitialTransferGraph& initialTransferGraph, const ATTRIBUTE weight, const Profiler& profilerTemplate = Profiler()) :
        data(data),
        initialTransferGraph(initialTransferGraph),
        initialTransferWeight(initialTransferGraph[weight]),
        sourceVertex(noVertex),
        tripReached(data.numberOfTrips(), TripFlag()),
        arrivalTime(initialTransferGraph.numVertices(), never),
        parent(PathRetrieval ? data.numberOfStops() : 0, noVertex),
        parentTrip(PathRetrieval ? data.numberOfStops() : 0, noTripId),
        dijkstraParent(PathRetrieval ? initialTransferGraph.numVertices() : 0, noVertex),
        profiler(profilerTemplate) {
        AssertMsg(Vector::isSorted(data.connections), "Connections must be sorted in ascending order!");
        profiler.registerPhases({PHASE_CLEAR, PHASE_INITIALIZATION, PHASE_CONNECTION_SCAN, PHASE_FINAL_TRANSFERS});
        profiler.registerMetrics({METRIC_CONNECTIONS, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
        for (const Vertex vertex : initialTransferGraph.vertices()) {
            dijkstraLabels.emplace_back(&arrivalTime[vertex]);
        }
    }

    template<typename T = TransferGraph, typename = std::enable_if_t<Meta::Equals<T, TransferGraph>() && Meta::Equals<T, InitialTransferGraph>()>>
    OneToAllDijkstraCSA(const Data& data, const Profiler& profilerTemplate = Profiler()) :
        OneToAllDijkstraCSA(data, data.transferGraph, TravelTime, profilerTemplate) {
    }

    inline void run(const Vertex source, const int departureTime) noexcept {
        profiler.start();

        profiler.startPhase();
        clear();
        profiler.donePhase(PHASE_CLEAR);

        profiler.startPhase();
        sourceVertex = source;
        if constexpr (PathRetrieval) {
            dijkstraParent[source] = source;
        }
        arrivalByTransfer(source, departureTime, source);
        runInitialTransfers();
        const ConnectionId firstConnection = firstReachableConnection(departureTime);
        profiler.donePhase(PHASE_INITIALIZATION);

        profiler.startPhase();
        scanConnections(firstConnection, ConnectionId(data.connections.size()));
        profiler.donePhase(PHASE_CONNECTION_SCAN);

        profiler.startPhase();
        runDijkstra(INFTY);
        profiler.donePhase(PHASE_FINAL_TRANSFERS);

        profiler.done();
    }

    inline bool reachable(const Vertex vertex) const noexcept {
        return arrivalTime[vertex] < never;
    }

    inline int getEarliestArrivalTime(const Vertex vertex) const noexcept {
        return arrivalTime[vertex];
    }

    template<bool T = PathRetrieval, typename = std::enable_if_t<T == PathRetrieval && T>>
    inline Journey getJourney(const Vertex vertex) noexcept {
        Journey journey;
        if (!reachable(vertex)) return journey;
        StopId stop(vertex);
        if (!data.isStop(vertex)) {
            stop = StopId(dijkstraParent[vertex]);
            journey.emplace_back(stop, vertex, arrivalTime[stop], arrivalTime[vertex]);
        }
        while (stop != sourceVertex) {
            if (parentTrip[stop] == noTripId) {
                journey.emplace_back(parent[stop], stop, arrivalTime[parent[stop]], arrivalTime[stop]);
            } else {
                journey.emplace_back(parent[stop], stop, data.connections[tripReached[parentTrip[stop]]].departureTime, arrivalTime[stop], parentTrip[stop]);
            }
            stop = StopId(parent[stop]);
        }
        Vector::reverse(journey);
        return journey;
    }

    template<bool T = PathRetrieval, typename = std::enable_if_t<T == PathRetrieval && T>>
    inline std::vector<Vertex> getPath(const Vertex vertex) noexcept {
        return journeyToPath(getJourney(vertex));
    }

    template<bool T = PathRetrieval, typename = std::enable_if_t<T == PathRetrieval && T>>
    inline std::vector<std::string> getRouteDescription(const Vertex vertex) noexcept {
        return data.journeyToText(getJourney(vertex));
    }

    inline const Profiler& getProfiler() const noexcept {
        return profiler;
    }

private:
    inline void clear() {
        sourceVertex = noVertex;
        Vector::fill(tripReached, TripFlag());
        Vector::fill(arrivalTime, never);
        if constexpr (PathRetrieval) {
            Vector::fill(parent, noVertex);
            Vector::fill(parentTrip, noTripId);
            Vector::fill(dijkstraParent, noVertex);
        }
        queue.clear();
    }

    inline void runInitialTransfers() noexcept {
        while(!queue.empty()) {
            settle(initialTransferGraph, initialTransferWeight);
        }
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
        if (connectionIsReachableFromStop(connection)) {
            if constexpr (PathRetrieval) {
                tripReached[connection.tripId] = id;
            } else {
                suppressUnusedParameterWarning(id);
                tripReached[connection.tripId] = true;
            }
            return true;
        }
        return false;
    }

    inline void arrivalByTrip(const StopId stop, const int time, const TripId trip) noexcept {
        if (arrivalTime[stop] <= time) return;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        arrivalTime[stop] = time;
        queue.update(&dijkstraLabels[stop]);
        if constexpr (PathRetrieval) {
            parent[stop] = data.connections[tripReached[trip]].departureStopId;
            parentTrip[stop] = trip;
            dijkstraParent[stop] = stop;
        } else {
            suppressUnusedParameterWarning(trip);
        }
    }

    inline void runDijkstra(const int nextDepartureTime) noexcept {
        while ((!queue.empty()) && (queue.min().getArrivalTime() <= nextDepartureTime)) {
            settle(data.transferGraph, data.transferGraph[TravelTime]);
        }
    }

    template<typename GRAPH>
    inline void settle(const GRAPH& graph, const std::vector<int>& weight) noexcept {
        const Vertex u = Vertex(queue.extractFront() - &(dijkstraLabels[0]));
        const int time = arrivalTime[u];
        for (const Edge edge : graph.edgesFrom(u)) {
            profiler.countMetric(METRIC_EDGES);
            const Vertex v = graph.get(ToVertex, edge);
            const int newArrivalTime = time + weight[edge];
            if (newArrivalTime < arrivalTime[v]) {
                arrivalByTransfer(v, newArrivalTime, u);
            }
        }
        if (data.isStop(u)) {
            profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        }
    }

    inline void arrivalByTransfer(const Vertex vertex, const int time, const Vertex parentVertex) noexcept {
        arrivalTime[vertex] = time;
        queue.update(&dijkstraLabels[vertex]);
        if constexpr (PathRetrieval) {
            dijkstraParent[vertex] = dijkstraParent[parentVertex];
            if (data.isStop(vertex)) {
                parent[vertex] = dijkstraParent[parentVertex];
                parentTrip[vertex] = noTripId;
            }
        } else {
            suppressUnusedParameterWarning(parentVertex);
        }
    }

private:
    const Data& data;
    const InitialTransferGraph& initialTransferGraph;
    const std::vector<int>& initialTransferWeight;

    Vertex sourceVertex;

    std::vector<TripFlag> tripReached;
    std::vector<int> arrivalTime;
    std::vector<Vertex> parent;
    std::vector<TripId> parentTrip;
    std::vector<Vertex> dijkstraParent;

    ExternalKHeap<2, DijkstraLabel> queue;
    std::vector<DijkstraLabel> dijkstraLabels;

    Profiler profiler;

};
}
