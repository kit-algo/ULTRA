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
#include "../../DataStructures/Container/ExternalKHeap.h"
#include "Profiler.h"

namespace CSA {

template<typename INITIAL_TRANSFERS, bool PATH_RETRIEVAL = true, typename PROFILER = NoProfiler>
class DijkstraCSA {

public:
    using InitialTransferType = INITIAL_TRANSFERS;
    using InitialTransferGraph = typename InitialTransferType::Graph;
    constexpr static bool PathRetrieval = PATH_RETRIEVAL;
    using Profiler = PROFILER;
    using Type = DijkstraCSA<InitialTransferType, PathRetrieval, Profiler>;
    using TripFlag = Meta::IF<PathRetrieval, ConnectionId, bool>;

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
    DijkstraCSA(const Data& data, const InitialTransferGraph& forwardGraph, const InitialTransferGraph& backwardGraph, const ATTRIBUTE weight, const Profiler& profilerTemplate = Profiler()) :
        data(data),
        initialTransfers(forwardGraph, backwardGraph, data.numberOfStops(), weight),
        sourceVertex(noVertex),
        sourceDepartureTime(never),
        targetVertex(noVertex),
        tripReached(data.numberOfTrips(), TripFlag()),
        arrivalTime(data.numberOfStops() + 1, never),
        parentLabel(PathRetrieval ? data.numberOfStops() + 1 : 0),
        dijkstraLabels(data.transferGraph.numVertices()),
        profiler(profilerTemplate) {
        AssertMsg(Vector::isSorted(data.connections), "Connections must be sorted in ascending order!");
        profiler.registerPhases({PHASE_CLEAR, PHASE_INITIALIZATION, PHASE_CONNECTION_SCAN});
        profiler.registerMetrics({METRIC_CONNECTIONS, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
    }

    template<typename T = CHGraph, typename = std::enable_if_t<Meta::Equals<T, CHGraph>() && Meta::Equals<T, InitialTransferGraph>()>>
    DijkstraCSA(const Data& data, const CH::CH& chData, const Profiler& profilerTemplate = Profiler()) :
        DijkstraCSA(data, chData.forward, chData.backward, Weight, profilerTemplate) {
    }

    template<typename T = TransferGraph, typename = std::enable_if_t<Meta::Equals<T, TransferGraph>() && Meta::Equals<T, InitialTransferGraph>()>>
    DijkstraCSA(const Data& data, const TransferGraph& forwardGraph, const TransferGraph& backwardGraph, const Profiler& profilerTemplate = Profiler()) :
        DijkstraCSA(data, forwardGraph, backwardGraph, TravelTime, profilerTemplate) {
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target) noexcept {
        profiler.start();

        profiler.startPhase();
        clear();
        profiler.donePhase(PHASE_CLEAR);

        profiler.startPhase();
        sourceVertex = source;
        sourceDepartureTime = departureTime;
        targetVertex = target;
        targetStop = data.isStop(target) ? StopId(target) : StopId(data.numberOfStops());
        if (data.isStop(source)) {
            arrivalTime[source] = departureTime;
        }
        runInitialTransfers();
        const ConnectionId firstConnection = firstReachableConnection(departureTime);
        profiler.donePhase(PHASE_INITIALIZATION);

        profiler.startPhase();
        scanConnections(firstConnection, ConnectionId(data.connections.size()));
        profiler.donePhase(PHASE_CONNECTION_SCAN);

        profiler.done();
    }

    inline bool reachable(const Vertex vertex) const noexcept {
        const StopId stop = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        return arrivalTime[stop] < never;
    }

    inline int getEarliestArrivalTime(const Vertex vertex) const noexcept {
        const StopId stop = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        return arrivalTime[stop];
    }

    template<bool T = PathRetrieval, typename = std::enable_if_t<T == PathRetrieval && T>>
    inline Journey getJourney() noexcept {
        return getJourney(targetStop);
    }

    template<bool T = PathRetrieval, typename = std::enable_if_t<T == PathRetrieval && T>>
    inline Journey getJourney(const Vertex vertex) noexcept {
        StopId stop = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        Journey journey;
        if (!reachable(stop)) return journey;
        while (stop != sourceVertex) {
            const ParentLabel& label = parentLabel[stop];
            if (label.tripId == noTripId) {
                const int parentDepartureTime = (label.parent == sourceVertex) ? sourceDepartureTime : arrivalTime[label.parent];
                journey.emplace_back(label.parent, (stop == targetStop) ? targetVertex : stop, parentDepartureTime, arrivalTime[stop]);
            } else {
                journey.emplace_back(label.parent, stop, data.connections[tripReached[label.tripId]].departureTime, arrivalTime[stop], label.tripId);
            }
            stop = StopId(label.parent);
        };
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
        targetStop = noStop;
        Vector::fill(arrivalTime, never);
        Vector::fill(tripReached, TripFlag());
        Vector::fill(dijkstraLabels, DijkstraLabel());
        queue.clear();
        if constexpr (PathRetrieval) {
            Vector::fill(parentLabel, ParentLabel());
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
            if (targetStop != noStop && connection.departureTime > arrivalTime[targetStop]) break;
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
        if constexpr (PathRetrieval) {
            parentLabel[stop].parent = data.connections[tripReached[trip]].departureStopId;
            parentLabel[stop].tripId = trip;
        } else {
            suppressUnusedParameterWarning(trip);
        }

        arrivalByEdge(stop, time, stop);
        if (initialTransfers.getBackwardDistance(stop) != INFTY) {
            profiler.countMetric(METRIC_EDGES);
            const int newArrivalTime = time + initialTransfers.getBackwardDistance(stop);
            arrivalByTransfer(targetStop, newArrivalTime, stop);
        }
    }

    inline void runInitialTransfers() noexcept {
        initialTransfers.run(sourceVertex, targetVertex);
        for (const Vertex stop : initialTransfers.getForwardPOIs()) {
            AssertMsg(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
            AssertMsg(initialTransfers.getForwardDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
            profiler.countMetric(METRIC_EDGES);
            const int newArrivalTime = sourceDepartureTime + initialTransfers.getForwardDistance(stop);
            arrivalByTransfer(StopId(stop), newArrivalTime, sourceVertex);
        }
        if (initialTransfers.getDistance() != INFTY) {
            const int newArrivalTime = sourceDepartureTime + initialTransfers.getDistance();
            profiler.countMetric(METRIC_EDGES);
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
                profiler.countMetric(METRIC_EDGES);
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
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        arrivalTime[stop] = time;
        if constexpr (PathRetrieval) {
            parentLabel[stop].parent = parent;
            parentLabel[stop].tripId = noTripId;
        } else {
            suppressUnusedParameterWarning(parent);
        }
    }

private:
    const Data& data;
    InitialTransferType initialTransfers;

    Vertex sourceVertex;
    int sourceDepartureTime;
    Vertex targetVertex;
    StopId targetStop;

    std::vector<TripFlag> tripReached;
    std::vector<int> arrivalTime;
    std::vector<ParentLabel> parentLabel;
    std::vector<DijkstraLabel> dijkstraLabels;
    ExternalKHeap<2, DijkstraLabel> queue;

    Profiler profiler;

};
}
