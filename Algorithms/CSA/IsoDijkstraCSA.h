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

template<typename PROFILER = NoProfiler>
class IsoDijkstraCSA {

public:
    using Profiler = PROFILER;
    using Type = IsoDijkstraCSA<Profiler>;

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
    IsoDijkstraCSA(const Data& data, const Profiler& profilerTemplate = Profiler()) :
        data(data),
        sourceVertex(noVertex),
        maxArrivalTime(never),
        tripReached(data.numberOfTrips(), false),
        arrivalTime(data.transferGraph.numVertices(), never),
        isochrone(data.transferGraph.numVertices()),
        profiler(profilerTemplate) {
        AssertMsg(Vector::isSorted(data.connections), "Connections must be sorted in ascending order!");
        profiler.registerPhases({PHASE_CLEAR, PHASE_INITIALIZATION, PHASE_CONNECTION_SCAN, PHASE_FINAL_TRANSFERS});
        profiler.registerMetrics({METRIC_CONNECTIONS, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
        for (const Vertex vertex : data.transferGraph.vertices()) {
            dijkstraLabels.emplace_back(&arrivalTime[vertex]);
        }
    }

    inline void run(const Vertex source, const int departureTime, const int range) noexcept {
        profiler.start();

        profiler.startPhase();
        clear();
        profiler.donePhase(PHASE_CLEAR);

        profiler.startPhase();
        sourceVertex = source;
        maxArrivalTime = departureTime + range;
        arrivalTime[source] = departureTime;
        queue.update(&dijkstraLabels[source]);
        isochrone.insert(source);
        const ConnectionId connectionStart = firstReachableConnection(departureTime);
        const ConnectionId connectionEnd = firstUnreachableConnection(maxArrivalTime);
        profiler.donePhase(PHASE_INITIALIZATION);

        profiler.startPhase();
        scanConnections(connectionStart, connectionEnd);
        profiler.donePhase(PHASE_CONNECTION_SCAN);

        profiler.startPhase();
        runDijkstra(maxArrivalTime);
        profiler.donePhase(PHASE_FINAL_TRANSFERS);

        profiler.done();
    }

    inline bool reachable(const Vertex vertex) const noexcept {
        return isochrone.contains(vertex);
    }

    inline const std::vector<Vertex>& getIsochrone() const noexcept {
        return isochrone.getValues();
    }

    inline int getEarliestArrivalTime(const Vertex vertex) const noexcept {
        return arrivalTime[vertex];
    }

    inline const Profiler& getProfiler() const noexcept {
        return profiler;
    }

private:
    inline void clear() {
        sourceVertex = noVertex;
        maxArrivalTime = never;
        Vector::fill(tripReached, false);
        Vector::fill(arrivalTime, never);
        queue.clear();
        isochrone.clear();
    }

    inline ConnectionId firstReachableConnection(const int departureTime) const noexcept {
        return ConnectionId(Vector::lowerBound(data.connections, departureTime, [](const Connection& connection, const int time) {
            return connection.departureTime < time;
        }));
    }

    inline ConnectionId firstUnreachableConnection(const int departureTime) const noexcept {
        return ConnectionId(Vector::upperBound(data.connections, departureTime, [](const int time, const Connection& connection) {
            return time < connection.departureTime;
        }));
    }


    inline void scanConnections(const ConnectionId begin, const ConnectionId end) noexcept {
        for (ConnectionId i = begin; i < end; i++) {
            const Connection& connection = data.connections[i];
            if (connection.arrivalTime > maxArrivalTime) continue;
            runDijkstra(connection.departureTime);
            if (connectionIsReachable(connection)) {
                profiler.countMetric(METRIC_CONNECTIONS);
                arrivalByTrip(connection.arrivalStopId, connection.arrivalTime);
            }
        }
    }

    inline bool connectionIsReachableFromStop(const Connection& connection) const noexcept {
        return arrivalTime[connection.departureStopId] <= connection.departureTime - data.minTransferTime(connection.departureStopId);
    }

    inline bool connectionIsReachableFromTrip(const Connection& connection) const noexcept {
        return tripReached[connection.tripId];
    }

    inline bool connectionIsReachable(const Connection& connection) noexcept {
        if (connectionIsReachableFromTrip(connection)) return true;
        if (connectionIsReachableFromStop(connection)) {
            tripReached[connection.tripId] = true;
            return true;
        }
        return false;
    }

    inline void arrivalByTrip(const StopId stop, const int time) noexcept {
        if (arrivalTime[stop] <= time) return;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        arrivalTime[stop] = time;
        isochrone.insert(stop);
        queue.update(&dijkstraLabels[stop]);
    }

    inline void runDijkstra(const int nextDepartureTime) noexcept {
        while ((!queue.empty()) && (queue.min().getArrivalTime() <= nextDepartureTime)) {
            const Vertex u = Vertex(queue.extractFront() - &(dijkstraLabels[0]));
            const int time = arrivalTime[u];
            isochrone.insert(u);
            for (const Edge edge : data.transferGraph.edgesFrom(u)) {
                profiler.countMetric(METRIC_EDGES);
                const Vertex v = data.transferGraph.get(ToVertex, edge);
                const int newArrivalTime = time + data.transferGraph.get(TravelTime, edge);
                if (newArrivalTime < arrivalTime[v]) {
                    arrivalTime[v] = newArrivalTime;
                    queue.update(&dijkstraLabels[v]);
                }
            }
            if (data.isStop(u)) {
                profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
            }
        }
    }

private:
    const Data& data;

    Vertex sourceVertex;
    int maxArrivalTime;

    std::vector<bool> tripReached;
    std::vector<int> arrivalTime;

    ExternalKHeap<2, DijkstraLabel> queue;
    std::vector<DijkstraLabel> dijkstraLabels;

    IndexedSet<false, Vertex> isochrone;

    Profiler profiler;
};
}
