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

template<bool PATH_RETRIEVAL = true, typename PROFILER = NoProfiler>
class ULTRACSA {

public:
    using InitialTransferGraph = CHGraph;
    constexpr static bool PathRetrieval = PATH_RETRIEVAL;
    using Profiler = PROFILER;
    using Type = ULTRACSA<PathRetrieval, Profiler>;
    using TripFlag = Meta::IF<PathRetrieval, ConnectionId, bool>;

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
    ULTRACSA(const Data& data, const CH::CH& chData, const Profiler& profilerTemplate = Profiler()) :
        data(data),
        initialTransfers(chData, FORWARD, data.numberOfStops()),
        sourceVertex(noVertex),
        sourceDepartureTime(never),
        targetVertex(noVertex),
        tripReached(data.numberOfTrips(), TripFlag()),
        arrivalTime(data.numberOfStops() + 1, never),
        parentLabel(PathRetrieval ? data.numberOfStops() + 1 : 0),
        profiler(profilerTemplate) {
        AssertMsg(!Graph::hasLoops(data.transferGraph), "Shortcut graph may not have loops!");
        AssertMsg(Vector::isSorted(data.connections), "Connections must be sorted in ascending order!");
        profiler.registerPhases({PHASE_CLEAR, PHASE_INITIALIZATION, PHASE_CONNECTION_SCAN});
        profiler.registerMetrics({METRIC_CONNECTIONS, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
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
        if (target == noVertex) {
            targetStop = noStop;
        } else {
            targetStop = data.isStop(target) ? StopId(target) : StopId(data.numberOfStops());
        }
        if (data.isStop(sourceVertex)) {
            arrivalTime[sourceVertex] = departureTime;
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
            if (label.reachedByTransfer) {
                const int parentDepartureTime = (label.parent == sourceVertex) ? sourceDepartureTime : arrivalTime[label.parent];
                journey.emplace_back(label.parent, stop, parentDepartureTime, arrivalTime[stop], label.transferId);
            } else {
                journey.emplace_back(label.parent, stop, data.connections[tripReached[label.tripId]].departureTime, arrivalTime[stop], label.tripId);
            }
            stop = StopId(label.parent);
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
        targetStop = noStop;
        Vector::fill(arrivalTime, never);
        Vector::fill(tripReached, TripFlag());
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
            parentLabel[stop].reachedByTransfer = false;
            parentLabel[stop].tripId = trip;
        }

        for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
            profiler.countMetric(METRIC_EDGES);
            const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
            const int newArrivalTime = time + data.transferGraph.get(TravelTime, edge);
            arrivalByTransfer(toStop, newArrivalTime, stop, edge);
        }

        if (initialTransfers.getBackwardDistance(stop) != INFTY) {
            profiler.countMetric(METRIC_EDGES);
            const int newArrivalTime = time + initialTransfers.getBackwardDistance(stop);
            arrivalByTransfer(targetStop, newArrivalTime, stop, noEdge);
        }
    }

    inline void runInitialTransfers() noexcept {
        initialTransfers.run(sourceVertex, targetVertex);
        for (const Vertex stop : initialTransfers.getForwardPOIs()) {
            AssertMsg(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
            AssertMsg(initialTransfers.getForwardDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
            profiler.countMetric(METRIC_EDGES);
            const int newArrivalTime = sourceDepartureTime + initialTransfers.getForwardDistance(stop);
            arrivalByTransfer(StopId(stop), newArrivalTime, sourceVertex, noEdge);
        }
        if (initialTransfers.getDistance() != INFTY) {
            const int newArrivalTime = sourceDepartureTime + initialTransfers.getDistance();
            profiler.countMetric(METRIC_EDGES);
            arrivalByTransfer(targetStop, newArrivalTime, sourceVertex, noEdge);
        }
    }

    inline void arrivalByTransfer(const StopId stop, const int time, const Vertex parent, const Edge edge) noexcept {
        if (arrivalTime[stop] <= time) return;
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        arrivalTime[stop] = time;
        if constexpr (PathRetrieval) {
            parentLabel[stop].parent = parent;
            parentLabel[stop].reachedByTransfer = true;
            parentLabel[stop].transferId = edge;
        }
    }

private:
    const Data& data;
    RAPTOR::BucketCHInitialTransfers initialTransfers;

    Vertex sourceVertex;
    int sourceDepartureTime;
    Vertex targetVertex;
    StopId targetStop;

    std::vector<TripFlag> tripReached;
    std::vector<int> arrivalTime;
    std::vector<ParentLabel> parentLabel;

    Profiler profiler;

};
}
