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

template<bool USE_STOP_BUCKETS, bool USE_TARGET_BUCKETS, bool PATH_RETRIEVAL = true, typename PROFILER = NoProfiler>
class UPCSA {

public:
    constexpr static bool UseStopBuckets = USE_STOP_BUCKETS;
    constexpr static bool UseTargetBuckets = USE_TARGET_BUCKETS;
    constexpr static bool PathRetrieval = PATH_RETRIEVAL;
    using Profiler = PROFILER;
    constexpr static bool Debug = Meta::Equals<Profiler, SimpleProfiler>();
    using Type = UPCSA<UseStopBuckets, UseTargetBuckets, PathRetrieval, Profiler>;
    using InitialAndFinalTransfers = RAPTOR::BasicInitialAndFinalTransfers<Debug, UseStopBuckets, UseTargetBuckets>;
    using TripFlag = Meta::IF<PathRetrieval, ConnectionId, bool>;

private:
    inline static Order vertexOrder(const CH::CH& chData, const bool useDFSOrder) noexcept {
        if (useDFSOrder) {
            return Order(Vector::reverse(CH::getOrder(chData)));
        } else {
            return Order(CH::getLevelOrderTopDown(chData));
        }
    }

    struct QueryData {
        QueryData(const Data& oldData, const CH::CH& oldCHData, const IndexedSet<false, Vertex>& oldTargets, const bool reorder, const bool useDFSOrder) :
            data(oldData),
            chData(oldCHData),
            targets(oldTargets) {
            const Order chOrder = vertexOrder(chData, useDFSOrder);
            if (reorder) {
                internalToExternal = chOrder.splitAt(data.numberOfStops());
                externalToInternal = Permutation(Construct::Invert, internalToExternal);
                Order stopOrder = internalToExternal;
                stopOrder.resize(data.numberOfStops());

                data.applyStopOrder(stopOrder);
                chData.applyVertexOrder(internalToExternal);
                targets.applyPermutation(externalToInternal);

                size_t numStops = 0;
                size_t numVertices = data.numberOfStops();
                for (const size_t i : chOrder) {
                    if (i < data.numberOfStops()) {
                        phastOrder.emplace_back(numStops++);
                    } else {
                        phastOrder.emplace_back(numVertices++);
                    }
                }
            } else {
                phastOrder = chOrder;
                internalToExternal = Order(Construct::Id, chData.numVertices());
                externalToInternal = Permutation(Construct::Id, chData.numVertices());
            }
        }

        Data data;
        CH::CH chData;
        Order internalToExternal;
        Permutation externalToInternal;
        Order phastOrder;
        IndexedSet<false, Vertex> targets;
    };

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
    UPCSA(const Data& oldData, const CH::CH& oldCHData, const IndexedSet<false, Vertex>& oldTargets, const bool reorder, const bool useDFSOrder, const Profiler& profilerTemplate = Profiler()) :
        queryData(oldData, oldCHData, oldTargets, reorder, useDFSOrder),
        data(queryData.data),
        initialAndFinalTransfers(queryData.chData, queryData.phastOrder, data.numberOfStops(), queryData.targets),
        sourceVertex(noVertex),
        sourceDepartureTime(never),
        targetVertices(queryData.targets),
        tripReached(data.numberOfTrips(), TripFlag()),
        arrivalTime(data.numberOfStops(), never),
        parentLabel(PathRetrieval ? data.numberOfStops() : 0),
        profiler(profilerTemplate) {
        AssertMsg(Vector::isSorted(data.connections), "Connections must be sorted in ascending order!");
        AssertMsg(!Graph::hasLoops(data.transferGraph), "Shortcut graph may not have loops!");
        profiler.registerPhases({PHASE_CLEAR, PHASE_INITIALIZATION, PHASE_CONNECTION_SCAN, PHASE_UPWARD_SWEEP, PHASE_DOWNWARD_SEARCH});
        profiler.registerMetrics({METRIC_CONNECTIONS, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
    }

    inline void run(const Vertex source, const int departureTime) noexcept {
        profiler.start();

        profiler.startPhase();
        clear();
        profiler.donePhase(PHASE_CLEAR);

        profiler.startPhase();
        sourceVertex = Vertex(queryData.externalToInternal[source]);
        sourceDepartureTime = departureTime;
        initialAndFinalTransfers.initialize();
        if (data.isStop(sourceVertex)) {
            arrivalTime[sourceVertex] = departureTime;
        }
        runInitialTransfers();
        const ConnectionId firstConnection = firstReachableConnection(departureTime);
        profiler.donePhase(PHASE_INITIALIZATION);

        profiler.startPhase();
        scanConnections(firstConnection, ConnectionId(data.connections.size()));
        profiler.donePhase(PHASE_CONNECTION_SCAN);

        profiler.startPhase();
        for (const StopId stop : data.stops()) {
            initialAndFinalTransfers.template addSource<true>(stop, arrivalTime[stop], stop);
        }
        initialAndFinalTransfers.upwardSweep();
        profiler.donePhase(PHASE_UPWARD_SWEEP);
        profiler.startPhase();
        initialAndFinalTransfers.downwardSearchToTargets();
        profiler.donePhase(PHASE_DOWNWARD_SEARCH);

        profiler.done();
    }

    inline bool reachable(const Vertex vertex) noexcept {
        const Vertex internalVertex(queryData.externalToInternal[vertex]);
        AssertMsg(targetVertices.contains(internalVertex), "Vertex " << internalVertex << " is not a target!");
        return getEarliestArrivalTime(internalVertex) < never;
    }

    inline int getEarliestArrivalTime(const Vertex vertex) noexcept {
        const Vertex internalVertex(queryData.externalToInternal[vertex]);
        AssertMsg(targetVertices.contains(internalVertex), "Vertex " << internalVertex << " is not a target!");
        return initialAndFinalTransfers.getDistance(internalVertex);
    }

    template<bool T = PathRetrieval, typename = std::enable_if_t<T == PathRetrieval && T>>
    inline Journey getJourney(const Vertex vertex) noexcept {
        const Vertex internalVertex(queryData.externalToInternal[vertex]);
        Journey journey;
        const int distance = initialAndFinalTransfers.getDistance(internalVertex);
        if (distance == never) return journey;
        const Vertex parent = initialAndFinalTransfers.getParent(internalVertex);
        if (parent != internalVertex) {
            const int parentDepartureTime = (parent == sourceVertex) ? sourceDepartureTime : arrivalTime[parent];
            journey.emplace_back(parent, internalVertex, parentDepartureTime, distance, noEdge);
        }
        StopId stop = StopId(parent);
        while (stop != sourceVertex) {
            const ParentLabel& label = parentLabel[stop];
            if (label.reachedByTransfer) {
                const int parentDepartureTime = (label.parent == sourceVertex) ? sourceDepartureTime : arrivalTime[label.parent];
                if (!journey.empty() && !journey.back().usesTrip) {
                    journey.back().from = label.parent;
                    journey.back().departureTime = parentDepartureTime;
                    journey.back().transferId = noEdge;
                } else {
                    journey.emplace_back(label.parent, stop, parentDepartureTime, arrivalTime[stop], label.transferId);
                }
            } else {
                journey.emplace_back(label.parent, stop, data.connections[tripReached[label.tripId]].departureTime, arrivalTime[stop], label.tripId);
            }
            stop = StopId(label.parent);
        }
        Vector::reverse(journey);
        for (JourneyLeg& leg : journey) {
            leg.from = Vertex(queryData.internalToExternal[leg.from]);
            leg.to = Vertex(queryData.internalToExternal[leg.to]);
        }
        return journey;
    }

    inline std::vector<Vertex> getPath(const Vertex vertex) noexcept {
        const Vertex internalVertex(queryData.externalToInternal[vertex]);
        AssertMsg(targetVertices.contains(internalVertex), "Vertex " << internalVertex << " is not a target!");
        return journeyToPath(getJourney(internalVertex));
    }

    inline std::vector<std::string> getRouteDescription(const Vertex vertex) noexcept {
        const Vertex internalVertex(queryData.externalToInternal[vertex]);
        AssertMsg(targetVertices.contains(internalVertex), "Vertex " << internalVertex << " is not a target!");
        return data.journeyToText(getJourney(internalVertex));
    }

    inline const Profiler& getProfiler() const noexcept {
        return profiler;
    }

    inline long long getUpwardSweepGraphVertices() const noexcept {
        return initialAndFinalTransfers.getUpwardSweepGraphVertices();
    }

    inline long long getUpwardSweepGraphEdges() const noexcept {
        return initialAndFinalTransfers.getUpwardSweepGraphEdges();
    }

    inline long long getStopGraphVertices() const noexcept {
        return initialAndFinalTransfers.getStopGraphVertices();
    }

    inline long long getStopGraphEdges() const noexcept {
        return initialAndFinalTransfers.getStopGraphEdges();
    }

    inline long long getTargetGraphVertices() const noexcept {
        return initialAndFinalTransfers.getTargetGraphVertices();
    }

    inline long long getTargetGraphEdges() const noexcept {
        return initialAndFinalTransfers.getTargetGraphEdges();
    }

private:
    inline void clear() {
        sourceVertex = noVertex;
        sourceDepartureTime = never;
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
    }

    inline void runInitialTransfers() noexcept {
        initialAndFinalTransfers.template addSource<false>(sourceVertex, sourceDepartureTime, sourceVertex);
        initialAndFinalTransfers.upwardSearch();
        initialAndFinalTransfers.downwardSearchToStops();
        for (const StopId stop : data.stops()) {
            arrivalByTransfer(stop, initialAndFinalTransfers.getDistance(stop), sourceVertex, noEdge);
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
        } else {
            suppressUnusedParameterWarning(parent);
            suppressUnusedParameterWarning(edge);
        }
    }

private:
    const QueryData queryData;
    const Data& data;

    InitialAndFinalTransfers initialAndFinalTransfers;

    Vertex sourceVertex;
    int sourceDepartureTime;
    const IndexedSet<false, Vertex>& targetVertices;

    std::vector<TripFlag> tripReached;
    std::vector<int> arrivalTime;
    std::vector<ParentLabel> parentLabel;

    Profiler profiler;

};
}
