#pragma once

#include "../../RAPTOR/InitialTransfers.h"

#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../../DataStructures/TripBased/RouteLabel.h"

#include "Profiler.h"
#include "ReachedIndex.h"

namespace TripBased {

template<size_t GROUPED_ROUNDS, typename PROFILER = NoProfiler>
class UPQuery {

public:
    constexpr static size_t GroupedRounds = GROUPED_ROUNDS;
    constexpr static bool GroupSweeps = GroupedRounds != 0;
    using Profiler = PROFILER;
    constexpr static bool Debug = !Meta::Equals<Profiler, NoProfiler>();
    using Type = UPQuery<GroupedRounds, Profiler>;
    using InitialAndFinalTransfers = RAPTOR::ParetoInitialAndFinalTransfers<false, GroupedRounds>;

private:
    struct QueryData {
        QueryData(const Data& oldData, const TransferGraph& oldTransferGraph, const CH::CH& oldCHData, const IndexedSet<false, Vertex>& oldTargets, const bool reorder) :
            transferGraph(oldTransferGraph),
            chData(oldCHData),
            targets(oldTargets) {
            Order chOrder(Vector::reverse(CH::getOrder(chData)));
            if (reorder) {
                internalToExternal = chOrder.splitAt(oldData.numberOfStops());
                externalToInternal = Permutation(Construct::Invert, internalToExternal);
                Order stopOrder = internalToExternal;
                stopOrder.resize(oldData.numberOfStops());

                RAPTOR::Data newRaptorData = oldData.raptorData;
                newRaptorData.applyStopOrder(stopOrder);
                data = Data(newRaptorData);
                data.stopEventGraph = oldData.stopEventGraph;
                transferGraph.applyVertexOrder(internalToExternal);
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
                data = oldData;
                phastOrder = chOrder;
                internalToExternal = Order(Construct::Id, chData.numVertices());
                externalToInternal = Permutation(Construct::Id, chData.numVertices());
            }
        }

        Data data;
        TransferGraph transferGraph;
        CH::CH chData;
        Order internalToExternal;
        Permutation externalToInternal;
        IndexedSet<false, Vertex> targets;
        Order phastOrder;
    };

    struct TripLabel {
        TripLabel(const StopEventId begin = noStopEvent, const StopEventId end = noStopEvent, const u_int32_t parent = -1) :
            begin(begin),
            end(end),
            parent(parent) {
        }
        StopEventId begin;
        StopEventId end;
        u_int32_t parent;
    };

    struct EdgeRange {
        EdgeRange() : begin(noEdge), end(noEdge) {}

        Edge begin;
        Edge end;
    };

    struct EdgeLabel {
        EdgeLabel(const StopEventId stopEvent = noStopEvent, const TripId trip = noTripId, const StopEventId firstEvent = noStopEvent) :
            stopEvent(stopEvent),
            trip(trip),
            firstEvent(firstEvent) {
        }
        StopEventId stopEvent;
        TripId trip;
        StopEventId firstEvent;
    };

    struct StopLabel {
        StopLabel(const int arrivalTime = INFTY, const u_int32_t parent = -1) :
            arrivalTime(arrivalTime),
            parent(parent) {
        }

        int arrivalTime;
        u_int32_t parent;
    };

public:
    UPQuery(const Data& oldData, const TransferGraph& oldTransferGraph, const CH::CH& oldCHData, const IndexedSet<false, Vertex>& oldTargets, const bool reorder) :
        queryData(oldData, oldTransferGraph, oldCHData, oldTargets, reorder),
        data(queryData.data),
        initialAndFinalTransfers(queryData.transferGraph, queryData.chData, std::move(queryData.phastOrder), data.numberOfStops(), queryData.targets),
        queue(data.numberOfStopEvents()),
        edgeRanges(data.numberOfStopEvents()),
        queueSize(0),
        reachedIndex(data),
        edgeLabels(data.stopEventGraph.numEdges()),
        sourceVertex(noVertex),
        sourceDepartureTime(never) {
        for (const Edge edge : data.stopEventGraph.edges()) {
            edgeLabels[edge].stopEvent = StopEventId(data.stopEventGraph.get(ToVertex, edge) + 1);
            edgeLabels[edge].trip = data.tripOfStopEvent[data.stopEventGraph.get(ToVertex, edge)];
            edgeLabels[edge].firstEvent = data.firstStopEventOfTrip[edgeLabels[edge].trip];
        }
        routeLabels.reserve(data.numberOfRoutes());
        for (const RouteId route : data.routes()) {
            routeLabels.emplace_back(data, route);
        }
        profiler.registerPhases({PHASE_SCAN_INITIAL, PHASE_EVALUATE_INITIAL, PHASE_SCAN_TRIPS, PHASE_FINAL_TRANSFERS});
        profiler.registerMetrics({METRIC_ROUNDS, METRIC_SCANNED_TRIPS, METRIC_SCANNED_STOPS, METRIC_RELAXED_TRANSFERS, METRIC_ENQUEUES, METRIC_ADD_JOURNEYS});
    }

public:
    inline void run(const Vertex source, const int departureTime) noexcept {
        profiler.start();
        clear();
        sourceVertex = Vertex(queryData.externalToInternal[source]);
        sourceDepartureTime = departureTime;
        computeInitialAndFinalTransfers();
        evaluateInitialTransfers();
        scanTrips();
        if constexpr (GroupSweeps) {
            profiler.startPhase();
            initialAndFinalTransfers.finalize();
            profiler.donePhase(PHASE_FINAL_TRANSFERS);
        }
        profiler.done();
    }

    inline std::vector<RAPTOR::Journey> getJourneys(const Vertex vertex) noexcept {
        const Vertex internalVertex(queryData.externalToInternal[vertex]);
        std::vector<RAPTOR::Journey> journeys;
        for (size_t i = 0; i <= stopLabels.size(); i++) {
            getJourney(journeys, i, internalVertex);
        }
        return journeys;
    }

    inline std::vector<RAPTOR::ArrivalLabel> getArrivals(const Vertex vertex) noexcept {
        const Vertex internalVertex(queryData.externalToInternal[vertex]);
        std::vector<RAPTOR::ArrivalLabel> labels;
        for (size_t i = 0; i <= stopLabels.size(); i++) {
            getArrival(labels, i, internalVertex);
        }
        return labels;
    }

    inline Profiler& getProfiler() noexcept {
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
    inline void clear() noexcept {
        queueSize = 0;
        reachedIndex.clear();
        stopLabels.clear();
    }

    inline void computeInitialAndFinalTransfers() noexcept {
        profiler.startPhase();
        initialAndFinalTransfers.initialize();
        initialAndFinalTransfers.startNewRound();
        initialAndFinalTransfers.template addSource<false>(sourceVertex, sourceDepartureTime, sourceVertex);
        initialAndFinalTransfers.initialUpwardSearch();
        if constexpr (GroupSweeps) {
            initialAndFinalTransfers.downwardSearchToStops();
        } else {
            initialAndFinalTransfers.downwardSearchToStopsAndTargets();
        }
        profiler.donePhase(PHASE_SCAN_INITIAL);
    }

    inline void evaluateInitialTransfers() noexcept {
        profiler.startPhase();
        for (const RouteId route : data.routes()) {
            const RouteLabel& label = routeLabels[route];
            const StopIndex endIndex = label.end();
            const TripId firstTrip = data.firstTripOfRoute[route];
            TripId tripIndex = noTripId;
            for (StopIndex stopIndex(0); stopIndex < endIndex; stopIndex++) {
                const StopId stop = data.getStop(firstTrip, stopIndex);
                const int stopDepartureTime = initialAndFinalTransfers.getDistance(0, stop);
                if (stopDepartureTime == INFTY) continue;
                if (!label.findEarliestTrip(stopIndex, stopDepartureTime, tripIndex)) continue;
                enqueue(firstTrip + tripIndex, StopIndex(stopIndex + 1));
                if (tripIndex == 0) break;
            }
        }
        profiler.donePhase(PHASE_EVALUATE_INITIAL);
    }

    inline void scanTrips() noexcept {
        profiler.startPhase();
        size_t roundBegin = 0;
        size_t roundEnd = queueSize;
        while (roundBegin < roundEnd) {
            profiler.countMetric(METRIC_ROUNDS);
            initialAndFinalTransfers.startNewRound();
            if (stopLabels.empty()) {
                stopLabels.emplace_back(data.numberOfStops());
            } else {
                stopLabels.emplace_back(stopLabels.back());
            }
            // Register stop arrival times for final transfers
            for (size_t i = roundBegin; i < roundEnd; i++) {
                const TripLabel& label = queue[i];
                profiler.countMetric(METRIC_SCANNED_TRIPS);
                for (StopEventId j = label.begin; j < label.end; j++) {
                    profiler.countMetric(METRIC_SCANNED_STOPS);
                    const int arrivalTime = data.arrivalEvents[j].arrivalTime;
                    const StopId stop = data.arrivalEvents[j].stop;
                    StopLabel& stopLabel = stopLabels.back()[stop];
                    if (stopLabel.arrivalTime <= arrivalTime) continue;
                    stopLabel.arrivalTime = arrivalTime;
                    stopLabel.parent = i;
                    initialAndFinalTransfers.template addSource<true>(stop, arrivalTime, stop);
                }
            }
            // Find the range of transfers for each trip
            for (size_t i = roundBegin; i < roundEnd; i++) {
                TripLabel& label = queue[i];
                StopEventId oldEnd = label.end;
                label.end = label.begin;
                for (StopEventId j = label.begin; j < oldEnd; j++) {
                    const int arrivalTime = data.arrivalEvents[j].arrivalTime;
                    const StopId stop = data.arrivalEvents[j].stop;
                    const StopLabel& stopLabel = stopLabels.back()[stop];
                    if (stopLabel.arrivalTime < arrivalTime) continue;
                    label.end = StopEventId(j+1);
                }
                edgeRanges[i].begin = data.stopEventGraph.beginEdgeFrom(Vertex(label.begin));
                edgeRanges[i].end = data.stopEventGraph.beginEdgeFrom(Vertex(label.end));
            }
            // Relax the transfers for each trip
            for (size_t i = roundBegin; i < roundEnd; i++) {
                const EdgeRange& label = edgeRanges[i];
                for (Edge edge = label.begin; edge < label.end; edge++) {
                    profiler.countMetric(METRIC_RELAXED_TRANSFERS);
                    enqueue(edge, i);
                }
            }
            roundBegin = roundEnd;
            roundEnd = queueSize;
            profiler.donePhase(PHASE_SCAN_TRIPS);
            profiler.startPhase();
            initialAndFinalTransfers.relaxFinalTransfers();
            profiler.donePhase(PHASE_FINAL_TRANSFERS);
            profiler.startPhase();
        }
        profiler.donePhase(PHASE_SCAN_TRIPS);
    }

    inline void enqueue(const TripId trip, const StopIndex index) noexcept {
        profiler.countMetric(METRIC_ENQUEUES);
        if (reachedIndex.alreadyReached(trip, index)) return;
        const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
        queue[queueSize] = TripLabel(StopEventId(firstEvent + index), StopEventId(firstEvent + reachedIndex(trip)));
        queueSize++;
        AssertMsg(queueSize <= queue.size(), "Queue is overfull!");
        reachedIndex.update(trip, index);
    }

    inline void enqueue(const Edge edge, const size_t parent) noexcept {
        profiler.countMetric(METRIC_ENQUEUES);
        const EdgeLabel& label = edgeLabels[edge];
        if (reachedIndex.alreadyReached(label.trip, label.stopEvent - label.firstEvent)) return;
        queue[queueSize] = TripLabel(label.stopEvent, StopEventId(label.firstEvent + reachedIndex(label.trip)), parent);
        queueSize++;
        AssertMsg(queueSize <= queue.size(), "Queue is overfull!");
        reachedIndex.update(label.trip, StopIndex(label.stopEvent - label.firstEvent));
    }

    inline void getJourney(std::vector<RAPTOR::Journey>& journeys, size_t round, Vertex vertex) noexcept {
        const int arrivalTime = initialAndFinalTransfers.getTargetDistance(round, vertex);
        if (arrivalTime >= (journeys.empty() ? never : journeys.back().back().arrivalTime)) return;
        RAPTOR::Journey result;
        if (round == 0) {
            result.emplace_back(Vertex(queryData.internalToExternal[sourceVertex]), Vertex(queryData.internalToExternal[vertex]), sourceDepartureTime, arrivalTime, false);
            journeys.emplace_back(result);
            return;
        }
        Vertex parentVertex = initialAndFinalTransfers.getTargetParent(round, vertex);
        const StopLabel& stopLabel = stopLabels[round - 1][parentVertex];
        u_int32_t parent = stopLabel.parent;
        StopEventId departureStopEvent = noStopEvent;
        Vertex departureStop = vertex;
        while (parent != u_int32_t(-1)) {
            Ensure(parent < queueSize, "Parent " << parent << " is out of range!");
            const TripLabel& label = queue[parent];
            StopEventId arrivalStopEvent;
            Edge edge;
            std::tie(arrivalStopEvent, edge) = (departureStopEvent == noStopEvent) ? getParent(label, parentVertex, stopLabel) : getParent(label, StopEventId(departureStopEvent + 1));

            const StopId arrivalStop = data.getStopOfStopEvent(arrivalStopEvent);
            const int arrivalTime = data.arrivalTime(arrivalStopEvent);
            const int transferArrivalTime = (edge == noEdge) ? arrivalTime : arrivalTime + data.stopEventGraph.get(TravelTime, edge);
            result.emplace_back(arrivalStop, departureStop, arrivalTime, transferArrivalTime, edge);

            departureStopEvent = StopEventId(label.begin - 1);
            departureStop = data.getStopOfStopEvent(departureStopEvent);
            const RouteId route = data.getRouteOfStopEvent(departureStopEvent);
            const int departureTime = data.departureTime(departureStopEvent);
            result.emplace_back(departureStop, arrivalStop, departureTime, arrivalTime, true, route);

            parent = label.parent;
        }
        const int initialTransferTime = initialAndFinalTransfers.getDistance(0, departureStop);
        result.emplace_back(sourceVertex, departureStop, sourceDepartureTime, initialTransferTime, noEdge);
        Vector::reverse(result);
        for (RAPTOR::JourneyLeg& leg : result) {
            leg.from = Vertex(queryData.internalToExternal[leg.from]);
            leg.to = Vertex(queryData.internalToExternal[leg.to]);
        }
        journeys.emplace_back(result);
    }

    inline std::pair<StopEventId, Edge> getParent(const TripLabel& parentLabel, const StopEventId departureStopEvent) const noexcept {
        for (StopEventId i = parentLabel.begin; i < parentLabel.end; i++) {
            for (const Edge edge : data.stopEventGraph.edgesFrom(Vertex(i))) {
                if (edgeLabels[edge].stopEvent == departureStopEvent) return std::make_pair(i, edge);
            }
        }
        Ensure(false, "Could not find parent stop event!");
        return std::make_pair(noStopEvent, noEdge);
    }

    inline std::pair<StopEventId, Edge> getParent(const TripLabel& parentLabel, const Vertex stop, const StopLabel& stopLabel) noexcept {
        //Final transfer to target may start exactly at parentLabel.end if it has length 0
        const TripId trip = data.tripOfStopEvent[parentLabel.begin];
        const StopEventId end = data.firstStopEventOfTrip[trip + 1];
        for (StopEventId i = parentLabel.begin; i < end; i++) {
            if (data.arrivalEvents[i].stop != stop) continue;
            if (stopLabel.arrivalTime != data.arrivalEvents[i].arrivalTime) continue;
            return std::make_pair(i, noEdge);
        }
        Ensure(false, "Could not find parent stop event!");
        return std::make_pair(noStopEvent, noEdge);
    }

    inline void getArrival(std::vector<RAPTOR::ArrivalLabel>& labels, size_t round, const Vertex vertex) noexcept {
        const int arrivalTime = initialAndFinalTransfers.getTargetDistance(round, vertex);
        if (arrivalTime >= (labels.empty() ? never : labels.back().arrivalTime)) return;
        labels.emplace_back(arrivalTime, round);
    }

private:
    const QueryData queryData;
    const Data& data;

    InitialAndFinalTransfers initialAndFinalTransfers;

    std::vector<TripLabel> queue;
    std::vector<EdgeRange> edgeRanges;
    size_t queueSize;
    ReachedIndex reachedIndex;

    std::vector<std::vector<StopLabel>> stopLabels;

    std::vector<EdgeLabel> edgeLabels;
    std::vector<RouteLabel> routeLabels;

    Vertex sourceVertex;
    int sourceDepartureTime;

    Profiler profiler;
};

}
