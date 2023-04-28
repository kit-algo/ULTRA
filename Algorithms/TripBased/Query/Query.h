#pragma once

#include "../../CH/Query/BucketQuery.h"

#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../../DataStructures/TripBased/RouteLabel.h"

#include "Profiler.h"
#include "ReachedIndex.h"

namespace TripBased {

template<typename PROFILER = NoProfiler>
class Query {

public:
    using Profiler = PROFILER;
    using Type = Query<Profiler>;

private:
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

    struct TargetLabel {
        TargetLabel(const int arrivalTime = INFTY, const u_int32_t parent = -1) :
            arrivalTime(arrivalTime),
            parent(parent) {
        }

        int arrivalTime;
        u_int32_t parent;
    };

public:
    Query(const Data& data, const RAPTOR::BucketCHInitialTransfers& initialTransfers) :
        data(data),
        bucketQuery(initialTransfers),
        queue(data.numberOfStopEvents()),
        edgeRanges(data.numberOfStopEvents()),
        queueSize(0),
        reachedIndex(data),
        targetLabels(1),
        minArrivalTime(INFTY),
        edgeLabels(data.stopEventGraph.numEdges()),
        sourceVertex(noVertex),
        targetVertex(noVertex),
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
        profiler.registerPhases({PHASE_SCAN_INITIAL, PHASE_EVALUATE_INITIAL, PHASE_SCAN_TRIPS});
        profiler.registerMetrics({METRIC_ROUNDS, METRIC_SCANNED_TRIPS, METRIC_SCANNED_STOPS, METRIC_RELAXED_TRANSFERS, METRIC_ENQUEUES, METRIC_ADD_JOURNEYS});
    }

    Query(const Data& data, const CH::CH& chData) :
        Query(data, RAPTOR::BucketCHInitialTransfers(chData.forward, chData.backward, data.numberOfStops(), Weight)) {
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target) noexcept {
        profiler.start();
        clear();
        sourceVertex = source;
        targetVertex = target;
        sourceDepartureTime = departureTime;
        computeInitialAndFinalTransfers();
        evaluateInitialTransfers();
        scanTrips();
        profiler.done();
    }

    inline int getEarliestArrivalTime() const noexcept {
        return targetLabels.back().arrivalTime;
    }

    inline int getEarliestArrivalNumberOfTrips() const noexcept {
        const int eat = targetLabels.back().arrivalTime;
        for (size_t i = 0; i < targetLabels.size(); i++) {
            if (targetLabels[i].arrivalTime == eat) return i;
        }
        return -1;
    }

    inline std::vector<RAPTOR::Journey> getJourneys() const noexcept {
        std::vector<RAPTOR::Journey> result;
        int bestArrivalTime = INFTY;
        for (const TargetLabel& label : targetLabels) {
            if (label.arrivalTime >= bestArrivalTime) continue;
            bestArrivalTime = label.arrivalTime;
            result.emplace_back(getJourney(label));
        }
        return result;
    }

    inline std::vector<RAPTOR::ArrivalLabel> getArrivals() const noexcept {
        std::vector<RAPTOR::ArrivalLabel> result;
        for (size_t i = 0; i < targetLabels.size(); i++) {
            if (targetLabels[i].arrivalTime >= INFTY) continue;
            if ((result.size() >= 1) && (result.back().arrivalTime == targetLabels[i].arrivalTime)) continue;
            result.emplace_back(targetLabels[i].arrivalTime, i);
        }
        return result;
    }

    inline Profiler& getProfiler() noexcept {
        return profiler;
    }

private:
    inline void clear() noexcept {
        queueSize = 0;
        reachedIndex.clear();
        targetLabels.resize(1);
        targetLabels[0] = TargetLabel();
        minArrivalTime = INFTY;
    }

    inline void computeInitialAndFinalTransfers() noexcept {
        profiler.startPhase();
        bucketQuery.run(sourceVertex, targetVertex);
        if (bucketQuery.getDistance() != INFTY) {
            addTargetLabel(sourceDepartureTime + bucketQuery.getDistance());
        }
        profiler.donePhase(PHASE_SCAN_INITIAL);
    }

    inline void evaluateInitialTransfers() noexcept {
        profiler.startPhase();
        std::vector<bool> reachedRoutes(data.numberOfRoutes(), false);
        for (const Vertex stop : bucketQuery.getForwardPOIs()) {
            for (const RAPTOR::RouteSegment& route : data.routesContainingStop(StopId(stop))) {
                reachedRoutes[route.routeId] = true;
            }
        }
        for (const RouteId route : data.routes()) {
            if (!reachedRoutes[route]) continue;
            const RouteLabel& label = routeLabels[route];
            const StopIndex endIndex = label.end();
            const TripId firstTrip = data.firstTripOfRoute[route];
            TripId tripIndex = noTripId;
            for (StopIndex stopIndex(0); stopIndex < endIndex; stopIndex++) {
                const StopId stop = data.getStop(firstTrip, stopIndex);
                const int timeFromSource = bucketQuery.getForwardDistance(stop);
                if (timeFromSource == INFTY) continue;
                const int stopDepartureTime = sourceDepartureTime + timeFromSource;
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
            targetLabels.emplace_back(targetLabels.back());
            // Evaluate final transfers in order to check if the target is reachable
            for (size_t i = roundBegin; i < roundEnd; i++) {
                const TripLabel& label = queue[i];
                profiler.countMetric(METRIC_SCANNED_TRIPS);
                for (StopEventId j = label.begin; j < label.end; j++) {
                    profiler.countMetric(METRIC_SCANNED_STOPS);
                    if (data.arrivalEvents[j].arrivalTime >= minArrivalTime) break;
                    const int timeToTarget = bucketQuery.getBackwardDistance(data.arrivalEvents[j].stop);
                    if (timeToTarget != INFTY) addTargetLabel(data.arrivalEvents[j].arrivalTime + timeToTarget, i);
                }
            }
            // Find the range of transfers for each trip
            for (size_t i = roundBegin; i < roundEnd; i++) {
                TripLabel& label = queue[i];
                for (StopEventId j = label.begin; j < label.end; j++) {
                    if (data.arrivalEvents[j].arrivalTime >= minArrivalTime) label.end = j;
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

    inline void addTargetLabel(const int newArrivalTime, const u_int32_t parent = -1) noexcept {
        profiler.countMetric(METRIC_ADD_JOURNEYS);
        if (newArrivalTime < targetLabels.back().arrivalTime) {
            targetLabels.back() = TargetLabel(newArrivalTime, parent);
            minArrivalTime = newArrivalTime;
        }
    }

    inline RAPTOR::Journey getJourney(const TargetLabel& targetLabel) const noexcept {
        RAPTOR::Journey result;
        u_int32_t parent = targetLabel.parent;
        if (parent == u_int32_t(-1)) {
            result.emplace_back(sourceVertex, targetVertex, sourceDepartureTime, targetLabel.arrivalTime, false);
            return result;
        }
        StopEventId departureStopEvent = noStopEvent;
        Vertex departureStop = targetVertex;
        while (parent != u_int32_t(-1)) {
            AssertMsg(parent < queueSize, "Parent " << parent << " is out of range!");
            const TripLabel& label = queue[parent];
            StopEventId arrivalStopEvent;
            Edge edge;
            std::tie(arrivalStopEvent, edge) = (departureStopEvent == noStopEvent) ? getParent(label, targetLabel) : getParent(label, StopEventId(departureStopEvent + 1));

            const StopId arrivalStop = data.getStopOfStopEvent(arrivalStopEvent);
            const int arrivalTime = data.arrivalTime(arrivalStopEvent);
            const int transferArrivalTime = (edge == noEdge) ? targetLabel.arrivalTime : arrivalTime + data.stopEventGraph.get(TravelTime, edge);
            result.emplace_back(arrivalStop, departureStop, arrivalTime, transferArrivalTime, edge);

            departureStopEvent = StopEventId(label.begin - 1);
            departureStop = data.getStopOfStopEvent(departureStopEvent);
            const RouteId route = data.getRouteOfStopEvent(departureStopEvent);
            const int departureTime = data.departureTime(departureStopEvent);
            result.emplace_back(departureStop, arrivalStop, departureTime, arrivalTime, true, route);

            parent = label.parent;
        }
        const int timeFromSource = bucketQuery.getForwardDistance(departureStop);
        result.emplace_back(sourceVertex, departureStop, sourceDepartureTime, sourceDepartureTime + timeFromSource, noEdge);
        Vector::reverse(result);
        return result;
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

    inline std::pair<StopEventId, Edge> getParent(const TripLabel& parentLabel, const TargetLabel& targetLabel) const noexcept {
        //Final transfer to target may start exactly at parentLabel.end if it has length 0
        const TripId trip = data.tripOfStopEvent[parentLabel.begin];
        const StopEventId end = data.firstStopEventOfTrip[trip + 1];
        for (StopEventId i = parentLabel.begin; i < end; i++) {
            const int timeToTarget = bucketQuery.getBackwardDistance(data.arrivalEvents[i].stop);
            if (timeToTarget == INFTY) continue;
            if (data.arrivalEvents[i].arrivalTime + timeToTarget == targetLabel.arrivalTime) return std::make_pair(i, noEdge);
        }
        Ensure(false, "Could not find parent stop event!");
        return std::make_pair(noStopEvent, noEdge);
    }

private:
    const Data& data;

    RAPTOR::BucketCHInitialTransfers bucketQuery;
    std::vector<TripLabel> queue;
    std::vector<EdgeRange> edgeRanges;
    size_t queueSize;
    ReachedIndex reachedIndex;

    std::vector<TargetLabel> targetLabels;
    int minArrivalTime;

    std::vector<EdgeLabel> edgeLabels;
    std::vector<RouteLabel> routeLabels;

    Vertex sourceVertex;
    Vertex targetVertex;
    int sourceDepartureTime;

    Profiler profiler;
};

}
