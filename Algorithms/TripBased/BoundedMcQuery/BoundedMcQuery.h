#pragma once

#include "../../CH/Query/BucketQuery.h"

#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/RAPTOR/Entities/Bags.h"
#include "../../../DataStructures/TripBased/Data.h"

#include "../Query/Profiler.h"
#include "ForwardPruningQuery.h"
#include "BackwardPruningQuery.h"
#include "TimestampedWalkingDistanceData.h"

namespace TripBased {

template<typename PROFILER = NoProfiler>
class BoundedMcQuery {

public:
    using Profiler = PROFILER;
    using Type = BoundedMcQuery<Profiler>;

private:
    struct TripLabel {
        TripLabel(const StopEventId begin = noStopEvent, const StopEventId end = noStopEvent, const int walkingDistance = INFTY, const u_int32_t parent = -1) :
            begin(begin),
            end(end),
            walkingDistance(walkingDistance),
            parent(parent),
            edgeBegin(noEdge),
            edgeEnd(noEdge) {
        }
        StopEventId begin;
        StopEventId end;
        int walkingDistance;
        u_int32_t parent;
        Edge edgeBegin;
        Edge edgeEnd;
    };

    struct TripInfo {
        StopEventId tripStart;
        StopEventId tripEnd;
        StopEventId routeEnd;
        StopIndex tripLength;
        TripId reverseTrip;
    };

    struct EdgeLabel {
        int walkingDistance;
        StopEventId stopEvent;
        TripId trip;
        TripId reverseTrip;
        StopIndex reverseStopIndex;
        StopEventId tripEnd;
        StopEventId routeEnd;
        StopIndex tripLength;
    };

    struct TargetLabel {
        TargetLabel(const int arrivalTime = never, int walkingDistance = INFTY, const u_int32_t parent = -1) :
            arrivalTime(arrivalTime),
            walkingDistance(walkingDistance),
            parent(parent) {
        }

        inline bool dominates(const TargetLabel& other) const noexcept {
            return arrivalTime <= other.arrivalTime && walkingDistance <= other.walkingDistance;
        }

        int arrivalTime;
        int walkingDistance;
        u_int32_t parent;
    };

    using TargetBag = RAPTOR::Bag<TargetLabel>;

public:
    BoundedMcQuery(const Data& data, const Data& forwardBoundedData, const Data& backwardBoundedData, const CH::CH& chData) :
        data(data),
        bucketQuery(chData.forward, chData.backward, data.numberOfStops(), Weight),
        forwardPruningQuery(forwardBoundedData, bucketQuery, profiler),
        backwardPruningQuery(backwardBoundedData, forwardPruningQuery, bucketQuery, profiler),
        walkingDistanceData(data),
        targetBags(1),
        tripInfo(data.numberOfTrips()),
        edgeLabels(data.stopEventGraph.numEdges()),
        offsets(data.numberOfStopEvents()),
        sourceVertex(noVertex),
        targetVertex(noVertex),
        sourceDepartureTime(never),
        maxTrips(-1) {
        queue.reserve(data.numberOfStopEvents());
        for (const TripId trip : data.trips()) {
            tripInfo[trip].tripStart = data.firstStopEventOfTrip[trip];
            tripInfo[trip].tripEnd = data.firstStopEventOfTrip[trip + 1];
            tripInfo[trip].routeEnd = data.firstStopEventOfTrip[data.firstTripOfRoute[data.routeOfTrip[trip] + 1]];
            tripInfo[trip].tripLength = StopIndex(data.numberOfStopsInTrip(trip));
            const RouteId route = data.routeOfTrip[trip];
            const TripId tripOffset = trip - data.firstTripOfRoute[route];
            tripInfo[trip].reverseTrip = TripId(data.firstTripOfRoute[route + 1] - tripOffset - 1);
        }
        for (const Edge edge : data.stopEventGraph.edges()) {
            edgeLabels[edge].walkingDistance = data.stopEventGraph.get(TravelTime, edge);
            edgeLabels[edge].stopEvent = StopEventId(data.stopEventGraph.get(ToVertex, edge) + 1);
            edgeLabels[edge].trip = data.tripOfStopEvent[data.stopEventGraph.get(ToVertex, edge)];
            edgeLabels[edge].reverseTrip = tripInfo[edgeLabels[edge].trip].reverseTrip;
            const StopIndex index = data.indexOfStopEvent[edgeLabels[edge].stopEvent - 1];
            edgeLabels[edge].reverseStopIndex = StopIndex(data.numberOfStopsInTrip(edgeLabels[edge].trip) - index - 1);
            edgeLabels[edge].tripEnd = tripInfo[edgeLabels[edge].trip].tripEnd;
            edgeLabels[edge].routeEnd = tripInfo[edgeLabels[edge].trip].routeEnd;
            edgeLabels[edge].tripLength = tripInfo[edgeLabels[edge].trip].tripLength;
        }
        for (StopEventId stopEvent(0); stopEvent < data.numberOfStopEvents(); stopEvent++) {
            const TripId trip = data.tripOfStopEvent[stopEvent];
            const bool hasPreviousTrip = trip > data.firstTripOfRoute[data.routeOfTrip[trip]];
            offsets[stopEvent] = hasPreviousTrip ? data.numberOfStopsInTrip(trip) : 0;
        }
        profiler.registerPhases({PHASE_FORWARD, PHASE_BACKWARD, PHASE_MAIN});
        profiler.registerMetrics({METRIC_ROUNDS, METRIC_SCANNED_TRIPS, METRIC_SCANNED_STOPS, METRIC_ENQUEUES, METRIC_ADD_JOURNEYS});
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target, const double arrivalSlack, const double tripSlack) noexcept {
        profiler.start();
        profiler.startPhase();
        clear();
        sourceVertex = source;
        targetVertex = target;
        sourceDepartureTime = departureTime;
        profiler.donePhase(PHASE_MAIN);

        profiler.startPhase();
        forwardPruningQuery.run(source, departureTime, target, arrivalSlack, tripSlack);
        profiler.donePhase(PHASE_FORWARD);
        if (forwardPruningQuery.getAnchorLabels().empty()) return;
        maxTrips = forwardPruningQuery.getMaxTrips();
        profiler.startPhase();
        backwardPruningQuery.run(target, departureTime, source, arrivalSlack, tripSlack);
        profiler.donePhase(PHASE_BACKWARD);

        profiler.startPhase();
        computeInitialAndFinalTransfers();
        evaluateInitialTransfers();
        scanTrips();
        profiler.donePhase(PHASE_MAIN);
        profiler.done();
    }

    inline void verify(const double arrivalSlack, const double tripSlack, const int departureTime) const noexcept {
        const std::vector<RAPTOR::ArrivalLabel>& anchorLabels = forwardPruningQuery.getAnchorLabels();
        for (const RAPTOR::ArrivalLabel& anchorLabel : anchorLabels) {
            Ensure(isContained(anchorLabel), "Anchor label with arrival time " << anchorLabel.arrivalTime << " and " << anchorLabel.numberOfTrips << " was not found!");
        }
        for (const RAPTOR::WalkingParetoLabel& label : getResults()) {
            if (!label.isWithinSlack(anchorLabels, departureTime, arrivalSlack, tripSlack)) {
                std::cout << "No anchor label found for " << label << std::endl;
                std::cout << "Anchor labels:" << std::endl;
                for (const RAPTOR::ArrivalLabel& anchorLabel : anchorLabels) {
                    std::cout << "\t" << anchorLabel << std::endl;
                }
                Ensure(false, "");
            }
        }
    }

    inline const std::vector<RAPTOR::ArrivalLabel>& getAnchorLabels() const noexcept {
        return forwardPruningQuery.getAnchorLabels();
    }

    inline std::vector<RAPTOR::Journey> getJourneys() const noexcept {
        std::vector<RAPTOR::Journey> result;
        for (const TargetBag& bag : targetBags) {
            for (const TargetLabel& label : bag) {
                result.emplace_back(getJourney(label));
            }
        }
        return result;
    }

    inline std::vector<RAPTOR::WalkingParetoLabel> getResults() const noexcept {
        std::vector<RAPTOR::WalkingParetoLabel> result;
        for (size_t i = 0; i < targetBags.size(); i++) {
            for (const TargetLabel& label : targetBags[i]) {
                result.emplace_back(label, i);
            }
        }
        return result;
    }

    inline Profiler& getProfiler() noexcept {
        return profiler;
    }

private:
    inline void clear() noexcept {
        queue.clear();
        walkingDistanceData.clear();
        targetBags.resize(1);
        targetBags[0].clear();
        bestTargetBag.clear();
    }

    inline void computeInitialAndFinalTransfers() noexcept {
        const int walkingDistance = bucketQuery.getDistance();
        if (walkingDistance != INFTY) {
            TargetLabel label(sourceDepartureTime + walkingDistance, walkingDistance);
            addTargetLabel(label);
        }
    }

    inline void evaluateInitialTransfers() noexcept {
        for (const Vertex stop : bucketQuery.getForwardPOIs()) {
            const int timeFromSource = bucketQuery.getForwardDistance(stop);
            const int stopDepartureTime = sourceDepartureTime + timeFromSource;
            const int arrivalTime = -backwardPruningQuery.getArrivalTime(StopId(stop), maxTrips);
            if (stopDepartureTime > arrivalTime) continue;
            for (const RAPTOR::RouteSegment& segment : data.routesContainingStop(StopId(stop))) {
                const TripId trip = data.getEarliestTrip(segment, stopDepartureTime);
                if (trip != noTripId) {
                    enqueue(trip, StopIndex(segment.stopIndex + 1), timeFromSource);
                }
            }
        }
    }

    inline void scanTrips() noexcept {
        size_t roundBegin = 0;
        size_t roundEnd = queue.size();
        while (targetBags.size() <= maxTrips && roundBegin < roundEnd) {
            profiler.countMetric(METRIC_ROUNDS);
            targetBags.emplace_back();
            // Find the range of stop events for each trip
            for (size_t i = roundBegin; i < roundEnd; i++) {
                TripLabel& label = queue[i];
                profiler.countMetric(METRIC_SCANNED_TRIPS);
                for (StopEventId j(label.begin + 1); j < label.end; j++) {
                    const int walkingDistance = walkingDistanceData(j);
                    if (walkingDistance < label.walkingDistance) label.end = j;
                    else if (walkingDistance == label.walkingDistance && offsets[j] != 0) {
                        const u_int8_t offset = offsets[j];
                        for (; j < label.end; j++) {
                            if (walkingDistanceData(StopEventId(j - offset)) == label.walkingDistance) label.end = j;
                        }
                        break;
                    }
                }
            }
            // Evaluate final transfers in order to check if the target is reachable
            for (size_t i = roundBegin; i < roundEnd; i++) {
                const TripLabel& label = queue[i];
                for (StopEventId j = label.begin; j < label.end; j++) {
                    profiler.countMetric(METRIC_SCANNED_STOPS);
                    const int timeToTarget = bucketQuery.getBackwardDistance(data.arrivalEvents[j].stop);
                    if (timeToTarget == INFTY) continue;
                    const int arrivalTime = data.arrivalEvents[j].arrivalTime + timeToTarget;
                    if (arrivalTime > -backwardPruningQuery.getDepartureTime(maxTrips - currentNumberOfTrips())) continue;
                    const TargetLabel targetLabel(arrivalTime, label.walkingDistance + timeToTarget, i);
                    addTargetLabel(targetLabel);
                }
            }
            // Find the range of transfers for each trip
            for (size_t i = roundBegin; i < roundEnd; i++) {
                TripLabel& label = queue[i];
                label.edgeBegin = data.stopEventGraph.beginEdgeFrom(Vertex(label.begin));
                label.edgeEnd = data.stopEventGraph.beginEdgeFrom(Vertex(label.end));
            }
            // Relax the transfers for each trip
            for (size_t i = roundBegin; i < roundEnd; i++) {
                const TripLabel& label = queue[i];
                const TargetLabel pruningLabel(data.arrivalEvents[label.begin].arrivalTime, label.walkingDistance);
                if (bestTargetBag.dominates(pruningLabel)) continue;
                for (Edge edge = label.edgeBegin; edge < label.edgeEnd; edge++) {
                    enqueue(edge, label.walkingDistance, i);
                }
            }
            roundBegin = roundEnd;
            roundEnd = queue.size();
        }
    }

    inline size_t currentNumberOfTrips() const noexcept {
        return targetBags.size() - 1;
    }

    inline void enqueue(const TripId trip, const StopIndex index, const int walkingDistance) noexcept {
        profiler.countMetric(METRIC_ENQUEUES);
        const TripInfo& info = tripInfo[trip];
        const StopEventId stopEvent = StopEventId(info.tripStart + index);
        if (walkingDistance >= walkingDistanceData(stopEvent)) return;
        const StopIndex reverseStopIndex(info.tripLength - index);
        if (backwardPruningQuery.getReachedIndex(info.reverseTrip, maxTrips - currentNumberOfTrips()) > reverseStopIndex) return;
        const StopEventId end = walkingDistanceData.getScanEnd(StopEventId(stopEvent + 1), info.tripEnd, walkingDistance);
        queue.emplace_back(stopEvent, end, walkingDistance);
        walkingDistanceData.update(stopEvent, info.tripEnd, info.routeEnd, info.tripLength, walkingDistance);
    }

    inline void enqueue(const Edge edge, int walkingDistance, const u_int32_t parent) noexcept {
        profiler.countMetric(METRIC_ENQUEUES);
        const EdgeLabel& label = edgeLabels[edge];
        walkingDistance += label.walkingDistance;
        if (walkingDistance >= walkingDistanceData(label.stopEvent)) return;
        if (backwardPruningQuery.getReachedIndex(label.reverseTrip, maxTrips - currentNumberOfTrips()) > label.reverseStopIndex) return;
        const StopEventId end = walkingDistanceData.getScanEnd(StopEventId(label.stopEvent + 1), label.tripEnd, walkingDistance);
        queue.emplace_back(label.stopEvent, end, walkingDistance, parent);
        walkingDistanceData.update(label.stopEvent, label.tripEnd, label.routeEnd, label.tripLength, walkingDistance);
    }

    inline void addTargetLabel(const TargetLabel& newLabel) noexcept {
        profiler.countMetric(METRIC_ADD_JOURNEYS);
        if (!bestTargetBag.merge(newLabel)) return;
        targetBags.back().mergeUndominated(newLabel);
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
            AssertMsg(parent < queue.size(), "Parent " << parent << " is out of range!");
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
        for (StopEventId i = parentLabel.begin; i < parentLabel.end; i++) {
            const int timeToTarget = bucketQuery.getBackwardDistance(data.arrivalEvents[i].stop);
            if (timeToTarget == INFTY) continue;
            if (data.arrivalEvents[i].arrivalTime + timeToTarget == targetLabel.arrivalTime) return std::make_pair(i, noEdge);
        }
        Ensure(false, "Could not find parent stop event!");
        return std::make_pair(noStopEvent, noEdge);
    }

    inline bool isContained(const RAPTOR::ArrivalLabel& anchorLabel) const noexcept {
        Ensure(anchorLabel.numberOfTrips < targetBags.size(), "Label with " << anchorLabel.numberOfTrips << " is out of bounds!");
        for (const TargetLabel& label : targetBags[anchorLabel.numberOfTrips]) {
            if (label.arrivalTime == anchorLabel.arrivalTime) return true;
        }
        return false;
    }

private:
    const Data& data;
    CH::BucketQuery<CHGraph, true, false> bucketQuery;
    ForwardPruningQuery<Profiler> forwardPruningQuery;
    BackwardPruningQuery<Profiler> backwardPruningQuery;

    std::vector<TripLabel> queue;
    TimestampedWalkingDistanceData walkingDistanceData;

    std::vector<TargetBag> targetBags;
    TargetBag bestTargetBag;

    std::vector<TripInfo> tripInfo;
    std::vector<EdgeLabel> edgeLabels;
    std::vector<u_int8_t> offsets;

    Vertex sourceVertex;
    Vertex targetVertex;
    int sourceDepartureTime;

    size_t maxTrips;

    Profiler profiler;
};

}
