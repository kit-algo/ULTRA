#pragma once

#include "../../CH/Query/BucketQuery.h"

#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../Query/ReachedIndex.h"
#include "../Query/Profiler.h"

#include "StopArrivalTimes.h"

namespace TripBased {

template<typename PROFILER = NoProfiler, typename INITIAL_TRANSFERS = RAPTOR::BucketCHInitialTransfers>
class ForwardPruningQuery {

public:
    using Profiler = PROFILER;
    using InitialTransferType = INITIAL_TRANSFERS;
    using Type = ForwardPruningQuery<Profiler, InitialTransferType>;

private:
    struct TripLabel {
        TripLabel(const StopEventId begin = noStopEvent, const StopEventId end = noStopEvent) :
            begin(begin),
            end(end) {
        }
        StopEventId begin;
        StopEventId end;
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

    struct RouteLabel {
        RouteLabel() :
            numberOfTrips(0) {
        }
        inline StopIndex end() const noexcept {
            return StopIndex(departureTimes.size() / numberOfTrips);
        }
        u_int32_t numberOfTrips;
        std::vector<int> departureTimes;
    };

public:
    ForwardPruningQuery(const Data& data, InitialTransferType& bucketQuery, Profiler& profiler) :
        data(data),
        bucketQuery(bucketQuery),
        queue(data.numberOfStopEvents()),
        edgeRanges(data.numberOfStopEvents()),
        queueSize(0),
        reachedIndex(data),
        stopArrivalTimes(data),
        targetLabels(1),
        minArrivalTime(INFTY),
        edgeLabels(data.stopEventGraph.numEdges()),
        routeLabels(data.numberOfRoutes()),
        sourceVertex(noVertex),
        targetVertex(noVertex),
        sourceDepartureTime(never),
        arrivalSlack(INFTY),
        maxTrips(0),
        profiler(profiler) {
        for (const Edge edge : data.stopEventGraph.edges()) {
            edgeLabels[edge].stopEvent = StopEventId(data.stopEventGraph.get(ToVertex, edge) + 1);
            edgeLabels[edge].trip = data.tripOfStopEvent[data.stopEventGraph.get(ToVertex, edge)];
            edgeLabels[edge].firstEvent = data.firstStopEventOfTrip[edgeLabels[edge].trip];
        }
        for (const RouteId route : data.raptorData.routes()) {
            const size_t numberOfStops = data.numberOfStopsInRoute(route);
            const size_t numberOfTrips = data.raptorData.numberOfTripsInRoute(route);
            const RAPTOR::StopEvent* stopEvents = data.raptorData.firstTripOfRoute(route);
            routeLabels[route].numberOfTrips = numberOfTrips;
            routeLabels[route].departureTimes.resize((numberOfStops - 1) * numberOfTrips);
            for (size_t trip = 0; trip < numberOfTrips; trip++) {
                for (size_t stopIndex = 0; stopIndex + 1 < numberOfStops; stopIndex++) {
                    routeLabels[route].departureTimes[(stopIndex * numberOfTrips) + trip] = stopEvents[(trip * numberOfStops) + stopIndex].departureTime;
                }
            }
        }
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target, const double arrSlack, const double tripSlack) noexcept {
        clear();
        sourceVertex = source;
        targetVertex = target;
        sourceDepartureTime = departureTime;
        arrivalSlack = arrSlack;
        computeInitialAndFinalTransfers();
        evaluateInitialTransfers();
        scanTrips();
        computeAnchorLabels(tripSlack);
    }

    inline int getTargetArrivalTime(const size_t numTrips) const noexcept {
        return targetLabels[std::min(numTrips, targetLabels.size() - 1)];
    }

    inline const std::vector<RAPTOR::ArrivalLabel>& getAnchorLabels() const noexcept {
        return anchorLabels;
    }

    inline size_t getMaxTrips() const noexcept {
        return maxTrips;
    }

    inline int getArrivalTime(const StopId stop, const size_t round) const noexcept {
        return stopArrivalTimes(stop, round);
    }

private:
    inline void clear() noexcept {
        queueSize = 0;
        reachedIndex.clear();
        stopArrivalTimes.clear();
        std::vector<int>(1, INFTY).swap(targetLabels);
        minArrivalTime = INFTY;
        anchorLabels.clear();
        maxTrips = 0;
    }

    inline void computeInitialAndFinalTransfers() noexcept {
        bucketQuery.run(sourceVertex, targetVertex, arrivalSlack);
        if (bucketQuery.getDistance() != INFTY) {
            addTargetLabel(sourceDepartureTime + bucketQuery.getDistance());
        }
    }

    inline void evaluateInitialTransfers() noexcept {
        std::vector<bool> reachedRoutes(data.raptorData.numberOfRoutes(), false);
        for (const Vertex stop : bucketQuery.getForwardPOIs()) {
            for (const RAPTOR::RouteSegment& route : data.raptorData.routesContainingStop(StopId(stop))) {
                reachedRoutes[route.routeId] = true;
            }
        }
        for (const RouteId route : data.raptorData.routes()) {
            if (!reachedRoutes[route]) continue;
            const RouteLabel& label = routeLabels[route];
            const StopIndex endIndex = label.end();
            const TripId firstTrip = data.firstTripOfRoute[route];
            const StopId* stops = data.raptorData.stopArrayOfRoute(route);
            TripId tripIndex = noTripId;
            for (StopIndex stopIndex(0); stopIndex < endIndex; stopIndex++) {
                const int timeFromSource = bucketQuery.getForwardDistance(stops[stopIndex]);
                if (timeFromSource == INFTY) continue;
                const int stopDepartureTime = sourceDepartureTime + timeFromSource;
                const u_int32_t labelIndex = stopIndex * label.numberOfTrips;
                if (tripIndex >= label.numberOfTrips) {
                    tripIndex = std::lower_bound(TripId(0), TripId(label.numberOfTrips), stopDepartureTime, [&](const TripId trip, const int time) {
                        return label.departureTimes[labelIndex + trip] < time;
                    });
                    if (tripIndex >= label.numberOfTrips) continue;
                } else {
                    if (label.departureTimes[labelIndex + tripIndex - 1] < stopDepartureTime) continue;
                    tripIndex--;
                    while ((tripIndex > 0) && (label.departureTimes[labelIndex + tripIndex - 1] >= stopDepartureTime)) {
                        tripIndex--;
                    }
                }
                enqueue(firstTrip + tripIndex, StopIndex(stopIndex + 1));
                if (tripIndex == 0) break;
            }
        }
    }

    inline void scanTrips() noexcept {
        size_t roundBegin = 0;
        size_t roundEnd = queueSize;
        while (roundBegin < roundEnd) {
            stopArrivalTimes.startNewRound();
            targetLabels.emplace_back(targetLabels.back());
            // Evaluate final transfers in order to check if the target is reachable
            for (size_t i = roundBegin; i < roundEnd; i++) {
                const TripLabel& label = queue[i];
                profiler.countMetric(METRIC_SCANNED_TRIPS);
                for (StopEventId j = label.begin; j < label.end; j++) {
                    profiler.countMetric(METRIC_SCANNED_STOPS);
                    if (data.arrivalEvents[j].arrivalTime >= minArrivalTime) break;
                    const int timeToTarget = bucketQuery.getBackwardDistance(data.arrivalEvents[j].stop);
                    if (timeToTarget != INFTY) addTargetLabel(data.arrivalEvents[j].arrivalTime + timeToTarget);
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
            // Update stops
            for (size_t i = roundBegin; i < roundEnd; i++) {
                const TripLabel& label = queue[i];
                for (StopEventId j = label.begin; j < label.end; j++) {
                    stopArrivalTimes.update(j);
                }
            }
            // Relax the transfers for each trip
            for (size_t i = roundBegin; i < roundEnd; i++) {
                const EdgeRange& label = edgeRanges[i];
                for (Edge edge = label.begin; edge < label.end; edge++) {
                    enqueue(edge);
                }
            }
            roundBegin = roundEnd;
            roundEnd = queueSize;
        }
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

    inline void enqueue(const Edge edge) noexcept {
        profiler.countMetric(METRIC_ENQUEUES);
        const EdgeLabel& label = edgeLabels[edge];
        if (reachedIndex.alreadyReached(label.trip, label.stopEvent - label.firstEvent)) return;
        queue[queueSize] = TripLabel(label.stopEvent, StopEventId(label.firstEvent + reachedIndex(label.trip)));
        queueSize++;
        AssertMsg(queueSize <= queue.size(), "Queue is overfull!");
        reachedIndex.update(label.trip, StopIndex(label.stopEvent - label.firstEvent));
    }

    inline void addTargetLabel(const int newArrivalTime) noexcept {
        profiler.countMetric(METRIC_ADD_JOURNEYS);
        if (newArrivalTime < targetLabels.back()) {
            targetLabels.back() = newArrivalTime;
            minArrivalTime = (newArrivalTime - sourceDepartureTime) * arrivalSlack + sourceDepartureTime;
        }
    }

    inline void computeAnchorLabels(const double tripSlack) noexcept {
        for (size_t i = 0; i < targetLabels.size(); i++) {
            if (targetLabels[i] >= (anchorLabels.empty() ? never : anchorLabels.back().arrivalTime)) continue;
            anchorLabels.emplace_back(targetLabels[i], i);
            maxTrips = i;
        }
        maxTrips = std::ceil(maxTrips * tripSlack);
        Vector::reverse(anchorLabels);
    }

private:
    const Data& data;
    InitialTransferType& bucketQuery;

    std::vector<TripLabel> queue;
    std::vector<EdgeRange> edgeRanges;
    size_t queueSize;
    ReachedIndex reachedIndex;
    StopArrivalTimes stopArrivalTimes;

    std::vector<int> targetLabels;
    int minArrivalTime;

    std::vector<EdgeLabel> edgeLabels;
    std::vector<RouteLabel> routeLabels;

    Vertex sourceVertex;
    Vertex targetVertex;
    int sourceDepartureTime;
    double arrivalSlack;

    std::vector<RAPTOR::ArrivalLabel> anchorLabels;
    size_t maxTrips;

    Profiler& profiler;
};

}
