#pragma once

#include "../../CH/Query/BucketQuery.h"

#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/TripBased/Data.h"

#include "../Query/Profiler.h"
#include "ReachedIndexRounds.h"
#include "ForwardPruningQuery.h"
#include "StopArrivalTimes.h"

namespace TripBased {

template<typename PROFILER = NoProfiler, typename INITIAL_TRANSFERS = RAPTOR::BucketCHInitialTransfers>
class BackwardPruningQuery {

public:
    using Profiler = PROFILER;
    using InitialTransferType = INITIAL_TRANSFERS;
    using Type = BackwardPruningQuery<Profiler, InitialTransferType>;

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
        StopEventId stopEvent;
        TripId trip;
        StopEventId firstEvent;
        StopId departureStop;
        int departureTime;
    };

public:
    BackwardPruningQuery(const Data& data, const ForwardPruningQuery<Profiler, InitialTransferType>& forwardPruningQuery, const InitialTransferType& bucketQuery, Profiler& profiler) :
        data(data),
        forwardPruningQuery(forwardPruningQuery),
        bucketQuery(bucketQuery),
        queue(data.numberOfStopEvents()),
        edgeRanges(data.numberOfStopEvents()),
        queueSize(0),
        reachedIndex(data),
        stopArrivalTimes(data),
        minArrivalTime(INFTY),
        edgeLabels(data.stopEventGraph.numEdges()),
        sourceVertex(noVertex),
        targetVertex(noVertex),
        sourceDepartureTime(never),
        roundOffset(-1),
        round(-1),
        maxTrips(-1),
        properArrivalTimes(data.numberOfStopEvents()),
        profiler(profiler) {
        for (const Edge edge : data.stopEventGraph.edges()) {
            const StopEventId departureStopEvent(data.stopEventGraph.get(ToVertex, edge));
            edgeLabels[edge].stopEvent = StopEventId(departureStopEvent + 1);
            edgeLabels[edge].trip = data.tripOfStopEvent[departureStopEvent];
            edgeLabels[edge].firstEvent = data.firstStopEventOfTrip[edgeLabels[edge].trip];
            edgeLabels[edge].departureStop = data.getStopOfStopEvent(departureStopEvent);
            edgeLabels[edge].departureTime = -data.departureTime(departureStopEvent);
        }
        for (StopEventId stopEvent(0); stopEvent < data.numberOfStopEvents(); stopEvent++) {
            properArrivalTimes[stopEvent] = data.arrivalEvents[stopEvent].arrivalTime - data.minTransferTime(data.getStopOfStopEvent(stopEvent));
        }
    }

    inline void run(const Vertex source, const int originalDepartureTime, const Vertex target, const double arrivalSlack, const double tripSlack) noexcept {
        clear<true>();
        sourceVertex = source;
        targetVertex = target;
        minArrivalTime = -originalDepartureTime;
        size_t lastNumberOfTrips = INFTY;
        maxTrips = forwardPruningQuery.getMaxTrips();
        departureTimes.resize(maxTrips + 1, INFTY);
        for (const RAPTOR::ArrivalLabel& label : forwardPruningQuery.getAnchorLabels()) {
            sourceDepartureTime = -((label.arrivalTime - originalDepartureTime) * arrivalSlack + originalDepartureTime);
            roundOffset = maxTrips - std::min(size_t(std::ceil(label.numberOfTrips * tripSlack)), lastNumberOfTrips - 1);
            lastNumberOfTrips = label.numberOfTrips;
            runIteration();
        }
    }

    inline StopIndex getReachedIndex(const TripId trip, const size_t numTrips) const noexcept {
        return reachedIndex(trip, numTrips);
    }

    inline int getDepartureTime(const size_t numTrips) const noexcept {
        return departureTimes[numTrips];
    }

    inline int getArrivalTime(const StopId stop, const size_t round) const noexcept {
        return stopArrivalTimes(stop, round);
    }

    inline TripId getReverseTrip(const RouteId route, const size_t tripOffset) const noexcept {
        return TripId(data.firstTripOfRoute[route + 1] - tripOffset - 1);
    }

    inline TripId getReverseTrip(const TripId trip) const noexcept {
        const RouteId route = data.routeOfTrip[trip];
        const size_t tripOffset = trip - data.firstTripOfRoute[route];
        return TripId(data.firstTripOfRoute[route + 1] - tripOffset - 1);
    }

private:
    inline void runIteration() noexcept {
        clear<false>();
        for (size_t i = round; i <= maxTrips; i++) {
            if (departureTimes[i] < sourceDepartureTime) break;
            departureTimes[i] = sourceDepartureTime;
        }
        evaluateInitialTransfers();
        scanTrips();
    }

    template<bool RESET>
    inline void clear() noexcept {
        queueSize = 0;
        round = roundOffset;
        if constexpr (RESET) {
            reachedIndex.clear();
            stopArrivalTimes.clear();
            minArrivalTime = INFTY;
            departureTimes.clear();
        } else {
            reachedIndex.startNewRound(round);
            stopArrivalTimes.startNewRound(round);
        }
    }

    inline void evaluateInitialTransfers() noexcept {
        for (const Vertex stop : bucketQuery.getBackwardPOIs()) {
            const int stopDepartureTime = sourceDepartureTime + bucketQuery.getBackwardDistance(stop);
            const int arrivalTime = forwardPruningQuery.getArrivalTime(StopId(stop), maxTrips - round);
            if (-stopDepartureTime < arrivalTime) continue;
            for (const RAPTOR::RouteSegment& segment : data.routesContainingStop(StopId(stop))) {
                const TripId trip = data.getEarliestTrip(segment, stopDepartureTime);
                if (trip != noTripId) {
                    enqueue(trip, StopIndex(segment.stopIndex + 1));
                }
            }
        }
    }

    inline void scanTrips() noexcept {
        size_t roundBegin = 0;
        size_t roundEnd = queueSize;
        while (round + 1 <= maxTrips && roundBegin < roundEnd) {
            round++;
            reachedIndex.startNewRound(round);
            stopArrivalTimes.startNewRound(round);
            // Find the range of transfers for each trip
            for (size_t i = roundBegin; i < roundEnd; i++) {
                TripLabel& label = queue[i];
                profiler.countMetric(METRIC_SCANNED_TRIPS);
                for (StopEventId j = label.begin; j < label.end; j++) {
                    profiler.countMetric(METRIC_SCANNED_STOPS);
                    if (properArrivalTimes[j] > minArrivalTime) label.end = j;
                }
                edgeRanges[i].begin = data.stopEventGraph.beginEdgeFrom(Vertex(label.begin));
                edgeRanges[i].end = data.stopEventGraph.beginEdgeFrom(Vertex(label.end));
            }
            // Update stops
            for (size_t i = roundBegin; i < roundEnd; i++) {
                const TripLabel& label = queue[i];
                for (StopEventId j = label.begin; j < label.end; j++) {
                    stopArrivalTimes.updateCopyForward(j);
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
        reachedIndex.updateCopyForward(trip, index);
    }

    inline void enqueue(const Edge edge) noexcept {
        profiler.countMetric(METRIC_ENQUEUES);
        const EdgeLabel& label = edgeLabels[edge];
        if (reachedIndex.alreadyReached(label.trip, label.stopEvent - label.firstEvent)) return;
        if (label.departureTime < forwardPruningQuery.getArrivalTime(label.departureStop, maxTrips - round)) return;
        queue[queueSize] = TripLabel(label.stopEvent, StopEventId(label.firstEvent + reachedIndex(label.trip)));
        queueSize++;
        AssertMsg(queueSize <= queue.size(), "Queue is overfull!");
        reachedIndex.updateCopyForward(label.trip, StopIndex(label.stopEvent - label.firstEvent));
    }

private:
    const Data& data;
    const ForwardPruningQuery<Profiler, InitialTransferType>& forwardPruningQuery;
    const InitialTransferType& bucketQuery;

    std::vector<TripLabel> queue;
    std::vector<EdgeRange> edgeRanges;
    size_t queueSize;
    ReachedIndexRounds reachedIndex;
    StopArrivalTimes stopArrivalTimes;

    int minArrivalTime;
    std::vector<int> departureTimes;

    std::vector<EdgeLabel> edgeLabels;

    Vertex sourceVertex;
    Vertex targetVertex;
    int sourceDepartureTime;

    size_t roundOffset;
    size_t round;
    size_t maxTrips;

    std::vector<int> properArrivalTimes;

    Profiler& profiler;
};

}
