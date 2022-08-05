#pragma once

#include <vector>

#include "../../../DataStructures/Container/Map.h"
#include "../../../DataStructures/Container/Set.h"
#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"

#include "../Profiler.h"

#include "ForwardPruningULTRARAPTOR.h"

namespace RAPTOR {

template<typename PROFILER, typename INITIAL_TRANSFERS = BucketCHInitialTransfers>
class BackwardPruningULTRARAPTOR {

public:
    using Profiler = PROFILER;
    using InitialTransferType = INITIAL_TRANSFERS;
    using Type = BackwardPruningULTRARAPTOR<Profiler, InitialTransferType>;

public:
    struct EarliestArrivalLabel {
        EarliestArrivalLabel() : arrivalTime(never), timestamp(0) {}
        int arrivalTime;
        int timestamp;
    };
    using Round = std::vector<EarliestArrivalLabel>;

public:
    BackwardPruningULTRARAPTOR(const Data& data, const InitialTransferType& initialTransfers, const ForwardPruningULTRARAPTOR<Profiler, InitialTransferType>& forwardPruningRAPTOR, Profiler& profiler) :
        data(data),
        initialTransfers(initialTransfers),
        forwardPruningRAPTOR(forwardPruningRAPTOR),
        roundOffset(-1),
        roundIndex(-1),
        maxTrips(-1),
        stopsUpdatedByRoute(data.numberOfStops()),
        stopsUpdatedByTransfer(data.numberOfStops()),
        routesServingUpdatedStops(data.numberOfRoutes()),
        sourceVertex(noVertex),
        targetVertex(noVertex),
        sourceStop(noStop),
        targetStop(noStop),
        sourceDepartureTime(intMax),
        timestamp(0),
        profiler(profiler) {
        AssertMsg(data.hasImplicitBufferTimes(), "Departure buffer times have to be implicit!");
    }

    inline void run(const Vertex source, const Vertex target, const int originalDepartureTime, const double arrivalSlack, const double tripSlack) noexcept {
        profiler.startPhase();
        clear<true>();
        sourceVertex = source;
        targetVertex = target;
        sourceStop = data.isStop(source) ? StopId(source) : StopId(data.numberOfStops());
        targetStop = data.isStop(target) ? StopId(target) : StopId(data.numberOfStops() + 1);

        size_t lastNumberOfTrips = INFTY;
        maxTrips = forwardPruningRAPTOR.getMaxTrips();
        for (const ArrivalLabel& label : forwardPruningRAPTOR.getAnchorLabels()) {
            sourceDepartureTime = -((label.arrivalTime - originalDepartureTime) * arrivalSlack + originalDepartureTime);
            roundOffset = 2 * (maxTrips - std::min(size_t(std::ceil(label.numberOfTrips * tripSlack)), lastNumberOfTrips - 1));
            lastNumberOfTrips = label.numberOfTrips;
            runIteration();
        }
        profiler.donePhase(PHASE_INITIALIZATION);
    }

    inline int getArrivalTime(const StopId source, const size_t round) noexcept {
        return roundLabel(std::min(2 * round + 1, rounds.size() - 1), source).arrivalTime;
    }

private:
    template<bool RESET = true>
    inline void clear() noexcept {
        roundIndex = roundOffset - 1;
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        if constexpr (RESET) {
            timestamp = 1;
            std::vector<Round>().swap(rounds);
        } else {
            timestamp++;
        }
    }

    inline void runIteration() noexcept {
        clear<false>();
        initialize();
        profiler.donePhase(PHASE_INITIALIZATION);
        profiler.startPhase();
        relaxInitialTransfers();
        profiler.donePhase(PHASE_TRANSFERS);

        for (size_t i = (roundIndex + 1) / 2; i <= maxTrips; i++) {
            profiler.startPhase();
            startNewRound();
            profiler.donePhase(PHASE_INITIALIZATION);
            profiler.startPhase();
            collectRoutesServingUpdatedStops();
            profiler.donePhase(PHASE_COLLECT);
            profiler.startPhase();
            scanRoutes();
            profiler.donePhase(PHASE_SCAN);
            profiler.startPhase();
            startNewRound();
            profiler.donePhase(PHASE_INITIALIZATION);
            profiler.startPhase();
            relaxIntermediateTransfers();
            profiler.donePhase(PHASE_TRANSFERS);
        }
        profiler.startPhase();
    }

    inline void initialize() noexcept {
        startNewRound();
        arrivalByRoute(sourceStop, sourceDepartureTime);
        startNewRound();
    }

    inline void collectRoutesServingUpdatedStops() noexcept {
        for (const StopId stop : stopsUpdatedByTransfer) {
            for (const RouteSegment& route : data.routesContainingStop(stop)) {
                AssertMsg(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                AssertMsg(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId)) continue;
                if (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < previousRoundLabel(stop).arrivalTime) continue;
                if (routesServingUpdatedStops.contains(route.routeId)) {
                    routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                }
            }
        }
    }

    inline void scanRoutes() noexcept {
        stopsUpdatedByRoute.clear();
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            profiler.countMetric(METRIC_ROUTES);
            StopIndex stopIndex = routesServingUpdatedStops[route];
            const size_t tripSize = data.numberOfStopsInRoute(route);
            AssertMsg(stopIndex < tripSize - 1, "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " << stopIndex << ", TripSize: " << tripSize << ", RoundIndex: " << roundIndex << ")!");

            const StopId* stops = data.stopArrayOfRoute(route);
            const StopEvent* firstTrip = data.firstTripOfRoute(route);
            const StopEvent* trip;
            trip = data.lastTripOfRoute(route);
            StopId stop = stops[stopIndex];
            AssertMsg(trip[stopIndex].departureTime >= previousRoundLabel(stop).arrivalTime, "Cannot scan a route after the last trip has departed (Route: " << route << ", Stop: " << stop << ", StopIndex: " << stopIndex << ", Time: " << previousRoundLabel(stop).arrivalTime << ", LastDeparture: " << trip[stopIndex].departureTime << ", RoundIndex: " << roundIndex << ")!");

            StopIndex parentIndex = stopIndex;
            while (stopIndex < tripSize - 1) {
                while ((trip > firstTrip) && ((trip - tripSize + stopIndex)->departureTime >= previousRoundLabel(stop).arrivalTime)) {
                    trip -= tripSize;
                    parentIndex = stopIndex;
                }
                stopIndex++;
                stop = stops[stopIndex];
                profiler.countMetric(METRIC_ROUTE_SEGMENTS);
                arrivalByRoute(stop, trip[stopIndex].arrivalTime);
            }
        }
    }

    inline void relaxInitialTransfers() noexcept {
        for (const Vertex stop : initialTransfers.getBackwardPOIs()) {
            if (stop == targetStop) continue;
            AssertMsg(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
            AssertMsg(initialTransfers.getBackwardDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
            arrivalByTransfer(StopId(stop), sourceDepartureTime + initialTransfers.getBackwardDistance(stop));
        }
        if (initialTransfers.getDistance() != INFTY) {
            arrivalByTransfer(targetStop, sourceDepartureTime + initialTransfers.getDistance());
        }
        if (data.isStop(sourceStop)) stopsUpdatedByTransfer.insert(sourceStop);
    }

    inline void relaxIntermediateTransfers() noexcept {
        routesServingUpdatedStops.clear();
        const std::vector<StopId> stopsToScan = stopsUpdatedByTransfer.getValues();
        stopsUpdatedByTransfer.clear();
        for (const StopId stop : stopsUpdatedByRoute) {
            const int earliestArrivalTime = previousRoundLabel(stop).arrivalTime;
            relaxShortcuts(stop, earliestArrivalTime);
            if (initialTransfers.getForwardDistance(stop) != INFTY) {
                arrivalByTransfer(targetStop, earliestArrivalTime + initialTransfers.getForwardDistance(stop));
            }
            stopsUpdatedByTransfer.insert(stop);
        }
        for (const StopId stop : stopsToScan) {
            relaxShortcuts(stop, previousRoundLabel(stop).arrivalTime);
        }
    }

    inline void relaxShortcuts(const StopId stop, const int earliestArrivalTime) noexcept {
        for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
            profiler.countMetric(METRIC_EDGES);
            const int arrivalTime = earliestArrivalTime + data.transferGraph.get(TravelTime, edge);
            const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
            AssertMsg(data.isStop(toStop), "Graph contains edges to non stop vertices!");
            arrivalByTransfer(toStop, arrivalTime);
        }
    }

    inline EarliestArrivalLabel& roundLabel(const size_t round, const StopId stop) noexcept {
        EarliestArrivalLabel& result = rounds[round][stop];
        if (result.timestamp != timestamp) {
            if (round > 0) {
                result.arrivalTime = std::min(result.arrivalTime, roundLabel(round - 1, stop).arrivalTime);
            }
            result.timestamp = timestamp;
        }
        return result;
    }

    inline EarliestArrivalLabel& currentRoundLabel(const StopId stop) noexcept {
        AssertMsg(roundIndex < rounds.size(), "Round index is out of bounds (roundIndex = " << roundIndex << ", rounds.size() = " << rounds.size() << ")!");
        return roundLabel(roundIndex, stop);
    }

    inline EarliestArrivalLabel& previousRoundLabel(const StopId stop) noexcept {
        AssertMsg(roundIndex - 1 < rounds.size(), "Round index is out of bounds (roundIndex = " << roundIndex << ", rounds.size() = " << rounds.size() << ")!");
        AssertMsg(roundIndex > 0, "Cannot return previous round, because no round exists!");
        return roundLabel(roundIndex - 1, stop);
    }

    inline void startNewRound() noexcept {
        roundIndex++;
        while (roundIndex >= rounds.size()) {
            if (rounds.empty()) {
                rounds.emplace_back(data.numberOfStops() + 2);
            } else {
                rounds.emplace_back(rounds.back());
            }
        }
    }

    inline void arrival(const StopId stop, const int arrivalTime, IndexedSet<false, StopId>& updatedStops, Metric metric) noexcept {
        if (forwardPruningRAPTOR.getArrivalTime(stop, maxTrips - roundIndex / 2) > -arrivalTime) return;
        EarliestArrivalLabel& label = currentRoundLabel(stop);
        if (label.arrivalTime <= arrivalTime) return;
        profiler.countMetric(metric);
        label.arrivalTime = arrivalTime;
        if (data.isStop(stop)) updatedStops.insert(stop);
    }

    inline void arrivalByRoute(const StopId stop, const int time) noexcept {
        arrival(stop, time, stopsUpdatedByRoute, METRIC_STOPS_BY_TRIP);
    }

    inline void arrivalByTransfer(const StopId stop, const int time) noexcept {
        arrival(stop, time, stopsUpdatedByTransfer, METRIC_STOPS_BY_TRANSFER);
    }

private:
    const Data& data;
    const InitialTransferType& initialTransfers;
    const ForwardPruningULTRARAPTOR<Profiler, InitialTransferType>& forwardPruningRAPTOR;

    std::vector<Round> rounds;
    size_t roundOffset;
    size_t roundIndex;
    size_t maxTrips;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    Vertex sourceVertex;
    Vertex targetVertex;
    StopId sourceStop;
    StopId targetStop;
    int sourceDepartureTime;
    int timestamp;

    Profiler& profiler;
};

}
