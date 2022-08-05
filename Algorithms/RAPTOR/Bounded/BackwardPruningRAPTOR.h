#pragma once

#include <vector>

#include "../../../DataStructures/Container/Map.h"
#include "../../../DataStructures/Container/Set.h"
#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "ForwardPruningRAPTOR.h"

#include "../Profiler.h"


namespace RAPTOR {

template<typename PROFILER>
class BackwardPruningRAPTOR {

public:
    using Profiler = PROFILER;
    using Type = BackwardPruningRAPTOR<Profiler>;

public:
    struct EarliestArrivalLabel {
        EarliestArrivalLabel() : arrivalTimeByRoute(never), arrivalTimeByTransfer(never), timestamp(0) {}
        int arrivalTimeByRoute;
        int arrivalTimeByTransfer;
        int timestamp;

        inline void setArrivalTimeByRoute(const int time) noexcept {
            arrivalTimeByRoute = time;
            arrivalTimeByTransfer = std::min(arrivalTimeByTransfer, time);
        }

        inline void minimize(const EarliestArrivalLabel& other) noexcept {
            arrivalTimeByRoute = std::min(arrivalTimeByRoute, other.arrivalTimeByRoute);
            arrivalTimeByTransfer = std::min(arrivalTimeByTransfer, other.arrivalTimeByTransfer);
        }
    };
    using Round = std::vector<EarliestArrivalLabel>;

public:
    BackwardPruningRAPTOR(const Data& data, const ForwardPruningRAPTOR<Profiler>& forwardPruningRAPTOR, Profiler& profiler) :
        data(data),
        forwardPruningRAPTOR(forwardPruningRAPTOR),
        roundOffset(-1),
        roundIndex(-1),
        maxTrips(-1),
        stopsUpdatedByRoute(data.numberOfStops()),
        stopsUpdatedByTransfer(data.numberOfStops()),
        routesServingUpdatedStops(data.numberOfRoutes()),
        sourceStop(noStop),
        targetStop(noStop),
        sourceDepartureTime(intMax),
        timestamp(0),
        profiler(profiler) {
        AssertMsg(data.hasImplicitBufferTimes(), "Departure buffer times have to be implicit!");
    }

    inline void run(const StopId source, const StopId target, const int originalDepartureTime, const double arrivalSlack, const double tripSlack) noexcept {
        profiler.startPhase();
        clear<true>();
        sourceStop = source;
        targetStop = target;
        profiler.donePhase(PHASE_INITIALIZATION);

        size_t lastNumberOfTrips = INFTY;
        maxTrips = forwardPruningRAPTOR.getMaxTrips();
        for (const ArrivalLabel& label : forwardPruningRAPTOR.getAnchorLabels()) {
            sourceDepartureTime = -((label.arrivalTime - originalDepartureTime) * arrivalSlack + originalDepartureTime);
            roundOffset = maxTrips - std::min(size_t(std::ceil(label.numberOfTrips * tripSlack)), lastNumberOfTrips - 1);
            lastNumberOfTrips = label.numberOfTrips;
            runIteration();
        }
        profiler.donePhase(PHASE_INITIALIZATION);
    }

    inline int getArrivalTimeByRoute(const StopId stop, const size_t round) noexcept {
        return roundLabel(std::min(round, rounds.size() - 1), stop).arrivalTimeByRoute;
    }

    inline int getArrivalTimeByTransfer(const StopId stop, const size_t round) noexcept {
        return roundLabel(std::min(round, rounds.size() - 1), stop).arrivalTimeByTransfer;
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
        profiler.startPhase();
        clear<false>();
        initialize();
        profiler.donePhase(PHASE_INITIALIZATION);
        profiler.startPhase();
        relaxTransfers();
        profiler.donePhase(PHASE_TRANSFERS);

        for (size_t i = roundIndex + 1; i <= maxTrips; i++) {
            profiler.startPhase();
            startNewRound();
            profiler.donePhase(PHASE_INITIALIZATION);
            profiler.startPhase();
            collectRoutesServingUpdatedStops();
            profiler.donePhase(PHASE_COLLECT);
            profiler.startPhase();
            scanRoutes();
            profiler.donePhase(PHASE_SCAN);
            if (stopsUpdatedByRoute.empty()) break;
            profiler.startPhase();
            relaxTransfers();
            profiler.donePhase(PHASE_TRANSFERS);
        }
        profiler.startPhase();
    }

    inline void initialize() noexcept {
        startNewRound();
        stopsUpdatedByRoute.insert(sourceStop);
        currentRoundLabel(sourceStop).setArrivalTimeByRoute(sourceDepartureTime);
    }

    inline void collectRoutesServingUpdatedStops() noexcept {
        for (const StopId stop : stopsUpdatedByTransfer) {
            for (const RouteSegment& route : data.routesContainingStop(stop)) {
                AssertMsg(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                AssertMsg(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId)) continue;
                if (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < previousRoundLabel(stop).arrivalTimeByTransfer) continue;
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
            AssertMsg(trip[stopIndex].departureTime >= previousRoundLabel(stop).arrivalTimeByTransfer, "Cannot scan a route after the last trip has departed (Route: " << route << ", Stop: " << stop << ", StopIndex: " << stopIndex << ", Time: " << previousRoundLabel(stop).arrivalTimeByTransfer << ", LastDeparture: " << trip[stopIndex].departureTime << ", RoundIndex: " << roundIndex << ")!");

            StopIndex parentIndex = stopIndex;
            while (stopIndex < tripSize - 1) {
                while ((trip > firstTrip) && ((trip - tripSize + stopIndex)->departureTime >= previousRoundLabel(stop).arrivalTimeByTransfer)) {
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
    inline void relaxTransfers() noexcept {
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        for (const StopId stop : stopsUpdatedByRoute) {
            const int earliestArrivalTime = currentRoundLabel(stop).arrivalTimeByRoute;
            for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
                profiler.countMetric(METRIC_EDGES);
                const int arrivalTime = earliestArrivalTime + data.transferGraph.get(TravelTime, edge);
                const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
                AssertMsg(data.isStop(toStop), "Graph contains edges to non stop vertices!");
                arrivalByTransfer(toStop, arrivalTime);
            }
            stopsUpdatedByTransfer.insert(stop);
        }
    }

    inline EarliestArrivalLabel& roundLabel(const size_t round, const StopId stop) noexcept {
        EarliestArrivalLabel& result = rounds[round][stop];
        if (result.timestamp != timestamp) {
            if (round > 0) {
                result.minimize(roundLabel(round - 1, stop));
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
                rounds.emplace_back(data.numberOfStops());
            } else {
                rounds.emplace_back(rounds.back());
            }
        }
    }

    inline void arrivalByRoute(const StopId stop, const int time) noexcept {
        if (forwardPruningRAPTOR.getArrivalTimeByTransfer(stop, maxTrips - roundIndex) > -time) return;
        EarliestArrivalLabel& label = currentRoundLabel(stop);
        if (label.arrivalTimeByRoute <= time) return;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        label.setArrivalTimeByRoute(time);
        stopsUpdatedByRoute.insert(stop);

    }

    inline void arrivalByTransfer(const StopId stop, const int time) noexcept {
        if (forwardPruningRAPTOR.getArrivalTimeByRoute(stop, maxTrips - roundIndex) > -time) return;
        EarliestArrivalLabel& label = currentRoundLabel(stop);
        if (label.arrivalTimeByTransfer <= time) return;
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        label.arrivalTimeByTransfer = time;
        stopsUpdatedByTransfer.insert(stop);

    }

private:
    const Data& data;
    const ForwardPruningRAPTOR<Profiler>& forwardPruningRAPTOR;

    std::vector<Round> rounds;
    size_t roundOffset;
    size_t roundIndex;
    size_t maxTrips;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    StopId sourceStop;
    StopId targetStop;
    int sourceDepartureTime;
    int timestamp;

    Profiler& profiler;
};

}
