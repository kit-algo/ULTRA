#pragma once

#include <vector>

#include "../../../DataStructures/Container/Map.h"
#include "../../../DataStructures/Container/Set.h"
#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"

#include "../InitialTransfers.h"
#include "../Profiler.h"

namespace RAPTOR {

template<typename PROFILER = NoProfiler, typename INITIAL_TRANSFERS = BucketCHInitialTransfers>
class ForwardPruningULTRARAPTOR {

public:
    using Profiler = PROFILER;
    using InitialTransferType = INITIAL_TRANSFERS;
    using Type = ForwardPruningULTRARAPTOR<Profiler, InitialTransferType>;

private:
    using Round = std::vector<int>;

public:
    ForwardPruningULTRARAPTOR(const Data& data, InitialTransferType& initialTransfers, Profiler& profiler) :
        data(data),
        initialTransfers(initialTransfers),
        stopsUpdatedByRoute(data.numberOfStops()),
        stopsUpdatedByTransfer(data.numberOfStops()),
        routesServingUpdatedStops(data.numberOfRoutes()),
        sourceVertex(noVertex),
        targetVertex(noVertex),
        sourceStop(noStop),
        targetStop(noStop),
        sourceDepartureTime(never),
        arrivalSlack(INFTY),
        maxTrips(0),
        profiler(profiler) {
        AssertMsg(data.hasImplicitBufferTimes(), "Departure buffer times have to be implicit!");
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target, const double arrSlack, const double tripSlack) noexcept {
        profiler.startPhase();
        clear();
        initialize(source, departureTime, target, arrSlack);
        profiler.donePhase(PHASE_INITIALIZATION);
        profiler.startPhase();
        relaxInitialTransfers();
        profiler.donePhase(PHASE_TRANSFERS);

        for (size_t i = 0; !stopsUpdatedByTransfer.empty(); i++) {
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
        computeAnchorLabels(tripSlack);
        profiler.donePhase(PHASE_INITIALIZATION);
    }

    inline const std::vector<ArrivalLabel>& getAnchorLabels() const noexcept {
        return anchorLabels;
    }

    inline size_t getMaxTrips() const noexcept {
        return maxTrips;
    }

    inline int getArrivalTime(const StopId stop, const size_t round) const noexcept {
        return rounds[std::min(2 * round + 1, rounds.size() - 1)][stop];
    }

private:
    inline void clear() noexcept {
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        anchorLabels.clear();
        maxTrips = 0;
        rounds.clear();
    }

    inline void initialize(const Vertex source, const int departureTime, const Vertex target, const double slack) noexcept {
        sourceVertex = source;
        targetVertex = target;
        sourceStop = data.isStop(source) ? StopId(source) : StopId(data.numberOfStops() + 1);
        targetStop = data.isStop(target) ? StopId(target) : StopId(data.numberOfStops());
        sourceDepartureTime = departureTime;
        arrivalSlack = slack;
        startNewRound();
        arrivalByRoute(sourceStop, sourceDepartureTime);
        startNewRound();
    }

    inline void collectRoutesServingUpdatedStops() noexcept {
        for (const StopId stop : stopsUpdatedByTransfer) {
            AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
            const int arrivalTime = previousRound()[stop];
            AssertMsg(arrivalTime < never, "Updated stop has arrival time = never!");
            for (const RouteSegment& route : data.routesContainingStop(stop)) {
                AssertMsg(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                AssertMsg(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId)) continue;
                if (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < arrivalTime) continue;
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
            AssertMsg(stopIndex < tripSize - 1, "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " << stopIndex << ", TripSize: " << tripSize << ")!");

            const StopId* stops = data.stopArrayOfRoute(route);
            const StopEvent* trip = data.lastTripOfRoute(route);
            StopId stop = stops[stopIndex];
            AssertMsg(trip[stopIndex].departureTime >= previousRound()[stop], "Cannot scan a route after the last trip has departed (Route: " << route << ", Stop: " << stop << ", StopIndex: " << stopIndex << ", Time: " << previousRound()[stop] << ", LastDeparture: " << trip[stopIndex].departureTime << ")!");

            StopIndex parentIndex = stopIndex;
            const StopEvent* firstTrip = data.firstTripOfRoute(route);
            while (stopIndex < tripSize - 1) {
                while ((trip > firstTrip) && ((trip - tripSize + stopIndex)->departureTime >= previousRound()[stop])) {
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
        initialTransfers.template run(sourceVertex, targetVertex, arrivalSlack);
        for (const Vertex stop : initialTransfers.getForwardPOIs()) {
            if (stop == targetStop) continue;
            AssertMsg(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
            AssertMsg(initialTransfers.getForwardDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
            arrivalByTransfer(StopId(stop), sourceDepartureTime + initialTransfers.getForwardDistance(stop));
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
            const int earliestArrivalTime = previousRound()[stop];
            relaxShortcuts(stop, earliestArrivalTime);
            if (initialTransfers.getBackwardDistance(stop) != INFTY) {
                arrivalByTransfer(targetStop, earliestArrivalTime + initialTransfers.getBackwardDistance(stop));
            }
            stopsUpdatedByTransfer.insert(stop);
        }
        for (const StopId stop : stopsToScan) {
            relaxShortcuts(stop, previousRound()[stop]);
        }
    }

    inline void relaxShortcuts(const StopId stop, const int earliestArrivalTime) noexcept {
        for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
            profiler.countMetric(METRIC_EDGES);
            AssertMsg(data.isStop(data.transferGraph.get(ToVertex, edge)), "Graph contains edges to non stop vertices!");
            const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
            arrivalByTransfer(toStop, earliestArrivalTime + data.transferGraph.get(TravelTime, edge));
        }
    }

    inline Round& currentRound() noexcept {
        AssertMsg(!rounds.empty(), "Cannot return current round, because no round exists!");
        return rounds.back();
    }

    inline Round& previousRound() noexcept {
        AssertMsg(rounds.size() >= 2, "Cannot return previous round, because less than two rounds exist!");
        return rounds[rounds.size() - 2];
    }

    inline void startNewRound() noexcept {
        if (rounds.empty()) {
            rounds.emplace_back(data.numberOfStops() + 2, never);
        } else {
            rounds.emplace_back(rounds.back());
        }
    }

    inline void arrival(const StopId stop, const int time, IndexedSet<false, StopId>& updatedStops, Metric metric) noexcept {
        if ((currentRound()[targetStop] - sourceDepartureTime) * arrivalSlack < time - sourceDepartureTime) return;
        if (currentRound()[stop] <= time) return;
        profiler.countMetric(metric);
        currentRound()[stop] = time;
        if (data.isStop(stop)) updatedStops.insert(stop);
    }

    inline void arrivalByRoute(const StopId stop, const int time) noexcept {
        arrival(stop, time, stopsUpdatedByRoute, METRIC_STOPS_BY_TRIP);
    }

    inline void arrivalByTransfer(const StopId stop, const int time) noexcept {
        arrival(stop, time, stopsUpdatedByTransfer, METRIC_STOPS_BY_TRANSFER);
    }

    inline void computeAnchorLabels(const double tripSlack) noexcept {
        for (size_t i = 1; i < rounds.size(); i += 2) {
            if (rounds[i][targetStop] >= (anchorLabels.empty() ? never : anchorLabels.back().arrivalTime)) continue;
            anchorLabels.emplace_back(rounds[i][targetStop], i / 2);
            maxTrips = i / 2;
        }
        maxTrips = std::ceil(maxTrips * tripSlack);
        Vector::reverse(anchorLabels);
    }

private:
    const Data& data;
    InitialTransferType& initialTransfers;

    std::vector<Round> rounds;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    Vertex sourceVertex;
    Vertex targetVertex;
    StopId sourceStop;
    StopId targetStop;
    int sourceDepartureTime;
    double arrivalSlack;

    std::vector<ArrivalLabel> anchorLabels;
    size_t maxTrips;

    Profiler& profiler;

};

}
