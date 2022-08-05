#pragma once

#include <vector>

#include "../../../DataStructures/Container/Map.h"
#include "../../../DataStructures/Container/Set.h"
#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"

#include "../Profiler.h"

namespace RAPTOR {

template<typename PROFILER = NoProfiler>
class ForwardPruningRAPTOR {

public:
    using Profiler = PROFILER;
    using Type = ForwardPruningRAPTOR<Profiler>;

private:
    struct ArrivalTimes {
        ArrivalTimes() : arrivalTimeByRoute(never), arrivalTimeByTransfer(never) {}

        int arrivalTimeByRoute;
        int arrivalTimeByTransfer;

        inline void setArrivalTimeByRoute(const int time) noexcept {
            arrivalTimeByRoute = time;
            arrivalTimeByTransfer = std::min(arrivalTimeByTransfer, time);
        }
    };

    using Round = std::vector<ArrivalTimes>;

public:
    ForwardPruningRAPTOR(const Data& data, Profiler& profiler) :
        data(data),
        stopsUpdatedByRoute(data.numberOfStops()),
        stopsUpdatedByTransfer(data.numberOfStops()),
        routesServingUpdatedStops(data.numberOfRoutes()),
        sourceStop(noStop),
        targetStop(noStop),
        sourceDepartureTime(never),
        arrivalSlack(INFTY),
        maxTrips(0),
        profiler(profiler) {
        AssertMsg(data.hasImplicitBufferTimes(), "Departure buffer times have to be implicit!");
    }

    inline void run(const StopId source, const int departureTime, const StopId target, const double arrSlack, const double tripSlack) noexcept {
        profiler.startPhase();
        clear();
        initialize(source, departureTime, target, arrSlack);
        profiler.donePhase(PHASE_INITIALIZATION);
        profiler.startPhase();
        relaxTransfers();
        profiler.donePhase(PHASE_TRANSFERS);

        for (size_t i = 0;; i++) {
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
        computeAnchorLabels(tripSlack);
        profiler.donePhase(PHASE_INITIALIZATION);
    }

    inline const std::vector<ArrivalLabel>& getAnchorLabels() const noexcept {
        return anchorLabels;
    }

    inline size_t getMaxTrips() const noexcept {
        return maxTrips;
    }

    inline int getArrivalTimeByRoute(const StopId stop, const size_t round) const noexcept {
        return rounds[std::min(round, rounds.size() - 1)][stop].arrivalTimeByRoute;
    }

    inline int getArrivalTimeByTransfer(const StopId stop, const size_t round) const noexcept {
        return rounds[std::min(round, rounds.size() - 1)][stop].arrivalTimeByTransfer;
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

    inline void initialize(const StopId source, const int departureTime, const StopId target, const double slack) noexcept {
        sourceStop = source;
        targetStop = target;
        sourceDepartureTime = departureTime;
        arrivalSlack = slack;
        startNewRound();
        arrivalByRoute(source, sourceDepartureTime);
    }

    inline void collectRoutesServingUpdatedStops() noexcept {
        for (const StopId stop : stopsUpdatedByTransfer) {
            AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
            const int arrivalTime = previousRound()[stop].arrivalTimeByTransfer;
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
            AssertMsg(trip[stopIndex].departureTime >= previousRound()[stop].arrivalTimeByTransfer, "Cannot scan a route after the last trip has departed (Route: " << route << ", Stop: " << stop << ", StopIndex: " << stopIndex << ", Time: " << previousRound()[stop].arrivalTimeByTransfer << ", LastDeparture: " << trip[stopIndex].departureTime << ")!");

            StopIndex parentIndex = stopIndex;
            const StopEvent* firstTrip = data.firstTripOfRoute(route);
            while (stopIndex < tripSize - 1) {
                while ((trip > firstTrip) && ((trip - tripSize + stopIndex)->departureTime >= previousRound()[stop].arrivalTimeByTransfer)) {
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
            const int earliestArrivalTime = currentRound()[stop].arrivalTimeByRoute;
            for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
                profiler.countMetric(METRIC_EDGES);
                const int arrivalTime = earliestArrivalTime + data.transferGraph.get(TravelTime, edge);
                AssertMsg(data.isStop(data.transferGraph.get(ToVertex, edge)), "Graph contains edges to non stop vertices!");
                const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
                arrivalByTransfer(toStop, arrivalTime);
            }
            stopsUpdatedByTransfer.insert(stop);
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
            rounds.emplace_back(data.numberOfStops());
        } else {
            rounds.emplace_back(rounds.back());
        }
    }

    inline void arrivalByRoute(const StopId stop, const int time) noexcept {
        AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
        if ((currentRound()[targetStop].arrivalTimeByTransfer - sourceDepartureTime) * arrivalSlack < time - sourceDepartureTime) return;
        if (currentRound()[stop].arrivalTimeByRoute <= time) return;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        currentRound()[stop].setArrivalTimeByRoute(time);
        stopsUpdatedByRoute.insert(stop);
    }

    inline void arrivalByTransfer(const StopId stop, const int time) noexcept {
        AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
        if ((currentRound()[targetStop].arrivalTimeByTransfer - sourceDepartureTime) * arrivalSlack < time - sourceDepartureTime) return;
        if (currentRound()[stop].arrivalTimeByTransfer <= time) return;
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        currentRound()[stop].arrivalTimeByTransfer = time;
        stopsUpdatedByTransfer.insert(stop);
    }

    inline void computeAnchorLabels(const double tripSlack) noexcept {
        for (size_t i = 0; i < rounds.size(); i++) {
            if (rounds[i][targetStop].arrivalTimeByTransfer >= (anchorLabels.empty() ? never : anchorLabels.back().arrivalTime)) continue;
            anchorLabels.emplace_back(rounds[i][targetStop].arrivalTimeByTransfer, i);
            maxTrips = i;
        }
        maxTrips = std::ceil(maxTrips * tripSlack);
        Vector::reverse(anchorLabels);
    }

private:
    const Data& data;

    std::vector<Round> rounds;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    StopId sourceStop;
    StopId targetStop;
    int sourceDepartureTime;
    double arrivalSlack;

    std::vector<ArrivalLabel> anchorLabels;
    size_t maxTrips;

    Profiler& profiler;

};

}
