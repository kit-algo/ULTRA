#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/RAPTOR/Entities/EarliestArrivalTime.h"
#include "../../DataStructures/Container/Set.h"
#include "../../DataStructures/Container/Map.h"

#include "Profiler.h"

namespace RAPTOR {

template<bool TARGET_PRUNING, typename PROFILER = NoProfiler, bool TRANSITIVE = true, bool USE_MIN_TRANSFER_TIMES = false, bool PREVENT_DIRECT_WALKING = false>
class RAPTOR {

public:
    static constexpr bool TargetPruning = TARGET_PRUNING;
    using Profiler = PROFILER;
    static constexpr bool Transitive = TRANSITIVE;
    static constexpr bool UseMinTransferTimes = USE_MIN_TRANSFER_TIMES;
    static constexpr bool PreventDirectWalking = PREVENT_DIRECT_WALKING;
    static constexpr bool SeparateRouteAndTransferEntries = !Transitive | UseMinTransferTimes | PreventDirectWalking;
    static constexpr int RoundFactor = SeparateRouteAndTransferEntries ? 2 : 1;
    using ArrivalTime = EarliestArrivalTime<SeparateRouteAndTransferEntries>;
    using Type = RAPTOR<TargetPruning, Profiler, Transitive, UseMinTransferTimes, PreventDirectWalking>;
    using InitialTransferGraph = TransferGraph;
    using SourceType = StopId;

private:
    struct EarliestArrivalLabel {
        EarliestArrivalLabel() : arrivalTime(never), parentDepartureTime(never), parent(noStop), usesRoute(false), routeId(noRouteId) {}
        int arrivalTime;
        int parentDepartureTime;
        StopId parent;
        bool usesRoute;
        union {
            RouteId routeId;
            Edge transferId;
        };
    };
    using Round = std::vector<EarliestArrivalLabel>;

public:
    RAPTOR(const Data& data, const Profiler& profilerTemplate = Profiler()) :
        data(data),
        earliestArrival(data.numberOfStops()),
        stopsUpdatedByRoute(data.numberOfStops()),
        stopsUpdatedByTransfer(data.numberOfStops()),
        routesServingUpdatedStops(data.numberOfRoutes()),
        sourceStop(noStop),
        targetStop(noStop),
        sourceDepartureTime(never),
        walkingDistance(INFTY),
        profiler(profilerTemplate) {
        if constexpr (UseMinTransferTimes) {
            AssertMsg(!data.hasImplicitBufferTimes(), "Either min transfer times have to be used OR departure buffer times have to be implicit!");
        } else {
            AssertMsg(data.hasImplicitBufferTimes(), "Either min transfer times have to be used OR departure buffer times have to be implicit!");
        }
        profiler.registerExtraRounds({EXTRA_ROUND_CLEAR, EXTRA_ROUND_INITIALIZATION});
        profiler.registerPhases({PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS});
        profiler.registerMetrics({METRIC_ROUTES, METRIC_ROUTE_SEGMENTS, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
    }

    template<typename ATTRIBUTE>
    RAPTOR(const Data& data, const InitialTransferGraph&, const InitialTransferGraph&, const ATTRIBUTE, const Profiler& = Profiler()) :
        RAPTOR(data) {
    }

    inline void run(const StopId source, const int departureTime, const StopId target = noStop, const size_t maxRounds = INFTY) noexcept {
        profiler.start();
        profiler.startExtraRound(EXTRA_ROUND_CLEAR);
        clear();
        profiler.doneRound();

        profiler.startExtraRound(EXTRA_ROUND_INITIALIZATION);
        profiler.startPhase();
        initialize(source, departureTime, target);
        profiler.donePhase(PHASE_INITIALIZATION);
        profiler.startPhase();
        relaxTransfers<true>();
        profiler.donePhase(PHASE_TRANSFERS);
        profiler.doneRound();

        for (size_t i = 0; i < maxRounds; i++) {
            profiler.startRound();
            profiler.startPhase();
            startNewRound();
            profiler.donePhase(PHASE_INITIALIZATION);
            profiler.startPhase();
            collectRoutesServingUpdatedStops();
            profiler.donePhase(PHASE_COLLECT);
            profiler.startPhase();
            scanRoutes();
            profiler.donePhase(PHASE_SCAN);
            if (stopsUpdatedByRoute.empty()) {
                profiler.doneRound();
                break;
            }
            if constexpr (SeparateRouteAndTransferEntries) {
                profiler.startPhase();
                startNewRound();
                profiler.donePhase(PHASE_INITIALIZATION);
            }
            profiler.startPhase();
            relaxTransfers<false>();
            profiler.donePhase(PHASE_TRANSFERS);
            profiler.doneRound();
        }
        profiler.done();
    }

    inline std::vector<Journey> getJourneys() const noexcept {
        return getJourneys(targetStop);
    }

    inline std::vector<Journey> getJourneys(const StopId stop) const noexcept {
        std::vector<Journey> journeys;
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
            getJourney(journeys, i, stop);
        }
        return journeys;
    }

    inline Journey getEarliestJourney(const StopId stop) const noexcept {
        std::vector<Journey> journeys = getJourneys(stop);
        return journeys.empty() ? Journey() : journeys.back();
    }

    inline std::vector<ArrivalLabel> getArrivals() const noexcept {
        return getArrivals(targetStop);
    }

    inline std::vector<ArrivalLabel> getArrivals(const StopId stop) const noexcept {
        AssertMsg(data.isStop(stop), "The StopId " << stop << " does not correspond to any stop!");
        std::vector<ArrivalLabel> labels;
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
            getArrival(labels, i, stop);
        }
        return labels;
    }

    inline std::vector<int> getArrivalTimes() const noexcept {
        return getArrivalTimes(targetStop);
    }

    inline std::vector<int> getArrivalTimes(const StopId stop) const noexcept {
        std::vector<int> arrivalTimes;
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
            getArrivalTime(arrivalTimes, i, stop);
        }
        return arrivalTimes;
    }

    inline bool reachable(const StopId stop) const noexcept {
        return earliestArrival[stop].getArrivalTime() < never;
    }

    inline int getEarliestArrivalTime(const StopId stop) const noexcept {
        return earliestArrival[stop].getArrivalTime();
    }

    inline int getWalkingArrivalTime() const noexcept {
        return sourceDepartureTime + walkingDistance;
    }

    inline int getWalkingTravelTime() const noexcept {
        return walkingDistance;
    }

    inline std::vector<Vertex> getPath(const StopId stop) const {
        return journeyToPath(getJourneys(stop).back());
    }

    inline std::vector<std::string> getRouteDescription(const StopId stop) const {
        return data.journeyToText(getJourneys(stop).back());
    }

    template<bool RESET_CAPACITIES = false>
    inline void clear() noexcept {
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        targetStop = noStop;
        sourceDepartureTime = never;
        walkingDistance = INFTY;
        if constexpr (RESET_CAPACITIES) {
            std::vector<Round>().swap(rounds);
            std::vector<ArrivalTime>(earliestArrival.size(), never).swap(earliestArrival);
        } else {
            rounds.clear();
            Vector::fill(earliestArrival);
        }
    }

    inline void reset() noexcept {
        clear<true>();
    }

    inline const Profiler& getProfiler() const noexcept {
        return profiler;
    }

    inline int getArrivalTime(const StopId stop, const size_t numberOfTrips) const noexcept {
        size_t round = numberOfTrips * RoundFactor;
        if constexpr (SeparateRouteAndTransferEntries) {
            if ((round + 1 < rounds.size()) && (rounds[round + 1][stop].arrivalTime < rounds[round][stop].arrivalTime)) round++;
        }
        AssertMsg(rounds[round][stop].arrivalTime < never, "No label found for stop " << stop << " in round " << round << "!");
        return rounds[round][stop].arrivalTime;
    }

private:
    inline void initialize(const StopId source, const int departureTime, const StopId target) noexcept {
        sourceStop = source;
        targetStop = target;
        sourceDepartureTime = departureTime;
        startNewRound();
        arrivalByRoute(source, sourceDepartureTime);
        currentRound()[source].parent = source;
        currentRound()[source].parentDepartureTime = sourceDepartureTime;
        currentRound()[source].usesRoute = false;
        if constexpr (SeparateRouteAndTransferEntries) startNewRound();
    }

    inline void collectRoutesServingUpdatedStops() noexcept {
        for (const StopId stop : stopsUpdatedByTransfer) {
            AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
            const int arrivalTime = previousRound()[stop].arrivalTime;
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
            AssertMsg(trip[stopIndex].departureTime >= previousRound()[stop].arrivalTime, "Cannot scan a route after the last trip has departed (Route: " << route << ", Stop: " << stop << ", StopIndex: " << stopIndex << ", Time: " << previousRound()[stop].arrivalTime << ", LastDeparture: " << trip[stopIndex].departureTime << ")!");

            StopIndex parentIndex = stopIndex;
            const StopEvent* firstTrip = data.firstTripOfRoute(route);
            while (stopIndex < tripSize - 1) {
                while ((trip > firstTrip) && ((trip - tripSize + stopIndex)->departureTime >= previousRound()[stop].arrivalTime)) {
                    trip -= tripSize;
                    parentIndex = stopIndex;
                }
                stopIndex++;
                stop = stops[stopIndex];
                profiler.countMetric(METRIC_ROUTE_SEGMENTS);
                if (arrivalByRoute(stop, trip[stopIndex].arrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[stop];
                    label.parent = stops[parentIndex];
                    label.parentDepartureTime = trip[parentIndex].departureTime;
                    label.usesRoute = true;
                    label.routeId = route;
                }
            }
        }
    }

    template<bool INITIAL_TRANSFERS = false>
    inline void relaxTransfers() noexcept {
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        for (const StopId stop : stopsUpdatedByRoute) {
            const int earliestArrivalTime = SeparateRouteAndTransferEntries ? previousRound()[stop].arrivalTime : currentRound()[stop].arrivalTime;
            for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
                if constexpr (INITIAL_TRANSFERS && PreventDirectWalking) {
                    if (data.transferGraph.get(ToVertex, edge) == targetStop) {
                        walkingDistance = data.transferGraph.get(TravelTime, edge);
                        continue;
                    }
                }
                profiler.countMetric(METRIC_EDGES);
                const int arrivalTime = earliestArrivalTime + data.transferGraph.get(TravelTime, edge);
                AssertMsg(data.isStop(data.transferGraph.get(ToVertex, edge)), "Graph contains edges to non stop vertices!");
                const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
                if (arrivalByTransfer(toStop, arrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[toStop];
                    label.parent = stop;
                    label.parentDepartureTime = earliestArrivalTime;
                    label.usesRoute = false;
                    label.transferId = edge;
                }
            }
            if constexpr (SeparateRouteAndTransferEntries) {
                const int arrivalTime = earliestArrivalTime + getMinTransferTime<INITIAL_TRANSFERS>(stop);
                if (arrivalByTransfer(stop, arrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[stop];
                    label.parent = stop;
                    label.parentDepartureTime = earliestArrivalTime;
                    label.usesRoute = false;
                }
            } else {
                stopsUpdatedByTransfer.insert(stop);
            }
        }
    }

    template<bool IGNORE_MIN_TRANSFER_TIMES>
    inline int getMinTransferTime(const StopId stop) const noexcept {
        if constexpr (IGNORE_MIN_TRANSFER_TIMES | !UseMinTransferTimes) {
            suppressUnusedParameterWarning(stop);
            return 0;
        } else {
            return data.stopData[stop].minTransferTime;
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
        rounds.emplace_back(data.numberOfStops());
    }

    inline bool arrivalByRoute(const StopId stop, const int time) noexcept {
        AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
        if constexpr (TargetPruning) if (earliestArrival[targetStop].getArrivalTimeByRoute() <= time) return false;
        if (earliestArrival[stop].getArrivalTimeByRoute() <= time) return false;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        currentRound()[stop].arrivalTime = time;
        earliestArrival[stop].setArrivalTimeByRoute(time);
        stopsUpdatedByRoute.insert(stop);
        return true;
    }

    inline bool arrivalByTransfer(const StopId stop, const int time) noexcept {
        AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
        if constexpr (TargetPruning) if (earliestArrival[targetStop].getArrivalTimeByTransfer() <= time) return false;
        if (earliestArrival[stop].getArrivalTimeByTransfer() <= time) return false;
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        currentRound()[stop].arrivalTime = time;
        earliestArrival[stop].setArrivalTimeByTransfer(time);
        stopsUpdatedByTransfer.insert(stop);
        return true;
    }

    inline void getJourney(std::vector<Journey>& journeys, size_t round, StopId stop) const noexcept {
        if constexpr (SeparateRouteAndTransferEntries) {
            if ((round + 1 < rounds.size()) && (rounds[round + 1][stop].arrivalTime < rounds[round][stop].arrivalTime)) round++;
        }
        if (rounds[round][stop].arrivalTime >= (journeys.empty() ? never : journeys.back().back().arrivalTime)) return;
        Journey journey;
        do {
            AssertMsg(round != size_t(-1), "Backtracking parent pointers did not pass through the source stop!");
            const EarliestArrivalLabel& label = rounds[round][stop];
            journey.emplace_back(label.parent, stop, label.parentDepartureTime, label.arrivalTime, label.usesRoute, label.routeId);
            stop = label.parent;
            if constexpr (SeparateRouteAndTransferEntries) {
                round--;
            } else {
                if (label.usesRoute) round--;
            }
        } while (journey.back().from != sourceStop);
        journeys.emplace_back(Vector::reverse(journey));
    }

    inline void getArrival(std::vector<ArrivalLabel>& labels, size_t round, const StopId stop) const noexcept {
        if constexpr (SeparateRouteAndTransferEntries) {
            if ((round + 1 < rounds.size()) && (rounds[round + 1][stop].arrivalTime < rounds[round][stop].arrivalTime)) round++;
        }
        if (rounds[round][stop].arrivalTime >= (labels.empty() ? never : labels.back().arrivalTime)) return;
        labels.emplace_back(rounds[round][stop].arrivalTime, round / RoundFactor);
    }

    inline void getArrivalTime(std::vector<int>& labels, size_t round, const StopId stop) const noexcept {
        if constexpr (SeparateRouteAndTransferEntries) {
            if ((round + 1 < rounds.size()) && (rounds[round + 1][stop].arrivalTime < rounds[round][stop].arrivalTime)) round++;
        }
        labels.emplace_back(std::min(rounds[round][stop].arrivalTime, (labels.empty()) ? (never) : (labels.back())));
    }

private:
    const Data& data;

    std::vector<Round> rounds;

    std::vector<ArrivalTime> earliestArrival;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    StopId sourceStop;
    StopId targetStop;
    int sourceDepartureTime;
    int walkingDistance;

    Profiler profiler;

};

}
