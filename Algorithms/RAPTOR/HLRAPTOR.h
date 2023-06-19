#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "InitialTransfers.h"

#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../DataStructures/RAPTOR/Entities/EarliestArrivalTime.h"
#include "../../DataStructures/Container/Set.h"
#include "../../DataStructures/Container/Map.h"

#include "Profiler.h"

namespace RAPTOR {

template<typename PROFILER = NoProfiler>
class HLRAPTOR {

public:
    using Profiler = PROFILER;
    using ArrivalTime = EarliestArrivalTime<false>;
    using Type = HLRAPTOR<Profiler>;
    using InitialTransferGraph = TransferGraph;
    using SourceType = Vertex;

private:
    struct EarliestArrivalLabel {
        EarliestArrivalLabel() : arrivalTime(never), parentDepartureTime(never), parent(noVertex), usesRoute(false), routeId(noRouteId) {}
        int arrivalTime;
        int parentDepartureTime;
        Vertex parent;
        bool usesRoute;
        RouteId routeId;
    };
    using Round = std::vector<EarliestArrivalLabel>;

    struct HubParentLabel {
        HubParentLabel() : parent(noVertex), parentDepartureTime(never) {}
        Vertex parent;
        int parentDepartureTime;
    };

public:
    HLRAPTOR(const Data& data, const InitialTransferGraph& outHubGraph, const InitialTransferGraph& inHubGraph, const Profiler& profilerTemplate = Profiler()) :
        data(data),
        outHubs(outHubGraph),
        inHubs(inHubGraph),
        reverseInHubs(inHubGraph),
        transferDistanceToTarget(inHubs.numVertices(), INFTY),
        hubParentLabels(inHubs.numVertices()),
        earliestArrival(inHubs.numVertices()),
        stopsUpdatedByRoute(data.numberOfStops() + 1),
        stopsUpdatedByTransfer(data.numberOfStops() + 1),
        updatedHubs(inHubs.numVertices()),
        routesServingUpdatedStops(data.numberOfRoutes()),
        sourceVertex(noVertex),
        targetVertex(noVertex),
        targetStop(noStop),
        lastTarget(Vertex(0)),
        sourceDepartureTime(never),
        profiler(profilerTemplate) {
        AssertMsg(data.hasImplicitBufferTimes(), "Departure buffer times have to be implicit!");
        profiler.registerExtraRounds({EXTRA_ROUND_CLEAR, EXTRA_ROUND_INITIALIZATION});
        profiler.registerPhases({PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS});
        profiler.registerMetrics({METRIC_ROUTES, METRIC_ROUTE_SEGMENTS, METRIC_VERTICES, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
        outHubs.sortEdges(TravelTime);
        inHubs.sortEdges(TravelTime);
        DynamicTransferGraph tempGraph;
        Graph::copy(reverseInHubs, tempGraph);
        for (const Vertex vertex : tempGraph.vertices()) {
            if (data.isStop(vertex)) continue;
            tempGraph.deleteAllOutgoingEdges(vertex);
        }
        tempGraph.revert();
        Graph::move(std::move(tempGraph), reverseInHubs);
        reverseInHubs.sortEdges(TravelTime);
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target, const size_t maxRounds = INFTY) noexcept {
        profiler.start();
        profiler.startExtraRound(EXTRA_ROUND_CLEAR);
        clear();
        profiler.doneRound();

        profiler.startExtraRound(EXTRA_ROUND_INITIALIZATION);
        profiler.startPhase();
        initialize(source, departureTime, target);
        profiler.donePhase(PHASE_INITIALIZATION);
        profiler.startPhase();
        relaxInitialTransfers(departureTime);
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
            profiler.startPhase();
            relaxIntermediateTransfers();
            profiler.donePhase(PHASE_TRANSFERS);
            profiler.doneRound();
        }
        profiler.done();
    }

    inline std::vector<Journey> getJourneys() const noexcept {
        return getJourneys(targetStop);
    }

    inline std::vector<Journey> getJourneys(const Vertex vertex) const noexcept {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        std::vector<Journey> journeys;
        for (size_t i = 0; i < rounds.size(); i++) {
            getJourney(journeys, i, target);
        }
        return journeys;
    }

    inline Journey getEarliestJourney(const Vertex vertex) const noexcept {
        std::vector<Journey> journeys = getJourneys(vertex);
        return journeys.empty() ? Journey() : journeys.back();
    }

    inline std::vector<ArrivalLabel> getArrivals() const noexcept {
        return getArrivals(targetStop);
    }

    inline std::vector<ArrivalLabel> getArrivals(const Vertex vertex) const noexcept {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        std::vector<ArrivalLabel> labels;
        for (size_t i = 0; i < rounds.size(); i++) {
            getArrival(labels, i, target);
        }
        return labels;
    }

    inline std::vector<int> getArrivalTimes() const noexcept {
        return getArrivalTimes(targetStop);
    }

    inline std::vector<int> getArrivalTimes(const Vertex vertex) const noexcept {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        std::vector<int> arrivalTimes;
        for (size_t i = 0; i < rounds.size(); i++) {
            getArrivalTime(arrivalTimes, i, target);
        }
        return arrivalTimes;
    }

    inline bool reachable(const Vertex vertex) const noexcept {
        return earliestArrival[vertex] < never;
    }

    inline int getEarliestArrivalTime(const Vertex vertex) const noexcept {
        return earliestArrival[vertex];
    }

    inline int getEarliestArrivalTime() const noexcept {
        return getEarliestArrivalTime(targetVertex);
    }

    inline int getEarliestArrivalNumberOfTrips() const noexcept {
        const int eat = getEarliestArrivalTime();
        for (size_t i = rounds.size() - 1; i < rounds.size(); i--) {
            if (rounds[i][targetStop].arrivalTime == eat) return i;
        }
        return -1;
    }

    inline std::vector<Vertex> getPath(const Vertex vertex) const noexcept {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        return journeyToPath(getJourneys(target).back());
    }

    inline std::vector<Vertex> getPath() const noexcept {
        return getPath(targetVertex);
    }

    inline std::vector<std::string> getRouteDescription(const Vertex vertex) const noexcept {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        return data.journeyToText(getJourneys(target).back());
    }

    inline Profiler& getProfiler() noexcept {
        return profiler;
    }

    inline int getArrivalTime(const Vertex vertex, const size_t numberOfTrips) const noexcept {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        AssertMsg(rounds[numberOfTrips][target].arrivalTime < never, "No label found for stop " << target << " in round " << numberOfTrips << "!");
        return rounds[numberOfTrips][target].arrivalTime;
    }

    template<bool RESET_CAPACITIES = false>
    inline void clear() noexcept {
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
        updatedHubs.clear();
        routesServingUpdatedStops.clear();
        targetStop = StopId(data.numberOfStops());
        sourceDepartureTime = never;
        if constexpr (RESET_CAPACITIES) {
            std::vector<Round>().swap(rounds);
            std::vector<HubParentLabel>(hubParentLabels.size()).swap(hubParentLabels);
            std::vector<int>(earliestArrival.size(), never).swap(earliestArrival);
        } else {
            rounds.clear();
            Vector::fill(hubParentLabels);
            Vector::fill(earliestArrival, never);
        }
    }

    inline void reset() noexcept {
        clear<true>();
    }

private:
    inline void initialize(const Vertex source, const int departureTime, const Vertex target) noexcept {
        sourceVertex = source;
        targetVertex = target;
        if (data.isStop(target)) {
            targetStop = StopId(target);
        }
        sourceDepartureTime = departureTime;
        startNewRound();
        if (data.isStop(source)) {
            arrivalByRoute(StopId(source), sourceDepartureTime);
            currentRound()[source].parent = source;
            currentRound()[source].parentDepartureTime = sourceDepartureTime;
            currentRound()[source].usesRoute = false;
            stopsUpdatedByTransfer.insert(StopId(source));
        }
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

    inline void relaxInitialTransfers(const int sourceDepartureTime) noexcept {
        updatedHubs.clear();

        transferDistanceToTarget[lastTarget] = INFTY;
        for (const Edge edge : inHubs.edgesFrom(lastTarget)) {
            transferDistanceToTarget[inHubs.get(ToVertex, edge)] = INFTY;
        }
        transferDistanceToTarget[targetVertex] = 0;
        for (const Edge edge : inHubs.edgesFrom(targetVertex)) {
            transferDistanceToTarget[inHubs.get(ToVertex, edge)] = inHubs.get(TravelTime, edge);
        }
        lastTarget = targetVertex;

        earliestArrival[sourceVertex] = sourceDepartureTime;
        hubParentLabels[sourceVertex].parent = sourceVertex;
        hubParentLabels[sourceVertex].parentDepartureTime = sourceDepartureTime;
        updatedHubs.insert(sourceVertex);

        for (const Edge edge : outHubs.edgesFrom(sourceVertex)) {
            const Vertex hub = outHubs.get(ToVertex, edge);
            profiler.countMetric(METRIC_EDGES);
            const int arrivalTime = sourceDepartureTime + outHubs.get(TravelTime, edge);
            if (earliestArrival[targetVertex] <= arrivalTime) break;
            arrivalAtHub(hub, arrivalTime, sourceVertex, sourceDepartureTime);
        }

        relaxTransfersFromHubs();
    }

    inline void relaxIntermediateTransfers() noexcept {
        stopsUpdatedByTransfer.clear();
        updatedHubs.clear();

        for (const StopId stop : stopsUpdatedByRoute) {
            const int earliestArrivalTime = currentRound()[stop].arrivalTime;
            hubParentLabels[stop].parent = stop;
            hubParentLabels[stop].parentDepartureTime = earliestArrivalTime;
            updatedHubs.insert(stop);
            stopsUpdatedByTransfer.insert(stop);

            for (const Edge edge : outHubs.edgesFrom(stop)) {
                const Vertex hub = outHubs.get(ToVertex, edge);
                profiler.countMetric(METRIC_EDGES);
                const int arrivalTime = earliestArrivalTime + outHubs.get(TravelTime, edge);
                if (earliestArrival[targetVertex] <= arrivalTime) break;
                arrivalAtHub(hub, arrivalTime, stop, earliestArrivalTime);
            }
        }

        relaxTransfersFromHubs();
    }

    inline void relaxTransfersFromHubs() noexcept {
        routesServingUpdatedStops.clear();
        for (const Vertex hub : updatedHubs) {
            for (const Edge edge : reverseInHubs.edgesFrom(hub)) {
                const StopId toStop = StopId(reverseInHubs.get(ToVertex, edge));
                AssertMsg(data.isStop(toStop), "Hubs have edges to non-stop vertices!");
                profiler.countMetric(METRIC_EDGES);
                const int arrivalTime = earliestArrival[hub] + reverseInHubs.get(TravelTime, edge);
                if (earliestArrival[targetVertex] <= arrivalTime) break;
                arrivalByTransfer(toStop, arrivalTime, hubParentLabels[hub].parent, hubParentLabels[hub].parentDepartureTime);
            }
            if (transferDistanceToTarget[hub] != INFTY) {
                profiler.countMetric(METRIC_EDGES);
                const int arrivalTime = earliestArrival[hub] + transferDistanceToTarget[hub];
                arrivalByTransfer(targetVertex, arrivalTime, hubParentLabels[hub].parent, hubParentLabels[hub].parentDepartureTime);
            }
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
        rounds.emplace_back(data.numberOfStops() + 1);
    }

    inline bool arrivalByRoute(const StopId stop, const int time) noexcept {
        AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
        if (earliestArrival[targetVertex] <= time) return false;
        if (earliestArrival[stop] <= time) return false;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        earliestArrival[stop] = time;
        currentRound()[stop].arrivalTime = time;
        stopsUpdatedByRoute.insert(stop);
        return true;
    }

    inline void arrivalAtHub(const Vertex hub, const int time, const Vertex parent, const int parentDepartureTime) noexcept {
        if (earliestArrival[hub] <= time) return;
        profiler.countMetric(METRIC_VERTICES);
        earliestArrival[hub] = time;
        hubParentLabels[hub].parent = parent;
        hubParentLabels[hub].parentDepartureTime = parentDepartureTime;
        updatedHubs.insert(hub);
        if (data.isStop(hub) || hub == targetVertex) {
            profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
            const StopId hubStop = (hub == targetVertex) ? targetStop : StopId(hub);
            currentRound()[hubStop].arrivalTime = time;
            currentRound()[hubStop].parent = parent;
            currentRound()[hubStop].parentDepartureTime = parentDepartureTime;
            currentRound()[hubStop].usesRoute = false;
            if (data.isStop(hub)) stopsUpdatedByTransfer.insert(hubStop);
        }
    }

    inline void arrivalByTransfer(const Vertex vertex, const int time, const Vertex parent, const int parentDepartureTime) noexcept {
        AssertMsg(data.isStop(vertex) || vertex == targetVertex, "Stop " << vertex << " is out of range!");
        if (earliestArrival[vertex] <= time) return;
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        earliestArrival[vertex] = time;
        const StopId stop = (vertex == targetVertex) ? targetStop : StopId(vertex);
        currentRound()[stop].arrivalTime = time;
        currentRound()[stop].parent = parent;
        currentRound()[stop].parentDepartureTime = parentDepartureTime;
        currentRound()[stop].usesRoute = false;
        if (data.isStop(stop)) stopsUpdatedByTransfer.insert(stop);
    }

    inline void getJourney(std::vector<Journey>& journeys, size_t round, StopId stop) const noexcept {
        if (rounds[round][stop].arrivalTime >= (journeys.empty() ? never : journeys.back().back().arrivalTime)) return;
        Journey journey;
        do {
            AssertMsg(round != size_t(-1), "Backtracking parent pointers did not pass through the source stop!");
            const EarliestArrivalLabel& label = rounds[round][stop];
            journey.emplace_back(label.parent, stop, label.parentDepartureTime, label.arrivalTime, label.usesRoute, label.routeId);
            AssertMsg(data.isStop(label.parent) || label.parent == sourceVertex, "Backtracking parent pointers reached a vertex (" << label.parent << ")!");
            stop = StopId(label.parent);
            if (label.usesRoute) round--;
        } while (journey.back().from != sourceVertex);
        journeys.emplace_back(Vector::reverse(journey));
    }

    inline void getArrival(std::vector<ArrivalLabel>& labels, size_t round, const StopId stop) const noexcept {
        if (rounds[round][stop].arrivalTime >= (labels.empty() ? never : labels.back().arrivalTime)) return;
        labels.emplace_back(rounds[round][stop].arrivalTime, round);
    }

    inline void getArrivalTime(std::vector<int>& labels, size_t round, const StopId stop) const noexcept {
        labels.emplace_back(std::min(rounds[round][stop].arrivalTime, (labels.empty()) ? (never) : (labels.back())));
    }

private:
    const Data& data;
    TransferGraph outHubs;
    TransferGraph inHubs;
    TransferGraph reverseInHubs;

    std::vector<int> transferDistanceToTarget;

    std::vector<Round> rounds;
    std::vector<HubParentLabel> hubParentLabels;

    std::vector<int> earliestArrival;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedSet<false, Vertex> updatedHubs;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    Vertex sourceVertex;
    Vertex targetVertex;
    StopId targetStop;
    Vertex lastTarget;
    int sourceDepartureTime;

    Profiler profiler;

};

}
