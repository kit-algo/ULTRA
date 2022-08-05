#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../Helpers/Vector/Vector.h"

#include "InitialTransfers.h"
#include "Profiler.h"

#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../DataStructures/Intermediate/Data.h"
#include "../../DataStructures/Container/Set.h"
#include "../../DataStructures/Container/Map.h"
#include "../../DataStructures/Container/ExternalKHeap.h"

namespace RAPTOR {

template<typename INITIAL_TRANSFERS, typename PROFILER, bool TARGET_PRUNING = true, bool USE_MIN_TRANSFER_TIMES = false, bool PREVENT_DIRECT_WALKING = false>
class DijkstraRAPTOR {

public:
    using InitialTransferType = INITIAL_TRANSFERS;
    using InitialTransferGraph = typename InitialTransferType::Graph;
    using Profiler = PROFILER;
    static constexpr bool TargetPruning = TARGET_PRUNING;
    static constexpr bool UseMinTransferTimes = USE_MIN_TRANSFER_TIMES;
    static constexpr bool PreventDirectWalking = PREVENT_DIRECT_WALKING;
    static constexpr bool SeparateRouteAndTransferEntries = UseMinTransferTimes | PreventDirectWalking;
    static constexpr int RoundFactor = SeparateRouteAndTransferEntries ? 2 : 1;
    using Type = DijkstraRAPTOR<InitialTransferType, Profiler, TargetPruning, UseMinTransferTimes, PreventDirectWalking>;
    using SourceType = Vertex;

public:
    struct EarliestArrivalLabel {
        EarliestArrivalLabel() : arrivalTime(never), parentDepartureTime(never), parent(noVertex), usesRoute(false), routeId(noRouteId) {}
        int arrivalTime;
        int parentDepartureTime;
        Vertex parent;
        bool usesRoute;
        RouteId routeId;
    };
    using Round = std::vector<EarliestArrivalLabel>;

    struct DijkstraLabel : public ExternalKHeapElement {
        DijkstraLabel() : arrivalTime(never), parent(noVertex) {}
        int arrivalTime;
        Vertex parent;
        inline bool hasSmallerKey(const DijkstraLabel* const other) const noexcept {
            return arrivalTime < other->arrivalTime;
        }
    };

public:
    DijkstraRAPTOR(const Data& data, const InitialTransferType initialTransfers, const Profiler& profilerTemplate = Profiler()) :
        data(data),
        initialTransfers(initialTransfers),
        earliestArrivalByRoute(data.numberOfStops() + 1, never),
        stopsUpdatedByRoute(data.numberOfStops() + 1),
        stopsUpdatedByTransfer(data.numberOfStops() + 1),
        routesServingUpdatedStops(data.numberOfRoutes()),
        sourceVertex(noVertex),
        targetVertex(noVertex),
        targetStop(noStop),
        sourceDepartureTime(intMax),
        label(data.transferGraph.numVertices()),
        profiler(profilerTemplate) {
        if constexpr (UseMinTransferTimes) {
            minChangeTimeGraph = data.minChangeTimeGraph();
            AssertMsg(!data.hasImplicitBufferTimes(), "Either min transfer times have to be used OR departure buffer times have to be implicit!");
        } else {
            AssertMsg(data.hasImplicitBufferTimes(), "Either min transfer times have to be used OR departure buffer times have to be implicit!");
        }
        profiler.registerExtraRounds({EXTRA_ROUND_CLEAR, EXTRA_ROUND_INITIALIZATION});
        profiler.registerPhases({PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS});
        profiler.registerMetrics({METRIC_ROUTES, METRIC_ROUTE_SEGMENTS, METRIC_VERTICES, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
    }


    template<typename ATTRIBUTE>
    DijkstraRAPTOR(const Data& data, const InitialTransferGraph& forwardGraph, const InitialTransferGraph& backwardGraph, const ATTRIBUTE weight, const Profiler& profilerTemplate = Profiler()) :
        DijkstraRAPTOR(data, InitialTransferType(forwardGraph, backwardGraph, data.numberOfStops(), weight), profilerTemplate) {
    }

    template<typename T = CHGraph, typename = std::enable_if_t<Meta::Equals<T, CHGraph>() && Meta::Equals<T, InitialTransferGraph>()>>
    DijkstraRAPTOR(const Data& data, const CH::CH& chData, const Profiler& profilerTemplate = Profiler()) :
        DijkstraRAPTOR(data, chData.forward, chData.backward, Weight, profilerTemplate) {
    }

    template<typename T = TransferGraph, typename = std::enable_if_t<Meta::Equals<T, TransferGraph>() && Meta::Equals<T, InitialTransferGraph>()>>
    DijkstraRAPTOR(const Data& data, const TransferGraph& forwardGraph, const TransferGraph& backwardGraph, const Profiler& profilerTemplate = Profiler()) :
        DijkstraRAPTOR(data, forwardGraph, backwardGraph, TravelTime, profilerTemplate) {
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target, const size_t maxRounds = INFTY) noexcept {
        runInitialize(source, departureTime, target);
        runInitialTransfers();
        runRounds(maxRounds);
    }

    inline void runInitialize(const Vertex source, const int departureTime, const Vertex target) noexcept {
        profiler.start();
        profiler.startExtraRound(EXTRA_ROUND_CLEAR);
        clear();
        profiler.doneRound();

        profiler.startExtraRound(EXTRA_ROUND_INITIALIZATION);
        profiler.startPhase();
        initialize(source, departureTime, target);
        profiler.donePhase(PHASE_INITIALIZATION);
        profiler.doneRound();
    }

    inline void runInitialTransfers() noexcept {
        profiler.startExtraRound(EXTRA_ROUND_INITIALIZATION);
        profiler.startPhase();
        relaxInitialTransfers();
        profiler.donePhase(PHASE_TRANSFERS);
        profiler.doneRound();
    }

    inline void runAddSource(const StopId source, const int departureTime) noexcept {
        currentRound()[source].arrivalTime = departureTime;
        stopsUpdatedByTransfer.insert(source);
    }

    inline void runRounds(const size_t maxRounds = INFTY) noexcept {
        profiler.startRound();
        profiler.startPhase();
        startNewRound();
        profiler.donePhase(PHASE_INITIALIZATION);
        collectAndScanRoutes([&](const StopId stop) {
            if constexpr (PreventDirectWalking) {if (stop == targetStop) return never;}
            return sourceDepartureTime + initialTransfers.getForwardDistance(stop);
        });
        for (size_t i = 0; (i < maxRounds) && (!stopsUpdatedByRoute.empty()); i++) {
            if constexpr (SeparateRouteAndTransferEntries) {
                profiler.startPhase();
                startNewRound();
                profiler.donePhase(PHASE_INITIALIZATION);
            }
            profiler.startPhase();
            relaxIntermediateTransfers();
            profiler.donePhase(PHASE_TRANSFERS);
            profiler.doneRound();
            profiler.startRound();
            profiler.startPhase();
            startNewRound();
            profiler.donePhase(PHASE_INITIALIZATION);
            collectAndScanRoutes([&](const StopId stop) {
                return label[stop].arrivalTime;
            });
        }
        profiler.doneRound();
        profiler.done();
    }

    inline std::vector<Journey> getJourneys() const noexcept {
        return getJourneys(targetStop);
    }

    inline std::vector<Journey> getJourneys(const Vertex vertex) const noexcept {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        std::vector<Journey> journeys;
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
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
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
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
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
            getArrivalTime(arrivalTimes, i, target);
        }
        return arrivalTimes;
    }

    inline bool reachable(const Vertex vertex) const noexcept {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        return earliestArrivalByRoute[target] < never || label[targetVertex].arrivalTime < never;
    }

    inline int getEarliestArrivalTime(const Vertex vertex) const noexcept {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        return std::min(earliestArrivalByRoute[target], label[targetVertex].arrivalTime);
    }

    inline int getEarliestArrivalTime() const noexcept {
        return getEarliestArrivalTime(targetVertex);
    }

    inline int getEarliestArrivalNumberOfTrips() const noexcept {
        const int eat = getEarliestArrivalTime();
        for (size_t i = rounds.size() - 1; i < rounds.size(); i -= RoundFactor) {
            if (rounds[i][targetStop].arrivalTime == eat) return i;
        }
        return -1;
    }

    inline int getWalkingArrivalTime() const noexcept {
        return sourceDepartureTime + initialTransfers.getDistance();
    }

    inline int getWalkingArrivalTime(const Vertex vertex) const noexcept {
        return sourceDepartureTime + initialTransfers.getForwardDistance(vertex);
    }

    inline int getWalkingTravelTime() const noexcept {
        return initialTransfers.getDistance();
    }

    inline int getWalkingTravelTime(const Vertex vertex) const noexcept {
        return initialTransfers.getDistance(vertex);
    }

    inline std::vector<Vertex> getPath(const Vertex vertex) const {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        return journeyToPath(getJourneys(target).back());
    }

    inline std::vector<std::string> getRouteDescription(const Vertex vertex) const {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        return data.journeyToText(getJourneys(target).back());
    }

    inline int getArrivalTime(const Vertex vertex, const size_t numberOfTrips) const noexcept {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        size_t round = numberOfTrips * RoundFactor;
        if constexpr (SeparateRouteAndTransferEntries) {
            if ((round + 1 < rounds.size()) && (rounds[round + 1][target].arrivalTime < rounds[round][target].arrivalTime)) round++;
        }
        AssertMsg(rounds[round][target].arrivalTime < never, "No label found for stop " << target << " in round " << round << "!");
        return rounds[round][target].arrivalTime;
    }

    template<bool RESET_CAPACITIES = false>
    inline void clear() noexcept {
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        sourceVertex = noVertex;
        targetVertex = noVertex;
        targetStop = StopId(data.numberOfStops());
        queue.clear();
        if constexpr (RESET_CAPACITIES) {
            std::vector<Round>().swap(rounds);
            std::vector<int>(earliestArrivalByRoute.size(), never).swap(earliestArrivalByRoute);
            std::vector<DijkstraLabel>(label.size()).swap(label);
        } else {
            rounds.clear();
            Vector::fill(earliestArrivalByRoute, never);
            Vector::fill(label, DijkstraLabel());
        }
    }

    inline void reset() noexcept {
        clear<true>();
    }

    inline Profiler& getProfiler() noexcept {
        return profiler;
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
            arrivalByRoute(StopId(source), departureTime);
            currentRound()[source].parent = source;
            currentRound()[source].parentDepartureTime = departureTime;
            currentRound()[source].usesRoute = false;
            if constexpr (!SeparateRouteAndTransferEntries) stopsUpdatedByTransfer.insert(StopId(source));
        }
        if constexpr (SeparateRouteAndTransferEntries) startNewRound();
    }

    template<typename ARRIVAL_TIME>
    inline void collectRoutesServingUpdatedStops(const ARRIVAL_TIME& arrivalTime) noexcept {
        for (const StopId stop : stopsUpdatedByTransfer) {
            for (const RouteSegment& route : data.routesContainingStop(stop)) {
                AssertMsg(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                AssertMsg(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId)) continue;
                if (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < arrivalTime(stop)) continue;
                if (routesServingUpdatedStops.contains(route.routeId)) {
                    routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                }
            }
        }
    }

    template<typename ARRIVAL_TIME>
    inline void scanRoutes(const ARRIVAL_TIME& arrivalTime) noexcept {
        stopsUpdatedByRoute.clear();
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            profiler.countMetric(METRIC_ROUTES);
            StopIndex stopIndex = routesServingUpdatedStops[route];
            const size_t tripSize = data.numberOfStopsInRoute(route);
            AssertMsg(stopIndex < tripSize - 1, "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " << stopIndex << ", TripSize: " << tripSize << ", RoundIndex: " << rounds.size() - 1 << ")!");

            const StopId* stops = data.stopArrayOfRoute(route);
            const StopEvent* trip = data.lastTripOfRoute(route);
            StopId stop = stops[stopIndex];
            AssertMsg(trip[stopIndex].departureTime >= arrivalTime(stop), "Cannot scan a route after the last trip has departed (Route: " << route << ", Stop: " << stop << ", StopIndex: " << stopIndex << ", Time: " << arrivalTime(stop) << ", LastDeparture: " << trip[stopIndex].departureTime << ", RoundIndex: " << rounds.size() - 1 << ")!");

            StopIndex parentIndex = stopIndex;
            const StopEvent* firstTrip = data.firstTripOfRoute(route);
            while (stopIndex < tripSize - 1) {
                while ((trip > firstTrip) && ((trip - tripSize + stopIndex)->departureTime >= arrivalTime(stop))) {
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

    template<typename ARRIVAL_TIME>
    inline void collectAndScanRoutes(const ARRIVAL_TIME& arrivalTime) noexcept {
        profiler.startPhase();
        collectRoutesServingUpdatedStops(arrivalTime);
        profiler.donePhase(PHASE_COLLECT);
        profiler.startPhase();
        scanRoutes(arrivalTime);
        profiler.donePhase(PHASE_SCAN);
    }

    inline void relaxInitialTransfers() noexcept {
        routesServingUpdatedStops.clear();
        stopsUpdatedByTransfer.clear();
        initialTransfers.template run<!PreventDirectWalking>(sourceVertex, targetVertex);
        for (const Vertex stop : initialTransfers.getForwardPOIs()) {
            if (stop == targetStop) continue;
            AssertMsg(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
            AssertMsg(initialTransfers.getForwardDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
            const int arrivalTime = sourceDepartureTime + initialTransfers.getForwardDistance(stop);
            arrivalByTransfer(StopId(stop), arrivalTime, sourceVertex, sourceDepartureTime);
        }
        if constexpr (!PreventDirectWalking) {
            if (initialTransfers.getDistance() != INFTY) {
                const int arrivalTime = sourceDepartureTime + initialTransfers.getDistance();
                arrivalByEdge<false>(label[targetVertex], arrivalTime, sourceVertex);
                arrivalByTransfer(targetStop, arrivalTime, sourceVertex, sourceDepartureTime);
            }
        }
    }

    inline void relaxIntermediateTransfers() noexcept {
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        AssertMsg(queue.empty(), "Queue still has " << queue.size() << " elements!");
        for (const StopId stop : stopsUpdatedByRoute) {
            if (initialTransfers.getBackwardDistance(stop) != INFTY) {
                const int arrivalTime = earliestArrivalByRoute[stop] + initialTransfers.getBackwardDistance(stop);
                if (arrivalByEdge<false>(label[targetVertex], arrivalTime, stop)) {
                    arrivalByTransfer(targetStop, arrivalTime, stop, earliestArrivalByRoute[stop]);
                }
            }
            if constexpr (UseMinTransferTimes) {
                for (Edge edge : data.transferGraph.edgesFrom(stop)) {
                    const Vertex to = data.transferGraph.get(ToVertex, edge);
                    if (to == targetVertex) continue;
                    profiler.countMetric(METRIC_EDGES);
                    arrivalByEdge<true>(label[to], earliestArrivalByRoute[stop] + data.transferGraph.get(TravelTime, edge), stop);
                }
                for (Edge edge : minChangeTimeGraph.edgesFrom(stop)) {
                    const Vertex to = minChangeTimeGraph.get(ToVertex, edge);
                    if (to == targetVertex) continue;
                    profiler.countMetric(METRIC_EDGES);
                    arrivalByEdge<true>(label[to], earliestArrivalByRoute[stop] + minChangeTimeGraph.get(TravelTime, edge), stop);
                }
                arrivalByEdge<true>(label[stop], earliestArrivalByRoute[stop] + data.stopData[stop].minTransferTime, stop);
            } else {
                arrivalByEdge<true>(label[stop], earliestArrivalByRoute[stop], stop);
            }
        }
        dijkstra();
    }

    inline void dijkstra() noexcept {
        DijkstraLabel& targetLabel = label[TargetPruning ? targetVertex : 0];
        while (!queue.empty()) {
            DijkstraLabel* uLabel = queue.extractFront();
            if constexpr (TargetPruning) {if (uLabel->arrivalTime > targetLabel.arrivalTime) break;}
            const Vertex u = Vertex(uLabel - &(label[0]));
            for (Edge edge : data.transferGraph.edgesFrom(u)) {
                const Vertex v = data.transferGraph.get(ToVertex, edge);
                if (v == targetVertex || v == uLabel->parent) continue;
                profiler.countMetric(METRIC_EDGES);
                arrivalByEdge<true>(label[v], uLabel->arrivalTime + data.transferGraph.get(TravelTime, edge), uLabel->parent);
            }
            if constexpr (UseMinTransferTimes) {
                for (Edge edge : minChangeTimeGraph.edgesFrom(u)) {
                    const Vertex v = minChangeTimeGraph.get(ToVertex, edge);
                    profiler.countMetric(METRIC_EDGES);
                    if (v == targetVertex || v == uLabel->parent) continue;
                    arrivalByEdge<true>(label[v], uLabel->arrivalTime + minChangeTimeGraph.get(TravelTime, edge), uLabel->parent);
                }
            }
            if (data.isStop(u)) {
                arrivalByTransfer(StopId(u), uLabel->arrivalTime, uLabel->parent, earliestArrivalByRoute[uLabel->parent]);
            }
            profiler.countMetric(METRIC_VERTICES);
        }
        if (!queue.empty()) {
            queue.clear();
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

    inline bool arrivalByRoute(const StopId stop, const int arrivalTime) noexcept {
        AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
        AssertMsg(arrivalTime >= sourceDepartureTime, "Arriving by route BEFORE departing from the source (source departure time: " << String::secToTime(sourceDepartureTime) << " [" << sourceDepartureTime << "], arrival time: " << String::secToTime(arrivalTime) << " [" << arrivalTime << "], stop: " << stop << ")!");
        if constexpr (TargetPruning) {if (earliestArrivalByRoute[targetStop] <= arrivalTime) return false;}
        if (earliestArrivalByRoute[stop] <= arrivalTime) return false;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        currentRound()[stop].arrivalTime = arrivalTime;
        earliestArrivalByRoute[stop] = arrivalTime;
        stopsUpdatedByRoute.insert(stop);
        return true;
    }

    template<bool ADD_TO_QUEUE>
    inline bool arrivalByEdge(DijkstraLabel& label, const int arrivalTime, const Vertex parent) noexcept {
        AssertMsg(arrivalTime >= sourceDepartureTime, "Arriving by route BEFORE departing from the source (source departure time: " << String::secToTime(sourceDepartureTime) << " [" << sourceDepartureTime << "], arrival time: " << String::secToTime(arrivalTime) << " [" << arrivalTime << "])!");
        if (label.arrivalTime <= arrivalTime) return false;
        label.arrivalTime = arrivalTime;
        label.parent = parent;
        if constexpr (ADD_TO_QUEUE) queue.update(&label);
        return true;
    }

    inline void arrivalByTransfer(const StopId stop, const int arrivalTime, const Vertex parent, const int parentDepartureTime) noexcept {
        AssertMsg(data.isStop(stop) || stop == targetStop, "Stop " << stop << " is out of range!");
        AssertMsg(arrivalTime >= sourceDepartureTime, "Arriving by route BEFORE departing from the source (source departure time: " << String::secToTime(sourceDepartureTime) << " [" << sourceDepartureTime << "], arrival time: " << String::secToTime(arrivalTime) << " [" << arrivalTime << "])!");
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        if (data.isStop(stop)) stopsUpdatedByTransfer.insert(stop);
        EarliestArrivalLabel& label = currentRound()[stop];
        if (label.arrivalTime <= arrivalTime) return;
        label.arrivalTime = arrivalTime;
        label.parent = parent;
        label.parentDepartureTime = parentDepartureTime;
        label.usesRoute = false;
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
            journey.emplace_back(label.parent, (stop == targetStop) ? targetVertex : stop, label.parentDepartureTime, label.arrivalTime, label.usesRoute, label.routeId);
            AssertMsg(data.isStop(label.parent) || label.parent == sourceVertex, "Backtracking parent pointers reached a vertex (" << label.parent << ")!");
            stop = StopId(label.parent);
            if constexpr (SeparateRouteAndTransferEntries) {
                round--;
            } else {
                if (label.usesRoute) round--;
            }
        } while (journey.back().from != sourceVertex);
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

    inline void printRoundsForStop(const StopId stop) const noexcept {
        AssertMsg(data.isStop(stop), stop << " is not a valid stop!");
        std::cout << "Raptor Label for stop " << stop << ":" << std::endl;
        std::cout << std::setw(10) << "Round" << std::setw(14) << "arrivalTime" << std::setw(14) << "parent" << std::endl;
        for (size_t i = 0; i < rounds.size(); i++) {
            std::cout << std::setw(10) << i << std::setw(14) << rounds[i][stop].arrivalTime << std::setw(14) << rounds[i][stop].parent << std::endl;
        }
    }

private:
    const Data& data;
    TransferGraph minChangeTimeGraph;

    InitialTransferType initialTransfers;

    std::vector<Round> rounds;

    std::vector<int> earliestArrivalByRoute;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    Vertex sourceVertex;
    Vertex targetVertex;
    StopId targetStop;
    int sourceDepartureTime;

    std::vector<DijkstraLabel> label;
    ExternalKHeap<2, DijkstraLabel> queue;

    Profiler profiler;

};

}
