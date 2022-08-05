#pragma once

#include <vector>

#include "../../Helpers/Vector/Vector.h"

#include "InitialTransfers.h"
#include "Profiler.h"

#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../DataStructures/RAPTOR/Entities/Bags.h"
#include "../../DataStructures/Container/Set.h"
#include "../../DataStructures/Container/Map.h"
#include "../../DataStructures/Container/ExternalKHeap.h"

namespace RAPTOR {

template<bool TARGET_PRUNING, typename PROFILER = NoProfiler>
class MCR {

public:
    static constexpr bool TargetPruning = TARGET_PRUNING;
    using Profiler = PROFILER;
    using Type = MCR<TargetPruning, Profiler>;
    using InitialTransferGraph = CHGraph;
    using SourceType = Vertex;

public:
    struct Label {
        Label() : arrivalTime(never), walkingDistance(INFTY), parentStop(noStop), parentIndex(-1), parentDepartureTime(never), routeId(noRouteId) {}

        template<typename OTHER_LABEL>
        Label(const OTHER_LABEL& parentLabel, const int parentDepartureTime) :
            arrivalTime(parentLabel.arrivalTime),
            walkingDistance(parentLabel.walkingDistance),
            parentStop(parentLabel.parentStop),
            parentIndex(parentLabel.parentIndex),
            parentDepartureTime(parentDepartureTime),
            transferId(noEdge) {
        }

        Label(const Label& parentLabel, const StopId stop, const size_t parentIndex) :
            arrivalTime(parentLabel.arrivalTime),
            walkingDistance(parentLabel.walkingDistance),
            parentStop(stop),
            parentIndex(parentIndex),
            parentDepartureTime(parentLabel.arrivalTime),
            transferId(noEdge) {
        }

        Label(const int departureTime, const StopId sourceStop) :
            arrivalTime(departureTime),
            walkingDistance(0),
            parentStop(sourceStop),
            parentIndex(-1),
            parentDepartureTime(departureTime),
            routeId(noRouteId) {
        }

        int arrivalTime;
        int walkingDistance;

        StopId parentStop;
        size_t parentIndex;
        int parentDepartureTime;
        union {
            RouteId routeId;
            Edge transferId;
        };

        inline bool dominates(const Label& other) const noexcept {
            return arrivalTime <= other.arrivalTime && walkingDistance <= other.walkingDistance;
        }
    };

    struct RouteLabel {
        const StopEvent* trip;
        int walkingDistance;
        StopIndex parentStop;
        size_t parentIndex;

        inline bool dominates(const RouteLabel& other) const noexcept {
            return trip <= other.trip && walkingDistance <= other.walkingDistance;
        }
    };

    struct DijkstraLabel {
        DijkstraLabel() : arrivalTime(never), walkingDistance(INFTY), parentStop(noStop), parentIndex(-1) {}

        template<typename LABEL>
        DijkstraLabel(const LABEL& parentLabel, const int travelTime = 0) :
            arrivalTime(parentLabel.arrivalTime + travelTime),
            walkingDistance(parentLabel.walkingDistance + travelTime),
            parentStop(parentLabel.parentStop),
            parentIndex(parentLabel.parentIndex) {
        }

        DijkstraLabel(const Label& parentLabel, const StopId parentStop, const size_t parentIndex, const int travelTime = 0) :
            arrivalTime(parentLabel.arrivalTime + travelTime),
            walkingDistance(parentLabel.walkingDistance + travelTime),
            parentStop(parentStop),
            parentIndex(parentIndex) {
        }

        DijkstraLabel(const int departureTime, const StopId sourceStop) :
            arrivalTime(departureTime),
            walkingDistance(0),
            parentStop(sourceStop),
            parentIndex(0) {
        }

        inline int getKey() const noexcept {
            return arrivalTime + walkingDistance;
        }

        inline bool hasSmallerKey(const DijkstraLabel* const other) const noexcept {
            return getKey() < other->getKey();
        }

        template<typename OTHER_LABEL>
        inline bool dominates(const OTHER_LABEL& other) const noexcept {
            return arrivalTime <= other.arrivalTime && walkingDistance <= other.walkingDistance;
        }

        int arrivalTime;
        int walkingDistance;
        StopId parentStop;
        size_t parentIndex;
    };

    using BagType = Bag<Label>;
    using Round = std::vector<BagType>;
    using RouteBagType = RouteBag<RouteLabel>;
    using DijkstraBagType = DijkstraBag<DijkstraLabel>;

public:
    MCR(const Data& data, const CH::CH& chData, const Profiler& profilerTemplate = Profiler()) :
        data(data),
        initialTransfers(chData, FORWARD, data.numberOfStops()),
        stopsUpdatedByRoute(data.numberOfStops() + 1),
        stopsUpdatedByTransfer(data.numberOfStops() + 1),
        routesServingUpdatedStops(data.numberOfRoutes()),
        sourceVertex(noVertex),
        targetVertex(noVertex),
        targetStop(noStop),
        sourceDepartureTime(intMax),
        dijkstraBags(data.transferGraph.numVertices()),
        profiler(profilerTemplate) {
        AssertMsg(data.hasImplicitBufferTimes(), "Departure buffer times have to be implicit!");
        profiler.registerExtraRounds({EXTRA_ROUND_CLEAR, EXTRA_ROUND_INITIALIZATION});
        profiler.registerPhases({PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS});
        profiler.registerMetrics({METRIC_ROUTES, METRIC_ROUTE_SEGMENTS, METRIC_VERTICES, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
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
        profiler.doneRound();
        profiler.startPhase();
        relaxInitialTransfers();
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
            startNewRound();
            profiler.donePhase(PHASE_INITIALIZATION);
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
        for (size_t round = 0; round < rounds.size(); round += 2) {
            const size_t trueRound = std::min(round + 1, rounds.size() - 1);
            for (size_t i = 0; i < rounds[trueRound][target].size(); i++) {
                getJourney(journeys, trueRound, target, i);
            }
        }
        return journeys;
    }

    inline std::vector<WalkingParetoLabel> getResults() const noexcept {
        return getResults(targetStop);
    }

    inline std::vector<WalkingParetoLabel> getResults(const StopId stop) const noexcept {
        std::vector<WalkingParetoLabel> result;
        for (size_t round = 0; round < rounds.size(); round += 2) {
            const size_t trueRound = std::min(round + 1, rounds.size() - 1);
            for (const Label& label : rounds[trueRound][stop].labels) {
                result.emplace_back(label, round / 2);
            }
        }
        return result;
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
            std::vector<DijkstraBagType>(dijkstraBags.size()).swap(dijkstraBags);
        } else {
            rounds.clear();
            Vector::fill(dijkstraBags);
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
            profiler.countMetric(METRIC_STOPS_BY_TRIP);
            currentRound()[sourceVertex].mergeUndominated(Label(sourceDepartureTime, StopId(sourceVertex)));
        }
        startNewRound();
    }

    inline void collectRoutesServingUpdatedStops() noexcept {
        for (const StopId stop : stopsUpdatedByTransfer) {
            for (const RouteSegment& route : data.routesContainingStop(stop)) {
                AssertMsg(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                AssertMsg(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId)) continue;
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
            StopId stop = stops[stopIndex];

            const StopEvent* firstTrip = data.firstTripOfRoute(route);
            const StopEvent* lastTrip = data.lastTripOfRoute(route);

            RouteBagType routeBag;

            while (stopIndex < tripSize - 1) {
                for (size_t i = 0; i < previousRound()[stop].size(); i++) {
                    const Label& label = previousRound()[stop][i];
                    const StopEvent* trip = firstTrip;
                    while ((trip < lastTrip) && (trip[stopIndex].departureTime < label.arrivalTime)) {
                        trip += tripSize;
                    }
                    if (trip[stopIndex].departureTime < label.arrivalTime) continue;

                    RouteLabel newLabel;
                    newLabel.trip = trip;
                    newLabel.walkingDistance = label.walkingDistance;
                    newLabel.parentStop = stopIndex;
                    newLabel.parentIndex = i;
                    routeBag.merge(newLabel);
                }
                stopIndex++;
                stop = stops[stopIndex];
                profiler.countMetric(METRIC_ROUTE_SEGMENTS);
                for (const RouteLabel& label : routeBag.labels) {
                    Label newLabel;
                    newLabel.arrivalTime = label.trip[stopIndex].arrivalTime;
                    newLabel.walkingDistance = label.walkingDistance;
                    newLabel.parentStop = stops[label.parentStop];
                    newLabel.parentIndex = label.parentIndex;
                    newLabel.parentDepartureTime = label.trip[label.parentStop].departureTime;
                    newLabel.routeId = route;
                    arrivalByRoute(stop, newLabel);
                }
            }
        }
    }

    inline void relaxInitialTransfers() noexcept {
        routesServingUpdatedStops.clear();
        stopsUpdatedByTransfer.clear();
        DijkstraLabel sourceLabel(sourceDepartureTime, StopId(sourceVertex));
        dijkstraBags[sourceVertex].initialize(sourceLabel);
        if (data.isStop(sourceVertex)) {
            arrivalByTransfer(StopId(sourceVertex), sourceLabel);
        }
        initialTransfers.template run<true>(sourceVertex, targetVertex);
        for (const Vertex stop : initialTransfers.getForwardPOIs()) {
            if (stop == targetStop || stop == sourceVertex) continue;
            AssertMsg(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
            AssertMsg(initialTransfers.getForwardDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
            DijkstraLabel dijkstraLabel(sourceLabel, initialTransfers.getForwardDistance(stop));
            dijkstraBags[stop].initialize(dijkstraLabel);
            arrivalByTransfer(StopId(stop), dijkstraLabel);
        }
        if (initialTransfers.getDistance() != INFTY) {
            DijkstraLabel targetLabel(sourceLabel, initialTransfers.getDistance());
            dijkstraBags[targetVertex].initialize(targetLabel);
            arrivalByTransfer(targetStop, targetLabel);
        }
    }

    inline void relaxIntermediateTransfers() noexcept {
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        AssertMsg(queue.empty(), "Queue still has " << queue.size() << " elements!");
        for (const StopId stop : stopsUpdatedByRoute) {
            stopsUpdatedByTransfer.insert(stop);
            if (initialTransfers.getBackwardDistance(stop) == INFTY) continue;
            const BagType& bag = previousRound()[stop];
            for (size_t i = 0; i < bag.size(); i++) {
                DijkstraLabel targetLabel(bag[i], stop, i, initialTransfers.getBackwardDistance(stop));
                if (dijkstraBags[targetVertex].template merge<false>(targetLabel)) {
                    arrivalByTransfer(targetStop, targetLabel);
                }
        }
        }
        for (const StopId stop : stopsUpdatedByRoute) {
            const BagType& bag = previousRound()[stop];
            for (size_t i = 0; i < bag.size(); i++) {
                arrivalByEdge(stop, DijkstraLabel(bag[i], stop, i));
            }
        }
        dijkstra();
    }

    inline void dijkstra() noexcept {
        while (!queue.empty()) {
            DijkstraBagType* uBag = queue.extractFront();
            const DijkstraLabel& uLabel = uBag->extractFront();
            if (!uBag->heapEmpty()) queue.update(uBag);
            const Vertex u = Vertex(uBag - &(dijkstraBags[0]));
            for (Edge edge : data.transferGraph.edgesFrom(u)) {
                const Vertex v = data.transferGraph.get(ToVertex, edge);
                if (v == targetVertex || v == uLabel.parentStop) continue;
                profiler.countMetric(METRIC_EDGES);
                DijkstraLabel vLabel(uLabel, data.transferGraph.get(TravelTime, edge));
                arrivalByEdge(v, vLabel);
            }
            if (data.isStop(u)) {
                arrivalByTransfer(StopId(u), uLabel);
            }
            profiler.countMetric(METRIC_VERTICES);
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

    inline void arrivalByRoute(const StopId stop, const Label& label) noexcept {
        AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
        if constexpr (TargetPruning) {
            if (dijkstraBags[targetVertex].dominates(label)) return;
            if (currentRound()[targetStop].dominates(label)) return;
        }
        if (dijkstraBags[stop].dominates(label)) return;
        if (!currentRound()[stop].merge(label)) return;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        stopsUpdatedByRoute.insert(stop);
    }

    inline bool arrivalByEdge(const Vertex vertex, const DijkstraLabel& label) noexcept {
        AssertMsg(label.arrivalTime >= sourceDepartureTime, "Arriving by route BEFORE departing from the source (source departure time: " << String::secToTime(sourceDepartureTime) << " [" << sourceDepartureTime << "], arrival time: " << String::secToTime(label.arrivalTime) << " [" << label.arrivalTime << "])!");
        if constexpr (TargetPruning) if (dijkstraBags[targetVertex].dominates(label)) return false;
        if (!dijkstraBags[vertex].template merge<true>(label)) return false;
        queue.update(&dijkstraBags[vertex]);
        return true;
    }

    inline void arrivalByTransfer(const StopId stop, const DijkstraLabel& label) noexcept {
        AssertMsg(data.isStop(stop) || stop == targetStop, "Stop " << stop << " is out of range!");
        AssertMsg(label.arrivalTime >= sourceDepartureTime, "Arriving by route BEFORE departing from the source (source departure time: " << String::secToTime(sourceDepartureTime) << " [" << sourceDepartureTime << "], arrival time: " << String::secToTime(label.arrivalTime) << " [" << label.arrivalTime << "])!");
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        const int parentDepartureTime = (label.parentStop == sourceVertex) ? sourceDepartureTime : previousRound()[label.parentStop][label.parentIndex].arrivalTime;
        currentRound()[stop].mergeUndominated(Label(label, parentDepartureTime));
        if (data.isStop(stop)) stopsUpdatedByTransfer.insert(stop);
    }

    inline void getJourney(std::vector<Journey>& journeys, size_t round, StopId stop, size_t index) const noexcept {
        Journey journey;
        do {
            AssertMsg(round != size_t(-1), "Backtracking parent pointers did not pass through the source stop!");
            const Label& label = rounds[round][stop][index];
            journey.emplace_back(label.parentStop, stop, label.parentDepartureTime, label.arrivalTime, round % 2 == 0, label.routeId);
            stop = label.parentStop;
            index = label.parentIndex;
            round--;
        } while (journey.back().from != sourceVertex);
        journeys.emplace_back(Vector::reverse(journey));
    }

private:
    const Data& data;

    CoreCHInitialTransfers initialTransfers;

    std::vector<Round> rounds;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    Vertex sourceVertex;
    Vertex targetVertex;
    StopId targetStop;
    int sourceDepartureTime;

    std::vector<DijkstraBagType> dijkstraBags;
    ExternalKHeap<2, DijkstraBagType> queue;

    Profiler profiler;

};

}
