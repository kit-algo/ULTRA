#pragma once

#include <vector>

#include "../../../DataStructures/Container/Set.h"
#include "../../../DataStructures/Container/Map.h"
#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/RAPTOR/Entities/Bags.h"

#include "../InitialTransfers.h"
#include "../Profiler.h"

#include "ForwardPruningULTRARAPTOR.h"
#include "BackwardPruningULTRARAPTOR.h"

namespace RAPTOR {

template<typename PROFILER = NoProfiler>
class BoundedULTRAMcRAPTOR {

public:
    using Profiler = PROFILER;
    using Type = BoundedULTRAMcRAPTOR<Profiler>;

private:
    struct Label {
        Label() : arrivalTime(never), walkingDistance(INFTY), parentStop(noStop), parentIndex(-1), parentDepartureTime(never), routeId(noRouteId) {}

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

    struct BestLabel {
        BestLabel() : arrivalTime(never), walkingDistance(INFTY) {}

        BestLabel(const int arrivalTime, const int walkingDistance) :
            arrivalTime(arrivalTime),
            walkingDistance(walkingDistance) {
        }

        template<typename LABEL>
        BestLabel(const LABEL& label) :
            arrivalTime(label.arrivalTime),
            walkingDistance(label.walkingDistance) {
        }

        template<typename LABEL>
        inline bool dominates(const LABEL& other) const noexcept {
            return arrivalTime <= other.arrivalTime && walkingDistance <= other.walkingDistance;
        }

        int arrivalTime;
        int walkingDistance;
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

    using BagType = Bag<Label>;
    using BestBagType = Bag<BestLabel>;
    using Round = std::vector<BagType>;
    using RouteBagType = RouteBag<RouteLabel>;

public:
    BoundedULTRAMcRAPTOR(const Data& data, const Data& backwardData, const CH::CH& chData, const Profiler& profilerTemplate = Profiler()) :
        data(data),
        initialTransfers(chData, FORWARD, data.numberOfStops()),
        profiler(profilerTemplate),
        forwardPruningRAPTOR(data, initialTransfers, profiler),
        backwardPruningRAPTOR(backwardData, initialTransfers, forwardPruningRAPTOR, profiler),
        maxTrips(-1),
        bestLabels(data.numberOfStops() + 1),
        stopsUpdatedByRoute(data.numberOfStops()),
        stopsUpdatedByTransfer(data.numberOfStops()),
        routesServingUpdatedStops(data.numberOfRoutes()),
        sourceVertex(noVertex),
        targetVertex(noVertex),
        targetStop(noStop),
        sourceDepartureTime(never) {
        AssertMsg(data.hasImplicitBufferTimes(), "Departure buffer times have to be implicit!");
        profiler.registerExtraRounds({EXTRA_ROUND_CLEAR, EXTRA_ROUND_FORWARD_PRUNING, EXTRA_ROUND_BACKWARD_PRUNING, EXTRA_ROUND_INITIALIZATION});
        profiler.registerPhases({PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS});
        profiler.registerMetrics({METRIC_ROUTES, METRIC_ROUTE_SEGMENTS, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target, const double arrivalSlack, const double tripSlack) noexcept {
        profiler.start();
        profiler.startExtraRound(EXTRA_ROUND_CLEAR);
        clear();
        profiler.doneRound();

        profiler.startExtraRound(EXTRA_ROUND_FORWARD_PRUNING);
        forwardPruningRAPTOR.run(source, departureTime, target, arrivalSlack, tripSlack);
        profiler.doneRound();
        if (forwardPruningRAPTOR.getAnchorLabels().empty()) return;
        profiler.startExtraRound(EXTRA_ROUND_BACKWARD_PRUNING);
        backwardPruningRAPTOR.run(target, source, departureTime, arrivalSlack, tripSlack);
        profiler.doneRound();

        profiler.startExtraRound(EXTRA_ROUND_INITIALIZATION);
        profiler.startPhase();
        sourceVertex = source;
        targetVertex = target;
        targetStop = data.isStop(target) ? StopId(target) : StopId(data.numberOfStops());
        sourceDepartureTime = departureTime;
        maxTrips = forwardPruningRAPTOR.getMaxTrips();
        initialize();
        profiler.donePhase(PHASE_INITIALIZATION);
        profiler.startPhase();
        relaxInitialTransfers();
        profiler.donePhase(PHASE_TRANSFERS);
        profiler.doneRound();

        for (size_t i = 0; i < maxTrips; i++) {
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

    inline void verify(const double arrivalSlack, const double tripSlack, const int departureTime) const noexcept {
        const std::vector<ArrivalLabel>& anchorLabels = forwardPruningRAPTOR.getAnchorLabels();
        for (const ArrivalLabel& anchorLabel : anchorLabels) {
            Ensure(isContained(anchorLabel), "Anchor label with arrival time " << anchorLabel.arrivalTime << " and " << anchorLabel.numberOfTrips << " was not found!");
        }
        for (const WalkingParetoLabel& label : getResults()) {
            if (!label.isWithinSlack(anchorLabels, departureTime, arrivalSlack, tripSlack)) {
                std::cout << "No anchor label found for " << label << std::endl;
                std::cout << "Anchor labels:" << std::endl;
                for (const ArrivalLabel& anchorLabel : anchorLabels) {
                    std::cout << "\t" << anchorLabel << std::endl;
                }
                Ensure(false, "");
            }
        }
    }

    inline const std::vector<ArrivalLabel>& getAnchorLabels() const noexcept {
        return forwardPruningRAPTOR.getAnchorLabels();
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

    inline std::vector<WalkingParetoLabel> getResults(const Vertex vertex) const noexcept {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        std::vector<WalkingParetoLabel> result;
        for (size_t round = 0; round < rounds.size(); round += 2) {
            const size_t trueRound = std::min(round + 1, rounds.size() - 1);
            for (const Label& label : rounds[trueRound][target].labels) {
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
        if constexpr (RESET_CAPACITIES) {
            std::vector<Round>().swap(rounds);
            std::vector<BestBagType>(bestLabels.size()).swap(bestLabels);
        } else {
            rounds.clear();
            Vector::fill(bestLabels);
        }
    }

    inline void reset() noexcept {
        clear<true>();
    }

    inline const Profiler& getProfiler() const noexcept {
        return profiler;
    }

private:
    inline void initialize() noexcept {
        startNewRound();
        if (data.isStop(sourceVertex)) {
            Label initialLabel(sourceDepartureTime, StopId(sourceVertex));
            arrivalByRoute(StopId(sourceVertex), initialLabel);
        }
        startNewRound();
    }

    inline void collectRoutesServingUpdatedStops() noexcept {
        for (const StopId stop : stopsUpdatedByTransfer) {
            AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
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
        if (data.isStop(sourceVertex)) {
            stopsUpdatedByTransfer.insert(StopId(sourceVertex));
            currentRound()[sourceVertex].resize(1);
            currentRound()[sourceVertex][0] = Label(previousRound()[sourceVertex][0], StopId(sourceVertex), 0);
        }
        for (const Vertex stop : initialTransfers.getForwardPOIs()) {
            if (stop == targetStop) continue;
            AssertMsg(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
            AssertMsg(initialTransfers.getForwardDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
            Label newLabel;
            newLabel.arrivalTime = sourceDepartureTime + initialTransfers.getForwardDistance(stop);
            newLabel.walkingDistance = initialTransfers.getForwardDistance(stop);
            newLabel.parentStop = StopId(sourceVertex);
            newLabel.parentIndex = 0;
            newLabel.parentDepartureTime = sourceDepartureTime;
            newLabel.transferId = noEdge;
            arrivalByTransfer(StopId(stop), newLabel);
        }
        if (initialTransfers.getDistance() != INFTY) {
            Label newLabel;
            newLabel.arrivalTime = sourceDepartureTime + initialTransfers.getDistance();
            newLabel.walkingDistance = initialTransfers.getDistance();
            newLabel.parentStop = StopId(sourceVertex);
            newLabel.parentIndex = 0;
            newLabel.parentDepartureTime = sourceDepartureTime;
            newLabel.transferId = noEdge;
            arrivalByTransfer(targetStop, newLabel);
        }
    }

    inline void relaxIntermediateTransfers() noexcept {
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        for (const StopId stop : stopsUpdatedByRoute) {
            stopsUpdatedByTransfer.insert(stop);
            const BagType& bag = previousRound()[stop];
            currentRound()[stop].resize(bag.size());
            for (size_t i = 0; i < bag.size(); i++) {
                currentRound()[stop][i] = Label(bag[i], stop, i);
            }
        }

        for (const StopId stop : stopsUpdatedByRoute) {
            const BagType& bag = previousRound()[stop];
            for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
                profiler.countMetric(METRIC_EDGES);
                const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
                AssertMsg(data.isStop(toStop), "Graph contains edges to non-stop vertices!");
                const int travelTime = data.transferGraph.get(TravelTime, edge);
                for (size_t i = 0; i < bag.size(); i++) {
                    Label newLabel;
                    newLabel.arrivalTime = bag[i].arrivalTime + travelTime;
                    newLabel.walkingDistance = bag[i].walkingDistance + travelTime;
                    newLabel.parentStop = stop;
                    newLabel.parentIndex = i;
                    newLabel.parentDepartureTime = bag[i].arrivalTime;
                    newLabel.transferId = edge;
                    arrivalByTransfer(toStop, newLabel);
                }
            }
            if (initialTransfers.getBackwardDistance(stop) != INFTY) {
                const int travelTime = initialTransfers.getBackwardDistance(stop);
                for (size_t i = 0; i < bag.size(); i++) {
                    Label newLabel;
                    newLabel.arrivalTime = bag[i].arrivalTime + travelTime;
                    newLabel.walkingDistance = bag[i].walkingDistance + travelTime;
                    newLabel.parentStop = stop;
                    newLabel.parentIndex = i;
                    newLabel.parentDepartureTime = bag[i].arrivalTime;
                    newLabel.transferId = noEdge;
                    arrivalByTransfer(targetStop, newLabel);
                }
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

    inline size_t currentNumberOfTrips() const noexcept {
        return (rounds.size() - 1) / 2;
    }

    inline void arrival(const StopId stop, const Label& label, IndexedSet<false, StopId>& updatedStops, Metric metric) noexcept {
        AssertMsg(data.isStop(stop) || stop == targetStop, "Stop " << stop << " is out of range!");
        if (-backwardPruningRAPTOR.getArrivalTime(stop, maxTrips - currentNumberOfTrips()) < label.arrivalTime) return;
        if (bestLabels[targetStop].dominates(label)) return;
        if (!bestLabels[stop].merge(BestLabel(label))) return;
        profiler.countMetric(metric);
        currentRound()[stop].mergeUndominated(label);
        AssertMsg(bestLabels[stop].dominates(currentRound()[stop]), "Best bag does not dominate current bag!");
        if (data.isStop(stop)) updatedStops.insert(stop);
    }

    inline void arrivalByTransfer(const StopId stop, const Label& label) noexcept {
        arrival(stop, label, stopsUpdatedByTransfer, METRIC_STOPS_BY_TRANSFER);
    }

    inline void arrivalByRoute(const StopId stop, const Label& label) noexcept {
        arrival(stop, label, stopsUpdatedByRoute, METRIC_STOPS_BY_TRIP);
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

    inline bool isContained(const ArrivalLabel& anchorLabel) const noexcept {
        Ensure(anchorLabel.numberOfTrips * 2 < rounds.size(), "Label with " << anchorLabel.numberOfTrips << " is out of bounds!");
        const size_t round = std::min(anchorLabel.numberOfTrips * 2 + 1, rounds.size() - 1);
        for (const Label& label : rounds[round][targetStop]) {
            if (label.arrivalTime == anchorLabel.arrivalTime) return true;
        }
        return false;
    }

private:
    const Data& data;
    BucketCHInitialTransfers initialTransfers;
    Profiler profiler;
    ForwardPruningULTRARAPTOR<Profiler> forwardPruningRAPTOR;
    BackwardPruningULTRARAPTOR<Profiler> backwardPruningRAPTOR;

    std::vector<Round> rounds;

    size_t maxTrips;

    std::vector<BestBagType> bestLabels;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    Vertex sourceVertex;
    Vertex targetVertex;
    StopId targetStop;
    int sourceDepartureTime;

};

}
