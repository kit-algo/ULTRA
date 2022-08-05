#pragma once

#include <vector>

#include "../../../Helpers/Vector/Vector.h"

#include "../InitialTransfers.h"
#include "../Profiler.h"

#include "../../CH/CH.h"

#include "../../../DataStructures/RAPTOR/MultimodalData.h"
#include "../../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/RAPTOR/Entities/Bags.h"
#include "../../../DataStructures/Container/Set.h"
#include "../../../DataStructures/Container/Map.h"
#include "../../../DataStructures/Container/ExternalKHeap.h"
#include "../../../DataStructures/TripBased/MultimodalData.h"

#include "../../TripBased/BoundedMcQuery/ForwardPruningQuery.h"
#include "../../TripBased/BoundedMcQuery/BackwardPruningQuery.h"

namespace RAPTOR {

template<size_t NUM_MODES, typename PROFILER = NoProfiler>
class MultimodalUBMHydRA {

public:
    inline static constexpr size_t NumTransferModes = NUM_MODES;
    using Profiler = PROFILER;
    using Type = MultimodalUBMHydRA<NumTransferModes, Profiler>;
    using MultimodalInitialTransferType = MultimodalInitialTransfers<NumTransferModes, BucketCHInitialTransfers>;
    using InitialTransferGraph = CHGraph;
    using SourceType = Vertex;
    using ParetoLabel = MultimodalParetoLabel<NumTransferModes>;

private:
    struct Label {
        Label() : arrivalTime(never), transferTime{INFTY}, parentStop(noStop), parentIndex(-1), parentDepartureTime(never), routeId(noRouteId), stopEvent(noStopEvent) {}

        Label(const int sourceDepartureTime, const StopId sourceStop) :
            arrivalTime(sourceDepartureTime),
            transferTime{0},
            parentStop(sourceStop),
            parentIndex(-1),
            parentDepartureTime(sourceDepartureTime),
            routeId(noRouteId),
            stopEvent(noStopEvent) {
        }

        Label(const Label& parentLabel, const StopId stop, const size_t parentIndex) :
            arrivalTime(parentLabel.arrivalTime),
            parentStop(stop),
            parentIndex(parentIndex),
            parentDepartureTime(parentLabel.arrivalTime),
            transferId(noEdge),
            stopEvent(noStopEvent) {
            std::copy(std::begin(parentLabel.transferTime), std::end(parentLabel.transferTime), std::begin(transferTime));
        }

        template<typename ROUTE_LABEL>
        Label(const ROUTE_LABEL& routeLabel, const StopIndex stopIndex, const StopId parentStop, const RouteId route, const StopEventId stopEvent) :
            arrivalTime(routeLabel.getArrivalTime(stopIndex)),
            parentStop(parentStop),
            parentIndex(routeLabel.parentIndex),
            parentDepartureTime(routeLabel.parentDepartureTime()),
            routeId(route),
            stopEvent(stopEvent) {
            std::copy(std::begin(routeLabel.transferTime), std::end(routeLabel.transferTime), std::begin(transferTime));
        }

        Label(const Label& parentLabel, const size_t mode, const int travelTime, const StopId stop, const size_t parentIndex, const Edge transferId, const StopEventId stopEvent) :
            arrivalTime(parentLabel.arrivalTime + travelTime),
            parentStop(stop),
            parentIndex(parentIndex),
            parentDepartureTime(parentLabel.arrivalTime),
            transferId(transferId),
            stopEvent(stopEvent) {
            std::copy(std::begin(parentLabel.transferTime), std::end(parentLabel.transferTime), std::begin(transferTime));
            transferTime[mode] += travelTime;
        }

        Label(const Label& parentLabel, const int travelTime, const StopId stop, const size_t parentIndex, const Edge transferId, const StopEventId stopEvent) :
            arrivalTime(parentLabel.arrivalTime + travelTime),
            parentStop(stop),
            parentIndex(parentIndex),
            parentDepartureTime(parentLabel.arrivalTime),
            transferId(transferId),
            stopEvent(stopEvent) {
            std::copy(std::begin(parentLabel.transferTime), std::end(parentLabel.transferTime), std::begin(transferTime));
        }

        Label(const int parentDepartureTime, const size_t mode, const int travelTime, const StopId stop) :
            arrivalTime(parentDepartureTime + travelTime),
            transferTime{0},
            parentStop(stop),
            parentIndex(0),
            parentDepartureTime(parentDepartureTime),
            transferId(noEdge),
            stopEvent(noStopEvent) {
            transferTime[mode] = travelTime;
        }

        Label(const int parentDepartureTime, const int travelTime, const StopId stop) :
            arrivalTime(parentDepartureTime + travelTime),
            transferTime{0},
            parentStop(stop),
            parentIndex(0),
            parentDepartureTime(parentDepartureTime),
            transferId(noEdge),
            stopEvent(noStopEvent) {
        }

        Label(const int arrivalTime, const Label& parentLabel, const StopId parentStop, const size_t parentIndex, const int parentDepartureTime, const RouteId route, const StopEventId stopEvent) :
            arrivalTime(arrivalTime),
            parentStop(parentStop),
            parentIndex(parentIndex),
            parentDepartureTime(parentDepartureTime),
            routeId(route),
            stopEvent(stopEvent) {
            std::copy(std::begin(parentLabel.transferTime), std::end(parentLabel.transferTime), std::begin(transferTime));
        }

        int arrivalTime;
        int transferTime[NumTransferModes];

        StopId parentStop;
        size_t parentIndex;
        int parentDepartureTime;
        union {
            RouteId routeId;
            Edge transferId;
        };
        StopEventId stopEvent;

        inline bool dominates(const Label& other) const noexcept {
            if (arrivalTime > other.arrivalTime) return false;
            for (size_t i = 0; i < NumTransferModes; i++) {
                if (transferTime[i] > other.transferTime[i]) return false;
            }
            return true;
        }

        inline bool dominatesStrongly(const Label& other) const noexcept {
            if (!dominates(other)) return false;
            if (arrivalTime < other.arrivalTime) return true;
            for (size_t i = 0; i < NumTransferModes; i++) {
                if (transferTime[i] < other.transferTime[i]) return true;
            }
            return false;
        }

        inline bool operator==(const Label& other) const noexcept {
            if (arrivalTime != other.arrivalTime) return false;
            for (size_t i = 0; i < NumTransferModes; i++) {
                if (transferTime[i] != other.transferTime[i]) return false;
            }
            if (stopEvent != other.stopEvent) return false;
            return true;
        }
    };

    struct BestLabel {
        BestLabel() : arrivalTime(never), transferTime{INFTY} {}

        BestLabel(const int arrivalTime) :
            arrivalTime(arrivalTime),
            transferTime{0} {
        }

        template<typename LABEL>
        BestLabel(const LABEL& label) :
            arrivalTime(label.arrivalTime) {
            std::copy(std::begin(label.transferTime), std::end(label.transferTime), std::begin(transferTime));
        }

        template<typename LABEL>
        inline bool dominates(const LABEL& other) const noexcept {
            if (arrivalTime > other.arrivalTime) return false;
            for (size_t i = 0; i < NumTransferModes; i++) {
                if (transferTime[i] > other.transferTime[i]) return false;
            }
            return true;
        }

        template<typename LABEL>
        inline bool dominatesStrongly(const LABEL& other) const noexcept {
            if (!dominates(other)) return false;
            if (arrivalTime < other.arrivalTime) return true;
            for (size_t i = 0; i < NumTransferModes; i++) {
                if (transferTime[i] < other.transferTime[i]) return true;
            }
            return false;
        }

        int arrivalTime;
        int transferTime[NumTransferModes];
    };

    struct RouteLabel {
        RouteLabel() {}

        RouteLabel(const StopEvent* trip, const Label& label, const StopIndex parentStop, const size_t parentIndex, const size_t lastExitIndex) :
            trip(trip),
            parentStop(parentStop),
            parentIndex(parentIndex),
            lastExitIndex(lastExitIndex) {
            std::copy(std::begin(label.transferTime), std::end(label.transferTime), std::begin(transferTime));
        }

        inline bool dominates(const RouteLabel& other) const noexcept {
            if (trip > other.trip) return false;
            for (size_t i = 0; i < NumTransferModes; i++) {
                if (transferTime[i] > other.transferTime[i]) return false;
            }
            return true;
        }

        inline int getArrivalTime(const StopIndex stopIndex) const noexcept {
            return trip[stopIndex].arrivalTime;
        }

        inline int parentDepartureTime() const noexcept {
            return trip[parentStop].departureTime;
        }

        const StopEvent* trip;
        int transferTime[NumTransferModes];
        StopIndex parentStop;
        size_t parentIndex;
        size_t lastExitIndex;
    };

    using BagType = Bag<Label>;
    using BestBagType = Bag<BestLabel>;
    using Round = std::vector<BagType>;
    using RouteBagType = RouteBag<RouteLabel>;

public:
    MultimodalUBMHydRA(const TripBased::MultimodalData& data, const TripBased::Data& forwardPruningData, const TripBased::Data& backwardPruningData, const TransferGraph& backwardTransitiveGraph, const std::vector<CH::CH>& chData, const Profiler& profilerTemplate = Profiler()) :
        data(data),
        tripData(data.tripData),
        raptorData(tripData.raptorData),
        stopOfStopEvent(tripData.numberOfStopEvents(), noStop),
        transitiveInitialTransfers(raptorData.transferGraph, backwardTransitiveGraph),
        multimodalInitialTransfers(transitiveInitialTransfers, initialTransfers, data.modes, chData[0].numVertices(), raptorData.numberOfStops()),
        profiler(profilerTemplate),
        forwardPruningQuery(forwardPruningData, multimodalInitialTransfers, dummy),
        backwardPruningQuery(backwardPruningData, forwardPruningQuery, multimodalInitialTransfers, dummy),
        bestBagByRoute(raptorData.numberOfStops() + 1),
        bestBagByTransfer(raptorData.numberOfStops() + 1),
        maxTrips(-1),
        stopsUpdatedByRoute(raptorData.numberOfStops() + 1),
        stopsUpdatedByTransfer(raptorData.numberOfStops() + 1),
        routesServingUpdatedStops(raptorData.numberOfRoutes()),
        sourceVertex(noVertex),
        targetVertex(noVertex),
        targetStop(noStop),
        sourceDepartureTime(intMax) {
        AssertMsg(raptorData.hasImplicitBufferTimes(), "Departure buffer times have to be implicit!");
        AssertMsg(data.modes.size() == NumTransferModes, "Wrong number of modes");
        AssertMsg(chData.size() == NumTransferModes, "Wrong number of modes");
        for (size_t i = 0; i < chData.size(); i++) {
            initialTransfers.emplace_back(chData[i], FORWARD, raptorData.numberOfStops());
        }
        for (StopEventId stopEvent(0); stopEvent < stopOfStopEvent.size(); stopEvent++) {
            stopOfStopEvent[stopEvent] = tripData.getStopOfStopEvent(stopEvent);
        }
        profiler.registerExtraRounds({EXTRA_ROUND_CLEAR, EXTRA_ROUND_FORWARD_PRUNING, EXTRA_ROUND_BACKWARD_PRUNING, EXTRA_ROUND_INITIALIZATION});
        profiler.registerPhases({PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS});
        for (const size_t mode : data.modes) {
            profiler.registerPhases({getProfilerTransferPhase(mode)});
        }
        profiler.registerMetrics({METRIC_ROUTES, METRIC_ROUTE_SEGMENTS, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target, const double arrivalSlack, const double tripSlack) noexcept {
        profiler.start();
        profiler.startExtraRound(EXTRA_ROUND_CLEAR);
        clear();
        profiler.doneRound();

        profiler.startExtraRound(EXTRA_ROUND_FORWARD_PRUNING);
        forwardPruningQuery.run(source, departureTime, target, arrivalSlack, tripSlack);
        profiler.doneRound();
        if (forwardPruningQuery.getAnchorLabels().empty()) return;
        profiler.startExtraRound(EXTRA_ROUND_BACKWARD_PRUNING);
        backwardPruningQuery.run(target, departureTime, source, arrivalSlack, tripSlack);
        profiler.doneRound();

        profiler.startExtraRound(EXTRA_ROUND_INITIALIZATION);
        profiler.startPhase();
        sourceVertex = source;
        targetVertex = target;
        sourceDepartureTime = departureTime;
        maxTrips = forwardPruningQuery.getMaxTrips();
        initialize();
        profiler.donePhase(PHASE_INITIALIZATION);

        profiler.startPhase();
        relaxTransitiveInitialTransfers();
        profiler.donePhase(PHASE_TRANSFERS);
        for (size_t mode = 0; mode < data.modes.size(); mode++) {
            profiler.startPhase();
            relaxInitialTransfers(mode);
            profiler.donePhase(getProfilerTransferPhase(data.modes[mode]));
        }
        profiler.doneRound();

        for (size_t i = 0; i < maxTrips; i++) {
            profiler.startRound();
            profiler.startPhase();
            startNewRound();
            profiler.donePhase(PHASE_INITIALIZATION);
            profiler.startPhase();
            if (i == 0) {
                collectRoutesServingUpdatedStops();
                profiler.donePhase(PHASE_COLLECT);
                profiler.startPhase();
                scanInitialRoutes();
            } else {
                scanRoutes();
            }
            profiler.donePhase(PHASE_SCAN);
            if (stopsUpdatedByRoute.empty()) {
                profiler.doneRound();
                break;
            }
            profiler.startPhase();
            startNewRound();
            profiler.donePhase(PHASE_INITIALIZATION);

            profiler.startPhase();
            relaxTransitiveIntermediateTransfers();
            profiler.donePhase(PHASE_TRANSFERS);
            for (size_t mode = 0; mode < data.modes.size(); mode++) {
                profiler.startPhase();
                relaxIntermediateTransfers(mode);
                profiler.donePhase(getProfilerTransferPhase(data.modes[mode]));
            }

            profiler.doneRound();
        }
        profiler.done();
    }

    inline void verify(const double arrivalSlack, const double tripSlack, const int departureTime) const noexcept {
        const std::vector<ArrivalLabel>& anchorLabels = forwardPruningQuery.getAnchorLabels();
        for (const ArrivalLabel& anchorLabel : anchorLabels) {
            Ensure(isContained(anchorLabel), "Anchor label with arrival time " << anchorLabel.arrivalTime << " and " << anchorLabel.numberOfTrips << " was not found!");
        }
        for (const ParetoLabel& label : getResults()) {
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
        return forwardPruningQuery.getAnchorLabels();
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

    inline std::vector<ParetoLabel> getResults() const noexcept {
        return getResults(targetStop);
    }

    inline std::vector<ParetoLabel> getResults(const StopId stop) const noexcept {
        std::vector<ParetoLabel> result;
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
        targetStop = StopId(raptorData.numberOfStops());
        if constexpr (RESET_CAPACITIES) {
            std::vector<Round>().swap(rounds);
            std::vector<BestBagType>(bestBagByRoute.size()).swap(bestBagByRoute);
            std::vector<BestBagType>(bestBagByTransfer.size()).swap(bestBagByTransfer);
        } else {
            rounds.clear();
            Vector::fill(bestBagByRoute);
            Vector::fill(bestBagByTransfer);
        }
    }

    inline void reset() noexcept {
        clear<true>();
    }

    inline Profiler& getProfiler() noexcept {
        return profiler;
    }

private:
    inline void initialize() noexcept {
        startNewRound();
        if (raptorData.isStop(targetVertex)) {
            targetStop = StopId(targetVertex);
        }
        if (raptorData.isStop(sourceVertex)) {
            Label initialLabel(sourceDepartureTime, StopId(sourceVertex));
            arrivalByRoute(StopId(sourceVertex), initialLabel);
        }
        startNewRound();
    }

    inline void collectRoutesServingUpdatedStops() noexcept {
        for (const StopId stop : stopsUpdatedByTransfer) {
            for (const RouteSegment& route : raptorData.routesContainingStop(stop)) {
                AssertMsg(raptorData.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                AssertMsg(raptorData.stopIds[raptorData.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                if (route.stopIndex + 1 == raptorData.numberOfStopsInRoute(route.routeId)) continue;
                if (routesServingUpdatedStops.contains(route.routeId)) {
                    routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                }
            }
        }
    }

    inline void scanInitialRoutes() noexcept {
        stopsUpdatedByRoute.clear();
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            profiler.countMetric(METRIC_ROUTES);
            StopIndex stopIndex = routesServingUpdatedStops[route];
            const size_t tripSize = raptorData.numberOfStopsInRoute(route);
            AssertMsg(stopIndex < tripSize - 1, "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " << stopIndex << ", TripSize: " << tripSize << ")!");

            const StopId* stops = raptorData.stopArrayOfRoute(route);
            StopId stop = stops[stopIndex];

            const StopEvent* firstTrip = raptorData.firstTripOfRoute(route);
            const StopEvent* lastTrip = raptorData.lastTripOfRoute(route);

            RouteBagType routeBag;

            while (stopIndex < tripSize - 1) {
                for (size_t i = 0; i < previousRound()[stop].size(); i++) {
                    const Label& label = previousRound()[stop][i];
                    const StopEvent* trip = firstTrip;
                    while ((trip < lastTrip) && (trip[stopIndex].departureTime < label.arrivalTime)) {
                        trip += tripSize;
                    }
                    if (trip[stopIndex].departureTime < label.arrivalTime) continue;
                    const size_t tripOffset = (trip - firstTrip) / tripSize;
                    const TripId reverseTrip = backwardPruningQuery.getReverseTrip(route, tripOffset);
                    const StopIndex reachedIndex = StopIndex(tripSize - backwardPruningQuery.getReachedIndex(reverseTrip, maxTrips - currentNumberOfTrips()) - 1);
                    if (reachedIndex < stopIndex) continue;
                    routeBag.merge(RouteLabel(trip, label, stopIndex, i, reachedIndex));
                }
                stopIndex++;
                stop = stops[stopIndex];
                profiler.countMetric(METRIC_ROUTE_SEGMENTS);
                for (const RouteLabel& label : routeBag.labels) {
                    if (label.lastExitIndex + 1 < stopIndex) continue;
                    const StopEventId stopEvent(label.trip + stopIndex - &(raptorData.stopEvents[0]));
                    arrivalByRoute(stop, Label(label, stopIndex, stops[label.parentStop], route, stopEvent));
                }
            }
        }
    }

    inline void scanRoutes() noexcept {
        stopsUpdatedByRoute.clear();
        for (const StopId stop : stopsUpdatedByTransfer) {
            for (size_t i = 0; i < previousRound()[stop].size(); i++) {
                profiler.countMetric(METRIC_ROUTE_SEGMENTS);
                const Label& label = previousRound()[stop][i];
                const StopEventId enterEvent = label.stopEvent;
                const int parentDepartureTime = raptorData.stopEvents[enterEvent].departureTime;
                const TripId trip = tripData.tripOfStopEvent[enterEvent];
                const RouteId route = tripData.routeOfTrip[trip];
                const StopId enterStop = stopOfStopEvent[enterEvent];
                const TripId reverseTrip = backwardPruningQuery.getReverseTrip(trip);
                const size_t tripSize = tripData.numberOfStopsInTrip(trip);
                const StopIndex reachedIndex = StopIndex(tripSize - backwardPruningQuery.getReachedIndex(reverseTrip, maxTrips - currentNumberOfTrips()));
                StopIndex stopIndex = StopIndex(tripData.indexOfStopEvent[enterEvent] + 1);
                StopEventId stopEvent = StopEventId(tripData.firstStopEventOfTrip[trip] + stopIndex);
                for (; stopIndex < StopIndex(reachedIndex + 1); stopIndex++, stopEvent++) {
                    const StopId toStop = stopOfStopEvent[stopEvent];
                    const int arrivalTime = raptorData.stopEvents[stopEvent].arrivalTime;
                    arrivalByRoute(toStop, Label(arrivalTime, label, enterStop, i, parentDepartureTime, route, stopEvent));
                }
            }
        }
    }

    inline void relaxTransitiveInitialTransfers() noexcept {
        if (!raptorData.isStop(sourceVertex)) return;
        stopsUpdatedByTransfer.insert(StopId(sourceVertex));
        currentRound()[sourceVertex].resize(1);
        currentRound()[sourceVertex][0] = Label(previousRound()[sourceVertex][0], StopId(sourceVertex), 0);
        for (const Vertex toStop : transitiveInitialTransfers.getForwardPOIs()) {
            AssertMsg(raptorData.isStop(toStop), "Transitive transfer graph has edges to non-stop vertices!");
            const int travelTime = transitiveInitialTransfers.getForwardDistance(toStop);
            const Label newLabel(sourceDepartureTime, travelTime, StopId(sourceVertex));
            if (toStop == targetStop) {
                targetArrivalByTransfer(newLabel);
            } else {
                arrivalByTransfer(StopId(toStop), newLabel);
            }
        }
    }

    inline void relaxInitialTransfers(const size_t mode) noexcept {
        //Initial transfers have already been run in the forward pruning search
        for (const Vertex stop : initialTransfers[mode].getForwardPOIs()) {
            if (stop == targetStop) continue;
            AssertMsg(raptorData.isStop(stop), "Reached POI " << stop << " is not a stop!");
            AssertMsg(initialTransfers[mode].getForwardDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
            const Label newLabel(sourceDepartureTime, mode, initialTransfers[mode].getForwardDistance(stop) + TransferModeOverhead[mode], StopId(sourceVertex));
            arrivalByTransfer(StopId(stop), newLabel);
        }
        if (initialTransfers[mode].getDistance() != INFTY) {
            const Label newLabel(sourceDepartureTime, mode, initialTransfers[mode].getDistance() + TransferModeOverhead[mode], StopId(sourceVertex));
            targetArrivalByTransfer(newLabel);
        }
    }

    inline void relaxTransitiveIntermediateTransfers() noexcept {
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        for (const StopId stop : stopsUpdatedByRoute) {
            const BagType& bag = previousRound()[stop];
            for (size_t i = 0; i < bag.size(); i++) {
                for (const Edge edge : tripData.stopEventGraph.edgesFrom(Vertex(bag[i].stopEvent))) {
                    profiler.countMetric(METRIC_EDGES);
                    const StopEventId toStopEvent = StopEventId(tripData.stopEventGraph.get(ToVertex, edge));
                    const StopId toStop = stopOfStopEvent[toStopEvent];
                    if (toStop == targetStop) continue;
                    const int travelTime = tripData.stopEventGraph.get(TravelTime, edge);
                    const Label newLabel(bag[i], travelTime, stop, i, edge, toStopEvent);
                    arrivalByTransfer(toStop, newLabel);
                }
            }
            if (transitiveInitialTransfers.getBackwardDistance(stop) != INFTY) {
                const int travelTime = transitiveInitialTransfers.getBackwardDistance(stop);
                for (size_t i = 0; i < bag.size(); i++) {
                    const Label newLabel(bag[i], travelTime, stop, i, noEdge, noStopEvent);
                    targetArrivalByTransfer(newLabel);
                }
            }
        }
    }

    inline void relaxIntermediateTransfers(const size_t mode) noexcept {
        for (const StopId stop : stopsUpdatedByRoute) {
            const BagType& bag = previousRound()[stop];
            for (size_t i = 0; i < bag.size(); i++) {
                for (const Edge edge : getTransferGraph(mode).edgesFrom(Vertex(bag[i].stopEvent))) {
                    profiler.countMetric(METRIC_EDGES);
                    const StopEventId toStopEvent = StopEventId(getTransferGraph(mode).get(ToVertex, edge));
                    const StopId toStop = stopOfStopEvent[toStopEvent];
                    if (toStop == targetStop) continue;
                    const int travelTime = getTransferGraph(mode).get(TravelTime, edge);
                    const Label newLabel(bag[i], mode, travelTime, stop, i, edge, toStopEvent);
                    arrivalByTransfer(toStop, newLabel);
                }
            }
            if (initialTransfers[mode].getBackwardDistance(stop) != INFTY) {
                const int travelTime = initialTransfers[mode].getBackwardDistance(stop) + TransferModeOverhead[mode];
                for (size_t i = 0; i < bag.size(); i++) {
                    const Label newLabel(bag[i], mode, travelTime, stop, i, noEdge, noStopEvent);
                    targetArrivalByTransfer(newLabel);
                }
            }
        }
    }

    inline const TransferGraph& getTransferGraph(const size_t mode) const noexcept {
        return data.getTransferGraph(data.modes[mode]);
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
        rounds.emplace_back(raptorData.numberOfStops() + 1);
    }

    inline size_t currentNumberOfTrips() const noexcept {
        return (rounds.size() - 1) / 2;
    }

    template<typename LABEL>
    inline bool checkTargetPruning(const LABEL& label) noexcept {
        if (bestBagByRoute[targetStop].dominates(label)) return true;
        if (bestBagByTransfer[targetStop].dominates(label)) return true;
        return false;
    }

    inline void arrivalByRoute(const StopId stop, const Label& label) noexcept {
        AssertMsg(raptorData.isStop(stop), "Stop " << stop << " is out of range!");
        if (checkTargetPruning(label)) return;
        if (!bestBagByRoute[stop].mergeWithStrongDominance(BestLabel(label))) return;
        if (!currentRound()[stop].mergeUndominatedUnlessEqual(label)) return;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        stopsUpdatedByRoute.insert(stop);
    }

    inline void arrivalByTransfer(const StopId stop, const Label& label) noexcept {
        AssertMsg(raptorData.isStop(stop), "Stop " << stop << " is out of range!");
        if (-backwardPruningQuery.getArrivalTime(stop, maxTrips - currentNumberOfTrips()) < label.arrivalTime) return;
        if (checkTargetPruning(label)) return;
        if (!bestBagByTransfer[stop].mergeWithStrongDominance(BestLabel(label))) return;
        if (!currentRound()[stop].mergeUndominatedUnlessEqual(label)) return;
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        if (raptorData.isStop(stop)) stopsUpdatedByTransfer.insert(stop);
    }

       inline void targetArrivalByTransfer(const Label& label) noexcept {
        if (-backwardPruningQuery.getDepartureTime(maxTrips - currentNumberOfTrips()) < label.arrivalTime) return;
        if (!bestBagByTransfer[targetStop].merge(BestLabel(label))) return;
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        currentRound()[targetStop].mergeUndominated(label);
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
    const TripBased::MultimodalData& data;
    const TripBased::Data& tripData;
    const Data& raptorData;
    std::vector<StopId> stopOfStopEvent;

    TransitiveInitialTransfers transitiveInitialTransfers;
    std::vector<BucketCHInitialTransfers> initialTransfers;
    MultimodalInitialTransferType multimodalInitialTransfers;

    Profiler profiler;
    TripBased::NoProfiler dummy;
    TripBased::ForwardPruningQuery<TripBased::NoProfiler, MultimodalInitialTransferType> forwardPruningQuery;
    TripBased::BackwardPruningQuery<TripBased::NoProfiler, MultimodalInitialTransferType> backwardPruningQuery;

    std::vector<Round> rounds;
    std::vector<BestBagType> bestBagByRoute;
    std::vector<BestBagType> bestBagByTransfer;

    size_t maxTrips;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    Vertex sourceVertex;
    Vertex targetVertex;
    StopId targetStop;
    int sourceDepartureTime;
};

}
