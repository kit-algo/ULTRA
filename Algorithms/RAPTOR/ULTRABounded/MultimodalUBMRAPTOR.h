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

#include "BackwardPruningULTRARAPTOR.h"
#include "ForwardPruningULTRARAPTOR.h"

namespace RAPTOR {

template<size_t NUM_MODES, typename PROFILER = NoProfiler, typename INITIAL_TRANSFERS = BucketCHInitialTransfers, bool ADD_INTERMEDIATE_OVERHEAD = false>
class MultimodalUBMRAPTOR {

public:
    inline static constexpr size_t NumTransferModes = NUM_MODES;
    using Profiler = PROFILER;
    using InitialTransferType = INITIAL_TRANSFERS;
    inline static constexpr bool AddIntermediateOverhead = ADD_INTERMEDIATE_OVERHEAD;
    using MultimodalInitialTransferType = MultimodalInitialTransfers<NumTransferModes, InitialTransferType>;
    using InitialTransferGraph = typename InitialTransferType::Graph;
    using Type = MultimodalUBMRAPTOR<NumTransferModes, Profiler, InitialTransferType, AddIntermediateOverhead>;
    using SourceType = Vertex;
    using ParetoLabel = MultimodalParetoLabel<NumTransferModes>;

private:
    struct Label {
        Label() : arrivalTime(never), transferTime{INFTY}, parentStop(noStop), parentIndex(-1), parentDepartureTime(never), routeId(noRouteId) {}

        Label(const int sourceDepartureTime, const StopId sourceStop) :
            arrivalTime(sourceDepartureTime),
            transferTime{0},
            parentStop(sourceStop),
            parentIndex(-1),
            parentDepartureTime(sourceDepartureTime),
            routeId(noRouteId) {
        }

        Label(const Label& parentLabel, const StopId stop, const size_t parentIndex) :
            arrivalTime(parentLabel.arrivalTime),
            parentStop(stop),
            parentIndex(parentIndex),
            parentDepartureTime(parentLabel.arrivalTime),
            transferId(noEdge) {
            std::copy(std::begin(parentLabel.transferTime), std::end(parentLabel.transferTime), std::begin(transferTime));
        }

        template<typename ROUTE_LABEL>
        Label(const ROUTE_LABEL& routeLabel, const StopIndex stopIndex, const StopId parentStop, const RouteId route) :
            arrivalTime(routeLabel.getArrivalTime(stopIndex)),
            parentStop(parentStop),
            parentIndex(routeLabel.parentIndex),
            parentDepartureTime(routeLabel.parentDepartureTime()),
            routeId(route) {
            std::copy(std::begin(routeLabel.transferTime), std::end(routeLabel.transferTime), std::begin(transferTime));
        }

        Label(const Label& parentLabel, const size_t mode, const int travelTime, const StopId stop, const size_t parentIndex, const Edge transferId) :
            arrivalTime(parentLabel.arrivalTime + travelTime),
            parentStop(stop),
            parentIndex(parentIndex),
            parentDepartureTime(parentLabel.arrivalTime),
            transferId(transferId) {
            std::copy(std::begin(parentLabel.transferTime), std::end(parentLabel.transferTime), std::begin(transferTime));
            transferTime[mode] += travelTime;
        }

        Label(const Label& parentLabel, const int travelTime, const StopId stop, const size_t parentIndex, const Edge transferId) :
            arrivalTime(parentLabel.arrivalTime + travelTime),
            parentStop(stop),
            parentIndex(parentIndex),
            parentDepartureTime(parentLabel.arrivalTime),
            transferId(transferId) {
            std::copy(std::begin(parentLabel.transferTime), std::end(parentLabel.transferTime), std::begin(transferTime));
        }

        Label(const int parentDepartureTime, const size_t mode, const int travelTime, const StopId stop) :
            arrivalTime(parentDepartureTime + travelTime),
            transferTime{0},
            parentStop(stop),
            parentIndex(0),
            parentDepartureTime(parentDepartureTime),
            transferId(noEdge) {
            transferTime[mode] = travelTime;
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

        inline bool dominates(const Label& other) const noexcept {
            if (arrivalTime > other.arrivalTime) return false;
            for (size_t i = 0; i < NumTransferModes; i++) {
                if (transferTime[i] > other.transferTime[i]) return false;
            }
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

        int arrivalTime;
        int transferTime[NumTransferModes];
    };

    struct RouteLabel {
        RouteLabel() {}

        RouteLabel(const StopEvent* trip, const Label& label, const StopIndex parentStop, const size_t parentIndex) :
            trip(trip),
            parentStop(parentStop),
            parentIndex(parentIndex) {
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
    };

    using BagType = Bag<Label>;
    using BestBagType = Bag<BestLabel>;
    using Round = std::vector<BagType>;
    using RouteBagType = RouteBag<RouteLabel>;

private:
    MultimodalUBMRAPTOR(const MultimodalData& data, const Data& forwardPruningData, const Data& backwardPruningData, const TransferGraph& backwardTransitiveGraph, const size_t numVertices, const Profiler& profilerTemplate = Profiler()) :
        data(data),
        transitiveInitialTransfers(data.raptorData.transferGraph, backwardTransitiveGraph),
        multimodalInitialTransfers(transitiveInitialTransfers, initialTransfers, data.modes, numVertices, data.raptorData.numberOfStops()),
        profiler(profilerTemplate),
        forwardPruningRAPTOR(forwardPruningData, multimodalInitialTransfers, profiler),
        backwardPruningRAPTOR(backwardPruningData, multimodalInitialTransfers, forwardPruningRAPTOR, profiler),
        bestBagByRoute(data.raptorData.numberOfStops() + 1),
        bestBagByTransfer(data.raptorData.numberOfStops() + 1),
        maxTrips(-1),
        stopsUpdatedByRoute(data.raptorData.numberOfStops() + 1),
        stopsUpdatedByTransfer(data.raptorData.numberOfStops() + 1),
        routesServingUpdatedStops(data.raptorData.numberOfRoutes()),
        sourceVertex(noVertex),
        targetVertex(noVertex),
        targetStop(noStop),
        sourceDepartureTime(intMax) {
        AssertMsg(data.modes.size() == NumTransferModes, "Wrong number of modes");
        AssertMsg(data.raptorData.hasImplicitBufferTimes(), "Departure buffer times have to be implicit!");
        profiler.registerExtraRounds({EXTRA_ROUND_CLEAR, EXTRA_ROUND_FORWARD_PRUNING, EXTRA_ROUND_BACKWARD_PRUNING, EXTRA_ROUND_INITIALIZATION});
        profiler.registerPhases({PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS});
        for (const size_t mode : data.modes) {
            profiler.registerPhases({getProfilerTransferPhase(mode)});
        }
        profiler.registerMetrics({METRIC_ROUTES, METRIC_ROUTE_SEGMENTS, METRIC_VERTICES, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
    }

public:
    template<typename T = CHGraph, typename = std::enable_if_t<Meta::Equals<T, CHGraph>() && Meta::Equals<T, InitialTransferGraph>()>>
    MultimodalUBMRAPTOR(const MultimodalData& data, const Data& forwardPruningData, const Data& backwardPruningData, const TransferGraph& backwardTransitiveGraph, const std::vector<CH::CH>& chData, const Profiler& profilerTemplate = Profiler()) :
        MultimodalUBMRAPTOR(data, forwardPruningData, backwardPruningData, backwardTransitiveGraph, chData[0].forward.numVertices(), profilerTemplate) {
        AssertMsg(chData.size() == NumTransferModes, "Wrong number of modes");
        for (size_t i = 0; i < chData.size(); i++) {
            initialTransfers.emplace_back(chData[i], FORWARD, data.raptorData.numberOfStops());
        }
    }

    template<typename T = TransferGraph, typename = std::enable_if_t<Meta::Equals<T, TransferGraph>() && Meta::Equals<T, InitialTransferGraph>()>>
    MultimodalUBMRAPTOR(const MultimodalData& data, const Data& forwardPruningData, const Data& backwardPruningData, const TransferGraph& backwardTransitiveGraph, const std::vector<TransferGraph>& backwardGraphs, const Profiler& profilerTemplate = Profiler()) :
        MultimodalUBMRAPTOR(data, forwardPruningData, backwardPruningData, backwardTransitiveGraph, backwardGraphs[0].numVertices(), profilerTemplate) {
        AssertMsg(backwardGraphs.size() == NumTransferModes, "Wrong number of modes");
        for (size_t i = 0; i < backwardGraphs.size(); i++) {
            initialTransfers.emplace_back(getTransferGraph(i), backwardGraphs[i], data.raptorData.numberOfStops(), TravelTime);
        }
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
        targetStop = data.raptorData.isStop(target) ? StopId(target) : StopId(data.raptorData.numberOfStops());
        sourceDepartureTime = departureTime;
        maxTrips = forwardPruningRAPTOR.getMaxTrips();
        initialize();
        profiler.donePhase(PHASE_INITIALIZATION);

        profiler.startPhase();
        relaxTransitiveTransfers();
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
            relaxTransitiveTransfers();
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
        const std::vector<ArrivalLabel>& anchorLabels = forwardPruningRAPTOR.getAnchorLabels();
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
        targetStop = StopId(data.raptorData.numberOfStops());
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
        if (data.raptorData.isStop(sourceVertex)) {
            Label initialLabel(sourceDepartureTime, StopId(sourceVertex));
            arrivalByRoute(StopId(sourceVertex), initialLabel);
        }
        startNewRound();
    }

    inline void collectRoutesServingUpdatedStops() noexcept {
        for (const StopId stop : stopsUpdatedByTransfer) {
            for (const RouteSegment& route : data.raptorData.routesContainingStop(stop)) {
                AssertMsg(data.raptorData.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                AssertMsg(data.raptorData.stopIds[data.raptorData.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                if (route.stopIndex + 1 == data.raptorData.numberOfStopsInRoute(route.routeId)) continue;
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
            const size_t tripSize = data.raptorData.numberOfStopsInRoute(route);
            AssertMsg(stopIndex < tripSize - 1, "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " << stopIndex << ", TripSize: " << tripSize << ")!");

            const StopId* stops = data.raptorData.stopArrayOfRoute(route);
            StopId stop = stops[stopIndex];

            const StopEvent* firstTrip = data.raptorData.firstTripOfRoute(route);
            const StopEvent* lastTrip = data.raptorData.lastTripOfRoute(route);

            RouteBagType routeBag;

            while (stopIndex < tripSize - 1) {
                for (size_t i = 0; i < previousRound()[stop].size(); i++) {
                    const Label& label = previousRound()[stop][i];
                    const StopEvent* trip = firstTrip;
                    while ((trip < lastTrip) && (trip[stopIndex].departureTime < label.arrivalTime)) {
                        trip += tripSize;
                    }
                    if (trip[stopIndex].departureTime < label.arrivalTime) continue;
                    routeBag.merge(RouteLabel(trip, label, stopIndex, i));
                }
                stopIndex++;
                stop = stops[stopIndex];
                profiler.countMetric(METRIC_ROUTE_SEGMENTS);
                for (const RouteLabel& label : routeBag.labels) {
                    arrivalByRoute(stop, Label(label, stopIndex, stops[label.parentStop], route));
                }
            }
        }
    }

    inline void relaxInitialTransfers(const size_t mode) noexcept {
        //Initial transfers have already been run in the forward pruning search
        for (const Vertex stop : initialTransfers[mode].getForwardPOIs()) {
            if (stop == targetStop) continue;
            AssertMsg(data.raptorData.isStop(stop), "Reached POI " << stop << " is not a stop!");
            AssertMsg(initialTransfers[mode].getForwardDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
            const Label newLabel(sourceDepartureTime, mode, initialTransfers[mode].getForwardDistance(stop) + TransferModeOverhead[mode], StopId(sourceVertex));
            arrivalByTransfer(StopId(stop), newLabel);
        }
        if (initialTransfers[mode].getDistance() != INFTY) {
            const Label newLabel(sourceDepartureTime, mode, initialTransfers[mode].getDistance() + TransferModeOverhead[mode], StopId(sourceVertex));
            arrivalByTransfer(targetStop, newLabel);
        }
    }

    inline void relaxTransitiveTransfers() noexcept {
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
            for (const Edge edge : data.raptorData.transferGraph.edgesFrom(stop)) {
                profiler.countMetric(METRIC_EDGES);
                const Vertex toStop = data.raptorData.transferGraph.get(ToVertex, edge);
                AssertMsg(data.raptorData.isStop(toStop), "Graph contains edges to non-stop vertices!");
                const int travelTime = data.raptorData.transferGraph.get(TravelTime, edge);
                for (size_t i = 0; i < bag.size(); i++) {
                    const Label newLabel(bag[i], travelTime, stop, i, edge);
                    arrivalByTransfer(StopId(toStop), newLabel);
                }
            }
        }
    }

    inline void relaxIntermediateTransfers(const size_t mode) noexcept {
        for (const StopId stop : stopsUpdatedByRoute) {
            const BagType& bag = previousRound()[stop];
            for (const Edge edge : getTransferGraph(mode).edgesFrom(stop)) {
                profiler.countMetric(METRIC_EDGES);
                const StopId toStop = StopId(getTransferGraph(mode).get(ToVertex, edge));
                if (toStop == targetStop) continue;
                AssertMsg(data.raptorData.isStop(toStop), "Graph contains edges to non-stop vertices!");
                const int travelTime = getIntermediateTravelTime(mode, edge);
                for (size_t i = 0; i < bag.size(); i++) {
                    const Label newLabel(bag[i], mode, travelTime, stop, i, edge);
                    arrivalByTransfer(toStop, newLabel);
                }
            }
            if (initialTransfers[mode].getBackwardDistance(stop) != INFTY) {
                const int travelTime = initialTransfers[mode].getBackwardDistance(stop) + TransferModeOverhead[mode];
                for (size_t i = 0; i < bag.size(); i++) {
                    const Label newLabel(bag[i], mode, travelTime, stop, i, noEdge);
                    arrivalByTransfer(targetStop, newLabel);
                }
            }

        }
    }

    inline int getIntermediateTravelTime(const size_t mode, const Edge edge) const noexcept {
        int travelTime = getTransferGraph(mode).get(TravelTime, edge);
        if constexpr (AddIntermediateOverhead) {
            travelTime += TransferModeOverhead[mode];
        }
        return travelTime;
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
        rounds.emplace_back(data.raptorData.numberOfStops() + 1);
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
        AssertMsg(data.raptorData.isStop(stop), "Stop " << stop << " is out of range!");
        if (-backwardPruningRAPTOR.getArrivalTime(stop, maxTrips - currentNumberOfTrips()) < label.arrivalTime) return;
        if (checkTargetPruning(label)) return;
        if (!bestBagByRoute[stop].merge(BestLabel(label))) return;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        currentRound()[stop].mergeUndominated(label);
        stopsUpdatedByRoute.insert(stop);
    }

    inline void arrivalByTransfer(const StopId stop, const Label& label) noexcept {
        AssertMsg(data.raptorData.isStop(stop) || stop == targetStop, "Stop " << stop << " is out of range!");
        if (-backwardPruningRAPTOR.getArrivalTime(stop, maxTrips - currentNumberOfTrips()) < label.arrivalTime) return;
        if (checkTargetPruning(label)) return;
        if (bestBagByRoute[stop].dominates(label)) return;
        if (!bestBagByTransfer[stop].merge(BestLabel(label))) return;
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        currentRound()[stop].mergeUndominated(label);
        if (data.raptorData.isStop(stop)) stopsUpdatedByTransfer.insert(stop);
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
    const MultimodalData& data;

    TransitiveInitialTransfers transitiveInitialTransfers;
    std::vector<InitialTransferType> initialTransfers;
    MultimodalInitialTransferType multimodalInitialTransfers;

    Profiler profiler;
    ForwardPruningULTRARAPTOR<Profiler, MultimodalInitialTransferType> forwardPruningRAPTOR;
    BackwardPruningULTRARAPTOR<Profiler, MultimodalInitialTransferType> backwardPruningRAPTOR;

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
