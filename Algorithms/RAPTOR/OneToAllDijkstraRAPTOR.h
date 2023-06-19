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

template<typename INITIAL_TRANSFERS, typename PROFILER>
class OneToAllDijkstraRAPTOR {

public:
    using InitialTransferType = INITIAL_TRANSFERS;
    using InitialTransferGraph = typename InitialTransferType::Graph;
    using Profiler = PROFILER;
    using Type = OneToAllDijkstraRAPTOR<InitialTransferType, Profiler>;
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
    template<typename ATTRIBUTE>
    OneToAllDijkstraRAPTOR(const Data& data, const InitialTransferGraph& forwardGraph, const InitialTransferGraph& backwardGraph, const ATTRIBUTE weight, const Profiler& profilerTemplate = Profiler()) :
        data(data),
        initialTransfers(forwardGraph, backwardGraph, data.transferGraph.numVertices(), weight),
        stopsUpdatedByRoute(data.numberOfStops()),
        stopsUpdatedByTransfer(data.numberOfStops()),
        routesServingUpdatedStops(data.numberOfRoutes()),
        sourceVertex(noVertex),
        sourceDepartureTime(intMax),
        earliestArrival(data.transferGraph.numVertices(), never),
        profiler(profilerTemplate) {
        AssertMsg(data.hasImplicitBufferTimes(), "Departure buffer times have to be implicit!");
        profiler.registerExtraRounds({EXTRA_ROUND_CLEAR, EXTRA_ROUND_INITIALIZATION});
        profiler.registerPhases({PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS});
        profiler.registerMetrics({METRIC_ROUTES, METRIC_ROUTE_SEGMENTS, METRIC_VERTICES, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
    }

    template<typename T = CHGraph, typename = std::enable_if_t<Meta::Equals<T, CHGraph>() && Meta::Equals<T, InitialTransferGraph>()>>
    OneToAllDijkstraRAPTOR(const Data& data, const CH::CH& chData, const Profiler& profilerTemplate = Profiler()) :
        OneToAllDijkstraRAPTOR(data, chData.forward, chData.backward, Weight, profilerTemplate) {
    }

    template<typename T = TransferGraph, typename = std::enable_if_t<Meta::Equals<T, TransferGraph>() && Meta::Equals<T, InitialTransferGraph>()>>
    OneToAllDijkstraRAPTOR(const Data& data, const TransferGraph& forwardGraph, const TransferGraph& backwardGraph, const Profiler& profilerTemplate = Profiler()) :
        OneToAllDijkstraRAPTOR(data, forwardGraph, backwardGraph, TravelTime, profilerTemplate) {
    }

    inline void run(const Vertex source, const int departureTime, const size_t maxRounds = INFTY) noexcept {
        profiler.start();
        profiler.startExtraRound(EXTRA_ROUND_CLEAR);
        clear();
        profiler.doneRound();

        profiler.startExtraRound(EXTRA_ROUND_INITIALIZATION);
        profiler.startPhase();
        initialize(source, departureTime);
        profiler.donePhase(PHASE_INITIALIZATION);
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
            relaxTransfers();
            profiler.donePhase(PHASE_TRANSFERS);
            profiler.doneRound();
        }
        profiler.done();
    }

    inline std::vector<Journey> getJourneys(const Vertex vertex) const noexcept {
        std::vector<Journey> journeys;
        for (size_t i = 0; i < rounds.size(); i++) {
            getJourney(journeys, i, vertex);
        }
        return journeys;
    }

    inline Journey getEarliestJourney(const Vertex vertex) const noexcept {
        std::vector<Journey> journeys = getJourneys(vertex);
        return journeys.empty() ? Journey() : journeys.back();
    }

    inline std::vector<ArrivalLabel> getArrivals(const Vertex vertex) const noexcept {
        std::vector<ArrivalLabel> labels;
        for (size_t i = 0; i < rounds.size(); i++) {
            getArrival(labels, i, vertex);
        }
        return labels;
    }

    inline std::vector<int> getArrivalTimes(const Vertex vertex) const noexcept {
        std::vector<int> arrivalTimes;
        for (size_t i = 0; i < rounds.size(); i++) {
            getArrivalTime(arrivalTimes, i, vertex);
        }
        return arrivalTimes;
    }

    inline bool reachable(const Vertex vertex) const noexcept {
        return earliestArrival[vertex] < never;
    }

    inline int getEarliestArrivalTime(const Vertex vertex) const noexcept {
        return earliestArrival[vertex];
    }

    inline int getWalkingArrivalTime(const Vertex vertex) const noexcept {
        return sourceDepartureTime + initialTransfers.getForwardDistance(vertex);
    }

    inline int getWalkingTravelTime(const Vertex vertex) const noexcept {
        return initialTransfers.getForwardDistance(vertex);
    }

    inline std::vector<Vertex> getPath(const Vertex vertex) const {
        return journeyToPath(getJourneys(vertex).back());
    }

    inline std::vector<std::string> getRouteDescription(const Vertex vertex) const {
        return data.journeyToText(getJourneys(vertex).back());
    }

    inline int getArrivalTime(const Vertex vertex, const size_t round) const noexcept {
        if (data.isStop(vertex)) {
            AssertMsg(rounds[round][vertex].arrivalTime < never, "No label found for stop " << vertex << " in round " << round << "!");
            return rounds[round][vertex].arrivalTime;
        } else {
            AssertMsg(dijkstraLabels[round][vertex].arrivalTime < never, "No label found for vertex " << vertex << " in round " << round << "!");
            return dijkstraLabels[round][vertex].arrivalTime;
        }
    }

    inline void clear() noexcept {
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        sourceVertex = noVertex;
        queue.clear();
        std::vector<Round>().swap(rounds);
        std::vector<std::vector<DijkstraLabel>>().swap(dijkstraLabels);
        std::vector<int>(earliestArrival.size(), never).swap(earliestArrival);
    }

    inline Profiler& getProfiler() noexcept {
        return profiler;
    }

private:
    inline void initialize(const Vertex source, const int departureTime) noexcept {
        sourceVertex = source;
        sourceDepartureTime = departureTime;
        startNewRound();
    }

    inline void collectRoutesServingUpdatedStops() noexcept {
        for (const StopId stop : stopsUpdatedByTransfer) {
            for (const RouteSegment& route : data.routesContainingStop(stop)) {
                AssertMsg(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                AssertMsg(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId)) continue;
                if (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < previousRound()[stop].arrivalTime) continue;
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
            AssertMsg(stopIndex < tripSize - 1, "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " << stopIndex << ", TripSize: " << tripSize << ", RoundIndex: " << rounds.size() - 1 << ")!");

            const StopId* stops = data.stopArrayOfRoute(route);
            const StopEvent* trip = data.lastTripOfRoute(route);
            StopId stop = stops[stopIndex];
            AssertMsg(trip[stopIndex].departureTime >= previousRound()[stop].arrivalTime, "Cannot scan a route after the last trip has departed (Route: " << route << ", Stop: " << stop << ", StopIndex: " << stopIndex << ", Time: " << previousRound()[stop].arrivalTime << ", LastDeparture: " << trip[stopIndex].departureTime << ", RoundIndex: " << rounds.size() - 1 << ")!");

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

    inline void relaxInitialTransfers() noexcept {
        routesServingUpdatedStops.clear();
        stopsUpdatedByTransfer.clear();
        initialTransfers.template run<FORWARD, BACKWARD>(sourceVertex);
        for (const Vertex vertex : initialTransfers.getForwardPOIs()) {
            AssertMsg(initialTransfers.getForwardDistance(vertex) != INFTY, "Vertex " << vertex << " was not reached!");
            DijkstraLabel& label = dijkstraLabels[0][vertex];
            label.arrivalTime = sourceDepartureTime + initialTransfers.getForwardDistance(vertex);
            label.parent = sourceVertex;
            earliestArrival[vertex] = label.arrivalTime;
            if (data.isStop(vertex)) {
                arrivalByTransfer(StopId(vertex), label.arrivalTime, sourceVertex, sourceDepartureTime);
            }
        }
    }

    inline void relaxTransfers() noexcept {
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        AssertMsg(queue.empty(), "Queue still has " << queue.size() << " elements!");
        for (const StopId stop : stopsUpdatedByRoute) {
            DijkstraLabel& label = dijkstraLabels.back()[stop];
            label.arrivalTime = earliestArrival[stop];
            label.parent = stop;
            queue.update(&label);
        }
        dijkstra();
    }

    inline void dijkstra() noexcept {
        while (!queue.empty()) {
            DijkstraLabel* uLabel = queue.extractFront();
            const Vertex u = Vertex(uLabel - &(dijkstraLabels.back()[0]));
            for (Edge edge : data.transferGraph.edgesFrom(u)) {
                const Vertex v = data.transferGraph.get(ToVertex, edge);
                profiler.countMetric(METRIC_EDGES);
                arrivalByEdge(v, uLabel->arrivalTime + data.transferGraph.get(TravelTime, edge), uLabel->parent);
            }
            if (data.isStop(u)) {
                arrivalByTransfer(StopId(u), uLabel->arrivalTime, uLabel->parent, earliestArrival[uLabel->parent]);
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
        rounds.emplace_back(data.numberOfStops());
        dijkstraLabels.emplace_back(data.transferGraph.numVertices());
    }

    inline bool arrivalByRoute(const StopId stop, const int arrivalTime) noexcept {
        AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
        AssertMsg(arrivalTime >= sourceDepartureTime, "Arriving by route BEFORE departing from the source (source departure time: " << String::secToTime(sourceDepartureTime) << " [" << sourceDepartureTime << "], arrival time: " << String::secToTime(arrivalTime) << " [" << arrivalTime << "], stop: " << stop << ")!");
        if (earliestArrival[stop] <= arrivalTime) return false;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        currentRound()[stop].arrivalTime = arrivalTime;
        earliestArrival[stop] = arrivalTime;
        stopsUpdatedByRoute.insert(stop);
        return true;
    }

    inline bool arrivalByEdge(const Vertex vertex, const int arrivalTime, const Vertex parent) noexcept {
        AssertMsg(data.isStop(parent), "Parent vertex (" << parent << ") is not a stop");
        AssertMsg(arrivalTime >= sourceDepartureTime, "Arriving by route BEFORE departing from the source (source departure time: " << String::secToTime(sourceDepartureTime) << " [" << sourceDepartureTime << "], arrival time: " << String::secToTime(arrivalTime) << " [" << arrivalTime << "])!");
        if (earliestArrival[vertex] <= arrivalTime) return false;
        DijkstraLabel& label = dijkstraLabels.back()[vertex];
        label.arrivalTime = arrivalTime;
        label.parent = parent;
        earliestArrival[vertex] = arrivalTime;
        queue.update(&label);
        return true;
    }

    inline void arrivalByTransfer(const StopId stop, const int arrivalTime, const Vertex parent, const int parentDepartureTime) noexcept {
        AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
        AssertMsg(arrivalTime >= sourceDepartureTime, "Arriving by route BEFORE departing from the source (source departure time: " << String::secToTime(sourceDepartureTime) << " [" << sourceDepartureTime << "], arrival time: " << String::secToTime(arrivalTime) << " [" << arrivalTime << "])!");
        EarliestArrivalLabel& label = currentRound()[stop];
        stopsUpdatedByTransfer.insert(stop);
        if (label.arrivalTime <= arrivalTime) return;
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        label.arrivalTime = arrivalTime;
        label.parent = parent;
        label.parentDepartureTime = parentDepartureTime;
        label.usesRoute = false;
    }

    inline void getJourney(std::vector<Journey>& journeys, size_t round, Vertex vertex) const noexcept {
        Journey journey;
        if (data.isStop(vertex)) {
            if (rounds[round][vertex].arrivalTime >= (journeys.empty() ? never : journeys.back().back().arrivalTime)) return;
        } else {
            const DijkstraLabel& label = dijkstraLabels[round][vertex];
            if (label.arrivalTime >= (journeys.empty() ? never : journeys.back().back().arrivalTime)) return;
            AssertMsg(label.parent != noVertex, "Vertex " << vertex << ": Label with " << round << " trips has no parent! (arrival time: " << label.arrivalTime << ")");
            journey.emplace_back(label.parent, vertex, rounds[round][label.parent].arrivalTime, label.arrivalTime, false, noRouteId);
            vertex = label.parent;
        }
        while (vertex != sourceVertex) {
            AssertMsg(round != size_t(-1), "Backtracking parent pointers did not pass through the source stop!");
            const EarliestArrivalLabel& label = rounds[round][vertex];
            journey.emplace_back(label.parent, vertex, label.parentDepartureTime, label.arrivalTime, label.usesRoute, label.routeId);
            vertex = label.parent;
            if (label.usesRoute) round--;
        }
        journeys.emplace_back(Vector::reverse(journey));
    }

    inline void getArrival(std::vector<ArrivalLabel>& labels, const size_t round, const Vertex vertex) const noexcept {
        const int arrivalTime = data.isStop(vertex) ? rounds[round][vertex].arrivalTime : dijkstraLabels[round][vertex].arrivalTime;
        if (arrivalTime >= (labels.empty() ? never : labels.back().arrivalTime)) return;
        labels.emplace_back(arrivalTime, round);
    }

    inline void getArrivalTime(std::vector<int>& labels, size_t round, const Vertex vertex) const noexcept {
        const int arrivalTime = data.isStop(vertex) ? rounds[round][vertex].arrivalTime : dijkstraLabels[round][vertex].arrivalTime;
        if (arrivalTime >= (labels.empty() ? never : labels.back())) return;
        labels.emplace_back(arrivalTime);
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

    InitialTransferType initialTransfers;

    std::vector<Round> rounds;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    Vertex sourceVertex;
    int sourceDepartureTime;

    std::vector<std::vector<DijkstraLabel>> dijkstraLabels;
    ExternalKHeap<2, DijkstraLabel> queue;

    std::vector<int> earliestArrival;

    Profiler profiler;

};

}
