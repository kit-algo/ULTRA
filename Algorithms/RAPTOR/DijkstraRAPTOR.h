/**********************************************************************************

 Copyright (c) 2019 Jonas Sauer, Tobias Zündorf

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
 modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************************/

#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../Helpers/Vector/Vector.h"

#include "InitialTransfers.h"

#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/Container/Set.h"
#include "../../DataStructures/Container/Map.h"
#include "../../DataStructures/Container/ExternalKHeap.h"

namespace RAPTOR {

template<typename DEBUGGER>
class DijkstraRAPTOR {

public:
    using Debugger = DEBUGGER;
    using Type = DijkstraRAPTOR<Debugger>;

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
    DijkstraRAPTOR(const Data& data, const CHGraph& forwardGraph, const CHGraph& backwardGraph, const ATTRIBUTE weight, const Debugger& debuggerTemplate = Debugger()) :
        data(data),
        initialTransfers(forwardGraph, backwardGraph, data.numberOfStops(), weight),
        roundIndex(-1),
        earliestArrivalPerRoute(data.numberOfStops() + 1, never),
        stopsUpdatedByRoute(data.numberOfStops() + 1),
        stopsUpdatedByTransfer(data.numberOfStops() + 1),
        routesServingUpdatedStops(data.numberOfRoutes()),
        sourceVertex(noVertex),
        targetVertex(noVertex),
        targetStop(noStop),
        sourceDepartureTime(intMax),
        label(data.transferGraph.numVertices()),
        debugger(debuggerTemplate) {
        AssertMsg(data.hasImplicitBufferTimes(), "Departure buffer times have to be implicit!");
        debugger.initialize(data);
    }

    DijkstraRAPTOR(const Data& data, const CH::CH& chData, const Debugger& debuggerTemplate = Debugger()) :
        DijkstraRAPTOR(data, chData.forward, chData.backward, Weight, debuggerTemplate) {
    }

    template<bool CLEAR = true, bool PREVENT_DIRECT_WALKING = false>
    inline void run(const Vertex source, const int departureTime, const Vertex target, const size_t maxRounds = 50) noexcept {
        runInitialize<CLEAR>(source, departureTime, target);
        runInitialTransfers<PREVENT_DIRECT_WALKING>();
        runRounds<PREVENT_DIRECT_WALKING>(maxRounds);
    }

    template<bool RESET = true>
    inline void runInitialize(const Vertex source, const int departureTime, const Vertex target) noexcept {
        debugger.start();
        debugger.startInitialization();
        clear<RESET>();
        initialize(source, departureTime, target);
        debugger.doneInitialization();
    }

    template<bool PREVENT_DIRECT_WALKING = false>
    inline void runInitialTransfers() noexcept {
        relaxInitialTransfers<!PREVENT_DIRECT_WALKING>();
    }

    inline void runAddSource(const StopId source, const int departureTime) noexcept {
        debugger.updateStopByTransfer(source, departureTime);
        currentRound()[source].arrivalTime = departureTime;
        stopsUpdatedByTransfer.insert(source);
    }

    template<bool PREVENT_DIRECT_WALKING = false>
    inline void runRounds(const size_t maxRounds = 50) noexcept {
        startNewRound();
        debugger.newRound();
        collectAndScanRoutes([&](const StopId stop) {
            if constexpr (PREVENT_DIRECT_WALKING) {if (stop == targetStop) return intMax;}
            return sourceDepartureTime + initialTransfers.getForwardDistance(stop);
        });
        for (size_t i = 0; (i < maxRounds) && (!stopsUpdatedByRoute.empty()); i++) {
            relaxIntermediateTransfers();
            startNewRound();
            debugger.newRound();
            collectAndScanRoutes([&](const StopId stop) {
                return label[stop].arrivalTime;
            });
        }
        debugger.done();
    }

    template<bool RESET = true>
    inline void clear() noexcept {
        roundIndex = -1;
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        sourceVertex = noVertex;
        targetVertex = noVertex;
        targetStop = StopId(data.numberOfStops());
        queue.clear();
        if constexpr (RESET) {
            std::vector<Round>().swap(rounds);
            std::vector<int>(earliestArrivalPerRoute.size(), never).swap(earliestArrivalPerRoute);
            std::vector<DijkstraLabel>(label.size()).swap(label);
        }
    }

    inline void reset() noexcept {
        clear<true>();
    }

    inline const Debugger& getDebugger() const noexcept {
        return debugger;
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
            debugger.updateStopByRoute(StopId(source), departureTime);
            arrivalByRoute(StopId(source), departureTime);
            currentRound()[source].parent = source;
            currentRound()[source].parentDepartureTime = departureTime;
            currentRound()[source].usesRoute = false;
            stopsUpdatedByTransfer.insert(StopId(source));
        }
    }

    template<typename ARRIVAL_TIME>
    inline void collectRoutesServingUpdatedStops(const ARRIVAL_TIME& arrivalTime) noexcept {
        debugger.startCollectRoutes();
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
        debugger.stopCollectRoutes();
    }

    template<typename ARRIVAL_TIME>
    inline void scanRoutes(const ARRIVAL_TIME& arrivalTime) noexcept {
        debugger.startScanRoutes();
        stopsUpdatedByRoute.clear();
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            debugger.scanRoute(route);
            StopIndex stopIndex = routesServingUpdatedStops[route];
            const size_t tripSize = data.numberOfStopsInRoute(route);
            AssertMsg(stopIndex < tripSize - 1, "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " << stopIndex << ", TripSize: " << tripSize << ", RoundIndex: " << roundIndex << ")!");

            const StopId* stops = data.stopArrayOfRoute(route);
            const StopEvent* trip = data.lastTripOfRoute(route);
            StopId stop = stops[stopIndex];
            AssertMsg(trip[stopIndex].departureTime >= arrivalTime(stop), "Cannot scan a route after the last trip has departed (Route: " << route << ", Stop: " << stop << ", StopIndex: " << stopIndex << ", Time: " << arrivalTime(stop) << ", LastDeparture: " << trip[stopIndex].departureTime << ", RoundIndex: " << roundIndex << ")!");

            StopIndex parentIndex = stopIndex;
            const StopEvent* firstTrip = data.firstTripOfRoute(route);
            while (stopIndex < tripSize - 1) {
                while ((trip > firstTrip) && ((trip - tripSize + stopIndex)->departureTime >= arrivalTime(stop))) {
                    trip -= tripSize;
                    parentIndex = stopIndex;
                }
                stopIndex++;
                stop = stops[stopIndex];
                debugger.scanRouteSegment(data.getRouteSegmentNum(route, stopIndex));
                if (arrivalByRoute(stop, trip[stopIndex].arrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[stop];
                    label.parent = stops[parentIndex];
                    label.parentDepartureTime = trip[parentIndex].departureTime;
                    label.usesRoute = true;
                    label.routeId = route;
                }
            }
        }
        debugger.stopScanRoutes();
    }

    template<typename ARRIVAL_TIME>
    inline void collectAndScanRoutes(const ARRIVAL_TIME& arrivalTime) noexcept {
        collectRoutesServingUpdatedStops(arrivalTime);
        scanRoutes(arrivalTime);
    }

    template<bool ALLOW_DIRECT_WALKING>
    inline void relaxInitialTransfers() noexcept {
        debugger.startRelaxTransfers();
        routesServingUpdatedStops.clear();
        stopsUpdatedByTransfer.clear();
        initialTransfers.template run<ALLOW_DIRECT_WALKING>(sourceVertex, targetVertex);
        debugger.directWalking(initialTransfers.getDistance());
        for (const Vertex stop : initialTransfers.getForwardPOIs()) {
            if (stop == targetStop) continue;
            AssertMsg(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
            AssertMsg(initialTransfers.getForwardDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
            const int arrivalTime = sourceDepartureTime + initialTransfers.getForwardDistance(stop);
            arrivalByTransfer(StopId(stop), arrivalTime, sourceVertex, sourceDepartureTime);
        }
        if constexpr (ALLOW_DIRECT_WALKING) {
            if (initialTransfers.getDistance() != INFTY) {
                const int arrivalTime = sourceDepartureTime + initialTransfers.getDistance();
                arrivalByTransfer(targetStop, arrivalTime, sourceVertex, sourceDepartureTime);
            }
        }
        debugger.stopRelaxTransfers();
    }

    inline void relaxIntermediateTransfers() noexcept {
        debugger.startRelaxTransfers();
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        AssertMsg(queue.empty(), "Queue still has " << queue.size() << " elements!");
        for (const StopId stop : stopsUpdatedByRoute) {
            if (initialTransfers.getBackwardDistance(stop) != INFTY) {
                const int arrivalTime = earliestArrivalPerRoute[stop] + initialTransfers.getBackwardDistance(stop);
                if (arrivalByEdge<false>(label[targetVertex], arrivalTime, stop)) {
                    arrivalByTransfer(targetStop, arrivalTime, stop, earliestArrivalPerRoute[stop]);
                }
            }
            arrivalByEdge<true>(label[stop], earliestArrivalPerRoute[stop], stop);
        }
        dijkstra();
        debugger.stopRelaxTransfers();
    }

    inline void dijkstra() noexcept {
        DijkstraLabel& targetLabel = label[targetVertex];
        while (!queue.empty()) {
            DijkstraLabel* uLabel = queue.extractFront();
            if (uLabel->arrivalTime > targetLabel.arrivalTime) break;
            const Vertex u = Vertex(uLabel - &(label[0]));
            for (Edge edge : data.transferGraph.edgesFrom(u)) {
                const Vertex v = data.transferGraph.get(ToVertex, edge);
                if (v == targetVertex || v == uLabel->parent) continue;
                debugger.relaxEdge(edge);
                arrivalByEdge<true>(label[v], uLabel->arrivalTime + data.transferGraph.get(TravelTime, edge), uLabel->parent);
            }
            if (data.isStop(u)) {
                arrivalByTransfer(StopId(u), uLabel->arrivalTime, uLabel->parent, earliestArrivalPerRoute[uLabel->parent]);
            }
            debugger.settleVertex(u);
        }
        if (!queue.empty()) {
            queue.clear();
        }
    }

    inline Round& currentRound() noexcept {
        AssertMsg(roundIndex < rounds.size(), "Round index is out of bounds (roundIndex = " << roundIndex << ", rounds.size() = " << rounds.size() << ")!");
        return rounds[roundIndex];
    }

    inline Round& previousRound() noexcept {
        AssertMsg(roundIndex - 1 < rounds.size(), "Round index is out of bounds (roundIndex = " << roundIndex << ", rounds.size() = " << rounds.size() << ")!");
        AssertMsg(roundIndex > 0, "Cannot return previous round, because no round exists!");
        return rounds[roundIndex - 1];
    }

    inline void startNewRound() noexcept {
        AssertMsg(roundIndex + 1 <= rounds.size(), "Round index is out of bounds (roundIndex = " << roundIndex << ", rounds.size() = " << rounds.size() << ")!");
        roundIndex++;
        if (roundIndex == rounds.size()) rounds.emplace_back(data.numberOfStops() + 1);
    }

    inline bool arrivalByRoute(const StopId stop, const int arrivalTime) noexcept {
        AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
        AssertMsg(arrivalTime >= sourceDepartureTime, "Arriving by route BEFORE departing from the source (source departure time: " << String::secToTime(sourceDepartureTime) << " [" << sourceDepartureTime << "], arrival time: " << String::secToTime(arrivalTime) << " [" << arrivalTime << "], stop: " << stop << ")!");
        if (earliestArrivalPerRoute[targetStop] <= arrivalTime) return false;
        if (earliestArrivalPerRoute[stop] <= arrivalTime) return false;
        debugger.updateStopByRoute(stop, arrivalTime);
        currentRound()[stop].arrivalTime = arrivalTime;
        earliestArrivalPerRoute[stop] = arrivalTime;
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
        debugger.updateStopByTransfer(stop, arrivalTime);
        if (data.isStop(stop)) stopsUpdatedByTransfer.insert(stop);
        EarliestArrivalLabel& label = currentRound()[stop];
        if (label.arrivalTime <= arrivalTime) return;
        label.arrivalTime = arrivalTime;
        label.parent = parent;
        label.parentDepartureTime = parentDepartureTime;
        label.usesRoute = false;
    }

private:
    const Data& data;
    TransferGraph minChangeTimeGraph;

    CoreCHInitialTransfers initialTransfers;

    std::vector<Round> rounds;
    size_t roundIndex;

    std::vector<int> earliestArrivalPerRoute;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    Vertex sourceVertex;
    Vertex targetVertex;
    StopId targetStop;
    int sourceDepartureTime;

    std::vector<DijkstraLabel> label;
    ExternalKHeap<2, DijkstraLabel> queue;

    Debugger debugger;

};

}
