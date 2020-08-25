/**********************************************************************************

 Copyright (c) 2019 Tobias Zündorf

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

#include "../Dijkstra/Dijkstra.h"

#include "../../Helpers/Meta.h"
#include "../../Helpers/Helpers.h"

#include "../../DataStructures/Container/Map.h"
#include "../../DataStructures/Container/Set.h"
#include "../../DataStructures/Container/ExternalKHeap.h"
#include "../../DataStructures/RAPTOR/Data.h"

namespace ULTRA {

template<bool DEBUG = false, bool REQUIRE_DIRECT_TRANSFER = false>
class RangeSearchUsingStations {

public:
    inline static constexpr bool Debug = DEBUG;
    inline static constexpr bool RequireDirectTransfer = REQUIRE_DIRECT_TRANSFER;
    using Type = RangeSearchUsingStations<Debug, RequireDirectTransfer>;

public:
    struct ArrivalLabel : public ExternalKHeapElement {
        ArrivalLabel() : arrivalTime(never) {}
        int arrivalTime;
        inline bool hasSmallerKey(const ArrivalLabel* const other) const noexcept {
            return arrivalTime < other->arrivalTime;
        }
    };

    struct DepartureLabel {
        DepartureLabel(const RouteId routeId = noRouteId, const StopIndex stopIndex = noStopIndex, const int departureTime = never) : route(routeId, stopIndex), departureTime(departureTime) {}
        RAPTOR::RouteSegment route;
        int departureTime;
        inline bool operator<(const DepartureLabel& other) const noexcept {
            return (departureTime > other.departureTime) || ((departureTime == other.departureTime) && (route.routeId < other.route.routeId));
        }
    };

    struct ConsolidatedDepartureLabel {
        ConsolidatedDepartureLabel(const int departureTime = never) : departureTime(departureTime) {}
        std::vector<RAPTOR::RouteSegment> routes;
        int departureTime;
        inline bool operator<(const ConsolidatedDepartureLabel& other) const noexcept {
            return departureTime > other.departureTime;
        }
    };

    struct Station {
        Station() : representative(noStop) {}
        StopId representative;
        std::vector<StopId> stops;
        inline void add(const StopId stop) noexcept {
            if (representative > stop) {
                representative = stop;
            }
            stops.emplace_back(stop);
        }
    };

public:
    RangeSearchUsingStations(const RAPTOR::Data& data, DynamicTransferGraph& shortcutGraph, const int witnessTransferLimit) :
        data(data),
        shortcutGraph(shortcutGraph),
        stationOfStop(data.numberOfStops()),
        sourceStation(),
        sourceDepartureTime(0),
        shortcutCandidatesInQueue(0),
        shortcutOriginCandidates(data.numberOfStops() + 1),
        shortcutDestinationCandidates(data.numberOfStops()),
        routesServingUpdatedStops(data.numberOfRoutes()),
        stopsUpdatedByRoute(data.numberOfStops()),
        stopsUpdatedByTransfer(data.numberOfStops()),
        witnessTransferLimit(witnessTransferLimit),
        earliestDepartureTime(data.getMinDepartureTime()) {
        AssertMsg(data.hasImplicitBufferTimes(), "Shortcut search requires implicit departure buffer times!");
        Dijkstra<TransferGraph, false> dijkstra(data.transferGraph);
        for (const StopId stop : data.stops()) {
            dijkstra.run(stop, noVertex, [&](const Vertex u) {
                if (!data.isStop(u)) return;
                stationOfStop[stop].add(StopId(u));
            }, NoOperation, [&](const Vertex, const Edge edge) {
                return data.transferGraph.get(TravelTime, edge) > 0;
            });
        }
    }

    inline void run(const StopId source, const int minTime, const int maxTime) noexcept {
        AssertMsg(data.isStop(source), "source (" << source << ") is not a stop!");
        if (stationOfStop[source].representative != source) return;
        setSource(source);
        std::vector<ConsolidatedDepartureLabel> departures = collectDepartures(minTime, maxTime);
        for (const ConsolidatedDepartureLabel& label : departures) {
            runForDepartureTime(label);
            for (const StopId shortcutDestination : getShortcutDestinationStops()) {
                const StopId shortcutOrigin = getShortcutOriginStop(shortcutDestination);
                if (!shortcutGraph.hasEdge(shortcutOrigin, shortcutDestination)) {
                    shortcutGraph.addEdge(shortcutOrigin, shortcutDestination).set(TravelTime, getShortcutTravelTime(shortcutDestination));
                } else {
                    AssertMsg(shortcutGraph.get(TravelTime, shortcutGraph.findEdge(shortcutOrigin, shortcutDestination)) == getShortcutTravelTime(shortcutDestination), "Edge from " << shortcutOrigin << " to " << shortcutDestination << " has inconclusive travel time (" << shortcutGraph.get(TravelTime, shortcutGraph.findEdge(shortcutOrigin, shortcutDestination)) << ", " << getShortcutTravelTime(shortcutDestination) << ")");
                }
            }
        }
    }

private:
    inline void setSource(const StopId sourceStop) noexcept {
        AssertMsg(directTransferQueue.empty(), "Queue for round 0 is not empty!");
        AssertMsg(stationOfStop[sourceStop].representative == sourceStop, "Source " << sourceStop << " is not representative of its station!");
        clear();
        sourceStation = stationOfStop[sourceStop];
        dijkstra<-1>();
        sort(stopsReachedByDirectTransfer);
        if constexpr (Debug) std::cout << "   Source stop: " << sourceStop << std::endl;
        if constexpr (Debug) std::cout << "   Number of stops reached by direct transfer: " << String::prettyInt(stopsReachedByDirectTransfer.size()) << std::endl;
    }

    inline void runForDepartureTime(const ConsolidatedDepartureLabel& label) noexcept {
        if constexpr (Debug) std::cout << "   Running search for departure time: " << label.departureTime << " (" << String::secToTime(label.departureTime) << ")" << std::endl;

        shortcutCandidatesInQueue = 0;
        shortcutOriginCandidates.clear();
        shortcutDestinationCandidates.clear();
        shortcutDestinationStops.clear();

        routesServingUpdatedStops.clear();
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();

        sourceDepartureTime = label.departureTime;
        for (const StopId stop : sourceStation.stops) {
            zeroTripsArrivalLabels[stop].arrivalTime = label.departureTime;
            oneTripArrivalLabels[stop].arrivalTime = label.departureTime;
            twoTripsArrivalLabels[stop].arrivalTime = label.departureTime;
        }

        relaxTransfers<0>();
        for (const StopId stop : sourceStation.stops) {
            AssertMsg(!stopsUpdatedByTransfer.contains(stop), "Source was updated by transfer!");
        }
        collectRoutesServingUpdatedStops(label.routes);
        scanRoutes<1>();
        for (const StopId stop : sourceStation.stops) {
            stopsUpdatedByTransfer.insert(stop);
        }
        collectRoutesServingUpdatedStops<-1>();
        scanRoutes<1>();
        relaxTransfers<1>();
        collectRoutesServingUpdatedStops<1>();
        scanRoutes<2>();
        relaxTransfers<2>();
    }

    inline const std::vector<StopId>& getShortcutDestinationStops() const noexcept {
        return shortcutDestinationStops;
    }

    inline StopId getShortcutOriginStop(const StopId destinationStop) const noexcept {
        AssertMsg(data.isStop(destinationStop), "Destination " << destinationStop << " is not a stop!");
        AssertMsg(data.isStop(oneTripTransferParent[destinationStop]), "Origin " << oneTripTransferParent[destinationStop] << " is not a stop!");
        return oneTripTransferParent[destinationStop];
    }

    inline int getShortcutTravelTime(const StopId destinationStop) noexcept {
        AssertMsg(data.isStop(destinationStop), "");
        AssertMsg(data.isStop(oneTripTransferParent[destinationStop]), "");
        return oneTripArrivalLabels[destinationStop].arrivalTime - oneTripArrivalLabels[oneTripTransferParent[destinationStop]].arrivalTime;
    }

    inline std::vector<ConsolidatedDepartureLabel> collectDepartures(const int minTime, const int maxTime) noexcept {
        AssertMsg(directTransferArrivalLabels[sourceStation.representative].arrivalTime == 0, "Direct transfer for source " << sourceStation.representative << " is incorrect!");
        const int cutoffTime = std::max(minTime, earliestDepartureTime);
        std::vector<DepartureLabel> departureLabels;
        for (const RouteId route : data.routes()) {
            const StopId* stops = data.stopArrayOfRoute(route);
            const size_t tripSize = data.numberOfStopsInRoute(route);
            int minimalTransferTime = never;
            for (size_t stopIndex = 0; stopIndex + 1 < tripSize; stopIndex++) {
                if (directTransferArrivalLabels[stops[stopIndex]].arrivalTime > minimalTransferTime) continue;
                minimalTransferTime = directTransferArrivalLabels[stops[stopIndex]].arrivalTime;
                for (const RAPTOR::StopEvent* trip = data.firstTripOfRoute(route); trip <= data.lastTripOfRoute(route); trip += tripSize) {
                    const int departureTime = trip[stopIndex].departureTime - minimalTransferTime;
                    if (departureTime < cutoffTime) continue;
                    if (departureTime > maxTime) break;
                    if (stationOfStop[stops[stopIndex]].representative == sourceStation.representative) {
                        departureLabels.emplace_back(noRouteId, noStopIndex, departureTime);
                    } else {
                        departureLabels.emplace_back(route, StopIndex(stopIndex), departureTime);
                    }
                }
            }
        }
        sort(departureLabels);
        std::vector<ConsolidatedDepartureLabel> result(1);
        for (const DepartureLabel& label : departureLabels) {
            if (label.route.routeId == noRouteId) {
                if (label.departureTime == result.back().departureTime) continue;
                result.back().departureTime = label.departureTime;
                result.emplace_back(label.departureTime);
            } else {
                result.back().routes.emplace_back(label.route);
            }
        }
        result.pop_back();
        return result;
    }

private:
    inline void clear() noexcept {
        sourceStation = Station();

        std::vector<ArrivalLabel>(data.transferGraph.numVertices()).swap(directTransferArrivalLabels);
        stopsReachedByDirectTransfer.clear();

        std::vector<ArrivalLabel>(data.numberOfStops()).swap(zeroTripsArrivalLabels);

        oneTripQueue.clear();
        std::vector<ArrivalLabel>(data.transferGraph.numVertices()).swap(oneTripArrivalLabels);

        twoTripsQueue.clear();
        std::vector<ArrivalLabel>(data.transferGraph.numVertices()).swap(twoTripsArrivalLabels);

        std::vector<StopId>(data.transferGraph.numVertices(), noStop).swap(oneTripTransferParent);
        std::vector<StopId>(data.numberOfStops(), noStop).swap(twoTripsRouteParent);

        shortcutCandidatesInQueue = 0;
        shortcutOriginCandidates.clear();
        shortcutDestinationCandidates.clear();
        shortcutDestinationStops.clear();

        routesServingUpdatedStops.clear();
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
    }

    template<int ROUND>
    inline void collectRoutesServingUpdatedStops() noexcept {
        if constexpr (ROUND == 0) {
            for (const RouteId routeId : data.routes()) {
                const RAPTOR::StopEvent* lastTripOfRoute = data.lastTripOfRoute(routeId);
                for (StopIndex stopIndex = StopIndex(0); stopIndex + 1 < data.numberOfStopsInRoute(routeId); stopIndex++) {
                    const StopId stop = data.stopArrayOfRoute(routeId)[stopIndex];
                    if (!stopsUpdatedByTransfer.contains(stop)) continue;
                    AssertMsg(data.isRoute(routeId), "Route " << routeId << " is out of range!");
                    AssertMsg(data.stopIds[data.firstStopIdOfRoute[routeId] + stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                    if (lastTripOfRoute[stopIndex].departureTime < arrivalTime<0>(stop)) continue;
                    routesServingUpdatedStops.insert(routeId, stopIndex);
                    break;
                }
            }
        } else {
            for (const StopId stop : stopsUpdatedByTransfer) {
                for (const RAPTOR::RouteSegment& route : data.routesContainingStop(stop)) {
                    AssertMsg(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                    AssertMsg(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                    if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId)) continue;
                    if (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < arrivalTime<(ROUND == -1) ? (0) : (ROUND)>(stop)) continue;
                    if (routesServingUpdatedStops.contains(route.routeId)) {
                        routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
                    } else {
                        routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                    }
                }
            }
        }
    }

    inline void collectRoutesServingUpdatedStops(const std::vector<RAPTOR::RouteSegment>& routes) noexcept {
        for (const RAPTOR::RouteSegment& route : routes) {
            AssertMsg(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
            AssertMsg(route.stopIndex + 1 < data.numberOfStopsInRoute(route.routeId), "RouteSegment " << route << " is not a departure event!");
            AssertMsg(data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime >= arrivalTime<0>(data.stopOfRouteSegment(route)), "RouteSegment " << route << " is not reachable!");
            if (routesServingUpdatedStops.contains(route.routeId)) {
                routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
            } else {
                routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
            }
        }
    }

    template<int CURRENT>
    inline void scanRoutes() noexcept {
        for (const auto [route, stopIndex] : routesServingUpdatedStops.map()) {
            RAPTOR::TripIterator tripIterator = data.getTripIterator(route, stopIndex);
            StopIndex parentIndex = stopIndex;
            while (tripIterator.hasFurtherStops()) {
                if (tripIterator.hasEarlierTrip() && (tripIterator.previousDepartureTime() >= arrivalTime<CURRENT - 1>(tripIterator.stop()))) {
                    do {
                        tripIterator.previousTrip();
                    } while (tripIterator.hasEarlierTrip() && (tripIterator.previousDepartureTime() >= arrivalTime<CURRENT - 1>(tripIterator.stop())));
                    if (!stopsUpdatedByTransfer.contains(tripIterator.stop())) {
                        if (!tripIterator.hasEarlierTrip()) break;
                        do {
                            tripIterator.nextStop();
                        } while (tripIterator.hasFurtherStops() && ((!stopsUpdatedByTransfer.contains(tripIterator.stop())) || (tripIterator.previousDepartureTime() < arrivalTime<CURRENT - 1>(tripIterator.stop()))));
                        continue;
                    }
                    parentIndex = tripIterator.getStopIndex();
                }
                tripIterator.nextStop();
                const int newArrivalTime = tripIterator.arrivalTime();
                if (newArrivalTime < arrivalTime<CURRENT>(tripIterator.stop())) {
                    arrivalByRoute<CURRENT>(tripIterator.stop(), newArrivalTime, tripIterator.stop(parentIndex));
                }
            }
        }
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
    }

    template<int ROUND>
    inline void relaxTransfers() noexcept {
        AssertMsg(stopsUpdatedByTransfer.empty(), "StopsUpdatedByTransfer is not empty!");
        if constexpr (ROUND == 0) {
            for (const StopId stop : stopsReachedByDirectTransfer) {
                const int newArrivalTime = sourceDepartureTime + directTransferArrivalLabels[stop].arrivalTime;
                if (newArrivalTime < zeroTripsArrivalLabels[stop].arrivalTime) {
                    arrivalByEdge<ROUND>(stop, newArrivalTime, sourceStation.representative);
                    stopsUpdatedByTransfer.insert(stop);
                }
            }
        } else {
            dijkstra<ROUND>();
        }
    }

    template<int ROUND>
    inline void dijkstra() noexcept {
        static_assert((ROUND == -1) | (ROUND == 1) | (ROUND == 2), "Invalid round!");
        std::vector<ArrivalLabel>& label = getLabel<ROUND>();
        ExternalKHeap<2, ArrivalLabel>& queue = getQueue<ROUND>();

        int transferLimit = intMax;
        if constexpr (ROUND == 2) {
            transferLimit = -never;
        }

        if constexpr (ROUND == -1) {
            label[sourceStation.representative].arrivalTime = 0;
            queue.update(&(label[sourceStation.representative]));
        } else {
            if constexpr (ROUND == 1) {
                shortcutCandidatesInQueue = 0;
            }
            for (const StopId stop : stopsUpdatedByRoute) {
                queue.update(&(label[stop]));
                if constexpr (ROUND == 1) {
                    oneTripTransferParent[stop] = stop;
                    if (shortcutOriginCandidates.contains(stop)) shortcutCandidatesInQueue++;
                } else if constexpr (ROUND == 2) {
                    const StopId routeParent = twoTripsRouteParent[stop];
                    if (data.isStop(routeParent)) {
                        if (!shortcutDestinationCandidates.contains(routeParent)) {
                            shortcutDestinationCandidates.insert(routeParent);
                        }
                        shortcutDestinationCandidates[routeParent].insert(stop);
                        if (transferLimit < label[stop].arrivalTime) {
                            transferLimit = label[stop].arrivalTime;
                        }
                    }
                }
            }
            if constexpr (ROUND == 1) {
                if (shortcutCandidatesInQueue == 0) {
                    transferLimit = -never;
                }
            } else if constexpr (ROUND == 2) {
                if constexpr (Debug) std::cout << "   Transfer limit in round " << ROUND << ": " << String::secToString(transferLimit - sourceDepartureTime) << " (" << transferLimit << ")" << std::endl;
            }
        }

        while (!queue.empty()) {
            ArrivalLabel* currentLabel = queue.extractFront();
            const Vertex currentVertex = Vertex(currentLabel - &(label[0]));
            for (Edge edge : data.transferGraph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = data.transferGraph.get(ToVertex, edge);
                const int newArrivalTime = currentLabel->arrivalTime + data.transferGraph.get(TravelTime, edge);
                if (newArrivalTime < arrivalTime<ROUND>(neighborVertex)) {
                    arrivalByEdge<ROUND>(neighborVertex, newArrivalTime, currentVertex);
                }
            }
            if constexpr (ROUND == 1) {
                if (shortcutOriginCandidates.contains(oneTripTransferParent[currentVertex])) {
                    shortcutCandidatesInQueue--;
                }
                if (shortcutCandidatesInQueue == 0) {
                    shortcutCandidatesInQueue = -1;
                    transferLimit = oneTripArrivalLabels[currentVertex].arrivalTime + witnessTransferLimit;
                    if (transferLimit < oneTripArrivalLabels[currentVertex].arrivalTime) transferLimit = intMax;
                    if constexpr (Debug) std::cout << "   Walking limit in round " << ROUND << ": " << String::secToString(transferLimit - sourceDepartureTime) << std::endl;
                }
            }
            if (data.isStop(currentVertex)) {
                if constexpr (ROUND == -1) {
                    if (stationOfStop[currentVertex].representative != sourceStation.representative) {
                        stopsReachedByDirectTransfer.emplace_back(StopId(currentVertex));
                    }
                } if constexpr (ROUND == 1) {
                    stopsUpdatedByTransfer.insert(StopId(currentVertex));
                } else if constexpr (ROUND == 2) {
                    const StopId routeParent = twoTripsRouteParent[currentVertex];
                    if (data.isStop(routeParent)) {
                        if constexpr (RequireDirectTransfer) {
                            if (directTransferArrivalLabels[currentVertex].arrivalTime < never) {
                                shortcutDestinationStops.emplace_back(routeParent);
                            }
                        } else {
                            shortcutDestinationStops.emplace_back(routeParent);
                        }
                        AssertMsg(shortcutDestinationCandidates.contains(routeParent), "Vertex " << currentVertex << " has route parent " << routeParent << " but the route parent does not know about this!");
                        for (const StopId witness : shortcutDestinationCandidates[routeParent]) {
                            twoTripsRouteParent[witness] = noStop;
                        }
                        shortcutDestinationCandidates.remove(routeParent);
                    }
                    if (shortcutDestinationCandidates.empty()) {
                        break;
                    }
                }
            }
            if (currentLabel->arrivalTime > transferLimit) {
                break;
            }
        }

        if constexpr (ROUND == -1) {
            if (!queue.empty()) queue.clear();
        } else if constexpr (ROUND == 1) {
            for (const ArrivalLabel* unsettledLabel : queue.data()) {
                oneTripTransferParent[unsettledLabel - &(label[0])] = StopId(data.numberOfStops());
            }
        } else if constexpr (ROUND == 2) {
            AssertMsg(shortcutDestinationCandidates.empty(), "There are still shortcut destination candidates left (" << shortcutDestinationCandidates.size() << ")!");
        }

        stopsUpdatedByRoute.clear();
    }

    template<int ROUND>
    inline int arrivalTime(const Vertex vertex) const noexcept {
        if constexpr (ROUND == -1) {
            return directTransferArrivalLabels[vertex].arrivalTime;
        } else if constexpr (ROUND == 0) {
            AssertMsg(data.isStop(vertex), "Arrival time in round 0 only available for stops!");
            return zeroTripsArrivalLabels[vertex].arrivalTime;
        } else if constexpr (ROUND == 1) {
            return oneTripArrivalLabels[vertex].arrivalTime;
        } else if constexpr (ROUND == 2) {
            return twoTripsArrivalLabels[vertex].arrivalTime;
        } else {
            AssertMsg(false, "There is no arrival time for vertex " << vertex << " in round " << ROUND);
        }
    }

    template<int ROUND>
    inline void arrivalByRoute(const StopId stop, const int arrivalTime, const StopId parent) noexcept {
        if constexpr (ROUND == 1) {
            if (stationOfStop[parent].representative == sourceStation.representative) {
                shortcutOriginCandidates.insert(stop);
            }
            oneTripArrivalLabels[stop].arrivalTime = arrivalTime;
            if (oneTripArrivalLabels[stop].isOnHeap()) {
                oneTripQueue.remove(&(oneTripArrivalLabels[stop]));
            }
            if (twoTripsArrivalLabels[stop].arrivalTime > arrivalTime) {
                twoTripsArrivalLabels[stop].arrivalTime = arrivalTime;
                if (twoTripsArrivalLabels[stop].isOnHeap()) {
                    twoTripsQueue.remove(&(twoTripsArrivalLabels[stop]));
                }
            }
        } else if constexpr (ROUND == 2) {
            if (shortcutOriginCandidates.contains(oneTripTransferParent[parent]) && (oneTripTransferParent[parent] != parent) && (!shortcutAlreadyExists(parent))) {
                twoTripsRouteParent[stop] = parent;
            } else {
                twoTripsRouteParent[stop] = noStop;
            }
            twoTripsArrivalLabels[stop].arrivalTime = arrivalTime;
            if (twoTripsArrivalLabels[stop].isOnHeap()) {
                twoTripsQueue.remove(&(twoTripsArrivalLabels[stop]));
            }
        } else {
            AssertMsg(false, "Arrival by route in round " << ROUND);
        }
        stopsUpdatedByRoute.insert(stop);
    }

    inline bool shortcutAlreadyExists(const StopId parent) const noexcept {
        return shortcutGraph.hasEdge(oneTripTransferParent[parent], parent);
    }

    template<int ROUND>
    inline void arrivalByEdge(const Vertex vertex, const int arrivalTime, const Vertex parent) noexcept {
        if constexpr (ROUND == -1) {
            suppressUnusedParameterWarning(parent);
            directTransferArrivalLabels[vertex].arrivalTime = arrivalTime;
            directTransferQueue.update(&(directTransferArrivalLabels[vertex]));
        } else if constexpr (ROUND == 0) {
            suppressUnusedParameterWarning(parent);
            zeroTripsArrivalLabels[vertex].arrivalTime = arrivalTime;
            if (oneTripArrivalLabels[vertex].arrivalTime > arrivalTime) {
                oneTripArrivalLabels[vertex].arrivalTime = arrivalTime;
                if (oneTripArrivalLabels[vertex].isOnHeap()) {
                    oneTripQueue.remove(&(oneTripArrivalLabels[vertex]));
                }
                if (twoTripsArrivalLabels[vertex].arrivalTime > arrivalTime) {
                    twoTripsArrivalLabels[vertex].arrivalTime = arrivalTime;
                    if (twoTripsArrivalLabels[vertex].isOnHeap()) {
                        twoTripsQueue.remove(&(twoTripsArrivalLabels[vertex]));
                    }
                }
            }
        } else if constexpr (ROUND == 1) {
            if (isShortcutCandidate(vertex)) shortcutCandidatesInQueue--;
            if (shortcutOriginCandidates.contains(oneTripTransferParent[parent])) shortcutCandidatesInQueue++;
            oneTripTransferParent[vertex] = oneTripTransferParent[parent];
            oneTripArrivalLabels[vertex].arrivalTime = arrivalTime;
            if (twoTripsArrivalLabels[vertex].arrivalTime > arrivalTime) {
                twoTripsArrivalLabels[vertex].arrivalTime = arrivalTime;
                if (twoTripsArrivalLabels[vertex].isOnHeap()) {
                    twoTripsQueue.remove(&(twoTripsArrivalLabels[vertex]));
                }
            }
            oneTripQueue.update(&(oneTripArrivalLabels[vertex]));
        } else if constexpr (ROUND == 2) {
            suppressUnusedParameterWarning(parent);
            twoTripsArrivalLabels[vertex].arrivalTime = arrivalTime;
            twoTripsQueue.update(&(twoTripsArrivalLabels[vertex]));
        } else {
            suppressUnusedParameterWarning(parent);
            AssertMsg(false, "Arrival by edge in round " << ROUND);
        }
        if (!data.isStop(vertex)) return;
        if constexpr (ROUND == 1) {
            shortcutOriginCandidates.remove(StopId(vertex));
        } else if constexpr (ROUND == 2) {
            if (hasTwoTripsRouteParent(vertex)) {
                const StopId routeParent = twoTripsRouteParent[vertex];
                AssertMsg(shortcutDestinationCandidates.contains(routeParent), "Vertex " << vertex << " has route parent " << routeParent << " but the route parent does not know about this!");
                AssertMsg(shortcutDestinationCandidates[routeParent].contains(StopId(vertex)), "Vertex " << vertex << " is not contained in shortcutDestinationCandidates List of " << routeParent << "!");
                shortcutDestinationCandidates[routeParent].erase(StopId(vertex));
                if (shortcutDestinationCandidates[routeParent].empty()) {
                    shortcutDestinationCandidates.remove(routeParent);
                }
            }
            twoTripsRouteParent[vertex] = noStop;
        }
    }

    inline bool isShortcutCandidate(const Vertex vertex) const noexcept {
        if (oneTripArrivalLabels[vertex].isOnHeap()) {
            AssertMsg(isStop(oneTripTransferParent[vertex]), "Vertex " << vertex << " has no one trip transfer parent!");
            return shortcutOriginCandidates.contains(oneTripTransferParent[vertex]);
        }
        return false;
    }

    inline bool notIsShortcutCandidate(const Vertex vertex) const noexcept {
        if (oneTripArrivalLabels[vertex].isOnHeap()) {
            AssertMsg(isStop(oneTripTransferParent[vertex]), "Vertex " << vertex << " has no one trip transfer parent!");
            return !shortcutOriginCandidates.contains(oneTripTransferParent[vertex]);
        }
        return false;
    }

    inline bool hasTwoTripsRouteParent(const StopId stop) const noexcept {
        return twoTripsArrivalLabels[stop].isOnHeap() & data.isStop(twoTripsRouteParent[stop]);
    }

    inline bool hasTwoTripsRouteParent(const Vertex vertex) const noexcept {
        return data.isStop(vertex) && hasTwoTripsRouteParent(StopId(vertex));
    }

    inline bool isStop(const Vertex vertex) const noexcept {
        return vertex <= data.numberOfStops();
    }

    template<int ROUND>
    inline std::vector<ArrivalLabel>& getLabel() noexcept {
        if constexpr (ROUND == -1) {
            return directTransferArrivalLabels;
        } else if constexpr (ROUND == 1) {
            return oneTripArrivalLabels;
        } else if constexpr (ROUND == 2) {
            return twoTripsArrivalLabels;
        } else {
            AssertMsg(false, "There are no labels for round " << ROUND);
        }
    }

    template<int ROUND>
    inline ExternalKHeap<2, ArrivalLabel>& getQueue() noexcept {
        if constexpr (ROUND == -1) {
            return directTransferQueue;
        } else if constexpr (ROUND == 1) {
            return oneTripQueue;
        } else if constexpr (ROUND == 2) {
            return twoTripsQueue;
        } else {
            AssertMsg(false, "There is no queue for round " << ROUND);
        }
    }

private:
    const RAPTOR::Data& data;
    DynamicTransferGraph& shortcutGraph;
    std::vector<Station> stationOfStop;

    Station sourceStation;
    int sourceDepartureTime;

    std::vector<ArrivalLabel> directTransferArrivalLabels;
    ExternalKHeap<2, ArrivalLabel> directTransferQueue;
    std::vector<StopId> stopsReachedByDirectTransfer;

    std::vector<ArrivalLabel> zeroTripsArrivalLabels;

    std::vector<ArrivalLabel> oneTripArrivalLabels;
    ExternalKHeap<2, ArrivalLabel> oneTripQueue;

    std::vector<ArrivalLabel> twoTripsArrivalLabels;
    ExternalKHeap<2, ArrivalLabel> twoTripsQueue;

    std::vector<StopId> oneTripTransferParent;
    std::vector<StopId> twoTripsRouteParent;

    size_t shortcutCandidatesInQueue;
    IndexedSet<false, StopId> shortcutOriginCandidates;
    IndexedMap<Set<StopId>, false, StopId> shortcutDestinationCandidates;
    std::vector<StopId> shortcutDestinationStops;

    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;
    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;

    int witnessTransferLimit;

    int earliestDepartureTime;

};

}
