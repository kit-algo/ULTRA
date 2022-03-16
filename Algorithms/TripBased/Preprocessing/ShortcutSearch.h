#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../Dijkstra/Dijkstra.h"

#include "../../../Helpers/Meta.h"
#include "../../../Helpers/Helpers.h"
#include "../../../Helpers/Vector/Vector.h"

#include "../../../DataStructures/Container/Map.h"
#include "../../../DataStructures/Container/Set.h"
#include "../../../DataStructures/Container/ExternalKHeap.h"
#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../../DataStructures/TripBased/Shortcut.h"

namespace TripBased {

template<bool DEBUG = false, bool IGNORE_ISOLATED_CANDIDATES = false>
class ShortcutSearch {

public:
    inline static constexpr bool Debug = DEBUG;
    inline static constexpr bool IgnoreIsolatedCandidates = IGNORE_ISOLATED_CANDIDATES;
    using Type = ShortcutSearch<Debug, IgnoreIsolatedCandidates>;

public:
    struct ArrivalLabel : public ExternalKHeapElement {
        ArrivalLabel() : arrivalTime(never) {}
        int arrivalTime;
        inline bool hasSmallerKey(const ArrivalLabel* const other) const noexcept {
            return arrivalTime < other->arrivalTime || (arrivalTime == other->arrivalTime && this < other);
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
    ShortcutSearch(const Data& tripData, const int witnessTransferLimit) :
        tripData(tripData),
        data(tripData.raptorData),
        stationOfStop(data.numberOfStops()),
        sourceStation(),
        sourceDepartureTime(0),
        twoTripsRouteParentEvent(tripData.numberOfStopEvents()),
        shortcutCandidatesInQueue(0),
        shortcutDestinationCandidates(data.numberOfStopEvents()),
        routesServingUpdatedStops(data.numberOfRoutes()),
        stopsUpdatedByRoute(data.numberOfStops()),
        stopsUpdatedByTransfer(data.numberOfStops()),
        witnessTransferLimit(witnessTransferLimit),
        earliestDepartureTime(data.getMinDepartureTime()),
        timestamp(0) {
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
        for (const ConsolidatedDepartureLabel& label : collectDepartures(minTime, maxTime)) {
            runForDepartureTime(label);
        }
    }

    inline const std::vector<Shortcut>& getShortcuts() noexcept {
        return shortcuts;
    }

private:
    inline void setSource(const StopId source) noexcept {
        AssertMsg(directTransferQueue.empty(), "Queue for round 0 is not empty!");
        AssertMsg(stationOfStop[source].representative == source, "Source " << source << " is not representative of its station!");
        clear();
        sourceStation = stationOfStop[source];
        initialDijkstra();
        sort(stopsReachedByDirectTransfer);
        if constexpr (Debug) {
            std::cout << "   Source stop: " << source << std::endl;
            std::cout << "   Number of stops reached by direct transfer: " << String::prettyInt(stopsReachedByDirectTransfer.size()) << std::endl;
        }
    }

    inline void runForDepartureTime(const ConsolidatedDepartureLabel& label) noexcept {
        if constexpr (Debug) std::cout << "   Running search for departure time: " << label.departureTime << " (" << String::secToTime(label.departureTime) << ")" << std::endl;

        timestamp++;
        shortcutCandidatesInQueue = 0;
        shortcutDestinationCandidates.clear();

        routesServingUpdatedStops.clear();
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();

        sourceDepartureTime = label.departureTime;
        relaxInitialTransfers();
        collectRoutes1(label.routes);
        scanRoutes<1>();
        intermediateDijkstra();
        collectRoutes2();
        scanRoutes<2>();
        finalDijkstra();
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
                    }
                    departureLabels.emplace_back(route, StopIndex(stopIndex), departureTime);
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
        for (ConsolidatedDepartureLabel& label : result) {
            sort(label.routes);
        }
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
        std::vector<u_int16_t>(data.transferGraph.numVertices(), 0).swap(oneTripTimestamps);

        twoTripsQueue.clear();
        std::vector<ArrivalLabel>(data.transferGraph.numVertices()).swap(twoTripsArrivalLabels);
        std::vector<u_int16_t>(data.transferGraph.numVertices(), 0).swap(twoTripsTimestamps);

        std::vector<StopEventId>(data.transferGraph.numVertices(), noStopEvent).swap(oneTripTransferParent);
        std::vector<StopId>(data.numberOfStops(), noStop).swap(twoTripsRouteParent);

        shortcutCandidatesInQueue = 0;
        shortcutDestinationCandidates.clear();

        routesServingUpdatedStops.clear();
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
    }

    inline void collectRoutes1(const std::vector<RAPTOR::RouteSegment>& routes) noexcept {
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
        AssertMsg(routesServingUpdatedStops.isSortedByKeys(), "Collected route segments are not sorted!");
    }

    inline void collectRoutes2() noexcept {
        for (const StopId stop : stopsUpdatedByTransfer) {
            for (const RAPTOR::RouteSegment& route : data.routesContainingStop(stop)) {
                AssertMsg(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                AssertMsg(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId)) continue;
                if (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < oneTripArrivalLabels[stop].arrivalTime) continue;
                if (routesServingUpdatedStops.contains(route.routeId)) {
                    routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                }
            }
        }
        routesServingUpdatedStops.sortKeys();
    }

    template<int CURRENT>
    inline void scanRoutes() noexcept {
        static_assert((CURRENT == 1) | (CURRENT == 2), "Invalid round!");
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            const StopIndex stopIndex = routesServingUpdatedStops[route];
            RAPTOR::TripIterator tripIterator = data.getTripIterator(route, stopIndex);
            StopIndex parentIndex = stopIndex;
            while (tripIterator.hasFurtherStops()) {
                //Find earliest trip that can be entered
                if (tripIterator.hasEarlierTrip() && (tripIterator.previousDepartureTime() >= arrivalTime<CURRENT - 1>(tripIterator.stop()))) {
                    do {
                        tripIterator.previousTrip();
                    } while (tripIterator.hasEarlierTrip() && (tripIterator.previousDepartureTime() >= arrivalTime<CURRENT - 1>(tripIterator.stop())));
                    if (!stopsUpdatedByTransfer.contains(tripIterator.stop())) {
                        //Trip was improved by an arrival that was found during a previous RAPTOR iteration.
                        //We already explored this trip during that iteration.
                        //Fast forward to the next stop that was updated in the current iteration and can enter the current trip.
                        do {
                            tripIterator.nextStop();
                        } while (tripIterator.hasFurtherStops() && ((!stopsUpdatedByTransfer.contains(tripIterator.stop())) || (tripIterator.departureTime() < arrivalTime<CURRENT - 1>(tripIterator.stop()))));
                        parentIndex = tripIterator.getStopIndex();
                        continue;
                    }
                    parentIndex = tripIterator.getStopIndex();
                }
                //Candidates may dominate equivalent labels from previous iterations
                else if (stopsUpdatedByTransfer.contains(tripIterator.stop()) && !isFromCurrentIteration<CURRENT - 1>(tripIterator.stop(parentIndex)) && isCandidate<CURRENT>(tripIterator.stop()) && tripIterator.departureTime() >= arrivalTime<CURRENT - 1>(tripIterator.stop())) {
                    parentIndex = tripIterator.getStopIndex();
                }
                tripIterator.nextStop();
                const int newArrivalTime = tripIterator.arrivalTime();
                if (newArrivalTime < arrivalTime<CURRENT>(tripIterator.stop())) {
                    if constexpr (CURRENT == 1) {
                        arrivalByRoute<CURRENT>(tripIterator.stop(), newArrivalTime, tripIterator.stop(parentIndex), StopEventId(tripIterator.stopEvent() - (&(data.stopEvents[0]))));
                    } else {
                        arrivalByRoute<CURRENT>(tripIterator.stop(), newArrivalTime, tripIterator.stop(parentIndex), StopEventId(tripIterator.stopEvent(parentIndex) - (&(data.stopEvents[0]))));
                    }
                }
                //Candidates may dominate equivalent labels from previous iterations
                else if (newArrivalTime == arrivalTime<CURRENT>(tripIterator.stop()) && !isFromCurrentIteration<CURRENT>(tripIterator.stop()) && newArrivalTime < arrivalTime<CURRENT - 1>(tripIterator.stop())) {
                    const StopId parent = tripIterator.stop(parentIndex);
                    if constexpr (CURRENT == 1) {
                        if (stationOfStop[parent].representative == sourceStation.representative) {
                            arrivalByRoute<CURRENT>(tripIterator.stop(), newArrivalTime, parent, StopEventId(tripIterator.stopEvent() - (&(data.stopEvents[0]))));
                        }
                    } else {
                        if (oneTripTransferParent[parent] != noStopEvent) {
                            arrivalByRoute<CURRENT>(tripIterator.stop(), newArrivalTime, parent, StopEventId(tripIterator.stopEvent(parentIndex) - (&(data.stopEvents[0]))));
                        }
                    }
                }
            }
        }
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
    }

    template<int CURRENT>
    inline bool isCandidate(const StopId parent) const noexcept {
        static_assert((CURRENT == 1) | (CURRENT == 2), "Invalid round!");
        if constexpr (CURRENT == 1) {
            return stationOfStop[parent].representative == sourceStation.representative;
        } else {
            return oneTripTransferParent[parent] != noStopEvent;
        }
    }

    template<int CURRENT>
    inline bool isFromCurrentIteration(const StopId stop) const noexcept {
        static_assert((CURRENT == 0) | (CURRENT == 1) | (CURRENT == 2), "Invalid round!");
        if constexpr (CURRENT == 0) {
            suppressUnusedParameterWarning(stop);
            return true;
        } else if constexpr (CURRENT == 1) {
            return oneTripTimestamps[stop] == timestamp;
        } else {
            return twoTripsTimestamps[stop] == timestamp;
        }
    }

    inline void initialDijkstra() noexcept {
        directTransferArrivalLabels[sourceStation.representative].arrivalTime = 0;
        directTransferQueue.update(&(directTransferArrivalLabels[sourceStation.representative]));
        while (!directTransferQueue.empty()) {
            ArrivalLabel* currentLabel = directTransferQueue.extractFront();
            const Vertex currentVertex = Vertex(currentLabel - &(directTransferArrivalLabels[0]));
            for (Edge edge : data.transferGraph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = data.transferGraph.get(ToVertex, edge);
                const int newArrivalTime = currentLabel->arrivalTime + data.transferGraph.get(TravelTime, edge);
                if (newArrivalTime < directTransferArrivalLabels[neighborVertex].arrivalTime) {
                    directTransferArrivalLabels[neighborVertex].arrivalTime = newArrivalTime;
                    directTransferQueue.update(&(directTransferArrivalLabels[neighborVertex]));
                }
            }
            if (data.isStop(currentVertex)) {
                stopsReachedByDirectTransfer.emplace_back(StopId(currentVertex));
            }
        }
        for (const StopId sourceStop : sourceStation.stops) {
            AssertMsg(Vector::contains(stopsReachedByDirectTransfer, sourceStop), "Source was not updated by transfer!");
        }
    }

    inline void relaxInitialTransfers() noexcept {
        AssertMsg(stopsUpdatedByTransfer.empty(), "stopsUpdatedByTransfer is not empty!");
        for (const StopId stop : stopsReachedByDirectTransfer) {
            const int newArrivalTime = sourceDepartureTime + directTransferArrivalLabels[stop].arrivalTime;
            arrivalByEdge0(stop, newArrivalTime);
            stopsUpdatedByTransfer.insert(stop);
        }
    }

    inline void intermediateDijkstra() noexcept {
        AssertMsg(stopsUpdatedByTransfer.empty(), "stopsUpdatedByTransfer is not empty!");

        shortcutCandidatesInQueue = 0;
        for (const StopId stop : stopsUpdatedByRoute) {
            oneTripQueue.update(&(oneTripArrivalLabels[stop]));
            if (oneTripTransferParent[stop] != noStopEvent) shortcutCandidatesInQueue++;
        }
        if (shortcutCandidatesInQueue == 0) {
            stopsUpdatedByRoute.clear();
            return;
        }

        int transferLimit = intMax;
        while (!oneTripQueue.empty()) {
            ArrivalLabel* currentLabel = oneTripQueue.extractFront();
            const Vertex currentVertex = Vertex(currentLabel - &(oneTripArrivalLabels[0]));
            for (Edge edge : data.transferGraph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = data.transferGraph.get(ToVertex, edge);
                const int newArrivalTime = currentLabel->arrivalTime + data.transferGraph.get(TravelTime, edge);
                if (newArrivalTime < oneTripArrivalLabels[neighborVertex].arrivalTime) {
                    arrivalByEdge1(neighborVertex, newArrivalTime, currentVertex);
                }
                //Candidates may dominate equivalent labels from previous iterations
                else if (newArrivalTime == oneTripArrivalLabels[neighborVertex].arrivalTime) {
                    const bool isCandidate = oneTripTransferParent[currentVertex] != noStopEvent;
                    const bool isFromPreviousIteration = oneTripTimestamps[neighborVertex] != timestamp;
                    if (isCandidate && isFromPreviousIteration) {
                        arrivalByEdge1(neighborVertex, newArrivalTime, currentVertex);
                    }
                }
            }
            if (oneTripTransferParent[currentVertex] != noStopEvent) {
                shortcutCandidatesInQueue--;
            }
            if (shortcutCandidatesInQueue == 0) {
                //Once all candidates have been settled, leave the Dijkstra search running until witnessTransferLimit is met.
                //Note that witnesses above the limit may be pruned, leading to superfluous shortcuts.
                shortcutCandidatesInQueue = -1;
                transferLimit = currentLabel->arrivalTime + witnessTransferLimit;
                if (transferLimit < currentLabel->arrivalTime) transferLimit = intMax;
                if constexpr (Debug) std::cout << "   Transfer limit in round 1: " << String::secToString(transferLimit - sourceDepartureTime) << std::endl;
            }
            if (data.isStop(currentVertex)) {
                stopsUpdatedByTransfer.insert(StopId(currentVertex));
            }
            if (currentLabel->arrivalTime > transferLimit) break;
        }

        stopsUpdatedByRoute.clear();
    }

    inline void finalDijkstra() noexcept {
        AssertMsg(stopsUpdatedByTransfer.empty(), "stopsUpdatedByTransfer is not empty!");

        for (const StopId stop : stopsUpdatedByRoute) {
            twoTripsQueue.update(&(twoTripsArrivalLabels[stop]));
            const StopId routeParent = twoTripsRouteParent[stop];
            if (data.isStop(routeParent)) { // This is the only place where shortcutDestinationCandidates are added
                const StopEventId routeParentEvent = twoTripsRouteParentEvent[stop];
                if (!shortcutDestinationCandidates.contains(routeParentEvent)) {
                    shortcutDestinationCandidates.insert(routeParentEvent);
                }
                shortcutDestinationCandidates[routeParentEvent].insert(stop);
            }
        }

        while (!twoTripsQueue.empty()) {
            ArrivalLabel* currentLabel = twoTripsQueue.extractFront();
            const Vertex currentVertex = Vertex(currentLabel - &(twoTripsArrivalLabels[0]));
            for (Edge edge : data.transferGraph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = data.transferGraph.get(ToVertex, edge);
                const int newArrivalTime = currentLabel->arrivalTime + data.transferGraph.get(TravelTime, edge);
                if (newArrivalTime < twoTripsArrivalLabels[neighborVertex].arrivalTime) {
                    arrivalByEdge2(neighborVertex, newArrivalTime, currentVertex);
                }
            }
            if (data.isStop(currentVertex)) {
                const StopId routeParent = twoTripsRouteParent[currentVertex];
                if (data.isStop(routeParent)) {
                    //No witness dominates this candidate journey => insert shortcut
                    const StopEventId routeParentEvent = twoTripsRouteParentEvent[currentVertex];
                    const StopEventId transferParentEvent = oneTripTransferParent[routeParent];
                    const int walkingDistance = oneTripArrivalLabels[routeParent].arrivalTime - data.stopEvents[transferParentEvent].arrivalTime;
                    if constexpr (IgnoreIsolatedCandidates) {
                        if (directTransferArrivalLabels[currentVertex].arrivalTime < never) {
                            shortcuts.emplace_back(transferParentEvent, routeParentEvent, walkingDistance);
                        }
                    }
                    else {
                        shortcuts.emplace_back(transferParentEvent, routeParentEvent, walkingDistance);
                    }
                    AssertMsg(shortcutDestinationCandidates.contains(routeParentEvent), "Vertex " << currentVertex << " has route parent " << routeParentEvent << " but the route parent does not know about this!");
                    //Unmark other candidates using this shortcut, since we don't need them anymore
                    for (const StopId obsoleteCandidate : shortcutDestinationCandidates[routeParentEvent]) {
                        twoTripsRouteParent[obsoleteCandidate] = noStop;
                    }
                    shortcutDestinationCandidates.remove(routeParentEvent);
                }
            }
            if (shortcutDestinationCandidates.empty()) break;
        }

        AssertMsg(shortcutDestinationCandidates.empty(), "There are still shortcut destination candidates left (" << shortcutDestinationCandidates.size() << ")!");
        stopsUpdatedByRoute.clear();
    }

    template<int ROUND>
    inline int arrivalTime(const Vertex vertex) const noexcept {
        static_assert((ROUND == 0) | (ROUND == 1) | (ROUND == 2), "Invalid round!");
        if constexpr (ROUND == 0) {
            AssertMsg(data.isStop(vertex), "Arrival time in round 0 only available for stops!");
            return zeroTripsArrivalLabels[vertex].arrivalTime;
        } else if constexpr (ROUND == 1) {
            return oneTripArrivalLabels[vertex].arrivalTime;
        } else if constexpr (ROUND == 2) {
            return twoTripsArrivalLabels[vertex].arrivalTime;
        }
    }

    template<int ROUND>
    inline void arrivalByRoute(const StopId stop, const int arrivalTime, const StopId parent, const StopEventId relevantStopEvent) noexcept {
        static_assert((ROUND == 1) | (ROUND == 2), "Invalid round!");
        if constexpr (ROUND == 1) {
            arrivalByRoute1(stop, arrivalTime, parent, relevantStopEvent);
        } else if constexpr (ROUND == 2) {
            arrivalByRoute2(stop, arrivalTime, parent, relevantStopEvent);
        }
    }

    inline void arrivalByRoute1(const StopId stop, const int arrivalTime, const StopId parent, const StopEventId arrivalStopEvent) noexcept {
        //Mark journey as candidate or witness
        if (stationOfStop[parent].representative == sourceStation.representative) {
            oneTripTransferParent[stop] = arrivalStopEvent;
        } else {
            oneTripTransferParent[stop] = noStopEvent;
        }
        updateArrival<1>(stop, arrivalTime, timestamp);
        //If the arrival label was improved, remove it from the queue - it will be re-added with the correct key later.
        if (oneTripArrivalLabels[stop].isOnHeap()) {
            oneTripQueue.remove(&(oneTripArrivalLabels[stop]));
        }
        if (twoTripsArrivalLabels[stop].arrivalTime > arrivalTime) {
            updateArrival<2>(stop, arrivalTime, timestamp);
            if (twoTripsArrivalLabels[stop].isOnHeap()) {
                twoTripsQueue.remove(&(twoTripsArrivalLabels[stop]));
            }
        }
        stopsUpdatedByRoute.insert(stop);
    }

    inline void arrivalByRoute2(const StopId stop, const int arrivalTime, const StopId parent, const StopEventId parentStopEvent) noexcept {
        //Mark journey as candidate or witness
        if (oneTripTransferParent[parent] != noStopEvent) {
            twoTripsRouteParent[stop] = parent;
            twoTripsRouteParentEvent[stop] = parentStopEvent;
        } else {
            twoTripsRouteParent[stop] = noStop;
        }
        updateArrival<2>(stop, arrivalTime, oneTripTimestamps[parent]);
        if (twoTripsArrivalLabels[stop].isOnHeap()) {
            twoTripsQueue.remove(&(twoTripsArrivalLabels[stop]));
        }
        stopsUpdatedByRoute.insert(stop);
    }

    inline void arrivalByEdge0(const Vertex vertex, const int arrivalTime) noexcept {
        updateArrival<0>(vertex, arrivalTime, timestamp);
        if (oneTripArrivalLabels[vertex].arrivalTime > arrivalTime) {
            updateArrival<1>(vertex, arrivalTime, timestamp);
            if (oneTripArrivalLabels[vertex].isOnHeap()) {
                oneTripQueue.remove(&(oneTripArrivalLabels[vertex]));
            }
            if (twoTripsArrivalLabels[vertex].arrivalTime > arrivalTime) {
                updateArrival<2>(vertex, arrivalTime, timestamp);
                if (twoTripsArrivalLabels[vertex].isOnHeap()) {
                    twoTripsQueue.remove(&(twoTripsArrivalLabels[vertex]));
                }
            }
        }
    }

    inline void arrivalByEdge1(const Vertex vertex, const int arrivalTime, const Vertex parent) noexcept {
        if (isShortcutCandidate(vertex)) shortcutCandidatesInQueue--;
        if (oneTripTransferParent[parent] != noStopEvent) shortcutCandidatesInQueue++;
        oneTripTransferParent[vertex] = oneTripTransferParent[parent];
        updateArrival<1>(vertex, arrivalTime, oneTripTimestamps[parent]);
        if (twoTripsArrivalLabels[vertex].arrivalTime > arrivalTime) {
            updateArrival<2>(vertex, arrivalTime, oneTripTimestamps[parent]);
            if (twoTripsArrivalLabels[vertex].isOnHeap()) {
                twoTripsQueue.remove(&(twoTripsArrivalLabels[vertex]));
            }
        }
        oneTripQueue.update(&(oneTripArrivalLabels[vertex]));
    }

    inline void arrivalByEdge2(const Vertex vertex, const int arrivalTime, const Vertex parent) noexcept {
        updateArrival<2>(vertex, arrivalTime, twoTripsTimestamps[parent]);
        twoTripsQueue.update(&(twoTripsArrivalLabels[vertex]));
        if (!data.isStop(vertex)) return;
        if (data.isStop(twoTripsRouteParent[vertex])) {
            //Candidate was dominated by a witness => remove from shortcutDestinationCandidates list.
            const StopEventId routeParentEvent = twoTripsRouteParentEvent[vertex];
            AssertMsg(shortcutDestinationCandidates.contains(routeParentEvent), "Vertex " << vertex << " has route parent " << routeParentEvent << " but the route parent does not know about this!");
            AssertMsg(shortcutDestinationCandidates[routeParentEvent].contains(StopId(vertex)), "Vertex " << vertex << " is not contained in shortcutDestinationCandidates List of " << routeParentEvent << "!");
            shortcutDestinationCandidates[routeParentEvent].erase(StopId(vertex));
            if (shortcutDestinationCandidates[routeParentEvent].empty()) {
                shortcutDestinationCandidates.remove(routeParentEvent);
            }
        }
        twoTripsRouteParent[vertex] = noStop;
    }

    template<int ROUND>
    inline void updateArrival(const Vertex vertex, const int arrivalTime, const u_int16_t labelTimestamp) noexcept {
        if constexpr (ROUND == 0) {
            zeroTripsArrivalLabels[vertex].arrivalTime = arrivalTime;
            suppressUnusedParameterWarning(labelTimestamp);
        } else if constexpr (ROUND == 1) {
            oneTripArrivalLabels[vertex].arrivalTime = arrivalTime;
            oneTripTimestamps[vertex] = labelTimestamp;
        } else if constexpr (ROUND == 2) {
            twoTripsArrivalLabels[vertex].arrivalTime = arrivalTime;
            twoTripsTimestamps[vertex] = labelTimestamp;
        }
    }

    inline bool isShortcutCandidate(const Vertex vertex) noexcept {
        return oneTripArrivalLabels[vertex].isOnHeap() && (oneTripTransferParent[vertex] != noStopEvent);
    }

private:
    const Data& tripData;
    const RAPTOR::Data& data;
    std::vector<Station> stationOfStop;

    Station sourceStation;
    int sourceDepartureTime;

    std::vector<ArrivalLabel> directTransferArrivalLabels;
    ExternalKHeap<2, ArrivalLabel> directTransferQueue;
    std::vector<StopId> stopsReachedByDirectTransfer;

    std::vector<ArrivalLabel> zeroTripsArrivalLabels;

    std::vector<ArrivalLabel> oneTripArrivalLabels;
    ExternalKHeap<2, ArrivalLabel> oneTripQueue;
    std::vector<u_int16_t> oneTripTimestamps;

    std::vector<ArrivalLabel> twoTripsArrivalLabels;
    ExternalKHeap<2, ArrivalLabel> twoTripsQueue;
    std::vector<u_int16_t> twoTripsTimestamps;

    //Only valid for candidates, dummy value for witnesses
    std::vector<StopEventId> oneTripTransferParent;
    //Only valid for candidates, dummy value for witnesses
    std::vector<StopId> twoTripsRouteParent;
    //Only valid for candidates
    std::vector<StopEventId> twoTripsRouteParentEvent;

    size_t shortcutCandidatesInQueue;
    //Maps potential shortcut destinations to the final stops of the candidate journeys using that shortcut
    IndexedMap<Set<StopId>, false, StopEventId> shortcutDestinationCandidates;
    std::vector<Shortcut> shortcuts;

    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;
    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;

    int witnessTransferLimit;

    int earliestDepartureTime;

    u_int16_t timestamp;

};

}
