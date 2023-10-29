#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../Dijkstra/Dijkstra.h"
#include "../Query/ReachedIndex.h"

#include "../../../Helpers/Meta.h"
#include "../../../Helpers/Helpers.h"
#include "../../../Helpers/Vector/Vector.h"

#include "../../../DataStructures/Container/Map.h"
#include "../../../DataStructures/Container/Set.h"
#include "../../../DataStructures/Container/ExternalKHeap.h"
#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../../DataStructures/TripBased/Shortcut.h"
#include "../../../DataStructures/TripBased/ShortcutCollection.h"

namespace TripBased {

template<bool DEBUG = false>
class DelayShortcutSearch {

public:
    inline static constexpr bool Debug = DEBUG;
    using Type = DelayShortcutSearch<Debug>;

public:
    struct ArrivalLabel : public ExternalKHeapElement {
        ArrivalLabel() : arrivalTime(never) {}
        int arrivalTime;
        inline bool hasSmallerKey(const ArrivalLabel* const other) const noexcept {
            return arrivalTime < other->arrivalTime;
        }
    };

    struct DepartureLabel {
        DepartureLabel(const RouteId routeId = noRouteId, const StopIndex stopIndex = noStopIndex, const int departureTime = never, const bool isCandidate = true) :
            route(routeId, stopIndex),
            departureTime(departureTime),
            isCandidate(isCandidate) {
        }

        RAPTOR::RouteSegment route;
        int departureTime;
        bool isCandidate;

        inline bool operator<(const DepartureLabel& other) const noexcept {
            return (departureTime > other.departureTime) || ((departureTime == other.departureTime) && (isCandidate < other.isCandidate));
        }
    };

    struct ConsolidatedDepartureLabel {
        ConsolidatedDepartureLabel(const int departureTime = never) : departureTime(departureTime) {}
        std::vector<RAPTOR::RouteSegment> witnessRoutes;
        std::vector<RAPTOR::RouteSegment> candidateRoutes;
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

    struct CandidateOriginLabel {
        CandidateOriginLabel(const int arrivalTime, const StopEventId stopEvent) :
            arrivalTime(arrivalTime),
            stopEvent(stopEvent) {
        }

        int arrivalTime;
        StopEventId stopEvent;
    };

    struct CandidateDijkstraLabel : public ExternalKHeapElement {
        CandidateDijkstraLabel() :
            arrivalTime(never),
            timestamp(-1) {
        }

        inline void checkTimestamp(const int newTimestamp) noexcept {
            if (timestamp < newTimestamp) arrivalTime = never;
            timestamp = newTimestamp;
        }

        inline bool hasSmallerKey(const CandidateDijkstraLabel* const other) const noexcept {
            return arrivalTime < other->arrivalTime;
        }

        int arrivalTime;
        int timestamp;
    };

    struct CandidateDestinationStopLabel {
        CandidateDestinationStopLabel(const int arrivalTime, const int maxOriginDelay) :
            arrivalTime(arrivalTime),
            maxOriginDelay(maxOriginDelay) {
        }

        int arrivalTime;
        int maxOriginDelay;
    };

    struct CandidateRouteLabel {
        CandidateRouteLabel(const StopIndex begin, const StopIndex end) :
            begin(begin),
            end(end) {
        }

        StopIndex begin;
        StopIndex end;
    };

    struct CandidateDestinationLabel {
        CandidateDestinationLabel(const int travelTime, const int minOriginDelay, const int maxOriginDelay) :
            travelTime(travelTime),
            minOriginDelay(minOriginDelay),
            maxOriginDelay(maxOriginDelay) {
        }

        int travelTime;
        int minOriginDelay;
        int maxOriginDelay;
    };

    struct TripMinOriginDelay {
        TripMinOriginDelay(const int minOriginDelay = 0, const int timestamp = -1) :
            minOriginDelay(minOriginDelay),
            timestamp(timestamp) {
        }

        inline void checkTimestamp(const int newTimestamp) noexcept {
            if (timestamp < newTimestamp) minOriginDelay = 0;
            timestamp = newTimestamp;
        }

        int minOriginDelay;
        int timestamp;
    };

public:
    DelayShortcutSearch(const Data& tripData, const int arrivalDelayBuffer, const int departureDelayBuffer, ShortcutCollection& shortcuts) :
        tripData(tripData),
        data(tripData.raptorData),
        stationOfStop(data.numberOfStops()),
        sourceStation(),
        sourceDepartureTime(0),
        witnessReachedIndex1(ReachedIndex(tripData)),
        witnessReachedIndex2(ReachedIndex(tripData)),
        sourceTimestamp(0),
        originTimestamp(0),
        sharedSourceArrivals1(data.transferGraph.numVertices()),
        sharedSourceArrivals2(data.transferGraph.numVertices()),
        sharedSourceReachedIndex2(tripData),
        sharedSourceTimestamp(data.transferGraph.numVertices(), -1),
        witnessDominatedIndex(tripData),
        routeTimestamp(data.numberOfRoutes(), -1),
        sharedOriginArrivals2(data.transferGraph.numVertices()),
        sharedOriginMaxDelay(data.transferGraph.numVertices(), -1),
        sharedOriginTimestamp(data.transferGraph.numVertices(), -1),
        shortcutOriginCandidates(data.numberOfStops()),
        shortcutDestinationStopCandidates(data.numberOfStops()),
        shortcutDestinationCandidates(data.numberOfStopEvents()),
        tripMinOriginDelay(data.numberOfTrips()),
        routesServingUpdatedStops(data.numberOfRoutes()),
        candidateRoutes(data.numberOfRoutes()),
        tripsServingUpdatedStops(data.numberOfTrips()),
        stopsUpdatedByRoute(data.numberOfStops()),
        stopsUpdatedByTransfer(data.numberOfStops()),
        earliestDepartureTime(data.getMinDepartureTime()),
        arrivalDelayBuffer(arrivalDelayBuffer),
        departureDelayBuffer(departureDelayBuffer),
        shortcuts(shortcuts) {
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
            sourceDepartureTime = label.departureTime;
            findWitnesses(label);
            findCandidates(label);
        }

        shortcuts.mergeUnmerged();
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

    inline std::vector<ConsolidatedDepartureLabel> collectDepartures(const int minTime, const int maxTime) noexcept {
        AssertMsg(directTransferArrivalLabels[sourceStation.representative].arrivalTime == 0, "Direct transfer for source " << sourceStation.representative << " is incorrect!");
        const int cutoffTime = std::max(minTime, earliestDepartureTime);
        std::vector<DepartureLabel> departureLabels;
        for (const RouteId route : data.routes()) {
            const StopId* stops = data.stopArrayOfRoute(route);
            const size_t tripSize = data.numberOfStopsInRoute(route);
            for (size_t stopIndex = 0; stopIndex + 1 < tripSize; stopIndex++) {
                for (const RAPTOR::StopEvent* trip = data.firstTripOfRoute(route); trip <= data.lastTripOfRoute(route); trip += tripSize) {
                    const int departureTime = trip[stopIndex].departureTime - directTransferArrivalLabels[stops[stopIndex]].arrivalTime;
                    if (departureTime < cutoffTime) continue;
                    if (departureTime > maxTime) break;
                    if (stationOfStop[stops[stopIndex]].representative == sourceStation.representative) {
                        departureLabels.emplace_back(route, StopIndex(stopIndex), departureTime + departureDelayBuffer, true);
                    }
                    departureLabels.emplace_back(route, StopIndex(stopIndex), departureTime, false);
                }
            }
        }
        sort(departureLabels);
        std::vector<ConsolidatedDepartureLabel> result(1);
        for (const DepartureLabel& label : departureLabels) {
            if (label.isCandidate) {
                if (result.back().departureTime == never) {
                    result.back().departureTime = label.departureTime;
                } else if (result.back().departureTime != label.departureTime) {
                    result.emplace_back(label.departureTime);
                }
                result.back().candidateRoutes.emplace_back(label.route);
            } else {
                if (result.back().departureTime != never) {
                    result.emplace_back(never);
                }
                result.back().witnessRoutes.emplace_back(label.route);
            }
        }
        if (result.back().departureTime == never) {
            result.pop_back();
        }

        return result;
    }

    inline void findWitnesses(const ConsolidatedDepartureLabel& label) noexcept {
        if constexpr (Debug) std::cout << "   Find witnesses for departure time: " << label.departureTime << " (" << String::secToTime(label.departureTime) << ")" << std::endl;

        routesServingUpdatedStops.clear();
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();

        relaxInitialTransfers();
        collectRoutes1(label.witnessRoutes);
        scanRoutes<1>();
        intermediateDijkstra();
        collectRoutes2();
        scanRoutes<2>();
        finalDijkstra();
    }

    inline void clear() noexcept {
        sourceStation = Station();

        std::vector<ArrivalLabel>(data.transferGraph.numVertices()).swap(directTransferArrivalLabels);
        stopsReachedByDirectTransfer.clear();

        std::vector<ArrivalLabel>(data.numberOfStops()).swap(witnessArrivals0);
        std::vector<ArrivalLabel>(data.transferGraph.numVertices()).swap(witnessArrivals1);
        std::vector<ArrivalLabel>(data.transferGraph.numVertices()).swap(witnessArrivals2);
        witnessReachedIndex1.clear();
        witnessReachedIndex2.clear();

        queue1.clear();
        queue2.clear();

        std::vector<int>(data.numberOfRoutes(), -1).swap(routeTimestamp);

        sourceTimestamp = 0;
        originTimestamp = 0;

        std::vector<ArrivalLabel>(data.transferGraph.numVertices()).swap(sharedSourceArrivals1);
        std::vector<ArrivalLabel>(data.transferGraph.numVertices()).swap(sharedSourceArrivals2);
        std::vector<int>(data.transferGraph.numVertices(), -1).swap(sharedSourceTimestamp);

        std::vector<ArrivalLabel>(data.transferGraph.numVertices()).swap(sharedOriginArrivals2);
        std::vector<int>(data.transferGraph.numVertices(), -1).swap(sharedOriginMaxDelay);
        std::vector<int>(data.transferGraph.numVertices(), -1).swap(sharedOriginTimestamp);

        std::vector<TripMinOriginDelay>(data.numberOfTrips()).swap(tripMinOriginDelay);

        AssertMsg(candidateQueue.empty(), "candidateQueue not empty!");
        std::vector<CandidateDijkstraLabel>(data.transferGraph.numVertices()).swap(preOriginDijkstraLabels);
        std::vector<CandidateDijkstraLabel>(data.transferGraph.numVertices()).swap(candidateDijkstraLabels);

        routesServingUpdatedStops.clear();
        candidateRoutes.clear();
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
    }

    inline void collectRoutes1(const std::vector<RAPTOR::RouteSegment>& routes) noexcept {
        for (const RAPTOR::RouteSegment& route : routes) {
            AssertMsg(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
            AssertMsg(route.stopIndex + 1 < data.numberOfStopsInRoute(route.routeId), "RouteSegment " << route << " is not a departure event!");
            if (routesServingUpdatedStops.contains(route.routeId)) {
                routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
            } else {
                routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
            }
        }
    }

    inline void collectRoutes2() noexcept {
        for (const StopId stop : stopsUpdatedByTransfer) {
            for (const RAPTOR::RouteSegment& route : data.routesContainingStop(stop)) {
                AssertMsg(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                AssertMsg(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId)) continue;
                if (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < witnessArrivals1[stop].arrivalTime) continue;
                if (routesServingUpdatedStops.contains(route.routeId)) {
                    routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                }
            }
        }
    }

    template<int CURRENT>
    inline void scanRoutes() noexcept {
        static_assert((CURRENT == 1) | (CURRENT == 2), "Invalid round!");
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            const StopIndex stopIndex = routesServingUpdatedStops[route];
            RAPTOR::TripIterator tripIterator = data.getTripIterator(route, stopIndex);
            updateWitnessReachedIndex<CURRENT>(route, tripIterator);
            while (tripIterator.hasFurtherStops()) {
                //Find earliest trip that can be entered
                if (tripIterator.hasEarlierTrip() && (tripIterator.previousDepartureTime() >= arrivalTime<CURRENT - 1>(tripIterator.stop()))) {
                    do {
                        tripIterator.previousTrip();
                    } while (tripIterator.hasEarlierTrip() && (tripIterator.previousDepartureTime() >= arrivalTime<CURRENT - 1>(tripIterator.stop())));
                    if (!stopsUpdatedByTransfer.contains(tripIterator.stop())) {
                        //Trip was improved by an arrival that was found during a previous RAPTOR iteration.
                        //We already explored this trip during that iteration.
                        //Fast forward to the next stop that was updated in the current iteration and can enter an earlier trip.
                        if (!tripIterator.hasEarlierTrip()) break;
                        do {
                            tripIterator.nextStop();
                        } while (tripIterator.hasFurtherStops() && ((!stopsUpdatedByTransfer.contains(tripIterator.stop())) || (tripIterator.previousDepartureTime() < arrivalTime<CURRENT - 1>(tripIterator.stop()))));
                        continue;
                    }
                    updateWitnessReachedIndex<CURRENT>(route, tripIterator);
                }
                tripIterator.nextStop();
                const int newArrivalTime = tripIterator.arrivalTime() + arrivalDelayBuffer;
                if (newArrivalTime < arrivalTime<CURRENT>(tripIterator.stop())) {
                    arrivalByRoute<CURRENT>(tripIterator.stop(), newArrivalTime);
                }
            }
        }
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
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
            witnessArrivals0[stop].arrivalTime = newArrivalTime;
            stopsUpdatedByTransfer.insert(stop);
        }
    }

    inline void intermediateDijkstra() noexcept {
        AssertMsg(stopsUpdatedByTransfer.empty(), "stopsUpdatedByTransfer is not empty!");

        for (const StopId stop : stopsUpdatedByRoute) {
            queue1.update(&(witnessArrivals1[stop]));
        }

        while (!queue1.empty()) {
            ArrivalLabel* currentLabel = queue1.extractFront();
            const Vertex currentVertex = Vertex(currentLabel - &(witnessArrivals1[0]));
            for (Edge edge : data.transferGraph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = data.transferGraph.get(ToVertex, edge);
                const int newArrivalTime = currentLabel->arrivalTime + data.transferGraph.get(TravelTime, edge);
                if (newArrivalTime < witnessArrivals1[neighborVertex].arrivalTime) {
                    arrivalByEdge1(neighborVertex, newArrivalTime);
                }
            }
            if (data.isStop(currentVertex)) {
                stopsUpdatedByTransfer.insert(StopId(currentVertex));
            }
        }

        stopsUpdatedByRoute.clear();
    }

    inline void finalDijkstra() noexcept {
        AssertMsg(stopsUpdatedByTransfer.empty(), "stopsUpdatedByTransfer is not empty!");

        for (const StopId stop : stopsUpdatedByRoute) {
            queue2.update(&(witnessArrivals2[stop]));
        }

        while (!queue2.empty()) {
            ArrivalLabel* currentLabel = queue2.extractFront();
            const Vertex currentVertex = Vertex(currentLabel - &(witnessArrivals2[0]));
            for (Edge edge : data.transferGraph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = data.transferGraph.get(ToVertex, edge);
                const int newArrivalTime = currentLabel->arrivalTime + data.transferGraph.get(TravelTime, edge);
                if (newArrivalTime < witnessArrivals2[neighborVertex].arrivalTime) {
                    arrivalByEdge2(neighborVertex, newArrivalTime);
                }
            }
         }

        stopsUpdatedByRoute.clear();
    }

    template<int ROUND>
    inline void updateWitnessReachedIndex(const RouteId route, const RAPTOR::TripIterator& tripIterator) noexcept {
        static_assert((ROUND == 1) | (ROUND == 2), "Invalid round!");
        const TripId trip = TripId(tripData.firstTripOfRoute[route] + tripIterator.getCurrentTripNumber());
        if constexpr (ROUND == 1) {
            witnessReachedIndex1.update(trip, tripIterator.getStopIndex());
        }
        witnessReachedIndex2.update(trip, tripIterator.getStopIndex());
    }

    template<int ROUND>
    inline int arrivalTime(const Vertex vertex) const noexcept {
        static_assert((ROUND == 0) | (ROUND == 1) | (ROUND == 2), "Invalid round!");
        if constexpr (ROUND == 0) {
            AssertMsg(data.isStop(vertex), "Arrival time in round 0 only available for stops!");
            return witnessArrivals0[vertex].arrivalTime;
        } else if constexpr (ROUND == 1) {
            return witnessArrivals1[vertex].arrivalTime;
        } else if constexpr (ROUND == 2) {
            return witnessArrivals2[vertex].arrivalTime;
        }
    }

    template<int ROUND>
    inline void arrivalByRoute(const StopId stop, const int arrivalTime) noexcept {
        static_assert((ROUND == 1) | (ROUND == 2), "Invalid round!");
        if constexpr (ROUND == 1) {
            arrivalByRoute1(stop, arrivalTime);
        } else if constexpr (ROUND == 2) {
            arrivalByRoute2(stop, arrivalTime);
        }
    }

    inline void arrivalByRoute1(const StopId stop, const int arrivalTime) noexcept {
        witnessArrivals1[stop].arrivalTime = arrivalTime;
        if (witnessArrivals2[stop].arrivalTime > arrivalTime) {
            witnessArrivals2[stop].arrivalTime = arrivalTime;
        }
        stopsUpdatedByRoute.insert(stop);
    }

    inline void arrivalByRoute2(const StopId stop, const int arrivalTime) noexcept {
        witnessArrivals2[stop].arrivalTime = arrivalTime;
        stopsUpdatedByRoute.insert(stop);
    }

    inline void arrivalByEdge1(const Vertex vertex, const int arrivalTime) noexcept {
        witnessArrivals1[vertex].arrivalTime = arrivalTime;
        if (witnessArrivals2[vertex].arrivalTime > arrivalTime) {
            witnessArrivals2[vertex].arrivalTime = arrivalTime;
        }
        queue1.update(&(witnessArrivals1[vertex]));
    }

    inline void arrivalByEdge2(const Vertex vertex, const int arrivalTime) noexcept {
        witnessArrivals2[vertex].arrivalTime = arrivalTime;
        queue2.update(&(witnessArrivals2[vertex]));
    }

    inline void findCandidates(const ConsolidatedDepartureLabel& label) noexcept {
        if constexpr (Debug) std::cout << "   Find candidates for departure time: " << label.departureTime << " (" << String::secToTime(label.departureTime) << ")" << std::endl;

        for (const RAPTOR::RouteSegment& routeSegment : label.candidateRoutes) {
            sourceTimestamp++;
            scanCandidateRoutes1(routeSegment);

            clearSharedSourceData();
            intermediateSharedSourceDijkstra();
            collectSharedSourceRoutes2();
            scanSharedSourceWitnessRoutes2();
            sharedSourceWitnessFinalDijkstra();

            for (const StopId origin : shortcutOriginCandidates.getKeys()) {
                //Check if strongly dominated by shared-source witness
                checkSharedSourceTimestamp(origin);
                if (shortcutOriginCandidates[origin].arrivalTime > sharedSourceArrivals1[origin].arrivalTime) continue;
                originTimestamp++;
                intermediatePreOriginDijkstra(routeSegment.stopIndex, origin);
                intermediateCandidateDijkstra(origin);
                collectCandidateRoutes2();
                registerDestinationStopEvents(origin);
                collectSharedOriginRoutes2();
                scanSharedOriginWitnessRoutes2();
                finalSharedOriginDijkstra();
                scanCandidateRoutes2(origin);
            }
        }
    }

    inline void scanCandidateRoutes1(const RAPTOR::RouteSegment& routeSegment) noexcept {
        shortcutOriginCandidates.clear();
        RAPTOR::TripIterator tripIterator = data.getTripIterator(routeSegment);
        //Find candidate trip
        while (tripIterator.hasEarlierTrip(sourceDepartureTime - departureDelayBuffer)) {
            tripIterator.previousTrip();
        }

        //Check if witness can enter this trip at an earlier stop
        const TripId trip = TripId(tripData.firstTripOfRoute[routeSegment.routeId] + tripIterator.getCurrentTripNumber());
        if (routeSegment.stopIndex > 0 && witnessReachedIndex1.alreadyReached(trip, routeSegment.stopIndex - 1)) return;

        while (tripIterator.hasFurtherStops()) {
            tripIterator.nextStop();
            const StopId stop = tripIterator.stop();
            const int candidateArrivalTime = tripIterator.arrivalTime();
            //Check if strongly dominated by witness
            if (candidateArrivalTime > witnessArrivals0[stop].arrivalTime) continue;
            if (candidateArrivalTime > witnessArrivals1[stop].arrivalTime) continue;
            // If a stop is reached multiple times by the same trip, the first arrival dominates the others (assuming trips do not travel backwards in time)
            if (shortcutOriginCandidates.contains(stop)) continue;
            const StopEventId stopEvent(tripIterator.stopEvent() - (&(data.stopEvents[0])));
            shortcutOriginCandidates.insert(stop, CandidateOriginLabel(candidateArrivalTime, stopEvent));
        }
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
    }

    inline void clearSharedSourceData() noexcept {
        sharedSourceReachedIndex2.clear();
    }

    inline void intermediateSharedSourceDijkstra() noexcept {
        for (const StopId origin : shortcutOriginCandidates.getKeys()) {
            sharedSourceArrivals1[origin].arrivalTime = shortcutOriginCandidates[origin].arrivalTime + arrivalDelayBuffer;
            sharedSourceArrivals2[origin].arrivalTime = sharedSourceArrivals1[origin].arrivalTime;
            sharedSourceTimestamp[origin] = sourceTimestamp;
            queue1.update(&(sharedSourceArrivals1[origin]));
        }
        while (!queue1.empty()) {
            const ArrivalLabel* currentLabel = queue1.extractFront();
            const Vertex currentVertex = Vertex(currentLabel - &(sharedSourceArrivals1[0]));
            for (const Edge edge : data.transferGraph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = data.transferGraph.get(ToVertex, edge);
                const int newArrivalTime = currentLabel->arrivalTime + data.transferGraph.get(TravelTime, edge);
                //Check if weakly dominated by witness
                if (newArrivalTime >= witnessArrivals1[neighborVertex].arrivalTime) continue;
                checkSharedSourceTimestamp(neighborVertex);
                if (newArrivalTime >= sharedSourceArrivals1[neighborVertex].arrivalTime) continue;
                sharedSourceArrivals1[neighborVertex].arrivalTime = newArrivalTime;
                sharedSourceArrivals2[neighborVertex].arrivalTime = newArrivalTime;
                queue1.update(&(sharedSourceArrivals1[neighborVertex]));
            }
            if (data.isStop(currentVertex)) {
                stopsUpdatedByTransfer.insert(StopId(currentVertex));
            }
        }
    }

    inline void collectSharedSourceRoutes2() noexcept {
        for (const StopId stop : stopsUpdatedByTransfer) {
            for (const RAPTOR::RouteSegment& route : data.routesContainingStop(stop)) {
                if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId)) continue;
                if (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < sharedSourceArrivals1[stop].arrivalTime) continue;
                if (routesServingUpdatedStops.contains(route.routeId)) {
                    routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                }
            }
        }
    }

    inline void scanSharedSourceWitnessRoutes2() noexcept {
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            const StopIndex stopIndex = routesServingUpdatedStops[route];
            RAPTOR::TripIterator tripIterator = data.getTripIterator(route, stopIndex);
            const TripId firstTrip = tripData.firstTripOfRoute[route];
            updateSharedSourceReachedIndex(firstTrip, tripIterator);

            while (tripIterator.hasFurtherStops()) {
                if (stopsUpdatedByTransfer.contains(tripIterator.stop())) {
                    const int destinationArrivalTime = sharedSourceArrivals1[tripIterator.stop()].arrivalTime;
                    if (tripIterator.hasEarlierTrip() && destinationArrivalTime <= tripIterator.previousDepartureTime()) {
                        do {
                            tripIterator.previousTrip();
                        } while (tripIterator.hasEarlierTrip() && destinationArrivalTime <= tripIterator.previousDepartureTime());
                        updateSharedSourceReachedIndex(firstTrip, tripIterator);
                    }
                }
                tripIterator.nextStop();
                const int newArrivalTime = tripIterator.arrivalTime() + arrivalDelayBuffer;
                //Check if weakly dominated by witness
                if (newArrivalTime >= witnessArrivals0[tripIterator.stop()].arrivalTime) continue;
                if (newArrivalTime >= witnessArrivals2[tripIterator.stop()].arrivalTime) continue;
                checkSharedSourceTimestamp(tripIterator.stop());
                if (newArrivalTime >= sharedSourceArrivals2[tripIterator.stop()].arrivalTime) continue;
                sharedSourceArrivals2[tripIterator.stop()].arrivalTime = newArrivalTime;
                stopsUpdatedByRoute.insert(tripIterator.stop());
            }
        }
    }

    inline void updateSharedSourceReachedIndex(const TripId firstTrip, const RAPTOR::TripIterator& tripIterator) noexcept {
        const TripId trip = TripId(firstTrip + tripIterator.getCurrentTripNumber());
        sharedSourceReachedIndex2.update(trip, tripIterator.getStopIndex());
    }

    inline void sharedSourceWitnessFinalDijkstra() noexcept {
        for (const StopId stop : stopsUpdatedByRoute) {
            queue2.update(&(sharedSourceArrivals2[stop]));
        }

        while (!queue2.empty()) {
            const ArrivalLabel* currentLabel = queue2.extractFront();
            const Vertex currentVertex = Vertex(currentLabel - &(sharedSourceArrivals2[0]));
            for (const Edge edge : data.transferGraph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = data.transferGraph.get(ToVertex, edge);
                const int newArrivalTime = currentLabel->arrivalTime + data.transferGraph.get(TravelTime, edge);
                //Check if weakly dominated by witness
                if (newArrivalTime >= witnessArrivals2[neighborVertex].arrivalTime) continue;
                checkSharedSourceTimestamp(neighborVertex);
                if (newArrivalTime >= sharedSourceArrivals2[neighborVertex].arrivalTime) continue;
                sharedSourceArrivals2[neighborVertex].arrivalTime = newArrivalTime;
                queue2.update(&(sharedSourceArrivals2[neighborVertex]));
            }
        }

        stopsUpdatedByRoute.clear();
    }

    inline void intermediatePreOriginDijkstra(const StopIndex sourceStopIndex, const StopId origin) noexcept {
        const int originArrivalTime = shortcutOriginCandidates[origin].arrivalTime;
        const StopEventId originStopEvent = shortcutOriginCandidates[origin].stopEvent;
        const TripId trip = tripData.tripOfStopEvent[originStopEvent];
        StopIndex index = StopIndex(tripData.indexOfStopEvent[originStopEvent] - 1);
        StopEventId stopEvent = StopEventId(tripData.firstStopEventOfTrip[trip] + index);
        const StopId* stops = tripData.stopArrayOfTrip(trip);
        for (; index != sourceStopIndex; index--, stopEvent--) {
            if (data.stopEvents[stopEvent].arrivalTime + arrivalDelayBuffer <= originArrivalTime) break;
            const StopId stop = stops[index];
            preOriginDijkstraLabels[stop].checkTimestamp(originTimestamp);
            preOriginDijkstraLabels[stop].arrivalTime = originArrivalTime;
            candidateQueue.update(&(preOriginDijkstraLabels[stop]));
        }
        while (!candidateQueue.empty()) {
            const CandidateDijkstraLabel* currentLabel = candidateQueue.extractFront();
            const Vertex currentVertex = Vertex(currentLabel - &(preOriginDijkstraLabels[0]));
            for (const Edge edge : data.transferGraph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = data.transferGraph.get(ToVertex, edge);
                const int newArrivalTime = currentLabel->arrivalTime + data.transferGraph.get(TravelTime, edge);
                if (isCandidateDominated1(newArrivalTime, neighborVertex)) continue;
                preOriginDijkstraLabels[neighborVertex].checkTimestamp(originTimestamp);
                if (newArrivalTime >= preOriginDijkstraLabels[neighborVertex].arrivalTime) continue;
                preOriginDijkstraLabels[neighborVertex].arrivalTime = newArrivalTime;
                candidateQueue.update(&(preOriginDijkstraLabels[neighborVertex]));
            }
        }
    }

    inline void intermediateCandidateDijkstra(const StopId origin) noexcept {
        shortcutDestinationStopCandidates.clear();
        candidateDijkstraLabels[origin].checkTimestamp(originTimestamp);
        candidateDijkstraLabels[origin].arrivalTime = shortcutOriginCandidates[origin].arrivalTime;
        candidateQueue.update(&(candidateDijkstraLabels[origin]));
        while (!candidateQueue.empty()) {
            const CandidateDijkstraLabel* currentLabel = candidateQueue.extractFront();
            const Vertex currentVertex = Vertex(currentLabel - &(candidateDijkstraLabels[0]));
            if (data.isStop(currentVertex)) {
                const int witnessArrivalTime = std::min(witnessArrivals0[currentVertex].arrivalTime, std::min(witnessArrivals1[currentVertex].arrivalTime, sharedSourceArrivals1[currentVertex].arrivalTime));
                const int maxOriginDelay = witnessArrivalTime - currentLabel->arrivalTime;
                shortcutDestinationStopCandidates.insert(StopId(currentVertex), CandidateDestinationStopLabel(currentLabel->arrivalTime, maxOriginDelay));
            }
            for (const Edge edge : data.transferGraph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = data.transferGraph.get(ToVertex, edge);
                const int newArrivalTime = currentLabel->arrivalTime + data.transferGraph.get(TravelTime, edge);
                if (isCandidateDominated1(newArrivalTime, neighborVertex)) continue;
                preOriginDijkstraLabels[neighborVertex].checkTimestamp(originTimestamp);
                if (newArrivalTime > preOriginDijkstraLabels[neighborVertex].arrivalTime) continue;
                candidateDijkstraLabels[neighborVertex].checkTimestamp(originTimestamp);
                if (newArrivalTime >= candidateDijkstraLabels[neighborVertex].arrivalTime) continue;
                candidateDijkstraLabels[neighborVertex].arrivalTime = newArrivalTime;
                candidateQueue.update(&(candidateDijkstraLabels[neighborVertex]));
            }
        }
    }

    inline void collectCandidateRoutes2() noexcept {
        candidateRoutes.clear();
        for (const StopId destinationStop : shortcutDestinationStopCandidates.getKeys()) {
            for (const RAPTOR::RouteSegment& route : data.routesContainingStop(destinationStop)) {
                if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId)) continue;
                if (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime + departureDelayBuffer < shortcutDestinationStopCandidates[destinationStop].arrivalTime) continue;
                if (candidateRoutes.contains(route.routeId)) {
                    candidateRoutes[route.routeId].begin = std::min(candidateRoutes[route.routeId].begin, route.stopIndex);
                } else {
                    updateRouteDominatedIndices(route.routeId);
                    const TripId firstTrip = tripData.firstTripOfRoute[route.routeId];
                    const StopIndex endIndex = getFirstDominatedIndex(firstTrip);
                    if (endIndex <= route.stopIndex) continue;
                    candidateRoutes.insert(route.routeId, CandidateRouteLabel(route.stopIndex, endIndex));
                }
            }
        }
    }

    inline void updateRouteDominatedIndices(const RouteId route) noexcept {
        if (routeTimestamp[route] == sourceTimestamp) return;
        routeTimestamp[route] = sourceTimestamp;
        witnessDominatedIndex.clear(route);

        const StopId* stops = data.stopArrayOfRoute(route);
        TripId trip = tripData.firstTripOfRoute[route];
        const TripId tripEnd = tripData.firstTripOfRoute[route + 1];
        const size_t tripLength = data.numberOfStopsInRoute(route);
        StopIndex stopIndex = StopIndex(tripLength - 1);
        StopEventId stopEvent = StopEventId(tripData.firstStopEventOfTrip[trip] + stopIndex);

        for (; stopIndex != 0; stopIndex--, stopEvent--) {
            const StopId stop = stops[stopIndex];
            const int witnessArrivalTime = getWitnessArrivalTime2(stop);
            if (data.stopEvents[stopEvent].arrivalTime <= witnessArrivalTime) {
                const TripId oldTrip = trip;
                do {
                    trip++;
                    stopEvent += tripLength;
                } while (trip < tripEnd && data.stopEvents[stopEvent].arrivalTime <= witnessArrivalTime);
                witnessDominatedIndex.updateRaw(oldTrip, trip, stopIndex);
                if (trip == tripEnd) return;
            }
        }
        witnessDominatedIndex.updateRaw(trip, tripEnd, StopIndex(0));
    }

    inline void registerDestinationStopEvents(const StopId origin) noexcept {
        shortcutDestinationCandidates.clear();
        const int originArrivalTime = shortcutOriginCandidates[origin].arrivalTime;
        std::vector<RouteId> routes = candidateRoutes.getKeys();
        std::sort(routes.begin(), routes.end(), [&](const RouteId a, const RouteId b) {
            return a > b;
        });
        for (const RouteId route : routes) {
            size_t addedTrips = 0;
            const StopId* stops = data.stopArrayOfRoute(route);
            const RAPTOR::StopEvent* lastTripBegin = data.lastTripOfRoute(route);
            const size_t tripSize = data.numberOfStopsInRoute(route);
            TripId firstTrip = tripData.firstTripOfRoute[route];
            TripId lastTrip = TripId(tripData.firstTripOfRoute[route + 1] - 1);

            for (StopIndex stopIndex = candidateRoutes[route].begin; stopIndex < candidateRoutes[route].end; stopIndex++) {
                const StopId destinationStop = stops[stopIndex];
                if (!shortcutDestinationStopCandidates.contains(destinationStop)) continue;

                while (lastTrip > firstTrip && getFirstDominatedIndex(lastTrip) <= stopIndex) {
                    lastTrip--;
                    lastTripBegin -= tripSize;
                }

                const RAPTOR::StopEvent* stopEvent = lastTripBegin + stopIndex;
                const int lastDepartureTime = stopEvent->departureTime + departureDelayBuffer;
                const int destinationArrivalTime = shortcutDestinationStopCandidates[destinationStop].arrivalTime;
                if (lastDepartureTime < destinationArrivalTime) continue;

                const int maxOriginDelay = shortcutDestinationStopCandidates[destinationStop].maxOriginDelay;
                const int travelTime = destinationArrivalTime - originArrivalTime;

                for (TripId trip = lastTrip; trip + 1 != firstTrip; trip--, stopEvent -= tripSize) {
                    const int delayedDepartureTime = stopEvent->departureTime + departureDelayBuffer;
                    if (delayedDepartureTime < destinationArrivalTime) break;

                    tripMinOriginDelay[trip].checkTimestamp(originTimestamp);
                    if (tripMinOriginDelay[trip].minOriginDelay > maxOriginDelay) continue;

                    if (!tripsServingUpdatedStops.contains(trip)) {
                        tripsServingUpdatedStops.insert(trip, stopIndex);
                        addedTrips++;
                    }

                    const StopEventId stopEventId = StopEventId(stopEvent - &(data.stopEvents[0]));
                    shortcutDestinationCandidates.insert(stopEventId, CandidateDestinationLabel(travelTime, tripMinOriginDelay[trip].minOriginDelay, maxOriginDelay));

                    const int maxOriginDelayAsWitness = std::min(stopEvent->departureTime - destinationArrivalTime, arrivalDelayBuffer);
                    tripMinOriginDelay[trip].minOriginDelay = std::max(tripMinOriginDelay[trip].minOriginDelay, maxOriginDelayAsWitness + 1);
                }
            }

            tripsServingUpdatedStops.sortLastNKeys(addedTrips, [&](const TripId a, const TripId b) {
                return a > b;
            });
        }
    }

    inline void collectSharedOriginRoutes2() noexcept {
        routesServingUpdatedStops.clear();
        for (const StopId stop : shortcutDestinationStopCandidates.getKeys()) {
            for (const RAPTOR::RouteSegment& route : data.routesContainingStop(stop)) {
                if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId)) continue;
                if (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < shortcutDestinationStopCandidates[stop].arrivalTime) continue;
                if (routesServingUpdatedStops.contains(route.routeId)) {
                    routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                }
            }
        }
    }

    inline void scanSharedOriginWitnessRoutes2() noexcept {
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            const StopIndex stopIndex = routesServingUpdatedStops[route];
            RAPTOR::TripIterator tripIterator = data.getTripIterator(route, stopIndex);
            StopEventId destination = StopEventId(tripIterator.stopEvent() - (&(data.stopEvents[0])));
            bool exit = shortcutDestinationCandidates.contains(destination);
            int maxDelay = std::min(tripIterator.departureTime() - shortcutDestinationStopCandidates[tripIterator.stop()].arrivalTime, arrivalDelayBuffer);

            while (tripIterator.hasFurtherStops()) {
                if (shortcutDestinationStopCandidates.contains(tripIterator.stop())) {
                    const int destinationArrivalTime = shortcutDestinationStopCandidates[tripIterator.stop()].arrivalTime;
                    if (tripIterator.hasEarlierTrip() && destinationArrivalTime <= tripIterator.previousDepartureTime()) {
                        do {
                            tripIterator.previousTrip();
                        } while (tripIterator.hasEarlierTrip() && destinationArrivalTime <= tripIterator.previousDepartureTime());
                        maxDelay = tripIterator.departureTime() - destinationArrivalTime;
                        destination = StopEventId(tripIterator.stopEvent() - (&(data.stopEvents[0])));
                        exit = shortcutDestinationCandidates.contains(destination);
                    }
                }
                tripIterator.nextStop();
                if (!exit) continue;
                const int newArrivalTime = tripIterator.arrivalTime() + arrivalDelayBuffer;
                //Check if weakly dominated by witness
                if (newArrivalTime >= witnessArrivals0[tripIterator.stop()].arrivalTime) continue;
                if (newArrivalTime >= witnessArrivals2[tripIterator.stop()].arrivalTime) continue;
                checkSharedSourceTimestamp(tripIterator.stop());
                if (newArrivalTime >= sharedSourceArrivals2[tripIterator.stop()].arrivalTime) continue;
                checkSharedOriginTimestamp(tripIterator.stop());
                if (newArrivalTime >= sharedOriginArrivals2[tripIterator.stop()].arrivalTime) continue;
                sharedOriginArrivals2[tripIterator.stop()].arrivalTime = newArrivalTime;
                sharedOriginMaxDelay[tripIterator.stop()] = maxDelay;
                stopsUpdatedByRoute.insert(tripIterator.stop());
            }
        }
    }

    inline void finalSharedOriginDijkstra() noexcept {
        for (const StopId stop : stopsUpdatedByRoute) {
            queue2.update(&(sharedOriginArrivals2[stop]));
        }
        while (!queue2.empty()) {
            const ArrivalLabel* currentLabel = queue2.extractFront();
            const Vertex currentVertex = Vertex(currentLabel - &(sharedOriginArrivals2[0]));
            for (const Edge edge : data.transferGraph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = data.transferGraph.get(ToVertex, edge);
                const int newArrivalTime = currentLabel->arrivalTime + data.transferGraph.get(TravelTime, edge);
                //Check if weakly dominated by witness
                if (newArrivalTime >= witnessArrivals2[neighborVertex].arrivalTime) continue;
                checkSharedSourceTimestamp(neighborVertex);
                if (newArrivalTime >= sharedSourceArrivals2[neighborVertex].arrivalTime) continue;
                checkSharedOriginTimestamp(neighborVertex);
                if (newArrivalTime >= sharedOriginArrivals2[neighborVertex].arrivalTime) continue;
                sharedOriginArrivals2[neighborVertex].arrivalTime = newArrivalTime;
                sharedOriginMaxDelay[neighborVertex] = sharedOriginMaxDelay[currentVertex];
                queue2.update(&(sharedOriginArrivals2[neighborVertex]));
            }
        }
        stopsUpdatedByRoute.clear();
    }

    inline void scanCandidateRoutes2(const StopId origin) noexcept {
        const StopEventId originStopEvent = shortcutOriginCandidates[origin].stopEvent;

        for (const TripId trip : tripsServingUpdatedStops.getKeys()) {
            const StopIndex firstCandidateIndex = tripsServingUpdatedStops[trip];
            const StopEventId firstStopEvent = tripData.firstStopEventOfTrip[trip];
            const StopId* stops = tripData.stopArrayOfTrip(trip);

            int minOriginDelayFromTarget = INFTY;
            int maxTargetArrivalTime = -INFTY;

            for (StopIndex stopIndex(witnessDominatedIndex(trip)); stopIndex != firstCandidateIndex; stopIndex--) {
                const StopId stop = stops[stopIndex];
                const StopEventId stopEvent = StopEventId(firstStopEvent + stopIndex);
                const int arrivalTime = data.stopEvents[stopEvent].arrivalTime;
                const int witnessArrivalTime = getWitnessArrivalTime2(stop);
                if (arrivalTime <= witnessArrivalTime) {
                    const int sharedOriginMaxDelay = getSharedOriginMaxDelay(stop, arrivalTime);
                    minOriginDelayFromTarget = std::min(minOriginDelayFromTarget, sharedOriginMaxDelay + 1);
                    maxTargetArrivalTime = std::max(maxTargetArrivalTime, witnessArrivalTime);
                }

                if (minOriginDelayFromTarget > arrivalDelayBuffer) continue;
                const StopEventId destination = StopEventId(stopEvent - 1);
                if (!shortcutDestinationCandidates.contains(destination)) continue;
                const StopId destinationStop = stops[stopIndex - 1];
                const CandidateDestinationLabel& l = shortcutDestinationCandidates[destination];
                const int minOriginDelay = std::max(l.minOriginDelay, minOriginDelayFromTarget);
                const int bestMaxOriginDelay = std::min(l.maxOriginDelay, maxTargetArrivalTime - shortcutDestinationStopCandidates[destinationStop].arrivalTime);
                if (minOriginDelay > bestMaxOriginDelay) continue;
                const int travelTime = shortcutDestinationCandidates[destination].travelTime;
                shortcuts.add(originStopEvent, DelayShortcut(destination, travelTime, minOriginDelay, bestMaxOriginDelay));
            }
        }
        tripsServingUpdatedStops.clear();
    }

    inline int getSharedOriginMaxDelay(const StopId stop, const int arrivalTime) noexcept {
        int maxDelay = -1;
        candidateDijkstraLabels[stop].checkTimestamp(originTimestamp);
        if (candidateDijkstraLabels[stop].arrivalTime < never) {
            maxDelay = arrivalTime - candidateDijkstraLabels[stop].arrivalTime;
        }
        checkSharedOriginTimestamp(stop);
        if (sharedOriginArrivals2[stop].arrivalTime < arrivalTime) {
            maxDelay = std::max(maxDelay, sharedOriginMaxDelay[stop]);
        }
        return maxDelay;
    }

    inline StopIndex getFirstDominatedIndex(const TripId trip) const noexcept {
        AssertMsg(routeTimestamp[tripData.routeOfTrip[trip]] == sourceTimestamp, "witnessDominatedIndex for trip " << trip << " has not been updated yet!");
        return std::min(getWitnessReachedIndex2(trip), witnessDominatedIndex(trip));
    }

    inline StopIndex getWitnessReachedIndex2(const TripId trip) const noexcept {
        return StopIndex(std::min(witnessReachedIndex2(trip), sharedSourceReachedIndex2(trip)) + 1);
    }

    inline bool isCandidateDominated1(const int arrivalTime, const Vertex vertex) noexcept {
        //Check if strongly dominated by witness
        if (arrivalTime > witnessArrivals1[vertex].arrivalTime) return true;
        //Check if strongly dominated by shared-source witness
        checkSharedSourceTimestamp(vertex);
        if (arrivalTime > sharedSourceArrivals1[vertex].arrivalTime) return true;
        return false;
    }

    inline int getWitnessArrivalTime2(const StopId stop) noexcept {
        checkSharedSourceTimestamp(stop);
        return std::min(witnessArrivals0[stop].arrivalTime, std::min(witnessArrivals2[stop].arrivalTime, sharedSourceArrivals2[stop].arrivalTime));
    }

    inline void checkSharedOriginTimestamp(const Vertex vertex) noexcept {
        if (sharedOriginTimestamp[vertex] == originTimestamp) return;
        sharedOriginArrivals2[vertex].arrivalTime = never;
        sharedOriginMaxDelay[vertex] = -1;
        sharedOriginTimestamp[vertex] = originTimestamp;
    }

    inline void checkSharedSourceTimestamp(const Vertex vertex) noexcept {
        if (sharedSourceTimestamp[vertex] == sourceTimestamp) return;
        sharedSourceArrivals1[vertex].arrivalTime = never;
        sharedSourceArrivals2[vertex].arrivalTime = never;
        sharedSourceTimestamp[vertex] = sourceTimestamp;
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

    std::vector<ArrivalLabel> witnessArrivals0;

    std::vector<ArrivalLabel> witnessArrivals1;
    ExternalKHeap<2, ArrivalLabel> queue1;

    std::vector<ArrivalLabel> witnessArrivals2;
    ExternalKHeap<2, ArrivalLabel> queue2;

    ReachedIndex witnessReachedIndex1;
    ReachedIndex witnessReachedIndex2;

    int sourceTimestamp;
    int originTimestamp;

    std::vector<ArrivalLabel> sharedSourceArrivals1;
    std::vector<ArrivalLabel> sharedSourceArrivals2;
    ReachedIndex sharedSourceReachedIndex2;
    std::vector<int> sharedSourceTimestamp;

    //First stop index along the trip where a candidate doesn't need to enter because it would be dominated when exiting at any subsequent stop
    ReachedIndex witnessDominatedIndex;
    std::vector<int> routeTimestamp;

    std::vector<ArrivalLabel> sharedOriginArrivals2;
    std::vector<int> sharedOriginMaxDelay;
    std::vector<int> sharedOriginTimestamp;

    IndexedMap<CandidateOriginLabel, false, StopId> shortcutOriginCandidates;
    IndexedMap<CandidateDestinationStopLabel, false, StopId> shortcutDestinationStopCandidates;
    IndexedMap<CandidateDestinationLabel, false, StopEventId> shortcutDestinationCandidates;

    std::vector<TripMinOriginDelay> tripMinOriginDelay;

    std::vector<CandidateDijkstraLabel> preOriginDijkstraLabels;
    std::vector<CandidateDijkstraLabel> candidateDijkstraLabels;
    ExternalKHeap<2, CandidateDijkstraLabel> candidateQueue;

    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;
    IndexedMap<CandidateRouteLabel, false, RouteId> candidateRoutes;
    IndexedMap<StopIndex, false, TripId> tripsServingUpdatedStops;
    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;

    int earliestDepartureTime;

    const int arrivalDelayBuffer;
    const int departureDelayBuffer;

    ShortcutCollection& shortcuts;
};

}
