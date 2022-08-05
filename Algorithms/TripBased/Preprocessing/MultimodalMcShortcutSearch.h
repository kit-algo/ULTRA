#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../Dijkstra/Dijkstra.h"

#include "../../../Helpers/Meta.h"
#include "../../../Helpers/Helpers.h"
#include "../../../Helpers/Types.h"

#include "../../../DataStructures/Container/Map.h"
#include "../../../DataStructures/Container/Set.h"
#include "../../../DataStructures/Container/ExternalKHeap.h"
#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../../DataStructures/TripBased/Shortcut.h"

namespace TripBased {

template<bool DEBUG = false, int TIME_FACTOR = 1>
class MultimodalMcShortcutSearch {

public:
    inline static constexpr bool Debug = DEBUG;
    inline static constexpr int TimeFactor = TIME_FACTOR;
    using KeyType = std::tuple<int, int>;
    using Type = MultimodalMcShortcutSearch<Debug, TimeFactor>;

public:
    struct ShortcutInfo {
        ShortcutInfo(const StopEventId origin, const int length, const StopEventId target, const bool isProper) :
            origin(origin),
            length(length),
            isProper(isProper) {
            targets.insert(target);
        }

        StopEventId origin;
        int length;
        Set<StopEventId> targets;
        bool isProper;
    };

    struct ArrivalLabel : public ExternalKHeapElement {
        ArrivalLabel() : arrivalTime(never) {}
        int arrivalTime;
        inline bool hasSmallerKey(const ArrivalLabel* const other) const noexcept {
            return arrivalTime < other->arrivalTime;
        }
    };

    struct InitialTransferLabel : public ExternalKHeapElement {
        InitialTransferLabel(const int transferDistance = INFTY, const int transferCost = INFTY) :
            transferDistance(transferDistance),
            transferCost(transferCost) {
        }

        inline bool hasSmallerKey(const InitialTransferLabel* const other) const noexcept {
            return transferDistance < other->transferDistance;
        }

        int transferDistance;
        int transferCost;
    };

    struct OneTripLabel : public ExternalKHeapElement {
        OneTripLabel(const int arrivalTime = never, const int walkingDistance = INFTY, const StopEventId shortcutOrigin = noStopEvent, const size_t timestamp = -1) :
            arrivalTime(arrivalTime),
            walkingDistance(walkingDistance),
            shortcutOrigin(shortcutOrigin),
            timestamp(timestamp) {
        }

        OneTripLabel(const OneTripLabel& parentLabel, const int walkingDistance) :
            arrivalTime(parentLabel.arrivalTime + walkingDistance),
            walkingDistance(parentLabel.walkingDistance + walkingDistance),
            shortcutOrigin(parentLabel.shortcutOrigin),
            timestamp(parentLabel.timestamp) {
        }

        int arrivalTime;
        int walkingDistance;
        StopEventId shortcutOrigin; //Only valid for candidates
        size_t timestamp;

        inline int getDominanceWalkingDistance() const noexcept {
            if constexpr (TimeFactor == 1) {
                return walkingDistance;
            } else {
                return ((walkingDistance - 1) / TimeFactor + 1) * TimeFactor;
            }
        }

        template<typename LABEL>
        inline bool dominates(const LABEL& other) const noexcept {
            return arrivalTime <= other.arrivalTime && getDominanceWalkingDistance() <= other.getDominanceWalkingDistance();
        }

        inline bool dominatesStrongly(const OneTripLabel& other) const noexcept {
            return (arrivalTime < other.arrivalTime && getDominanceWalkingDistance() <= other.getDominanceWalkingDistance()) || (arrivalTime <= other.arrivalTime && getDominanceWalkingDistance() < other.getDominanceWalkingDistance());
        }

        inline bool isCandidate() const noexcept {
            return shortcutOrigin != noStopEvent;
        }

        inline void witnessify() noexcept {
            shortcutOrigin = noStopEvent;
        }

        inline int getKey() const noexcept {
            return arrivalTime;
        }

        inline KeyType getQueueKey() const noexcept {
            return std::make_tuple(arrivalTime, walkingDistance);
        }

        inline bool hasSmallerKey(const OneTripLabel* const other) const noexcept {
            return getQueueKey() < other->getQueueKey();
        }

        inline bool operator==(const OneTripLabel& other) const noexcept {
            return arrivalTime == other.arrivalTime && walkingDistance == other.walkingDistance;
        }

        inline bool operator!=(const OneTripLabel& other) const noexcept {
            return arrivalTime != other.arrivalTime || walkingDistance != other.walkingDistance;
        }
    };

    struct TwoTripsLabel : public ExternalKHeapElement {
        TwoTripsLabel(const int arrivalTime = never, const int walkingDistance = INFTY, const StopEventId finalStopEvent = noStopEvent, const size_t timestamp = -1) :
            arrivalTime(arrivalTime),
            walkingDistance(walkingDistance),
            finalStopEvent(finalStopEvent),
            timestamp(timestamp) {
        }

        TwoTripsLabel(const TwoTripsLabel& parentLabel, const int walkingDistance) :
            arrivalTime(parentLabel.arrivalTime + walkingDistance),
            walkingDistance(parentLabel.walkingDistance + walkingDistance),
            finalStopEvent(noStopEvent),
            timestamp(parentLabel.timestamp) {
        }

        TwoTripsLabel(const OneTripLabel& oneTripLabel) :
            arrivalTime(oneTripLabel.arrivalTime),
            walkingDistance(oneTripLabel.walkingDistance),
            finalStopEvent(noStopEvent),
            //Set timestamp to INFTY so label can dominate equivalent candidates
            timestamp(INFTY) {
        }

        int arrivalTime;
        int walkingDistance;
        StopEventId finalStopEvent; //Only valid for candidates
        size_t timestamp;

        inline int getDominanceWalkingDistance() const noexcept {
            if constexpr (TimeFactor == 1) {
                return walkingDistance;
            } else {
                return ((walkingDistance - 1) / TimeFactor + 1) * TimeFactor;
            }
        }

        template<typename LABEL>
        inline bool dominates(const LABEL& other) const noexcept {
            return arrivalTime <= other.arrivalTime && getDominanceWalkingDistance() <= other.getDominanceWalkingDistance();
        }

        inline bool dominatesStrongly(const TwoTripsLabel& other) const noexcept {
            return (arrivalTime < other.arrivalTime && getDominanceWalkingDistance() <= other.getDominanceWalkingDistance()) || (arrivalTime <= other.arrivalTime && getDominanceWalkingDistance() < other.getDominanceWalkingDistance());
        }

        inline bool isCandidate() const noexcept {
            return finalStopEvent != noStopEvent;
        }

        inline void witnessify() noexcept {
            finalStopEvent = noStopEvent;
        }

        inline int getKey() const noexcept {
            return arrivalTime;
        }

        inline KeyType getQueueKey() const noexcept {
            return std::make_tuple(arrivalTime, walkingDistance);
        }

        inline bool hasSmallerKey(const TwoTripsLabel* const other) const noexcept {
            return getQueueKey() < other->getQueueKey();
        }

        inline bool operator==(const TwoTripsLabel& other) const noexcept {
            return arrivalTime == other.arrivalTime && walkingDistance == other.walkingDistance;
        }

        inline bool operator!=(const TwoTripsLabel& other) const noexcept {
            return arrivalTime != other.arrivalTime || walkingDistance != other.walkingDistance;
        }
    };

    template<typename LABEL>
    class Bag : public ExternalKHeapElement {
    public:
        using Label = LABEL;
        using Iterator = typename std::vector<Label>::const_iterator;
        static constexpr int logK = 2;
        static constexpr int K = 1 << logK;

        Bag() : heapSize(0) {}

        inline Label& operator[](const size_t i) noexcept {
            return labels[i];
        }

        inline const Label& operator[](const size_t i) const noexcept {
            return labels[i];
        }

        inline Iterator begin() const noexcept {
            return labels.begin();
        }

        inline Iterator end() const noexcept {
            return labels.end();
        }

        inline bool empty() const noexcept {
            return labels.empty();
        }

        inline size_t size() const noexcept {
            return labels.size();
        }

        inline bool heapEmpty() const noexcept {
            return heapSize == 0;
        }

        inline const Label& front() const noexcept {
            AssertMsg(!empty(), "An empty heap has no front!");
            AssertMsg(heapSize > 0, "An empty heap has no front!");
            return labels[0];
        }

        inline int getKey() const noexcept {
            return front().getKey();
        }

        inline bool hasSmallerKey(const Bag* const other) const noexcept {
            const KeyType thisKey = front().getQueueKey();
            const KeyType otherKey = other->front().getQueueKey();
            return thisKey < otherKey || (thisKey == otherKey && this < other);
        }

        inline const Label& extractFront() noexcept {
            AssertMsg(!empty(), "An empty heap has no front!");
            AssertMsg(heapSize > 0, "An empty heap has no front!");
            heapSize--;
            if (heapSize > 0) {
                std::swap(labels[0], labels[heapSize]);
                siftDown(0);
            }
            return labels[heapSize];
        }

        inline bool prune(const Label& newLabel) noexcept {
            return internalMerge<false>(newLabel, NoOperation, [&](const Label& label) {
                return label.dominatesStrongly(newLabel);
            });
        }

        template<typename OTHER_BAG>
        inline bool pruneWithBag(const OTHER_BAG& otherBag) noexcept {
            bool pruned = false;
            for (const typename OTHER_BAG::Label& label : otherBag) {
                pruned |= prune(label);
            }
            return pruned;
        }

        template<typename FUNCTION = NO_OPERATION>
        inline std::tuple<bool, bool> merge(const Label& newLabel, const FUNCTION& processRemovedHeapLabel = NoOperation) noexcept {
            bool isProper = true;
            const bool result = internalMerge<true>(newLabel, processRemovedHeapLabel, [&](const Label& label) {
                if (newLabel.isCandidate() && label.timestamp < newLabel.timestamp) {
                    isProper &= (label != newLabel);
                    return label.dominatesStrongly(newLabel);
                } else {
                    return label.dominates(newLabel);
                }
            });
            return std::tuple<bool, bool>{result, isProper};
        }

        template<typename FUNCTION = NO_OPERATION>
        inline bool mergeWitness(const Label& newLabel, const FUNCTION& processRemovedHeapLabel = NoOperation) noexcept {
            return internalMerge<true>(newLabel, processRemovedHeapLabel, [&](const Label& label) {
                return label.dominates(newLabel);
            });
        }

        template<typename FUNCTION = NO_OPERATION>
        inline std::tuple<bool, bool> mergeTransitiveWitness(const Label& newLabel, const FUNCTION& processRemovedHeapLabel = NoOperation) noexcept {
            size_t removedLabels = 0;
            size_t removedHeapLabels = 0;
            for (size_t i = 0; i < labels.size(); i++) {
                if (labels[i].dominates(newLabel)) return std::tuple<bool, bool>{false, false};
                if (newLabel.dominates(labels[i])) {
                    removedLabels++;
                    if (i < heapSize) {
                        processRemovedHeapLabel(labels[i]);
                        removedHeapLabels++;
                    }
                    continue;
                }
                labels[i - removedLabels] = labels[i];
            }
            heapSize -= removedHeapLabels;
            labels.resize(labels.size() - removedLabels + 1);
            labels.back() = newLabel;
            if (removedHeapLabels > 0) heapify();
            return std::tuple<bool, bool>{true, removedHeapLabels > 0};
        }

        template<typename OTHER_LABEL>
        inline bool dominates(const OTHER_LABEL& other) noexcept {
            for (const Label& label : labels) {
                if (label.dominates(other)) return true;
            }
            return false;
        }

        inline void witnessify() noexcept {
            for (size_t i = 0; i < heapSize; i++) {
                labels[i].witnessify();
            }
        }

    private:
        template<bool ONTO_HEAP, typename FUNCTION1 = NO_OPERATION, typename FUNCTION2 = NO_OPERATION>
        inline bool internalMerge(const Label& newLabel, const FUNCTION1& processRemovedHeapLabel = NoOperation, const FUNCTION2& isDominatedBy = NoOperation) noexcept {
            size_t removedLabels = 0;
            size_t removedHeapLabels = 0;
            for (size_t i = 0; i < labels.size(); i++) {
                if (isDominatedBy(labels[i])) return false;
                if (newLabel.dominates(labels[i])) {
                    removedLabels++;
                    if (i < heapSize) {
                        processRemovedHeapLabel(labels[i]);
                        removedHeapLabels++;
                    }
                    continue;
                }
                labels[i - removedLabels] = labels[i];
            }
            heapSize -= removedHeapLabels;
            labels.resize(labels.size() - removedLabels + 1);
            labels.back() = newLabel;
            if constexpr (ONTO_HEAP) {
                std::swap(labels.back(), labels[heapSize]);
                heapSize++;
                heapify();
                return true;
            } else {
                if (removedHeapLabels > 0) heapify();
                return (removedHeapLabels > 0);
            }
        }

        inline void heapify() noexcept {
            if (heapSize <= 1) return;
            for (size_t i = parent(heapSize - 1); i != size_t(-1); i--) {
                siftDown(i);
            }
        }

        inline void siftDown(size_t i) noexcept {
            AssertMsg(i < heapSize, "siftDown index out of range!");
            while (true) {
                size_t minIndex = i;
                const size_t childrenStart = firstChild(i);
                const size_t childrenEnd = std::min(childrenStart + K, heapSize);
                for (size_t j = childrenStart; j < childrenEnd; j++) {
                    if (labels[j].hasSmallerKey(&labels[minIndex])) {
                        minIndex = j;
                    }
                }
                if (minIndex == i) break;
                std::swap(labels[i], labels[minIndex]);
                i = minIndex;
            }
        }

        inline size_t parent(const size_t i) const noexcept {
            return (i - 1) >> logK;
        }

        inline size_t firstChild(const size_t i) const noexcept {
            return (i << logK) + 1;
        }

        std::vector<Label> labels;
        size_t heapSize;
    };

    using OneTripBag = Bag<OneTripLabel>;
    using TwoTripsBag = Bag<TwoTripsLabel>;

    struct RouteLabel {
        const RAPTOR::StopEvent* trip;
        int walkingDistance;
        StopEventId shortcutOrigin;
        StopIndex parentStopIndex;
        size_t timestamp;

        inline const RAPTOR::StopEvent* parentStopEvent() const noexcept {
            return trip + parentStopIndex;
        }

        inline int getDominanceWalkingDistance() const noexcept {
            if constexpr (TimeFactor == 1) {
                return walkingDistance;
            } else {
                return ((walkingDistance - 1) / TimeFactor + 1) * TimeFactor;
            }
        }

        inline bool dominates(const RouteLabel& other) const noexcept {
            return trip <= other.trip && getDominanceWalkingDistance() <= other.getDominanceWalkingDistance();
        }

        inline bool isCandidate() const noexcept {
            return shortcutOrigin != noStopEvent;
        }
    };

    struct RouteBag {
        using Iterator = typename std::vector<RouteLabel>::const_iterator;

        inline bool merge(const RouteLabel& newLabel) noexcept {
            size_t removedLabels = 0;
            for (size_t i = 0; i < labels.size(); i++) {
                if (labels[i].dominates(newLabel)) return false;
                if (newLabel.dominates(labels[i])) {
                    removedLabels++;
                    continue;
                }
                labels[i - removedLabels] = labels[i];
            }
            labels.resize(labels.size() - removedLabels + 1);
            labels.back() = newLabel;
            return true;
        }

        inline Iterator begin() const noexcept {
            return labels.begin();
        }

        inline Iterator end() const noexcept {
            return labels.end();
        }

        std::vector<RouteLabel> labels;
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
    MultimodalMcShortcutSearch(const Data& tripData, const TransferGraph& transitiveTransferGraph, const int intermediateWitnessTransferLimit, const int finalWitnessTransferLimit) :
        tripData(tripData),
        data(tripData.raptorData),
        transitiveTransferGraph(transitiveTransferGraph),
        stationOfStop(data.numberOfStops()),
        sourceStation(),
        sourceDepartureTime(0),
        stopsReachedByDirectTransfer(data.numberOfStops()),
        shortcutCandidatesInQueue(0),
        shortcutDestinationCandidates(data.numberOfStopEvents()),
        properDestinationCandidates(0),
        routesServingUpdatedStops(data.numberOfRoutes()),
        stopsUpdatedByRoute(data.numberOfStops()),
        stopsUpdatedByTransfer(data.numberOfStops()),
        intermediateWitnessTransferLimit(intermediateWitnessTransferLimit),
        finalWitnessTransferLimit(finalWitnessTransferLimit),
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
        AssertMsg(initialTransferQueue.empty(), "Queue for round 0 is not empty!");
        AssertMsg(stationOfStop[source].representative == source, "Source " << source << " is not representative of its station!");
        clear();
        sourceStation = stationOfStop[source];
        initialDijkstra();
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
        properDestinationCandidates = 0;

        routesServingUpdatedStops.clear();
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();

        sourceDepartureTime = label.departureTime;
        for (const StopId stop : stopsReachedByDirectTransfer) {
            stopsUpdatedByTransfer.insert(stop);
        }
        collectRoutes1(label.routes);
        scanRoutes1();
        relaxIntermediateTransitiveTransfers();
        intermediateDijkstra();
        collectRoutes2();
        scanRoutes2();
        relaxFinalTransitiveTransfers();
        finalDijkstra();
    }

    inline std::vector<ConsolidatedDepartureLabel> collectDepartures(const int minTime, const int maxTime) noexcept {
        AssertMsg(initialTransferLabels[sourceStation.representative].transferDistance == 0, "Direct transfer for source " << sourceStation.representative << " is incorrect!");
        const int cutoffTime = std::max(minTime, earliestDepartureTime);
        std::vector<DepartureLabel> departureLabels;
        for (const RouteId route : data.routes()) {
            const StopId* stops = data.stopArrayOfRoute(route);
            const size_t tripSize = data.numberOfStopsInRoute(route);
            int minimalTransferTime = never;
            for (size_t stopIndex = 0; stopIndex + 1 < tripSize; stopIndex++) {
                const int initialTransferTime = initialTransferLabels[stops[stopIndex]].transferDistance;
                if (initialTransferTime > minimalTransferTime) continue;
                minimalTransferTime = initialTransferTime;
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

        std::vector<InitialTransferLabel>(data.transferGraph.numVertices()).swap(initialTransferLabels);
        stopsReachedByDirectTransfer.clear();

        oneTripQueue.clear();
        std::vector<OneTripBag>(data.transferGraph.numVertices()).swap(oneTripBags);

        twoTripsQueue.clear();
        std::vector<TwoTripsBag>(data.transferGraph.numVertices()).swap(twoTripsBags);

        std::vector<StopEventId>(data.numberOfStopEvents(), noStopEvent).swap(twoTripsRouteParent);

        shortcutCandidatesInQueue = 0;
        shortcutDestinationCandidates.clear();
        properDestinationCandidates = 0;

        routesServingUpdatedStops.clear();
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
    }

    inline void collectRoutes1(const std::vector<RAPTOR::RouteSegment>& routes) noexcept {
        for (const RAPTOR::RouteSegment& route : routes) {
            AssertMsg(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
            AssertMsg(route.stopIndex + 1 < data.numberOfStopsInRoute(route.routeId), "RouteSegment " << route << " is not a departure event!");
            AssertMsg(data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime >= initialArrivalTime(data.stopOfRouteSegment(route)), "RouteSegment " << route << " is not reachable!");
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
                if (routesServingUpdatedStops.contains(route.routeId)) {
                    routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                }
            }
        }
        routesServingUpdatedStops.sortKeys();
    }

    inline void scanRoutes1Full() noexcept {
        shortcutCandidatesInQueue = 0;
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            StopIndex stopIndex = routesServingUpdatedStops[route];
            const size_t tripSize = data.numberOfStopsInRoute(route);
            AssertMsg(stopIndex < tripSize - 1, "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " << stopIndex << ", TripSize: " << tripSize << ")!");

            const StopId* stops = data.stopArrayOfRoute(route);
            StopId stop = stops[stopIndex];

            const RAPTOR::StopEvent* firstTrip = data.firstTripOfRoute(route);
            const RAPTOR::StopEvent* lastTrip = data.lastTripOfRoute(route);

            RouteBag routeBag;

            while (stopIndex < tripSize - 1) {
                if (stopsUpdatedByTransfer.contains(stop)) {
                    const int walkingDistance = initialTransferLabels[stop].transferCost;
                    const int arrivalTime = initialArrivalTime(stop);
                    const RAPTOR::StopEvent* trip = firstTrip;
                    while ((trip < lastTrip) && (trip[stopIndex].departureTime < arrivalTime)) {
                        trip += tripSize;
                    }
                    if (trip[stopIndex].departureTime >= arrivalTime) {
                        RouteLabel newLabel;
                        newLabel.trip = trip;
                        newLabel.walkingDistance = walkingDistance;
                        newLabel.parentStopIndex = stopIndex;
                        routeBag.merge(newLabel);
                    }
                }
                stopIndex++;
                stop = stops[stopIndex];
                for (const RouteLabel& label : routeBag) {
                    const StopId parent = stops[label.parentStopIndex];
                    const bool isCandidate = stationOfStop[parent].representative == sourceStation.representative;
                    const StopEventId stopEvent = StopEventId(label.trip + stopIndex - &(data.stopEvents[0]));
                    const OneTripLabel newLabel(label.trip[stopIndex].arrivalTime, label.walkingDistance, isCandidate ? stopEvent : noStopEvent, timestamp);
                    arrivalByRoute1(stop, newLabel);
                }
            }
        }
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
    }

    inline void scanRoutes1() noexcept {
        shortcutCandidatesInQueue = 0;
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            const StopIndex stopIndex = routesServingUpdatedStops[route];
            RAPTOR::TripIterator tripIterator = data.getTripIterator(route, stopIndex);
            StopIndex parentIndex = stopIndex;
            while (tripIterator.hasFurtherStops()) {
                //Find earliest trip that can be entered
                if (tripIterator.hasEarlierTrip() && (tripIterator.previousDepartureTime() >= initialArrivalTime(tripIterator.stop()))) {
                    AssertMsg(stopsUpdatedByTransfer.contains(tripIterator.stop()), "Trip was entered at a stop that was not reached!");
                    do {
                        tripIterator.previousTrip();
                    } while (tripIterator.hasEarlierTrip() && (tripIterator.previousDepartureTime() >= initialArrivalTime(tripIterator.stop())));
                    parentIndex = tripIterator.getStopIndex();
                }
                tripIterator.nextStop();
                const StopId parent = tripIterator.stop(parentIndex);
                const int walkingDistance = initialTransferLabels[parent].transferCost;
                const bool isCandidate = stationOfStop[parent].representative == sourceStation.representative;
                const OneTripLabel label(tripIterator.arrivalTime(), walkingDistance, isCandidate ? StopEventId(tripIterator.stopEvent() - &(data.stopEvents[0])) : noStopEvent, timestamp);
                arrivalByRoute1(tripIterator.stop(), label);
            }
        }
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
    }

    inline void scanRoutes2() noexcept {
        properDestinationCandidates = 0;
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            StopIndex stopIndex = routesServingUpdatedStops[route];
            const size_t tripSize = data.numberOfStopsInRoute(route);
            AssertMsg(stopIndex < tripSize - 1, "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " << stopIndex << ", TripSize: " << tripSize << ")!");

            const StopId* stops = data.stopArrayOfRoute(route);
            StopId stop = stops[stopIndex];

            const RAPTOR::StopEvent* firstTrip = data.firstTripOfRoute(route);
            const RAPTOR::StopEvent* lastTrip = data.lastTripOfRoute(route);

            RouteBag routeBag;

            while (stopIndex < tripSize - 1) {
                for (const OneTripLabel& label : oneTripBags[stop]) {
                    if (label.timestamp != timestamp) continue;
                    const RAPTOR::StopEvent* trip = firstTrip;
                    while ((trip < lastTrip) && (trip[stopIndex].departureTime < label.arrivalTime)) {
                        trip += tripSize;
                    }
                    if (trip[stopIndex].departureTime < label.arrivalTime) continue;

                    RouteLabel newLabel;
                    newLabel.trip = trip;
                    newLabel.walkingDistance = label.walkingDistance;
                    newLabel.shortcutOrigin = label.shortcutOrigin;
                    newLabel.parentStopIndex = stopIndex;
                    newLabel.timestamp = label.timestamp;
                    routeBag.merge(newLabel);
                }
                stopIndex++;
                stop = stops[stopIndex];
                for (const RouteLabel& label : routeBag) {
                    TwoTripsLabel newLabel(label.trip[stopIndex].arrivalTime, label.walkingDistance, noStopEvent, label.timestamp);
                    const StopEventId finalStopEvent = StopEventId(label.trip + stopIndex - &(data.stopEvents[0]));
                    arrivalByRoute2(stop, newLabel, label, finalStopEvent);
                }
            }
        }
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
    }

    inline void initialDijkstra() noexcept {
        initialTransferLabels[sourceStation.representative] = InitialTransferLabel(0, 0);
        initialTransferQueue.update(&(initialTransferLabels[sourceStation.representative]));
        while (!initialTransferQueue.empty()) {
            InitialTransferLabel* currentLabel = initialTransferQueue.extractFront();
            const Vertex currentVertex = Vertex(currentLabel - &(initialTransferLabels[0]));
            for (const Edge edge : data.transferGraph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = data.transferGraph.get(ToVertex, edge);
                const int newDistance = currentLabel->transferDistance + data.transferGraph.get(TravelTime, edge);
                if (newDistance < initialTransferLabels[neighborVertex].transferDistance) {
                    initialTransferLabels[neighborVertex].transferDistance = newDistance;
                    initialTransferLabels[neighborVertex].transferCost = newDistance;
                    initialTransferQueue.update(&(initialTransferLabels[neighborVertex]));
                }
            }
            if (data.isStop(currentVertex)) {
                stopsReachedByDirectTransfer.insert(StopId(currentVertex));
            }
        }
        for (const StopId sourceStop : sourceStation.stops) {
            AssertMsg(stopsReachedByDirectTransfer.contains(sourceStop), "Source was not updated by transfer!");
        }

        for (const Edge edge : transitiveTransferGraph.edgesFrom(sourceStation.representative)) {
            const StopId neighborStop(transitiveTransferGraph.get(ToVertex, edge));
            const int travelTime = transitiveTransferGraph.get(TravelTime, edge);
            if (travelTime < initialTransferLabels[neighborStop].transferDistance) {
                initialTransferLabels[neighborStop].transferDistance = travelTime;
                initialTransferLabels[neighborStop].transferCost = 0;
            }
            stopsReachedByDirectTransfer.insert(neighborStop);
        }
    }

    inline void relaxIntermediateTransitiveTransfers() noexcept {
        AssertMsg(stopsUpdatedByTransfer.empty(), "stopsUpdatedByTransfer is not empty!");

        for (const StopId stop : stopsUpdatedByRoute) {
            for (const Edge edge : transitiveTransferGraph.edgesFrom(stop)) {
                const Vertex toStop = transitiveTransferGraph.get(ToVertex, edge);
                const int travelTime = transitiveTransferGraph.get(TravelTime, edge);
                for (const OneTripLabel& label : oneTripBags[stop]) {
                    if (label.timestamp != timestamp) continue;
                    OneTripLabel newLabel(label, travelTime);
                    newLabel.witnessify();
                    if (updateOneTripBagTransitiveWitness(toStop, newLabel)) {
                        stopsUpdatedByTransfer.insert(StopId(toStop));
                    }
                }
            }
        }
    }

    inline void intermediateDijkstra() noexcept {
        for (const StopId stop : stopsUpdatedByRoute) {
            if (oneTripBags[stop].heapEmpty()) continue;
            oneTripQueue.update(&(oneTripBags[stop]));
        }
        if (shortcutCandidatesInQueue == 0) {
            stopsUpdatedByRoute.clear();
            return;
        }

        int transferLimit = intMax;
        while (!oneTripQueue.empty()) {
            OneTripBag* currentBag = oneTripQueue.extractFront();
            const OneTripLabel& currentLabel = currentBag->extractFront();
            if (!currentBag->heapEmpty()) oneTripQueue.update(currentBag);
            const Vertex currentVertex = Vertex(currentBag - &(oneTripBags[0]));
            for (Edge edge : data.transferGraph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = data.transferGraph.get(ToVertex, edge);
                const OneTripLabel newLabel(currentLabel, data.transferGraph.get(TravelTime, edge));
                arrivalByEdge1(neighborVertex, newLabel);
            }
            if (currentLabel.isCandidate()) {
                shortcutCandidatesInQueue--;
            }
            if (shortcutCandidatesInQueue == 0) {
                //Once all candidates have been settled, leave the Dijkstra search running until witnessTransferLimit is met.
                //Note that witnesses above the limit may be pruned, leading to superfluous shortcuts.
                shortcutCandidatesInQueue = -1;
                transferLimit = currentLabel.getKey() + intermediateWitnessTransferLimit;
                if (transferLimit < currentLabel.getKey()) transferLimit = intMax;
                if constexpr (Debug) std::cout << "   Transfer limit in round 1: " << String::secToString(transferLimit - sourceDepartureTime) << ", travel time: " << (currentLabel.arrivalTime - sourceDepartureTime) << ", walking distance: " << currentLabel.walkingDistance << std::endl;
            }
            if (data.isStop(currentVertex)) {
                stopsUpdatedByTransfer.insert(StopId(currentVertex));
            }
            if (currentLabel.getKey() > transferLimit) break;
        }

        stopsUpdatedByRoute.clear();
    }

    inline void relaxFinalTransitiveTransfers() noexcept {
        AssertMsg(stopsUpdatedByTransfer.empty(), "stopsUpdatedByTransfer is not empty!");

        for (const StopId stop : stopsUpdatedByRoute) {
            for (const Edge edge : transitiveTransferGraph.edgesFrom(stop)) {
                const Vertex toStop = transitiveTransferGraph.get(ToVertex, edge);
                const int travelTime = transitiveTransferGraph.get(TravelTime, edge);
                for (const TwoTripsLabel& label : twoTripsBags[stop]) {
                    if (label.timestamp != timestamp) continue;
                    TwoTripsLabel newLabel(label, travelTime);
                    newLabel.witnessify();
                    updateTwoTripsBagTransitiveWitness(toStop, newLabel);
                }
            }
        }
    }

    inline void finalDijkstra() noexcept {
        for (const StopId stop : stopsUpdatedByRoute) {
            if (twoTripsBags[stop].heapEmpty()) continue;
            twoTripsQueue.update(&(twoTripsBags[stop]));
        }

        int transferLimit = intMax;
        while (!twoTripsQueue.empty()) {
            TwoTripsBag* currentBag = twoTripsQueue.extractFront();
            const TwoTripsLabel& currentLabel = currentBag->extractFront();
            if (!currentBag->heapEmpty()) twoTripsQueue.update(currentBag);
            const Vertex currentVertex = Vertex(currentBag - &(twoTripsBags[0]));
            for (Edge edge : data.transferGraph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = data.transferGraph.get(ToVertex, edge);
                const TwoTripsLabel newLabel(currentLabel, data.transferGraph.get(TravelTime, edge));
                arrivalByEdge2(neighborVertex, newLabel);
            }
            if (data.isStop(currentVertex) && currentLabel.isCandidate()) {
                const StopEventId shortcutDestination = twoTripsRouteParent[currentLabel.finalStopEvent];
                if (shortcutDestination != noStopEvent && shortcutDestinationCandidates.contains(shortcutDestination)) {
                    //No witness dominates this candidate journey => insert shortcut
                    const ShortcutInfo& info = shortcutDestinationCandidates[shortcutDestination];
                    shortcuts.emplace_back(info.origin, shortcutDestination, info.length);
                    AssertMsg(info.targets.contains(currentLabel.finalStopEvent), "Stop event " << currentLabel.finalStopEvent << " is not contained in shortcutDestinationCandidates list of " << shortcutDestination << "!");
                    //Unmark other candidates using this shortcut, since we don't need them anymore
                    for (const StopEventId obsoleteCandidate : info.targets) {
                        twoTripsRouteParent[obsoleteCandidate] = noStopEvent;
                    }
                    if (shortcutDestinationCandidates[shortcutDestination].isProper) {
                        properDestinationCandidates--;
                    }
                    shortcutDestinationCandidates.remove(shortcutDestination);
                }
            }
            if (properDestinationCandidates == 0) {
                //Once all proper candidates have been settled, leave the Dijkstra search running until witnessTransferLimit is met.
                //Note that witnesses above the limit may be pruned, leading to superfluous shortcuts.
                properDestinationCandidates = -1;
                transferLimit = currentLabel.getKey() + finalWitnessTransferLimit;
                if (transferLimit < currentLabel.getKey()) transferLimit = intMax;
                if constexpr (Debug) std::cout << "   Transfer limit in round 2: " << String::secToString(transferLimit - sourceDepartureTime) << ", travel time: " << (currentLabel.arrivalTime - sourceDepartureTime) << ", walking distance: " << currentLabel.walkingDistance << std::endl;
            }
            if (shortcutDestinationCandidates.empty()) break;
            if (currentLabel.getKey() > transferLimit) break;
        }

        //Insert shortcuts for remaining improper candidates
        for (const StopEventId shortcutDestination : shortcutDestinationCandidates.getKeys()) {
            const ShortcutInfo& info = shortcutDestinationCandidates[shortcutDestination];
            shortcuts.emplace_back(info.origin, shortcutDestination, info.length);
            //Unmark other candidates using this shortcut, since we don't need them anymore
            for (const StopEventId obsoleteCandidate : info.targets) {
                twoTripsRouteParent[obsoleteCandidate] = noStopEvent;
            }
        }

        //Clean up finalStopEvent pointers of obsolete candidates
        for (TwoTripsBag* unsettledBag : twoTripsQueue.data()) {
            unsettledBag->witnessify();
        }

        shortcutDestinationCandidates.clear();
        stopsUpdatedByRoute.clear();
    }

    inline void arrivalByRoute1(const StopId stop, const OneTripLabel& label) noexcept {
        if (!updateOneTripBag(stop, label)) return;
        if (label.isCandidate()) shortcutCandidatesInQueue++;
        //If the bag was improved, remove it from the queue - it will be re-added with the correct key later.
        if (oneTripBags[stop].isOnHeap()) {
            oneTripQueue.remove(&(oneTripBags[stop]));
        }
        stopsUpdatedByRoute.insert(stop);
    }

    inline void arrivalByRoute2(const StopId stop, TwoTripsLabel& label, const RouteLabel& routeLabel, const StopEventId finalStopEvent) noexcept {
        if (routeLabel.isCandidate()) {
            label.finalStopEvent = finalStopEvent;
        }
        bool wasInserted, isProper;
        std::tie(wasInserted, isProper) = updateTwoTripsBagCandidate(stop, label);
        if (!wasInserted) return;
        if (twoTripsBags[stop].isOnHeap()) {
            twoTripsQueue.remove(&(twoTripsBags[stop]));
        }
        stopsUpdatedByRoute.insert(stop);
        AssertMsg(!twoTripsBags[stop].heapEmpty(), "Inserting empty bag into queue!");
        if (routeLabel.isCandidate()) {
            const StopEventId shortcutDestination = StopEventId(routeLabel.parentStopEvent() - &(data.stopEvents[0]));
            twoTripsRouteParent[finalStopEvent] = shortcutDestination;
            if (!shortcutDestinationCandidates.contains(shortcutDestination)) {
                shortcutDestinationCandidates.insert(shortcutDestination, ShortcutInfo(routeLabel.shortcutOrigin, label.walkingDistance, finalStopEvent, isProper));
                if (isProper) properDestinationCandidates++;
            } else {
                shortcutDestinationCandidates[shortcutDestination].targets.insert(finalStopEvent);
            }
        } else {
            twoTripsRouteParent[finalStopEvent] = noStopEvent;
        }
    }

    inline void arrivalByEdge1(const Vertex vertex, const OneTripLabel& label) noexcept {
        if (!updateOneTripBag(vertex, label)) return;
        if (label.isCandidate()) shortcutCandidatesInQueue++;
        oneTripQueue.update(&(oneTripBags[vertex]));
    }

    inline void arrivalByEdge2(const Vertex vertex, const TwoTripsLabel& label) noexcept {
        if (!updateTwoTripsBagWitness(vertex, label)) return;
        twoTripsQueue.update(&(twoTripsBags[vertex]));
    }

    inline int initialArrivalTime(const StopId stop) const noexcept {
        return initialTransferLabels[stop].transferDistance + sourceDepartureTime;
    }

    inline bool updateOneTripBag(const Vertex vertex, const OneTripLabel& newLabel) noexcept {
        const OneTripLabel walkingLabel = getWalkingLabel(vertex);
        if (walkingLabel.dominates(newLabel)) return false;
        const bool merged = std::get<0>(oneTripBags[vertex].merge(newLabel, [&](const OneTripLabel& removedLabel) {
            if (removedLabel.isCandidate()) shortcutCandidatesInQueue--;
        }));
        if (merged && twoTripsBags[vertex].prune(TwoTripsLabel(newLabel))) {
            pruneBagInQueue(twoTripsBags[vertex], twoTripsQueue);
        }
        return merged;
    }

    inline bool updateOneTripBagTransitiveWitness(const Vertex vertex, const OneTripLabel& newLabel) noexcept {
        const OneTripLabel walkingLabel = getWalkingLabel(vertex);
        if (walkingLabel.dominates(newLabel)) return false;
        bool merged, removedHeapLabels;
        std::tie(merged, removedHeapLabels) = oneTripBags[vertex].mergeTransitiveWitness(newLabel, [&](const OneTripLabel& removedLabel) {
            if (removedLabel.isCandidate()) shortcutCandidatesInQueue--;
        });
        if (!merged) return false;
        if (removedHeapLabels && oneTripBags[vertex].isOnHeap()) {
            pruneBagInQueue(oneTripBags[vertex], oneTripQueue);
        }
        if (twoTripsBags[vertex].prune(TwoTripsLabel(newLabel))) {
            pruneBagInQueue(twoTripsBags[vertex], twoTripsQueue);
        }
        return true;
    }

    inline bool updateTwoTripsBagTransitiveWitness(const Vertex vertex, const TwoTripsLabel& newLabel) noexcept {
        const OneTripLabel walkingLabel = getWalkingLabel(vertex);
        if (walkingLabel.dominates(newLabel)) return false;
        bool merged, removedHeapLabels;
        std::tie(merged, removedHeapLabels) = twoTripsBags[vertex].mergeTransitiveWitness(newLabel, [&](const TwoTripsLabel& removedLabel) {
            pruneShortcutDestinationCandidates(removedLabel);
        });
        if (!merged) return false;
        if (removedHeapLabels && twoTripsBags[vertex].isOnHeap()) {
            pruneBagInQueue(twoTripsBags[vertex], twoTripsQueue);
        }
        return true;
    }

    inline bool updateTwoTripsBagWitness(const Vertex vertex, const TwoTripsLabel& newLabel) noexcept {
        const OneTripLabel walkingLabel = getWalkingLabel(vertex);
        if (walkingLabel.dominates(newLabel)) return false;
        return twoTripsBags[vertex].mergeWitness(newLabel, [&](const TwoTripsLabel& removedLabel) {
            pruneShortcutDestinationCandidates(removedLabel);
        });
    }

    inline std::tuple<bool, bool> updateTwoTripsBagCandidate(const Vertex vertex, const TwoTripsLabel& newLabel) noexcept {
        const OneTripLabel walkingLabel = getWalkingLabel(vertex);
        if (walkingLabel.dominates(newLabel)) return std::tuple<bool, bool>{false, false};
        return twoTripsBags[vertex].merge(newLabel, [&](const TwoTripsLabel& removedLabel) {
            pruneShortcutDestinationCandidates(removedLabel);
        });
    }

    inline void pruneShortcutDestinationCandidates(const TwoTripsLabel& removedLabel) noexcept {
        if (!removedLabel.isCandidate()) return;
        const StopEventId shortcutDestination = twoTripsRouteParent[removedLabel.finalStopEvent];
        if (shortcutDestination == noStopEvent || !shortcutDestinationCandidates.contains(shortcutDestination)) return;
        AssertMsg(shortcutDestinationCandidates[shortcutDestination].targets.contains(removedLabel.finalStopEvent), "Stop event " << removedLabel.finalStopEvent << " is not contained in shortcutDestinationCandidates list of " << shortcutDestination << "!");
        shortcutDestinationCandidates[shortcutDestination].targets.erase(removedLabel.finalStopEvent);
        if (shortcutDestinationCandidates[shortcutDestination].targets.empty()) {
            if (shortcutDestinationCandidates[shortcutDestination].isProper) {
                properDestinationCandidates--;
            }
            shortcutDestinationCandidates.remove(shortcutDestination);
        }
        twoTripsRouteParent[removedLabel.finalStopEvent] = noStopEvent;
    }

    template<typename BAG_TYPE, typename QUEUE_TYPE>
    inline void pruneBagInQueue(BAG_TYPE& bag, QUEUE_TYPE& queue) noexcept {
        AssertMsg(bag.isOnHeap(), "Heap labels were removed from bag, but bag is not in queue!");
        if (bag.heapEmpty()) {
            queue.remove(&bag);
        } else {
            queue.update(&bag);
        }
    }

    inline OneTripLabel getWalkingLabel(const Vertex vertex) noexcept {
        const int transferDistance = initialTransferLabels[vertex].transferDistance;
        return OneTripLabel(transferDistance == INFTY ? INFTY : sourceDepartureTime + transferDistance, initialTransferLabels[vertex].transferCost);
    }

private:
    const Data& tripData;
    const RAPTOR::Data& data;
    const TransferGraph& transitiveTransferGraph;
    std::vector<Station> stationOfStop;

    Station sourceStation;
    int sourceDepartureTime;

    std::vector<InitialTransferLabel> initialTransferLabels;
    ExternalKHeap<2, InitialTransferLabel> initialTransferQueue;
    IndexedSet<false, StopId> stopsReachedByDirectTransfer;

    std::vector<OneTripBag> oneTripBags;
    ExternalKHeap<2, OneTripBag> oneTripQueue;

    std::vector<TwoTripsBag> twoTripsBags;
    ExternalKHeap<2, TwoTripsBag> twoTripsQueue;

    //For each stop event. If a candidate leads there, points to the corresponding shortcut destination stop event
    std::vector<StopEventId> twoTripsRouteParent;

    size_t shortcutCandidatesInQueue;
    IndexedMap<ShortcutInfo, false, StopEventId> shortcutDestinationCandidates;
    size_t properDestinationCandidates;
    std::vector<Shortcut> shortcuts;

    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;
    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;

    int intermediateWitnessTransferLimit;
    int finalWitnessTransferLimit;

    int earliestDepartureTime;

    size_t timestamp;

};

}
