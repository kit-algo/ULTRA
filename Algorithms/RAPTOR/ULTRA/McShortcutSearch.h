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

namespace RAPTOR::ULTRA {

template<bool DEBUG = false, bool USE_ARRIVAL_KEY = true, bool FULL_ROUTE_SCANS = false>
class McShortcutSearch {

public:
    inline static constexpr bool Debug = DEBUG;
    inline static constexpr bool UseArrivalKey = USE_ARRIVAL_KEY;
    inline static constexpr bool FullRouteScans = FULL_ROUTE_SCANS;
    using KeyType = Meta::IF<UseArrivalKey, std::tuple<int, int>, std::tuple<int, int, int>>;
    using Type = McShortcutSearch<Debug, UseArrivalKey, FullRouteScans>;

public:
    struct ShortcutInfo {
        ShortcutInfo(const StopId origin, const StopId destination, const int length, const StopEventId target, const bool isProper) :
            origin(origin),
            destination(destination),
            length(length),
            isProper(isProper) {
            targets.insert(target);
        }

        StopId origin;
        StopId destination;
        int length;
        Set<StopEventId> targets;
        bool isProper;
    };

    struct Shortcut {
        Shortcut(const ShortcutInfo& info) :
            origin(info.origin),
            destination(info.destination),
            travelTime(info.length) {
        }

        StopId origin;
        StopId destination;
        int travelTime;
    };

    struct ArrivalLabel : public ExternalKHeapElement {
        ArrivalLabel() : arrivalTime(never) {}
        int arrivalTime;
        inline bool hasSmallerKey(const ArrivalLabel* const other) const noexcept {
            return arrivalTime < other->arrivalTime;
        }
    };

    struct OneTripLabel : public ExternalKHeapElement {
        OneTripLabel(const int arrivalTime = never, const int walkingDistance = INFTY, const StopId shortcutOrigin = noStop, const size_t timestamp = -1) :
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
        StopId shortcutOrigin; //Only valid for candidates
        size_t timestamp;

        template<typename LABEL>
        inline bool dominates(const LABEL& other) const noexcept {
            return arrivalTime <= other.arrivalTime && walkingDistance <= other.walkingDistance;
        }

        inline bool dominatesStrongly(const OneTripLabel& other) const noexcept {
            return (arrivalTime < other.arrivalTime && walkingDistance <= other.walkingDistance) || (arrivalTime <= other.arrivalTime && walkingDistance < other.walkingDistance);
        }

        inline bool isCandidate() const noexcept {
            return shortcutOrigin != noStop;
        }

        inline void witnessify() noexcept {
            shortcutOrigin = noStop;
        }

        inline int getKey() const noexcept {
            if constexpr (UseArrivalKey) {
                return arrivalTime;
            } else {
                return arrivalTime + walkingDistance;
            }
        }

        inline KeyType getQueueKey() const noexcept {
            if constexpr (UseArrivalKey) {
                return std::make_tuple(arrivalTime, walkingDistance);
            } else {
                return std::make_tuple(getKey(), arrivalTime, walkingDistance);
            }
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

        template<typename LABEL>
        inline bool dominates(const LABEL& other) const noexcept {
            return arrivalTime <= other.arrivalTime && walkingDistance <= other.walkingDistance;
        }

        inline bool dominatesStrongly(const TwoTripsLabel& other) const noexcept {
            return (arrivalTime < other.arrivalTime && walkingDistance <= other.walkingDistance) || (arrivalTime <= other.arrivalTime && walkingDistance < other.walkingDistance);
        }

        inline bool isCandidate() const noexcept {
            return finalStopEvent != noStopEvent;
        }

        inline void witnessify() noexcept {
            finalStopEvent = noStopEvent;
        }

        inline int getKey() const noexcept {
            if constexpr (UseArrivalKey) {
                return arrivalTime;
            } else {
                return arrivalTime + walkingDistance;
            }
        }

        inline KeyType getQueueKey() const noexcept {
            if constexpr (UseArrivalKey) {
                return std::make_tuple(arrivalTime, walkingDistance);
            } else {
                return std::make_tuple(getKey(), arrivalTime, walkingDistance);
            }
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
        const StopEvent* trip;
        int walkingDistance;
        StopId shortcutOrigin;
        StopIndex parentStopIndex;
        size_t timestamp;

        inline const StopEvent* parentStopEvent() const noexcept {
            return trip + parentStopIndex;
        }

        inline bool dominates(const RouteLabel& other) const noexcept {
            return trip <= other.trip && walkingDistance <= other.walkingDistance;
        }

        inline bool isCandidate() const noexcept {
            return shortcutOrigin != noStop;
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
        RouteSegment route;
        int departureTime;
        inline bool operator<(const DepartureLabel& other) const noexcept {
            return (departureTime > other.departureTime) || ((departureTime == other.departureTime) && (route.routeId < other.route.routeId));
        }
    };

    struct ConsolidatedDepartureLabel {
        ConsolidatedDepartureLabel(const int departureTime = never) : departureTime(departureTime) {}
        std::vector<RouteSegment> routes;
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
    McShortcutSearch(const Data& data, DynamicTransferGraph& shortcutGraph, const int intermediateWitnessTransferLimit, const int finalWitnessTransferLimit) :
        data(data),
        shortcutGraph(shortcutGraph),
        stationOfStop(data.numberOfStops()),
        sourceStation(),
        sourceDepartureTime(0),
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
            for (const Shortcut& shortcut : shortcuts) {
                if (!shortcutGraph.hasEdge(shortcut.origin, shortcut.destination)) {
                    shortcutGraph.addEdge(shortcut.origin, shortcut.destination).set(TravelTime, shortcut.travelTime);
                } else {
                    AssertMsg(shortcutGraph.get(TravelTime, shortcutGraph.findEdge(shortcut.origin, shortcut.destination)) == shortcut.travelTime, "Edge from " << shortcut.origin << " to " << shortcut.destination << " has inconclusive travel time (" << shortcutGraph.get(TravelTime, shortcutGraph.findEdge(shortcut.origin, shortcut.destination)) << ", " << shortcut.travelTime << ")");
                }
            }
        }
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
        properDestinationCandidates = 0;
        shortcuts.clear();

        routesServingUpdatedStops.clear();
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();

        sourceDepartureTime = label.departureTime;
        for (const StopId stop : stopsReachedByDirectTransfer) {
            stopsUpdatedByTransfer.insert(stop);
        }
        collectRoutes1(label.routes);
        if constexpr (FullRouteScans) {
            scanRoutes1Full();
        } else {
            scanRoutes1();
        }
        intermediateDijkstra();
        collectRoutes2();
        scanRoutes2();
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
                for (const StopEvent* trip = data.firstTripOfRoute(route); trip <= data.lastTripOfRoute(route); trip += tripSize) {
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

        oneTripQueue.clear();
        std::vector<OneTripBag>(data.transferGraph.numVertices()).swap(oneTripBags);

        twoTripsQueue.clear();
        std::vector<TwoTripsBag>(data.transferGraph.numVertices()).swap(twoTripsBags);

        std::vector<StopEventId>(data.numberOfStopEvents(), noStopEvent).swap(twoTripsRouteParent);

        shortcutCandidatesInQueue = 0;
        shortcutDestinationCandidates.clear();
        properDestinationCandidates = 0;
        shortcuts.clear();

        routesServingUpdatedStops.clear();
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
    }

    inline void collectRoutes1(const std::vector<RouteSegment>& routes) noexcept {
        for (const RouteSegment& route : routes) {
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

            const StopEvent* firstTrip = data.firstTripOfRoute(route);
            const StopEvent* lastTrip = data.lastTripOfRoute(route);

            RouteBag routeBag;

            while (stopIndex < tripSize - 1) {
                if (stopsUpdatedByTransfer.contains(stop)) {
                    const int walkingDistance = directTransferArrivalLabels[stop].arrivalTime;
                    const int arrivalTime = initialArrivalTime(stop);
                    const StopEvent* trip = firstTrip;
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
                    const OneTripLabel newLabel(label.trip[stopIndex].arrivalTime, label.walkingDistance, isCandidate ? stop : noStop, timestamp);
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
            TripIterator tripIterator = data.getTripIterator(route, stopIndex);
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
                const int walkingDistance = directTransferArrivalLabels[parent].arrivalTime;
                const bool isCandidate = stationOfStop[parent].representative == sourceStation.representative;
                const OneTripLabel label(tripIterator.arrivalTime(), walkingDistance, isCandidate ? tripIterator.stop() : noStop, timestamp);
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

            const StopEvent* firstTrip = data.firstTripOfRoute(route);
            const StopEvent* lastTrip = data.lastTripOfRoute(route);

            RouteBag routeBag;

            while (stopIndex < tripSize - 1) {
                for (const OneTripLabel& label : oneTripBags[stop]) {
                    if (label.timestamp != timestamp) continue;
                    const StopEvent* trip = firstTrip;
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
                    arrivalByRoute2(stop, newLabel, label, stops[label.parentStopIndex], finalStopEvent);
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

    inline void intermediateDijkstra() noexcept {
        AssertMsg(stopsUpdatedByTransfer.empty(), "stopsUpdatedByTransfer is not empty!");

        for (const StopId stop : stopsUpdatedByRoute) {
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

    inline void finalDijkstra() noexcept {
        AssertMsg(stopsUpdatedByTransfer.empty(), "stopsUpdatedByTransfer is not empty!");

        for (const StopId stop : stopsUpdatedByRoute) {
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
                    shortcuts.emplace_back(info);
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
            shortcuts.emplace_back(info);
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

    inline void arrivalByRoute2(const StopId stop, TwoTripsLabel& label, const RouteLabel& routeLabel, const StopId parentStop, const StopEventId finalStopEvent) noexcept {
        const bool isCandidate = routeLabel.isCandidate() && (routeLabel.shortcutOrigin != parentStop) && !shortcutGraph.hasEdge(routeLabel.shortcutOrigin, parentStop);
        if (isCandidate) {
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
        if (isCandidate) {
            const StopEventId shortcutDestination = StopEventId(routeLabel.parentStopEvent() - &(data.stopEvents[0]));
            twoTripsRouteParent[finalStopEvent] = shortcutDestination;
            if (!shortcutDestinationCandidates.contains(shortcutDestination)) {
                shortcutDestinationCandidates.insert(shortcutDestination, ShortcutInfo(routeLabel.shortcutOrigin, parentStop, label.walkingDistance, finalStopEvent, isProper));
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

    inline int initialArrivalTime(const Vertex vertex) const noexcept {
        return directTransferArrivalLabels[vertex].arrivalTime + sourceDepartureTime;
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
        const int walkingDistance = directTransferArrivalLabels[vertex].arrivalTime;
        return OneTripLabel(walkingDistance == INFTY ? INFTY : sourceDepartureTime + walkingDistance, walkingDistance);
    }

private:
    const Data& data;
    DynamicTransferGraph& shortcutGraph;
    std::vector<Station> stationOfStop;

    Station sourceStation;
    int sourceDepartureTime;

    std::vector<ArrivalLabel> directTransferArrivalLabels;
    ExternalKHeap<2, ArrivalLabel> directTransferQueue;
    std::vector<StopId> stopsReachedByDirectTransfer;

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
