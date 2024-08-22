#pragma once

#include "../../../DataStructures/TripBased/Data.h"
#include "../../../DataStructures/TripBased/DelayUpdateData.h"
#include "../../../DataStructures/TripBased/Shortcut.h"
#include "../../RAPTOR/InitialTransfers.h"

namespace TripBased {

class ReplacementShortcutSearch {

public:
    struct ShortcutData {
        StopEventId destination;
        int travelTime;
    };

    struct ShortcutSet {
        ShortcutSet(const size_t capacity) :
            data(capacity) {
        }

        inline void insert(const StopEventId origin, const ShortcutData& shortcut) noexcept {
            if (!data.contains(origin)) {
                data.insert(origin);
            }
            for (const ShortcutData& other : data[origin]) {
                if (shortcut.destination == other.destination) return;
            }
            data[origin].emplace_back(shortcut);
        }

        inline void clear() noexcept {
            data.clear();
        }

        inline const std::vector<StopEventId>& getKeys() const noexcept {
            return data.getKeys();
        }

        inline const std::vector<ShortcutData>& operator[](const StopEventId origin) const noexcept {
            return data[origin];
        }

        IndexedMap<std::vector<ShortcutData>, false, StopEventId> data;
    };


    struct BackwardDijkstraLabel : public ExternalKHeapElement {
        BackwardDijkstraLabel() : arrivalTime(never) {}
        int arrivalTime;
        inline bool hasSmallerKey(const BackwardDijkstraLabel* const other) const noexcept {
            return arrivalTime < other->arrivalTime || (arrivalTime == other->arrivalTime && this < other);
        }
    };

    struct ForwardDijkstraLabel : public ExternalKHeapElement {
        ForwardDijkstraLabel() : arrivalTime(never), exitEvent(noStopEvent), timestamp(0) {}
        int arrivalTime;
        StopEventId exitEvent;
        int timestamp;

        inline void check(const int newTimestamp) noexcept {
            if (timestamp == newTimestamp) return;
            arrivalTime = never;
            exitEvent = noStopEvent;
            timestamp = newTimestamp;
        }

        inline bool hasSmallerKey(const ForwardDijkstraLabel* const other) const noexcept {
            return arrivalTime < other->arrivalTime || (arrivalTime == other->arrivalTime && this < other);
        }
    };

    struct EarliestArrivalLabel {
        EarliestArrivalLabel() : arrivalTime(never), enterEvent(noStopEvent), exitEvent(noStopEvent) {}
        int arrivalTime;
        StopEventId enterEvent;
        StopEventId exitEvent;
    };
    using Round = std::vector<EarliestArrivalLabel>;

    ReplacementShortcutSearch(const DelayUpdateData& data, const int targetSlack) :
        data(data),
        initialTransfers(data.initialTransfers),
        targetSlack(targetSlack),
        sourceEvents(data.raptorData.numberOfStopEvents()),
        earliestSourceDepartureTime(INFTY),
        targetStops(data.raptorData.numberOfStops()),
        targetArrivalTime(data.raptorData.numberOfStops(), INFTY),
        sourceEvent(noStopEvent),
        sourceDepartureTime(INFTY),
        stopsUpdatedByRoute(data.raptorData.numberOfStops()),
        stopsUpdatedByTransfer(data.raptorData.numberOfStops()),
        routesServingUpdatedStops(data.raptorData.numberOfRoutes()),
        earliestArrival(data.raptorData.numberOfStops(), INFTY),
        label(data.raptorData.transferGraph.numVertices()),
        shortcuts(data.tripData.numberOfStopEvents()),
        timestamp(0) {
    }

    inline void resetData() noexcept {
        routesServingUpdatedStops.clear();
        routesServingUpdatedStops.resize(data.raptorData.numberOfRoutes());
        shortcuts.clear();
    }

    inline void runForDelayedOrigin(const StopEventId originEvent) noexcept {
        clear();
        collectEventsForDelayedOrigin(originEvent);
        run();
    }

    inline void runForDelayedOrigins(const std::vector<StopEventId>& originEvents) noexcept {
        clear();
        for (const StopEventId originEvent : originEvents) {
            collectEventsForDelayedOrigin(originEvent);
        }
        run();
    }

    inline void runForDelayedTarget(const StopEventId targetEvent) noexcept {
        clear();
        collectEventsForDelayedTarget(targetEvent);
        run();
    }

    inline void runForDelayedTargets(const std::vector<StopEventId>& targetEvents) noexcept {
        clear();
        for (const StopEventId targetEvent : targetEvents) {
            collectEventsForDelayedTarget(targetEvent);
        }
        run();
    }

    inline const std::vector<StopEventId>& getShortcutOrigins() noexcept {
        return shortcuts.getKeys();
    }

    inline const std::vector<ShortcutData>& getShortcutsFrom(const StopEventId origin) noexcept {
        return shortcuts[origin];
    }

private:
    inline void clear() noexcept {
        sourceEvents.clear();
        earliestSourceDepartureTime = INFTY;
        targetStops.clear();
        Vector::fill(targetArrivalTime, INFTY);
    }

    inline void collectEventsForDelayedTarget(const StopEventId targetEvent) noexcept {
        const StopId targetStop = data.tripData.getStopOfStopEvent(targetEvent);
        targetStops.insert(targetStop);
        targetArrivalTime[targetStop] = data.tripData.arrivalTime(targetEvent);

        const TripId trip2 = data.tripData.tripOfStopEvent[targetEvent];
        const StopIndex targetIndex = data.tripData.indexOfStopEvent[targetEvent];
        StopEventId destinationEvent = data.tripData.firstStopEventOfTrip[trip2];
        for (StopIndex i(0); i < targetIndex; i++, destinationEvent++) {
            for (const Edge shortcut : data.reverseStopEventGraph.edgesFrom(Vertex(destinationEvent))) {
                const StopEventId originEvent(data.reverseStopEventGraph.get(ToVertex, shortcut));
                const int travelTime = data.reverseStopEventGraph.get(TravelTime, shortcut);
                //If shortcut is infeasible, we already handle it with the delayed origin search
                Assert(data.tripData.arrivalTime(originEvent) + travelTime <= data.tripData.departureTime(destinationEvent), "Shortcut is infeasible!");
                const TripId trip1 = data.tripData.tripOfStopEvent[originEvent];
                const StopIndex originIndex = data.tripData.indexOfStopEvent[originEvent];
                StopEventId sourceEvent = data.tripData.firstStopEventOfTrip[trip1];
                for (StopIndex j(0); j < originIndex; j++, sourceEvent++) {
                    sourceEvents.insert(sourceEvent);
                    earliestSourceDepartureTime = std::min(earliestSourceDepartureTime, data.tripData.departureTime(sourceEvent));
                }
            }
        }
    }

    inline void collectEventsForDelayedOrigin(const StopEventId originEvent) noexcept {
        const TripId trip1 = data.tripData.tripOfStopEvent[originEvent];
        const StopIndex originIndex = data.tripData.indexOfStopEvent[originEvent];
        StopEventId sourceEvent = data.tripData.firstStopEventOfTrip[trip1];
        for (StopIndex i(0); i < originIndex; i++, sourceEvent++) {
            sourceEvents.insert(sourceEvent);
            earliestSourceDepartureTime = std::min(earliestSourceDepartureTime, data.tripData.departureTime(sourceEvent));
        }

        for (const Edge shortcut : data.infeasibleShortcuts.edgesFrom(Vertex(originEvent))) {
            const StopEventId destinationEvent(data.infeasibleShortcuts.get(ToVertex, shortcut));
            const int travelTime = data.infeasibleShortcuts.get(TravelTime, shortcut);
            Assert(data.tripData.arrivalTime(originEvent) + travelTime > data.tripData.departureTime(destinationEvent), "Shortcut is not infeasible!");
            const TripId trip2 = data.tripData.tripOfStopEvent[destinationEvent];
            const RouteId route2 = data.tripData.routeOfTrip[trip2];
            const StopIndex index2 = data.tripData.indexOfStopEvent[destinationEvent];
            std::vector<TripId> replacementTrips;
            for (const RouteId route : data.routeGrouping.getRoutesWithSameSequence(route2)) {
                for (const TripId trip : data.tripData.tripsOfRoute(route)) {
                    const StopEventId enterEvent = data.tripData.getStopEventId(trip, index2);
                    if (data.tripData.arrivalTime(originEvent) + travelTime > data.tripData.departureTime(enterEvent)) continue;
                    replacementTrips.emplace_back(trip);
                    break;
                }
            }
            const StopIndex destinationIndex = data.tripData.indexOfStopEvent[destinationEvent];
            StopEventId targetEvent(destinationEvent + 1);
            for (StopIndex i(destinationIndex + 1); i < data.tripData.numberOfStopsInTrip(trip2); i++, targetEvent++) {
                const StopId targetStop = data.tripData.getStopOfStopEvent(targetEvent);
                targetStops.insert(targetStop);
                targetArrivalTime[targetStop] = std::min(targetArrivalTime[targetStop], data.tripData.arrivalTime(targetEvent) + targetSlack);
                for (const TripId trip : replacementTrips) {
                    const StopEventId exitEvent = data.tripData.getStopEventId(trip, i);
                    targetArrivalTime[targetStop] = std::min(targetArrivalTime[targetStop], data.tripData.arrivalTime(exitEvent));
                }
            }
        }

        if (targetStops.empty()) return;
        const StopId originStop = data.tripData.getStopOfStopEvent(originEvent);
        initialTransfers.template run<FORWARD, BACKWARD, false>(originStop);
        for (size_t i = 0; i < targetStops.size(); i++) {
            const int originTransferTime = data.tripData.arrivalTime(originEvent) + initialTransfers.getForwardDistance(targetStops[i]);
            targetArrivalTime[targetStops[i]] = std::min(targetArrivalTime[targetStops[i]], originTransferTime);
        }
    }

    inline void run() noexcept {
        if (sourceEvents.empty() || targetStops.empty()) return;
        runBackwardSearch();
        for (const StopEventId event : sourceEvents) {
            runForwardSearch(event);
        }
    }

    inline void runBackwardSearch() noexcept {
        clearBackwardSearch();
        startNewBackwardRound();
        initializeTargets();
        relaxBackwardTransfers();
        startNewBackwardRound();
        collectBackwardRoutes();
        scanBackwardRoutes();
        relaxBackwardTransfers();
        startNewBackwardRound();
        collectBackwardRoutes();
        scanBackwardRoutes();
        relaxBackwardTransfers();
    }

    inline void clearBackwardSearch() noexcept {
        backwardRounds.clear();
        backwardQueue.clear();
    }

    inline std::vector<BackwardDijkstraLabel>& currentBackwardRound() noexcept {
        Assert(!backwardRounds.empty(), "Cannot return current round, because no round exists!");
        return backwardRounds.back();
    }

    inline std::vector<BackwardDijkstraLabel>& previousBackwardRound() noexcept {
        Assert(backwardRounds.size() >= 2, "Cannot return previous round, because less than two rounds exist!");
        return backwardRounds[backwardRounds.size() - 2];
    }

    inline void startNewBackwardRound() noexcept {
        if (backwardRounds.empty()) {
            backwardRounds.emplace_back(data.raptorData.transferGraph.numVertices());
        } else {
            backwardRounds.emplace_back(backwardRounds.back());
        }
    }

    inline void initializeTargets() noexcept {
        stopsUpdatedByRoute.clear();
        for (size_t i = 0; i < targetStops.size(); i++) {
            backwardArrivalByRoute(targetStops[i], -targetArrivalTime[targetStops[i]]);
        }
    }

    inline void collectBackwardRoutes() noexcept {
        routesServingUpdatedStops.clear();
        for (const StopId stop : stopsUpdatedByTransfer) {
            Assert(data.reverseRaptorData.isStop(stop), "Stop " << stop << " is out of range!");
            const int arrivalTime = previousBackwardRound()[stop].arrivalTime;
            Assert(arrivalTime < never, "Updated stop has arrival time = never!");
            for (const RAPTOR::RouteSegment& route : data.reverseRaptorData.routesContainingStop(stop)) {
                if (route.stopIndex + 1 == data.reverseRaptorData.numberOfStopsInRoute(route.routeId)) continue;
                if (data.reverseRaptorData.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < arrivalTime) continue;
                if (routesServingUpdatedStops.contains(route.routeId)) {
                    routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                }
            }
        }
    }

    inline void scanBackwardRoutes() noexcept {
        stopsUpdatedByRoute.clear();
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            const StopIndex stopIndex = routesServingUpdatedStops[route];
            RAPTOR::TripIterator tripIterator = data.reverseRaptorData.getTripIterator(route, stopIndex);
            while (tripIterator.hasFurtherStops()) {
                while (tripIterator.hasEarlierTrip() && tripIterator.previousDepartureTime() >= previousBackwardRound()[tripIterator.stop()].arrivalTime) {
                    tripIterator.previousTrip();
                }
                tripIterator.nextStop();
                backwardArrivalByRoute(tripIterator.stop(), tripIterator.arrivalTime());
            }
        }
    }

    inline void relaxBackwardTransfers() noexcept {
        stopsUpdatedByTransfer.clear();
        for (const StopId stop : stopsUpdatedByRoute) {
            backwardQueue.update(&(currentBackwardRound()[stop]));
            stopsUpdatedByTransfer.insert(stop);
        }
        while (!backwardQueue.empty()) {
            BackwardDijkstraLabel* uLabel = backwardQueue.extractFront();
            const Vertex u = Vertex(uLabel - &(currentBackwardRound()[0]));
            for (const Edge edge : data.reverseRaptorData.transferGraph.edgesFrom(u)) {
                const Vertex v = data.reverseRaptorData.transferGraph.get(ToVertex, edge);
                const int arrivalTime = uLabel->arrivalTime + data.reverseRaptorData.transferGraph.get(TravelTime, edge);
                backwardArrivalByTransfer(v, arrivalTime);
            }
        }
    }

    inline void backwardArrivalByRoute(const StopId stop, const int arrivalTime) noexcept {
        if (-arrivalTime < earliestSourceDepartureTime) return;
        if (currentBackwardRound()[stop].arrivalTime <= arrivalTime) return;
        currentBackwardRound()[stop].arrivalTime = arrivalTime;
        stopsUpdatedByRoute.insert(stop);
    }

    inline void backwardArrivalByTransfer(const Vertex vertex, const int arrivalTime) noexcept {
        if (-arrivalTime < earliestSourceDepartureTime) return;
        if (currentBackwardRound()[vertex].arrivalTime <= arrivalTime) return;
        currentBackwardRound()[vertex].arrivalTime = arrivalTime;
        backwardQueue.update(&(currentBackwardRound()[vertex]));
        if (data.raptorData.isStop(vertex)) stopsUpdatedByTransfer.insert(StopId(vertex));
    }

    inline int getLatestDepartureTime(const Vertex vertex) const noexcept {
        Assert(rounds.size() <= backwardRounds.size(), "Too many rounds!");
        return -backwardRounds[backwardRounds.size() - rounds.size()][vertex].arrivalTime;
    }

    inline void runForwardSearch(const StopEventId event) noexcept {
        sourceEvent = event;
        sourceStop = data.tripData.getStopOfStopEvent(sourceEvent);
        sourceDepartureTime = data.tripData.departureTime(sourceEvent);

        clearForwardSearch();
        startNewRound();
        initializeSource();
        relaxTransfers();
        startNewRound();
        collectRoutes();
        scanRoutes();
        relaxTransfers();
        startNewRound();
        collectRoutes();
        scanRoutes();
        relaxTransfers();
        for (const StopId targetStop : targetStops) {
            unpackShortcut(targetStop);
        }
    }

    inline void clearForwardSearch() noexcept {
        rounds.clear();
        Vector::fill(earliestArrival, INFTY);
        timestamp++;
        queue.clear();
    }

    inline Round& currentRound() noexcept {
        Assert(!rounds.empty(), "Cannot return current round, because no round exists!");
        return rounds.back();
    }

    inline Round& previousRound() noexcept {
        Assert(rounds.size() >= 2, "Cannot return previous round, because less than two rounds exist!");
        return rounds[rounds.size() - 2];
    }

    inline void startNewRound() noexcept {
        rounds.emplace_back(data.raptorData.numberOfStops());
    }

    inline void initializeSource() noexcept {
        stopsUpdatedByRoute.clear();
        arrivalByRoute(sourceStop, sourceDepartureTime, noStopEvent, noStopEvent);
    }

    inline void collectRoutes() noexcept {
        routesServingUpdatedStops.clear();
        for (const StopId stop : stopsUpdatedByTransfer) {
            Assert(data.raptorData.isStop(stop), "Stop " << stop << " is out of range!");
            const int arrivalTime = previousRound()[stop].arrivalTime;
            Assert(arrivalTime < never, "Updated stop has arrival time = never!");
            for (const RAPTOR::RouteSegment& route : data.raptorData.routesContainingStop(stop)) {
                if (route.stopIndex + 1 == data.raptorData.numberOfStopsInRoute(route.routeId)) continue;
                if (data.raptorData.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < arrivalTime) continue;
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
            const StopIndex stopIndex = routesServingUpdatedStops[route];
            RAPTOR::TripIterator tripIterator = data.raptorData.getTripIterator(route, stopIndex);
            StopIndex parentIndex = stopIndex;
            while (tripIterator.hasFurtherStops()) {
                while (tripIterator.hasEarlierTrip() && tripIterator.previousDepartureTime() >= previousRound()[tripIterator.stop()].arrivalTime) {
                    tripIterator.previousTrip();
                    parentIndex = tripIterator.getStopIndex();
                }
                tripIterator.nextStop();
                const StopEventId enterEvent(tripIterator.stopEvent(parentIndex) - (&(data.raptorData.stopEvents[0])));
                const StopEventId exitEvent(tripIterator.stopEvent(tripIterator.getStopIndex()) - (&(data.raptorData.stopEvents[0])));
                arrivalByRoute(tripIterator.stop(), tripIterator.arrivalTime(), exitEvent, enterEvent);
            }
        }
    }

    inline void relaxTransfers() noexcept {
        stopsUpdatedByTransfer.clear();
        for (const StopId stop : stopsUpdatedByRoute) {
            arrivalByEdge(stop, currentRound()[stop].arrivalTime, currentRound()[stop].exitEvent);
            stopsUpdatedByTransfer.insert(stop);
        }
        while (!queue.empty()) {
            ForwardDijkstraLabel* uLabel = queue.extractFront();
            const Vertex u = Vertex(uLabel - &(label[0]));
            for (const Edge edge : data.raptorData.transferGraph.edgesFrom(u)) {
                const Vertex v = data.raptorData.transferGraph.get(ToVertex, edge);
                const int arrivalTime = uLabel->arrivalTime + data.raptorData.transferGraph.get(TravelTime, edge);
                arrivalByEdge(v, arrivalTime, uLabel->exitEvent);
            }
            if (data.raptorData.isStop(u)) {
                arrivalByTransfer(StopId(u), uLabel->arrivalTime, uLabel->exitEvent);
            }
        }
    }

    inline void unpackShortcut(const StopId targetStop) noexcept {
        if (earliestArrival[targetStop] == INFTY) return;
        if (rounds[0][targetStop].arrivalTime == earliestArrival[targetStop]) return;
        if (rounds[1][targetStop].arrivalTime == earliestArrival[targetStop]) return;
        const StopEventId targetEvent = rounds[2][targetStop].exitEvent;
        const StopId targetEventStop = data.tripData.getStopOfStopEvent(targetEvent);
        const StopEventId destinationEvent = rounds[2][targetEventStop].enterEvent;
        const StopId destinationStop = data.tripData.getStopOfStopEvent(destinationEvent);
        const StopEventId originEvent = rounds[1][destinationStop].exitEvent;
        const StopId originStop = data.tripData.getStopOfStopEvent(originEvent);
        const int travelTime = rounds[1][destinationStop].arrivalTime - rounds[1][originStop].arrivalTime;
        if (data.tripData.stopEventGraph.hasEdge(Vertex(originEvent), Vertex(destinationEvent))) return;
        shortcuts.insert(originEvent, ShortcutData{destinationEvent, travelTime});
    }

    inline void arrivalByRoute(const StopId stop, const int arrivalTime, const StopEventId exitEvent, const StopEventId enterEvent) noexcept {
        if (arrivalTime > getLatestDepartureTime(stop)) return;
        if (earliestArrival[stop] <= arrivalTime) return;
        earliestArrival[stop] = arrivalTime;
        currentRound()[stop].arrivalTime = arrivalTime;
        currentRound()[stop].enterEvent = enterEvent;
        currentRound()[stop].exitEvent = exitEvent;
        stopsUpdatedByRoute.insert(stop);
    }

    inline void arrivalByEdge(const Vertex vertex, const int arrivalTime, const StopEventId exitEvent) noexcept {
        if (arrivalTime > getLatestDepartureTime(vertex)) return;
        label[vertex].check(timestamp);
        if (label[vertex].arrivalTime <= arrivalTime) return;
        label[vertex].arrivalTime = arrivalTime;
        label[vertex].exitEvent = exitEvent;
        queue.update(&(label[vertex]));
    }

    inline void arrivalByTransfer(const StopId stop, const int arrivalTime, const StopEventId exitEvent) noexcept {
        if (arrivalTime > getLatestDepartureTime(stop)) return;
        if (earliestArrival[stop] <= arrivalTime) return;
        earliestArrival[stop] = arrivalTime;
        currentRound()[stop].arrivalTime = arrivalTime;
        currentRound()[stop].enterEvent = noStopEvent;
        currentRound()[stop].exitEvent = exitEvent;
        stopsUpdatedByTransfer.insert(stop);
    }

private:
    const DelayUpdateData& data;
    RAPTOR::BucketCHInitialTransfers initialTransfers;
    const int targetSlack;

    IndexedSet<false, StopEventId> sourceEvents;
    int earliestSourceDepartureTime;
    IndexedSet<false, StopId> targetStops;
    std::vector<int> targetArrivalTime;

    StopEventId sourceEvent;
    StopId sourceStop;
    int sourceDepartureTime;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    std::vector<std::vector<BackwardDijkstraLabel>> backwardRounds;
    ExternalKHeap<2, BackwardDijkstraLabel> backwardQueue;

    std::vector<Round> rounds;
    std::vector<int> earliestArrival;
    std::vector<ForwardDijkstraLabel> label;
    ExternalKHeap<2, ForwardDijkstraLabel> queue;

    ShortcutSet shortcuts;

    int timestamp;
};

}
