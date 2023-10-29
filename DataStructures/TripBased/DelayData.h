#pragma once

#include "Data.h"
#include "Delay.h"
#include "DelayUpdateData.h"
#include "Shortcut.h"

#include "../../Algorithms/RAPTOR/InitialTransfers.h"

namespace TripBased {

class DelayData {

public:
    DelayData(const RAPTOR::Data& raptorData, const TransferGraph& shortcuts, const int maxArrivalDelay, const int maxDepartureDelay) :
        DelayData(raptorData, maxArrivalDelay, maxDepartureDelay) {
        Graph::copy(shortcuts, stopEventGraph);
        for (const Edge edge : stopEventGraph.edges()) {
            stopEventGraph.set(MinOriginDelay, edge, 0);
            stopEventGraph.set(MaxOriginDelay, edge, maxArrivalDelay);
        }
    }

    DelayData(const TripBased::Data& data) :
        data(data),
        maxArrivalDelay(0),
        maxDepartureDelay(0),
        arrivalDelay(data.numberOfStopEvents(), 0),
        departureDelay(data.numberOfStopEvents(), 0) {
        Graph::copy(data.stopEventGraph, stopEventGraph);
        for (const Edge edge : stopEventGraph.edges()) {
            stopEventGraph.set(MinOriginDelay, edge, 0);
            stopEventGraph.set(MaxOriginDelay, edge, INFTY);
        }
    }

    DelayData(const RAPTOR::Data& raptorData, const int maxArrivalDelay, const int maxDepartureDelay) :
        data(raptorData),
        maxArrivalDelay(maxArrivalDelay),
        maxDepartureDelay(maxDepartureDelay),
        arrivalDelay(data.numberOfStopEvents(), 0),
        departureDelay(data.numberOfStopEvents(), 0) {
    }

    DelayData(const std::string& fileName) {
        deserialize(fileName);
        arrivalDelay.resize(data.numberOfStopEvents(), 0);
        departureDelay.resize(data.numberOfStopEvents(), 0);
    }

    inline void serialize(const std::string& fileName) const noexcept {
        data.serialize(fileName + ".tripBased");
        stopEventGraph.writeBinary(fileName + ".delayGraph");
        IO::serialize(fileName, maxArrivalDelay, maxDepartureDelay);
    }

    inline void deserialize(const std::string& fileName) noexcept {
        data.deserialize(fileName + ".tripBased");
        stopEventGraph.readBinary(fileName + ".delayGraph");
        IO::deserialize(fileName, maxArrivalDelay, maxDepartureDelay);
    }

    inline int delayedArrivalTime(const StopEventId stopEvent) const noexcept {
        return data.arrivalTime(stopEvent) + arrivalDelay[stopEvent];
    }

    inline int delayedDepartureTime(const StopEventId stopEvent) const noexcept {
        return data.departureTime(stopEvent) + departureDelay[stopEvent];
    }

    inline int getUndelayedTransferSlack(const StopEventId from, const StopEventId to, const int travelTime) const noexcept {
        return data.getTransferSlack(from, to, travelTime);
    }

    inline int getTransferSlack(const StopEventId from, const StopEventId to, const int travelTime) const noexcept {
        return delayedDepartureTime(to) - delayedArrivalTime(from) - travelTime;
    }

    inline bool isTransferFeasible(const StopEventId from, const StopEventId to, const int travelTime) const noexcept {
        return getTransferSlack(from, to, travelTime) >= 0;
    }

    inline void printInfo() noexcept {
        size_t infeasibleEdges = 0;
        size_t minOriginDelayEdges = 0;
        size_t maxOriginDelayEdges = 0;
        for (const Vertex from : stopEventGraph.vertices()) {
            for (const Edge edge : stopEventGraph.edgesFrom(from)) {
                const Vertex to = stopEventGraph.get(ToVertex, edge);
                if (!isTransferFeasible(StopEventId(from), StopEventId(to), stopEventGraph.get(TravelTime, edge))) infeasibleEdges++;
                else if (stopEventGraph.get(MinOriginDelay, edge) > arrivalDelay[from]) minOriginDelayEdges++;
                else if (stopEventGraph.get(MaxOriginDelay, edge) < arrivalDelay[from]) maxOriginDelayEdges++;
            }
        }
        data.printInfo();
        std::cout << "   Number of Shortcuts:        " << std::setw(12) << String::prettyInt(stopEventGraph.numEdges()) << std::endl;
        std::cout << "   Infeasible Shortcuts:       " << std::setw(12) << String::prettyInt(infeasibleEdges) << std::endl;
        std::cout << "   Min Origin Delay Shortcuts: " << std::setw(12) << String::prettyInt(minOriginDelayEdges) << std::endl;
        std::cout << "   Max Origin Delay Shortcuts: " << std::setw(12) << String::prettyInt(maxOriginDelayEdges) << std::endl;
    }

    inline DelayUpdateData createUpdateData(const RAPTOR::BucketCHInitialTransfers& initialTransfers) const noexcept {
        DelayUpdateData result(initialTransfers);
        refreshUpdateData(result);
        return result;
    }

    inline void refreshUpdateData(DelayUpdateData& updateData) const noexcept {
        updateData.update(getDelayedRaptorData(), filterShortcutGraph());
    }

    inline DelayQueryData createQueryData() const noexcept {
        DelayQueryData result;
        refreshQueryData(result);
        return result;
    }

    inline void refreshQueryData(DelayQueryData& queryData) const noexcept {
        queryData.update(getDelayedRaptorData(), filterShortcutGraph());
    }

    inline void delayTripFromEvent(const TripId trip, const StopIndex index, const int delay, const bool allowOvertaking, const bool ignoreMaxDelay) noexcept {
        const StopEventId startingEvent = StopEventId(data.firstStopEventOfTrip[trip] + index);
        const bool hasNextTrip = (data.routeOfTrip[trip] == data.routeOfTrip[trip + 1]);
        for (StopEventId currentEvent = startingEvent; currentEvent < data.firstStopEventOfTrip[trip + 1]; currentEvent++) {
            arrivalDelay[currentEvent] += delay;
            departureDelay[currentEvent] += delay;
            if (!ignoreMaxDelay) {
                arrivalDelay[currentEvent] = std::min(maxArrivalDelay, arrivalDelay[currentEvent]);
                departureDelay[currentEvent] = std::min(maxDepartureDelay, departureDelay[currentEvent]);
            }
            if (!allowOvertaking && hasNextTrip) {
                const StopEventId nextEvent = StopEventId(currentEvent + data.numberOfStopsInTrip(trip));
                arrivalDelay[currentEvent] = std::min(arrivalDelay[currentEvent], delayedArrivalTime(nextEvent) - data.arrivalTime(currentEvent));
                departureDelay[currentEvent] = std::min(departureDelay[currentEvent], delayedDepartureTime(nextEvent) - data.departureTime(currentEvent));
            }
        }
        const StopEventId lastEvent(data.firstStopEventOfTrip[trip + 1] - 1);
        const int realDepartureTime = data.departureTime(lastEvent) + data.raptorData.minTransferTime(data.getStopOfStopEvent(lastEvent));
        arrivalDelay[lastEvent] = std::min(arrivalDelay[lastEvent], realDepartureTime + departureDelay[lastEvent] - data.arrivalTime(lastEvent));
        for (StopEventId nextEvent = lastEvent; nextEvent != startingEvent; nextEvent--) {
            const StopEventId currentEvent = StopEventId(nextEvent - 1);
            const int realDepartureTime = data.departureTime(currentEvent) + data.raptorData.minTransferTime(data.getStopOfStopEvent(currentEvent));
            departureDelay[currentEvent] = std::min(departureDelay[currentEvent], delayedArrivalTime(nextEvent) - realDepartureTime);
            arrivalDelay[currentEvent] = std::min(arrivalDelay[currentEvent], realDepartureTime + departureDelay[currentEvent] - data.arrivalTime(currentEvent));
        }
    }

    inline void applyDelays(const std::vector<DelayIncident>& incidents, const bool ignoreMaxDelay, const bool allowDepartureDelays, const bool allowOvertaking = true) noexcept {
        for (const DelayIncident& incident : incidents) {
            delayTripFromEvent(incident.trip, incident.startIndex, incident.delay, allowOvertaking, ignoreMaxDelay);
        }
        if (!ignoreMaxDelay) validateDelays();
        if (!allowDepartureDelays) clearDepartureDelays();
    }

    inline void applyDelayUpdates(const std::vector<DelayUpdate>& updates, const bool allowDepartureDelays) noexcept {
        for (const DelayUpdate& update : updates) {
            StopEventId event = update.firstEvent;
            for (size_t i = 0; i < update.arrivalDelay.size(); i++, event++) {
                arrivalDelay[event] = update.arrivalDelay[i];
                if (allowDepartureDelays) departureDelay[event] = update.departureDelay[i];
            }
        }
    }

    inline DelayQueryData applyDelayScenario(const std::vector<TripBased::DelayIncident>& incidents, const bool ignoreMaxDelay, const bool allowDepartureDelays, const bool allowOvertaking = true) noexcept {
        applyDelays(incidents, ignoreMaxDelay, allowDepartureDelays, allowOvertaking);
        DelayQueryData queryData;
        refreshQueryData(queryData);
        return queryData;
    }

    inline void clearDepartureDelays() noexcept {
        Vector::fill(departureDelay, 0);
    }

    inline void validateDelays() noexcept {
        for (size_t i = 0; i < arrivalDelay.size(); i++) {
            Ensure(arrivalDelay[i] >= 0, "Arrival delay of stop event " << i << " is negative!");
            Ensure(arrivalDelay[i] <= maxArrivalDelay, "Arrival delay of stop event " << i << " exceeds max delay!");
            Ensure(departureDelay[i] >= 0, "Departure delay of stop event " << i << " is negative!");
            Ensure(departureDelay[i] <= maxDepartureDelay, "Departure delay of stop event " << i << " exceeds max delay!");
        }
    }

private:
    inline RAPTOR::Data getDelayedRaptorData() const noexcept {
        RAPTOR::Data delayedData = data.raptorData;
        for (StopEventId event(0); event < delayedData.numberOfStopEvents(); event++) {
            delayedData.stopEvents[event].arrivalTime += arrivalDelay[event];
            delayedData.stopEvents[event].departureTime += departureDelay[event];
        }
        return delayedData;
    }

    inline std::pair<TransferGraph, TransferGraph> filterShortcutGraph() const noexcept {
        TransferGraph result;
        TransferGraph infeasibleShortcuts;
        result.reserve(stopEventGraph.numVertices(), stopEventGraph.numEdges());
        infeasibleShortcuts.reserve(stopEventGraph.numVertices(), stopEventGraph.numEdges());
        for (const Vertex fromStopEvent : stopEventGraph.vertices()) {
            result.addVertex();
            infeasibleShortcuts.addVertex();
            for (const Edge shortcut : stopEventGraph.edgesFrom(fromStopEvent)) {
                if (stopEventGraph.get(MinOriginDelay, shortcut) > arrivalDelay[fromStopEvent]) continue;
                const Vertex toStopEvent = stopEventGraph.get(ToVertex, shortcut);
                const int travelTime = stopEventGraph.get(TravelTime, shortcut);
                if (!isTransferFeasible(StopEventId(fromStopEvent), StopEventId(toStopEvent), travelTime)) {
                    infeasibleShortcuts.addEdge(fromStopEvent, toStopEvent, stopEventGraph.edgeRecord(shortcut));
                } else {
                    const int maxOriginDelay = stopEventGraph.get(MaxOriginDelay, shortcut);
                    if (maxOriginDelay < maxArrivalDelay && maxOriginDelay < arrivalDelay[fromStopEvent]) continue;
                    result.addEdge(fromStopEvent, toStopEvent, stopEventGraph.edgeRecord(shortcut));
                }
            }
        }
        return std::make_pair(result, infeasibleShortcuts);
    }

public:

    Data data;
    DelayGraph stopEventGraph;
    int maxArrivalDelay;
    int maxDepartureDelay;
    std::vector<int> arrivalDelay;
    std::vector<int> departureDelay;
};

}
