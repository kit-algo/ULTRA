#pragma once

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>

#include "../../Helpers/Assert.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/Types.h"
#include "../../Helpers/Vector/Vector.h"
#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/CSA/Entities/Journey.h"
#include "Profiler.h"

namespace CSA {

template<bool PATH_RETRIEVAL = true, typename PROFILER = NoProfiler>
class CSA {

public:
    constexpr static bool PathRetrieval = PATH_RETRIEVAL;
    using Profiler = PROFILER;
    using Type = CSA<PathRetrieval, Profiler>;
    using TripFlag = Meta::IF<PathRetrieval, ConnectionId, bool>;

private:
    struct ParentLabel {
        ParentLabel(const StopId parent = noStop, const bool reachedByTransfer = false, const TripId tripId = noTripId) :
            parent(parent),
            reachedByTransfer(reachedByTransfer),
            tripId(tripId) {
        }

        StopId parent;
        bool reachedByTransfer;
        union {
            TripId tripId;
            Edge transferId;
        };
    };

public:
    CSA(const Data& data, const Profiler& profilerTemplate = Profiler()) :
        data(data),
        sourceStop(noStop),
        targetStop(noStop),
        tripReached(data.numberOfTrips(), TripFlag()),
        arrivalTime(data.numberOfStops(), never),
        parentLabel(PathRetrieval ? data.numberOfStops() : 0),
        profiler(profilerTemplate) {
        AssertMsg(Vector::isSorted(data.connections), "Connections must be sorted in ascending order!");
        profiler.registerPhases({PHASE_CLEAR, PHASE_INITIALIZATION, PHASE_CONNECTION_SCAN});
        profiler.registerMetrics({METRIC_CONNECTIONS, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
    }

    inline void run(const StopId source, const int departureTime, const StopId target = noStop) noexcept {
        profiler.start();

        profiler.startPhase();
        AssertMsg(data.isStop(source), "Source stop " << source << " is not a valid stop!");
        clear();
        profiler.donePhase(PHASE_CLEAR);

        profiler.startPhase();
        sourceStop = source;
        targetStop = target;
        arrivalTime[sourceStop] = departureTime;
        relaxEdges(sourceStop, departureTime);
        const ConnectionId firstConnection = firstReachableConnection(departureTime);
        profiler.donePhase(PHASE_INITIALIZATION);

        profiler.startPhase();
        scanConnections(firstConnection, ConnectionId(data.connections.size()));
        profiler.donePhase(PHASE_CONNECTION_SCAN);

        profiler.done();
    }

    inline bool reachable(const StopId stop) const noexcept {
        return arrivalTime[stop] < never;
    }

    inline int getEarliestArrivalTime(const StopId stop) const noexcept {
        return arrivalTime[stop];
    }

    template<bool T = PathRetrieval, typename = std::enable_if_t<T == PathRetrieval && T>>
    inline Journey getJourney() const noexcept {
        return getJourney(targetStop);
    }

    template<bool T = PathRetrieval, typename = std::enable_if_t<T == PathRetrieval && T>>
    inline Journey getJourney(StopId stop) const noexcept {
        Journey journey;
        if (!reachable(stop)) return journey;
        while (stop != sourceStop) {
            const ParentLabel& label = parentLabel[stop];
            if (label.reachedByTransfer) {
                const int travelTime = data.transferGraph.get(TravelTime, label.transferId);
                journey.emplace_back(label.parent, stop, arrivalTime[stop] - travelTime, arrivalTime[stop], label.transferId);
            } else {
                journey.emplace_back(label.parent, stop, data.connections[tripReached[label.tripId]].departureTime, arrivalTime[stop], label.tripId);
            }
            stop = label.parent;
        }
        Vector::reverse(journey);
        return journey;
    }

    inline std::vector<Vertex> getPath(const StopId stop) const noexcept {
        return journeyToPath(getJourney(stop));
    }

    inline std::vector<std::string> getRouteDescription(const StopId stop) const noexcept {
        return data.journeyToText(getJourney(stop));
    }

    inline const Profiler& getProfiler() const noexcept {
        return profiler;
    }

private:
    inline void clear() {
        sourceStop = noStop;
        targetStop = noStop;
        Vector::fill(arrivalTime, never);
        Vector::fill(tripReached, TripFlag());
        if constexpr (PathRetrieval) {
            Vector::fill(parentLabel, ParentLabel());
        }
    }

    inline ConnectionId firstReachableConnection(const int departureTime) const noexcept {
        return ConnectionId(Vector::lowerBound(data.connections, departureTime, [](const Connection& connection, const int time) {
            return connection.departureTime < time;
        }));
    }

    inline void scanConnections(const ConnectionId begin, const ConnectionId end) noexcept {
        for (ConnectionId i = begin; i < end; i++) {
            const Connection& connection = data.connections[i];
            if (targetStop != noStop && connection.departureTime > arrivalTime[targetStop]) break;
            if (connectionIsReachable(connection, i)) {
                profiler.countMetric(METRIC_CONNECTIONS);
                arrivalByTrip(connection.arrivalStopId, connection.arrivalTime, connection.tripId);
            }
        }
    }

    inline bool connectionIsReachableFromStop(const Connection& connection) const noexcept {
        return arrivalTime[connection.departureStopId] <= connection.departureTime - data.minTransferTime(connection.departureStopId);
    }

    inline bool connectionIsReachableFromTrip(const Connection& connection) const noexcept {
        return tripReached[connection.tripId] != TripFlag();
    }

    inline bool connectionIsReachable(const Connection& connection, const ConnectionId id) noexcept {
        if (connectionIsReachableFromTrip(connection)) return true;
        if (connectionIsReachableFromStop(connection)) {
            if constexpr (PathRetrieval) {
                tripReached[connection.tripId] = id;
            } else {
                suppressUnusedParameterWarning(id);
                tripReached[connection.tripId] = true;
            }
            return true;
        }
        return false;
    }

    inline void arrivalByTrip(const StopId stop, const int time, const TripId trip) noexcept {
        if (arrivalTime[stop] <= time) return;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        arrivalTime[stop] = time;
        if constexpr (PathRetrieval) {
            parentLabel[stop].parent = data.connections[tripReached[trip]].departureStopId;
            parentLabel[stop].reachedByTransfer = false;
            parentLabel[stop].tripId = trip;
        }
        relaxEdges(stop, time);
    }

    inline void relaxEdges(const StopId stop, const int time) noexcept {
        for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
            profiler.countMetric(METRIC_EDGES);
            const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
            const int newArrivalTime = time + data.transferGraph.get(TravelTime, edge);
            arrivalByTransfer(toStop, newArrivalTime, stop, edge);
        }
    }

    inline void arrivalByTransfer(const StopId stop, const int time, const StopId parent, const Edge edge) noexcept {
        if (arrivalTime[stop] <= time) return;
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        arrivalTime[stop] = time;
        if constexpr (PathRetrieval) {
            parentLabel[stop].parent = parent;
            parentLabel[stop].reachedByTransfer = true;
            parentLabel[stop].transferId = edge;
        }
    }

private:
    const Data& data;

    StopId sourceStop;
    StopId targetStop;

    std::vector<TripFlag> tripReached;
    std::vector<int> arrivalTime;
    std::vector<ParentLabel> parentLabel;

    Profiler profiler;
};
}
