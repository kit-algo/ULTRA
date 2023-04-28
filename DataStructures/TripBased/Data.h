#pragma once

#include <algorithm>
#include <vector>

#include "../RAPTOR/Data.h"
#include "../../Helpers/Console/Progress.h"

namespace TripBased {

struct ArrivalEvent {
    ArrivalEvent(const int arrivalTime = INFTY, const StopId stop = noStop) :
        arrivalTime(arrivalTime),
        stop(stop) {
    }
    int arrivalTime;
    StopId stop;
};

class Data {

public:
    Data() {}

    Data(const RAPTOR::Data& data) :
        raptorData(data) {
        for (const RouteId route : routes()) {
            firstTripOfRoute.emplace_back(TripId(routeOfTrip.size()));
            const size_t tripLength = raptorData.numberOfStopsInRoute(route);
            const size_t firstStopId = raptorData.firstStopIdOfRoute[route];
            for (StopEventId firstStopEvent = StopEventId(raptorData.firstStopEventOfRoute[route]); firstStopEvent < raptorData.firstStopEventOfRoute[route + 1]; firstStopEvent += tripLength) {
                const TripId trip = TripId(routeOfTrip.size());
                routeOfTrip.emplace_back(route);
                firstStopIdOfTrip.emplace_back(firstStopId);
                firstStopEventOfTrip.emplace_back(firstStopEvent);
                for (StopIndex i = StopIndex(0); i < tripLength; i++) {
                    tripOfStopEvent.emplace_back(trip);
                    indexOfStopEvent.emplace_back(i);
                    arrivalEvents.emplace_back(raptorData.stopEvents[arrivalEvents.size()].arrivalTime, raptorData.stopIds[firstStopId + i]);
                }
            }
        }
        firstTripOfRoute.emplace_back(TripId(routeOfTrip.size()));
        firstStopIdOfTrip.emplace_back(StopId(raptorData.stopIds.size()));
        firstStopEventOfTrip.emplace_back(raptorData.stopEvents.size());
        if (!raptorData.hasImplicitBufferTimes()) {
            raptorData.useImplicitDepartureBufferTimes();
        }
    }

    Data(const std::string& fileName) {
        deserialize(fileName);
    }

public:
    inline const TransferGraph& getTransferGraph() const noexcept {return raptorData.transferGraph;}

    inline size_t numberOfStops() const noexcept {return raptorData.numberOfStops();}
    inline bool isStop(const Vertex stop) const noexcept {return raptorData.isStop(stop);}
    inline Range<StopId> stops() const noexcept {return raptorData.stops();}

    inline size_t numberOfTrips() const noexcept {return routeOfTrip.size();}
    inline bool isTrip(const TripId trip) const noexcept {return trip < numberOfTrips();}
    inline Range<TripId> trips() const noexcept {return Range<TripId>(TripId(0), TripId(numberOfTrips()));}

    inline size_t numberOfRoutes() const noexcept {return raptorData.numberOfRoutes();}
    inline bool isRoute(const RouteId route) const noexcept {return raptorData.isRoute(route);}
    inline Range<RouteId> routes() const noexcept {return raptorData.routes();}

    inline size_t numberOfStopEvents() const noexcept {return raptorData.numberOfStopEvents();}
    inline size_t numberOfRouteSegments() const noexcept {return raptorData.numberOfRouteSegments();}

    inline int minTransferTime(const StopId stop) const noexcept {return raptorData.minTransferTime(stop);}

    inline size_t numberOfStopsInRoute(const RouteId route) const noexcept {return raptorData.numberOfStopsInRoute(route);}
    inline size_t numberOfTripsInRoute(const RouteId route) const noexcept {return raptorData.numberOfTripsInRoute(route);}
    inline size_t numberOfStopsInTrip(const TripId trip) const noexcept {
        AssertMsg(isTrip(trip), "The id " << trip << " does not represent a trip!");
        return firstStopEventOfTrip[trip + 1] - firstStopEventOfTrip[trip];
    }

    inline size_t maxTripLength() const noexcept {
        size_t maxLength = 0;
        for (const RouteId route : routes()) {
            maxLength = std::max(maxLength, numberOfStopsInRoute(route));
        }
        return maxLength;
    }

    inline StopId getStop(const TripId trip, const StopIndex index) const noexcept {
        AssertMsg(isTrip(trip), "The id " << trip << " does not represent a trip!");
        AssertMsg(index < numberOfStopsInTrip(trip), "The trip " << trip << " has only " << numberOfStopsInTrip(trip) << " stops!");
        return raptorData.stopIds[firstStopIdOfTrip[trip] + index];
    }

    inline RouteId getRouteOfStopEvent(const StopEventId stopEvent) const noexcept {
        return routeOfTrip[tripOfStopEvent[stopEvent]];
    }

    inline StopId getStopOfStopEvent(const StopEventId stopEvent) const noexcept {
        return getStop(tripOfStopEvent[stopEvent], indexOfStopEvent[stopEvent]);
    }

    inline StopEventId getStopEventId(const TripId trip, const StopIndex index) const noexcept {
        AssertMsg(isTrip(trip), "The id " << trip << " does not represent a trip!");
        AssertMsg(index < numberOfStopsInTrip(trip), "The trip " << trip << " has only " << numberOfStopsInTrip(trip) << " stops!");
        return StopEventId(firstStopEventOfTrip[trip] + index);
    }

    inline const RAPTOR::StopEvent& getStopEvent(const TripId trip, const StopIndex index) const noexcept {
        AssertMsg(isTrip(trip), "The id " << trip << " does not represent a trip!");
        AssertMsg(index < numberOfStopsInTrip(trip), "The trip " << trip << " has only " << numberOfStopsInTrip(trip) << " stops!");
        return raptorData.stopEvents[firstStopEventOfTrip[trip] + index];
    }

    inline const int& arrivalTime(const StopEventId stopEvent) const noexcept {
        return raptorData.stopEvents[stopEvent].arrivalTime;
    }

    inline int& arrivalTime(const StopEventId stopEvent) noexcept {
        return raptorData.stopEvents[stopEvent].arrivalTime;
    }

    inline const int& departureTime(const StopEventId stopEvent) const noexcept {
        return raptorData.stopEvents[stopEvent].departureTime;
    }

    inline int& departureTime(const StopEventId stopEvent) noexcept {
        return raptorData.stopEvents[stopEvent].departureTime;
    }

    inline SubRange<std::vector<RAPTOR::RouteSegment>> routesContainingStop(const StopId stop) const noexcept {
        AssertMsg(isStop(stop), "The id " << stop << " does not represent a stop!");
        return raptorData.routesContainingStop(stop);
    }

    inline Range<TripId> tripsOfRoute(const RouteId route) const noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        return Range<TripId>(firstTripOfRoute[route], firstTripOfRoute[route + 1]);
    }

    inline const StopId* stopArrayOfRoute(const RouteId route) const noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        return raptorData.stopArrayOfRoute(route);
    }

    inline const StopId* stopArrayOfTrip(const TripId trip) const noexcept {
        AssertMsg(isTrip(trip), "The id " << trip << " does not represent a trip!");
        return stopArrayOfRoute(routeOfTrip[trip]);
    }

    inline const RAPTOR::StopEvent* eventArrayOfTrip(const TripId trip) const noexcept {
        AssertMsg(isTrip(trip), "The id " << trip << " does not represent a trip!");
        return &(raptorData.stopEvents[firstStopEventOfTrip[trip]]);
    }

    inline TripId getEarliestTrip(const RouteId route, const StopIndex stopIndex, const int time) const noexcept {
        return getEarliestTripBinary(RAPTOR::RouteSegment(route, stopIndex), time);
    }

    inline TripId getEarliestTrip(const RAPTOR::RouteSegment& route, const int time) const noexcept {
        return getEarliestTripBinary(route, time);
    }

    inline TripId getEarliestTripLinear(const RAPTOR::RouteSegment& route, const int time) const noexcept {
        if (route.stopIndex + 1 == raptorData.numberOfStopsInRoute(route.routeId)) return noTripId;
        if (raptorData.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < time) return noTripId;
        for (const TripId trip : tripsOfRoute(route.routeId)) {
            if (getStopEvent(trip, route.stopIndex).departureTime >= time) return trip;
        }
        return noTripId;
    }

    inline TripId getEarliestTripBinary(const RAPTOR::RouteSegment& route, const int time) const noexcept {
        if (route.stopIndex + 1 == raptorData.numberOfStopsInRoute(route.routeId)) return noTripId;
        const TripId trip = std::lower_bound(firstTripOfRoute[route.routeId], firstTripOfRoute[route.routeId + 1], time, [&](const TripId trip, const int time) {
            return getStopEvent(trip, route.stopIndex).departureTime < time;
        });
        if (trip < firstTripOfRoute[route.routeId + 1]) return trip;
        return noTripId;
    }

    inline int getTransferSlack(const StopEventId from, const StopEventId to, const int travelTime) const noexcept {
        return departureTime(to) - arrivalTime(from) - travelTime;
    }

public:
    inline Data reverseNetwork() const noexcept {
        Permutation dummy;
        return reverseNetwork(dummy);
    }

    inline Data reverseNetwork(Permutation& stopEventPermutation) const noexcept {
        const RAPTOR::Data reverseRaptorData = raptorData.reverseNetwork(stopEventPermutation);
        Data reverseData(reverseRaptorData);
        Graph::copy(stopEventGraph, reverseData.stopEventGraph);
        reverseData.stopEventGraph.revert();
        reverseData.stopEventGraph.applyVertexPermutation(stopEventPermutation);
        reverseData.stopEventGraph.sortEdges(ToVertex);
        return reverseData;
    }

    inline void removeInfeasibleShortcuts() noexcept {
        TransferGraph temp;
        temp.reserve(stopEventGraph.numVertices(), stopEventGraph.numEdges());
        for (const Vertex fromStopEvent : stopEventGraph.vertices()) {
            temp.addVertex();
            for (const Edge shortcut : stopEventGraph.edgesFrom(fromStopEvent)) {
                const Vertex toStopEvent = stopEventGraph.get(ToVertex, shortcut);
                const int travelTime = stopEventGraph.get(TravelTime, shortcut);
                if (arrivalTime(StopEventId(fromStopEvent)) + travelTime > departureTime(StopEventId(toStopEvent))) continue;
                temp.addEdge(fromStopEvent, toStopEvent, stopEventGraph.edgeRecord(shortcut));
            }
        }
        Graph::move(std::move(temp), stopEventGraph);
    }

    inline void printInfo() const noexcept {
        int firstDay = std::numeric_limits<int>::max();
        int lastDay = std::numeric_limits<int>::min();
        for (const RAPTOR::StopEvent& stopEvent : raptorData.stopEvents) {
            if (firstDay > stopEvent.departureTime) firstDay = stopEvent.departureTime;
            if (lastDay < stopEvent.arrivalTime) lastDay = stopEvent.arrivalTime;
        }
        std::cout << "Trip-Based public transit data:" << std::endl;
        std::cout << "   Number of Stops:          " << std::setw(12) << String::prettyInt(numberOfStops()) << std::endl;
        std::cout << "   Number of Routes:         " << std::setw(12) << String::prettyInt(numberOfRoutes()) << std::endl;
        std::cout << "   Number of Trips:          " << std::setw(12) << String::prettyInt(numberOfTrips()) << std::endl;
        std::cout << "   Number of Stop Events:    " << std::setw(12) << String::prettyInt(numberOfStopEvents()) << std::endl;
        std::cout << "   Number of Connections:    " << std::setw(12) << String::prettyInt(numberOfStopEvents() - numberOfTrips()) << std::endl;
        std::cout << "   Number of Transfers:      " << std::setw(12) << String::prettyInt(stopEventGraph.numEdges()) << std::endl;
        std::cout << "   Number of Vertices:       " << std::setw(12) << String::prettyInt(raptorData.transferGraph.numVertices()) << std::endl;
        std::cout << "   Number of Edges:          " << std::setw(12) << String::prettyInt(raptorData.transferGraph.numEdges()) << std::endl;
        std::cout << "   First Day:                " << std::setw(12) << String::prettyInt(firstDay / (60 * 60 * 24)) << std::endl;
        std::cout << "   Last Day:                 " << std::setw(12) << String::prettyInt(lastDay / (60 * 60 * 24)) << std::endl;
        std::cout << "   Bounding Box:             " << std::setw(12) << raptorData.boundingBox() << std::endl;
    }

    inline void serialize(const std::string& fileName) const noexcept {
        raptorData.serialize(fileName + ".raptor");
        IO::serialize(fileName, firstTripOfRoute, routeOfTrip, firstStopIdOfTrip, firstStopEventOfTrip, tripOfStopEvent, indexOfStopEvent, arrivalEvents);
        stopEventGraph.writeBinary(fileName + ".graph");
    }

    inline void deserialize(const std::string& fileName) noexcept {
        raptorData.deserialize(fileName + ".raptor");
        IO::deserialize(fileName, firstTripOfRoute, routeOfTrip, firstStopIdOfTrip, firstStopEventOfTrip, tripOfStopEvent, indexOfStopEvent, arrivalEvents);
        stopEventGraph.readBinary(fileName + ".graph");
    }

public:
    RAPTOR::Data raptorData;

    std::vector<TripId> firstTripOfRoute;

    std::vector<RouteId> routeOfTrip;
    std::vector<size_t> firstStopIdOfTrip;
    std::vector<StopEventId> firstStopEventOfTrip;

    std::vector<TripId> tripOfStopEvent;
    std::vector<StopIndex> indexOfStopEvent;

    TransferGraph stopEventGraph;

    std::vector<ArrivalEvent> arrivalEvents;

};

}
