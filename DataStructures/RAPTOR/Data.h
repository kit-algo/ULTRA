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

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <map>

#include "Entities/Stop.h"
#include "Entities/StopEvent.h"
#include "Entities/Route.h"
#include "Entities/RouteSegment.h"
#include "Entities/TripIterator.h"

#include "../Container/Map.h"
#include "../Container/Set.h"
#include "../Graph/Graph.h"
#include "../Intermediate/Data.h"

#include "../../Helpers/Assert.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/IO/Serialization.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/String/Enumeration.h"
#include "../../Helpers/Ranges/Range.h"
#include "../../Helpers/Ranges/SubRange.h"

#include "../../Algorithms/Dijkstra/Dijkstra.h"

namespace RAPTOR {

using TransferGraph = ::TransferGraph;

class Data {

private:
    Data() :
        implicitDepartureBufferTimes(false),
        implicitArrivalBufferTimes(false) {
    }

public:
    inline static Data FromBinary(const std::string& fileName) noexcept {
        Data data;
        data.deserialize(fileName);
        return data;
    }

    inline static Data FromIntermediate(const Intermediate::Data& inter, const int routeType = 1) noexcept {
        Data data;
        std::vector<std::vector<Intermediate::Trip>> routes;
        if (routeType == 0) routes = inter.geographicRoutes();
        if (routeType == 1) routes = inter.fifoRoutes();
        if (routeType == 2) routes = inter.offsetRoutes();
        for (const Intermediate::Stop& stop : inter.stops) {
            data.stopData.emplace_back(stop);
        }
        std::vector<std::vector<RouteSegment>> routeSegmentsOfStop(inter.stops.size());
        for (RouteId i = RouteId(0); i < routes.size(); i++) {
            const std::vector<Intermediate::Trip>& route = routes[i];
            AssertMsg(!route.empty(), "A route should not be empty!");
            data.routeData.emplace_back(route[0].routeName, route[0].type);
            data.firstStopIdOfRoute.emplace_back(data.stopIds.size());
            for (StopIndex j = StopIndex(0); j < route[0].stopEvents.size(); j++) {
                const Intermediate::StopEvent& stopEvent = route[0].stopEvents[j];
                routeSegmentsOfStop[stopEvent.stopId].emplace_back(i, j);
                data.stopIds.emplace_back(stopEvent.stopId);
            }
            data.firstStopEventOfRoute.emplace_back(data.stopEvents.size());
            for (const Intermediate::Trip& trip : route) {
                for (const Intermediate::StopEvent& stopEvent : trip.stopEvents) {
                    data.stopEvents.emplace_back(stopEvent);
                }
            }
        }
        for (const std::vector<RouteSegment>& routeSegmentList : routeSegmentsOfStop) {
            data.firstRouteSegmentOfStop.emplace_back(data.routeSegments.size());
            for (const RouteSegment& routeSegment : routeSegmentList) {
                data.routeSegments.emplace_back(routeSegment);
            }
        }
        data.firstStopIdOfRoute.emplace_back(data.stopIds.size());
        data.firstStopEventOfRoute.emplace_back(data.stopEvents.size());
        data.firstRouteSegmentOfStop.emplace_back(data.routeSegments.size());
        Intermediate::TransferGraph transferGraph = inter.transferGraph;
        Graph::move(std::move(transferGraph), data.transferGraph);
        return data;
    }

public:
    inline size_t numberOfStops() const noexcept {return stopData.size();}
    inline bool isStop(const Vertex stop) const noexcept {return stop < numberOfStops();}
    inline Range<StopId> stops() const noexcept {return Range<StopId>(StopId(0), StopId(numberOfStops()));}

    inline size_t numberOfRoutes() const noexcept {return routeData.size();}
    inline bool isRoute(const RouteId route) const noexcept {return route < numberOfRoutes();}
    inline Range<RouteId> routes() const noexcept {return Range<RouteId>(RouteId(0), RouteId(numberOfRoutes()));}

    inline size_t numberOfStopEvents() const noexcept {return stopEvents.size();}
    inline size_t numberOfRouteSegments() const noexcept {return routeSegments.size();}

    inline int minTransferTime(const StopId stop) const noexcept {return stopData[stop].minTransferTime;}
    inline int minTransferTime(const Vertex vertex) const noexcept {return isStop(vertex) ? stopData[vertex].minTransferTime : 0;}

    inline bool hasImplicitBufferTimes() const noexcept {return implicitDepartureBufferTimes | implicitArrivalBufferTimes;}

    inline size_t numberOfRoutesContainingStop(const StopId stop) const noexcept {
        AssertMsg(isStop(stop), "The id " << stop << " does not represent a stop!");
        return firstRouteSegmentOfStop[stop + 1] - firstRouteSegmentOfStop[stop];
    }

    inline size_t numberOfTripsContainingStop(const StopId stop) const noexcept {
        AssertMsg(isStop(stop), "The id " << stop << " does not represent a stop!");
        int result = 0;
        for (const RouteSegment& route : routesContainingStop(stop)) {
            result += numberOfTripsInRoute(route.routeId);
        }
        return result;
    }

    inline size_t numberOfStopsInRoute(const RouteId route) const noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        return firstStopIdOfRoute[route + 1] - firstStopIdOfRoute[route];
    }

    inline size_t numberOfStopEventsInRoute(const RouteId route) const noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        return firstStopEventOfRoute[route + 1] - firstStopEventOfRoute[route];
    }

    inline size_t numberOfTripsInRoute(const RouteId route) const noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        return numberOfStopEventsInRoute(route) / numberOfStopsInRoute(route);
    }

    inline size_t numberOfTrips() const noexcept {
        size_t count = 0;
        for (const RouteId route : routes()) {
            count += numberOfTripsInRoute(route);
        }
        return count;
    }

    inline size_t getRouteSegmentNum(const RouteId route, const StopIndex stopIndex) const noexcept {
        return firstStopIdOfRoute[route] + stopIndex;
    }

    inline SubRange<std::vector<RouteSegment>> routesContainingStop(const StopId stop) const noexcept {
        AssertMsg(isStop(stop), "The id " << stop << " does not represent a stop!");
        return SubRange<std::vector<RouteSegment>>(routeSegments, firstRouteSegmentOfStop, stop);
    }

    inline SubRange<std::vector<StopEvent>> stopEventsOfRoute(const RouteId route) const noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        return SubRange<std::vector<StopEvent>>(stopEvents, firstStopEventOfRoute, route);
    }

    inline SubRange<std::vector<StopId>> stopsOfRoute(const RouteId route) const noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        return SubRange<std::vector<StopId>>(stopIds, firstStopIdOfRoute, route);
    }

    inline const StopId* stopArrayOfRoute(const RouteId route) const noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        return &(stopIds[firstStopIdOfRoute[route]]);
    }

    inline StopId stopOfRouteSegment(const RouteSegment& route) const noexcept {
        AssertMsg(isRoute(route.routeId), "The id " << route.routeId << " does not represent a route!");
        return stopIds[firstStopIdOfRoute[route.routeId] + route.stopIndex];
    }

    inline const StopEvent* firstTripOfRoute(const RouteId route) const noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        return &(stopEvents[firstStopEventOfRoute[route]]);
    }

    inline const StopEvent* lastTripOfRoute(const RouteId route) const noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        return &(stopEvents[firstStopEventOfRoute[route + 1] - numberOfStopsInRoute(route)]);
    }

    inline const StopEvent* tripOfRoute(const RouteId route, const size_t tripNum) const noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        AssertMsg(tripNum < numberOfTripsInRoute(route), "Trip number " << tripNum << " exceeds number of trips in route!");
        return firstTripOfRoute(route) + tripNum * numberOfStopsInRoute(route);
    }

    inline TripIterator getTripIterator(const RouteId route, const StopIndex stopIndex, const StopEvent* const currentTrip) const noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        AssertMsg(stopIndex < numberOfStopsInRoute(route), "The route " << route << " has only " << numberOfStopsInRoute(route) << " stops, but stop index " << stopIndex << " was requested!");
        AssertMsg(currentTrip >= firstTripOfRoute(route), "The specified trip is not part of the route!");
        AssertMsg(currentTrip <= lastTripOfRoute(route), "The specified trip is not part of the route!");
        AssertMsg((currentTrip - firstTripOfRoute(route)) % numberOfStopsInRoute(route) == 0, "The specified trip is not valid!");
        return TripIterator(numberOfStopsInRoute(route), stopArrayOfRoute(route), firstTripOfRoute(route), stopIndex, currentTrip);
    }

    inline TripIterator getTripIterator(const RouteId route, const StopIndex stopIndex, size_t currentTripNumber) const noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        AssertMsg(stopIndex < numberOfStopsInRoute(route), "The route " << route << " has only " << numberOfStopsInRoute(route) << " stops, but stop index " << stopIndex << " was requested!");
        currentTripNumber = std::min(currentTripNumber, numberOfTripsInRoute(route) - 1);
        const StopEvent* const currentTrip = tripOfRoute(route, currentTripNumber);
        AssertMsg(currentTrip <= lastTripOfRoute(route), "currentTrip is not a trip of the given route!");
        return TripIterator(numberOfStopsInRoute(route), stopArrayOfRoute(route), firstTripOfRoute(route), stopIndex, currentTrip);
    }

    inline TripIterator getTripIterator(const RouteId route, const StopIndex stopIndex = StopIndex(0)) const noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        AssertMsg(stopIndex < numberOfStopsInRoute(route), "The route " << route << " has only " << numberOfStopsInRoute(route) << " stops, but stop index " << stopIndex << " was requested!");
        return TripIterator(numberOfStopsInRoute(route), stopArrayOfRoute(route), firstTripOfRoute(route), stopIndex, lastTripOfRoute(route));
    }

    inline TripIterator getTripIterator(const RouteSegment& route) const noexcept {
        return getTripIterator(route.routeId, route.stopIndex);
    }

    inline std::vector<const StopEvent*> getLastTripByStopIndex() const noexcept {
        std::vector<const StopEvent*> result;
        result.reserve(stopIds.size());
        for (const RouteId route : routes()) {
            AssertMsg(numberOfStopsInRoute(route) > 0, "Route " << route << " has 0 stops!");
            result.emplace_back(lastTripOfRoute(route));
            result.resize(firstStopIdOfRoute[route + 1], result.back());
        }
        AssertMsg(result.size() == stopIds.size(), "Wrong number of trips!");
        return result;
    }

private:
    inline StopEvent* firstTripOfRoute(const RouteId route) noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        return &(stopEvents[firstStopEventOfRoute[route]]);
    }

    inline StopEvent* lastTripOfRoute(const RouteId route) noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        return &(stopEvents[firstStopEventOfRoute[route + 1] - numberOfStopsInRoute(route)]);
    }

    template<typename ADJUST>
    inline void adjustTimes(const ADJUST& adjust) noexcept {
        for (const RouteId route : routes()) {
            const StopId* stops = stopArrayOfRoute(route);
            const size_t tripSize = numberOfStopsInRoute(route);
            for (size_t stopIndex = 0; stopIndex < tripSize; stopIndex++) {
                for (StopEvent* trip = firstTripOfRoute(route); trip <= lastTripOfRoute(route); trip += tripSize) {
                    adjust(trip[stopIndex], stops[stopIndex]);
                }
            }
        }
    }

public:
    inline void useImplicitDepartureBufferTimes() noexcept {
        if (implicitDepartureBufferTimes | implicitArrivalBufferTimes) return;
        adjustTimes([&](StopEvent& stopEvent, const StopId stop){
            stopEvent.departureTime -= minTransferTime(stop);
        });
        implicitDepartureBufferTimes = true;
    }

    inline void dontUseImplicitDepartureBufferTimes() noexcept {
        if (!implicitDepartureBufferTimes) return;
        adjustTimes([&](StopEvent& stopEvent, const StopId stop){
            stopEvent.departureTime += minTransferTime(stop);
        });
        implicitDepartureBufferTimes = false;
    }

    inline void useImplicitArrivalBufferTimes() noexcept {
        if (implicitDepartureBufferTimes | implicitArrivalBufferTimes) return;
        adjustTimes([&](StopEvent& stopEvent, const StopId stop){
            stopEvent.arrivalTime += minTransferTime(stop);
        });
        implicitArrivalBufferTimes = true;
    }

    inline void dontUseImplicitArrivalBufferTimes() noexcept {
        if (!implicitArrivalBufferTimes) return;
        adjustTimes([&](StopEvent& stopEvent, const StopId stop){
            stopEvent.arrivalTime -= minTransferTime(stop);
        });
        implicitArrivalBufferTimes = false;
    }

    inline int getMinDepartureTime() const noexcept {
        int minDepartureTime = never;
        for (const RouteId route : routes()) {
            const int minDepartureTimeOfRoute = getMinDepartureTime(route);
            if (minDepartureTime > minDepartureTimeOfRoute) {
                minDepartureTime = minDepartureTimeOfRoute;
            }
        }
        return minDepartureTime;
    }

    inline int getMinDepartureTime(const RouteId route) const noexcept {
        if (implicitDepartureBufferTimes) {
            int minDepartureTimeOfRoute = never;
            const size_t tripSize = numberOfStopsInRoute(route);
            for (size_t stopIndex = 0; stopIndex + 1 < tripSize; stopIndex++) {
                if (minDepartureTimeOfRoute > firstTripOfRoute(route)[stopIndex].departureTime) {
                    minDepartureTimeOfRoute = firstTripOfRoute(route)[stopIndex].departureTime;
                }
            }
            return minDepartureTimeOfRoute;
        } else {
            return firstTripOfRoute(route)->departureTime;
        }
    }

    inline int getTripOffset(const RouteId route) const noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        const size_t tripCount = numberOfTripsInRoute(route);
        const size_t stopCount = numberOfStopsInRoute(route);
        if (tripCount <= 1) return 1;
        const int offset = stopEvents[firstStopEventOfRoute[route] + stopCount].departureTime - stopEvents[firstStopEventOfRoute[route]].departureTime;
        for (size_t i = 1; i < tripCount; i++) {
            AssertMsg(offset == stopEvents[firstStopEventOfRoute[route] + (i * stopCount)].departureTime - stopEvents[firstStopEventOfRoute[route] + ((i - 1) * stopCount)].departureTime, "The route " << route << " has no constant frequency!");
        }
        return offset;
    }

    inline const std::vector<Geometry::Point>& getCoordinates() const noexcept {
        return transferGraph[Coordinates];
    }

    inline Geometry::Rectangle boundingBox() const noexcept {
        Geometry::Rectangle result = Geometry::Rectangle::Empty();
        for (const Stop& stop : stopData) {
            result.extend(stop.coordinates);
        }
        return result;
    }

    inline void applyMinTravelTime(const double minTravelTime) noexcept {
        for (const Vertex from : transferGraph.vertices()) {
            for (const Edge edge : transferGraph.edgesFrom(from)) {
                if (transferGraph.get(TravelTime, edge) < minTravelTime) {
                    transferGraph.set(TravelTime, edge, minTravelTime);
                }
            }
        }
    }

public:
    inline void printInfo() const noexcept {
        size_t stopEventCount = stopEvents.size();
        size_t tripCount = numberOfTrips();
        int firstDay = std::numeric_limits<int>::max();
        int lastDay = std::numeric_limits<int>::min();
        for (const StopEvent& stopEvent : stopEvents) {
            if (firstDay > stopEvent.departureTime) firstDay = stopEvent.departureTime;
            if (lastDay < stopEvent.arrivalTime) lastDay = stopEvent.arrivalTime;
        }
        std::cout << "RAPTOR public transit data:" << std::endl;
        std::cout << "   Number of Stops:          " << std::setw(12) << String::prettyInt(numberOfStops()) << std::endl;
        std::cout << "   Number of Routes:         " << std::setw(12) << String::prettyInt(numberOfRoutes()) << std::endl;
        std::cout << "   Number of Trips:          " << std::setw(12) << String::prettyInt(tripCount) << std::endl;
        std::cout << "   Number of Stop Events:    " << std::setw(12) << String::prettyInt(stopEventCount) << std::endl;
        std::cout << "   Number of Connections:    " << std::setw(12) << String::prettyInt(stopEventCount - tripCount) << std::endl;
        std::cout << "   Number of Vertices:       " << std::setw(12) << String::prettyInt(transferGraph.numVertices()) << std::endl;
        std::cout << "   Number of Edges:          " << std::setw(12) << String::prettyInt(transferGraph.numEdges()) << std::endl;
        std::cout << "   First Day:                " << std::setw(12) << String::prettyInt(firstDay / (60 * 60 * 24)) << std::endl;
        std::cout << "   Last Day:                 " << std::setw(12) << String::prettyInt(lastDay / (60 * 60 * 24)) << std::endl;
        std::cout << "   Bounding Box:             " << std::setw(12) << boundingBox() << std::endl;
    }

    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, firstRouteSegmentOfStop, firstStopIdOfRoute, firstStopEventOfRoute, routeSegments, stopIds, stopEvents, stopData, routeData, implicitDepartureBufferTimes, implicitArrivalBufferTimes);
        transferGraph.writeBinary(fileName + ".graph");
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, firstRouteSegmentOfStop, firstStopIdOfRoute, firstStopEventOfRoute, routeSegments, stopIds, stopEvents, stopData, routeData, implicitDepartureBufferTimes, implicitArrivalBufferTimes);
        transferGraph.readBinary(fileName + ".graph");
    }

    inline long long byteSize() const noexcept {
        long long result = Vector::byteSize(firstRouteSegmentOfStop);
        result += Vector::byteSize(firstStopIdOfRoute);
        result += Vector::byteSize(firstStopEventOfRoute);
        result += Vector::byteSize(routeSegments);
        result += Vector::byteSize(stopIds);
        result += Vector::byteSize(stopEvents);
        result += Vector::byteSize(stopData);
        result += Vector::byteSize(routeData);
        result += transferGraph.byteSize();
        result += 2 * sizeof(bool);
        return result;
    }

public:
    std::vector<size_t> firstRouteSegmentOfStop;

    std::vector<size_t> firstStopIdOfRoute;
    std::vector<size_t> firstStopEventOfRoute;

    std::vector<RouteSegment> routeSegments;

    std::vector<StopId> stopIds;
    std::vector<StopEvent> stopEvents;

    std::vector<Stop> stopData;
    std::vector<Route> routeData;

    TransferGraph transferGraph;

    bool implicitDepartureBufferTimes;
    bool implicitArrivalBufferTimes;

};

}
