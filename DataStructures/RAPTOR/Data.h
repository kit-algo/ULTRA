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
#include "Entities/Journey.h"
#include "Entities/TripIterator.h"

#include "../Intermediate/Data.h"
#include "../Container/Map.h"
#include "../Container/Set.h"
#include "../Graph/Graph.h"
#include "../Geometry/Rectangle.h"

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

public:
    Data() :
        implicitDepartureBufferTimes(false),
        implicitArrivalBufferTimes(false) {
    }

    Data(const std::string& fileName) :
        implicitDepartureBufferTimes(false),
        implicitArrivalBufferTimes(false) {
        deserialize(fileName);
    }

    inline static Data FromBinary(const std::string& fileName) noexcept {
        Data data;
        data.deserialize(fileName);
        return data;
    }

    inline static Data FromIntermediate(const Intermediate::Data& inter, const int routeType = 1) noexcept {
        if (routeType == 0) return FromIntermediate(inter, inter.geographicRoutes());
        if (routeType == 1) return FromIntermediate(inter, inter.fifoRoutes());
        if (routeType == 2) return FromIntermediate(inter, inter.offsetRoutes());
        return FromIntermediate(inter, inter.fifoRoutes());
    }

    inline static Data FromIntermediate(const Intermediate::Data& inter, const std::vector<std::vector<Intermediate::Trip>>& routes) noexcept {
        Data data;
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

    inline SubRange<std::vector<StopEvent>> stopEventsOfTrip(const RouteId route, const size_t tripNum) const noexcept {
        AssertMsg(isRoute(route), "The id " << route << " does not represent a route!");
        AssertMsg(tripNum < numberOfTripsInRoute(route), tripNum << " is not a valid trip of route " << route << "!");
        const size_t tripSize = numberOfStopsInRoute(route);
        const size_t firstStopEvent = firstStopEventOfRoute[route] + tripNum * tripSize;
        const size_t lastStopEvent = firstStopEvent + tripSize;
        return SubRange<std::vector<StopEvent>>(stopEvents, firstStopEvent, lastStopEvent);
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

    inline double maxRouteDistance(const RouteSegment& route) const noexcept {
        AssertMsg(isRoute(route.routeId), "The id " << route << " does not represent a route!");
        double maxDist = 0;
        const StopId* stops = stopArrayOfRoute(route.routeId);
        const size_t tripSize = numberOfStopsInRoute(route.routeId);
        for (size_t stopIndex = route.stopIndex + 1; stopIndex < tripSize; stopIndex++) {
            const double dist = geoDistanceInCM(stopData[stops[route.stopIndex]].coordinates, stopData[stops[stopIndex]].coordinates) / 100000.0;
            if (maxDist >= dist) continue;
            maxDist = dist;
        }
        return maxDist;
    }

    inline double maxRouteSpeed(const RouteSegment& route) const noexcept {
        AssertMsg(isRoute(route.routeId), "The id " << route << " does not represent a route!");
        double maxSpeed = 0;
        const StopId* stops = stopArrayOfRoute(route.routeId);
        const size_t tripSize = numberOfStopsInRoute(route.routeId);
        for (const StopEvent* trip = firstTripOfRoute(route.routeId); trip <= lastTripOfRoute(route.routeId); trip += tripSize) {
            for (size_t stopIndex = route.stopIndex + 1; stopIndex < tripSize; stopIndex++) {
                const double speed = geoDistanceInCM(stopData[stops[route.stopIndex]].coordinates, stopData[stops[stopIndex]].coordinates) / (100000.0 * (trip[stopIndex].arrivalTime - trip[route.stopIndex].departureTime));
                if (maxSpeed >= speed) continue;
                maxSpeed = speed;
            }
        }
        return maxSpeed;
    }

    inline double maxRouteSpeedTimesDistance(const RouteSegment& route) const noexcept {
        AssertMsg(isRoute(route.routeId), "The id " << route << " does not represent a route!");
        double maxSpeedTimesDistance = 0;
        const StopId* stops = stopArrayOfRoute(route.routeId);
        const size_t tripSize = numberOfStopsInRoute(route.routeId);
        for (const StopEvent* trip = firstTripOfRoute(route.routeId); trip <= lastTripOfRoute(route.routeId); trip += tripSize) {
            for (size_t stopIndex = route.stopIndex + 1; stopIndex < tripSize; stopIndex++) {
                const double dist = geoDistanceInCM(stopData[stops[route.stopIndex]].coordinates, stopData[stops[stopIndex]].coordinates) / 100000.0;
                const double speedTimesDist = dist * dist / (trip[stopIndex].arrivalTime - trip[route.stopIndex].departureTime);
                if (maxSpeedTimesDistance >= speedTimesDist) continue;
                maxSpeedTimesDistance = speedTimesDist;
            }
        }
        return maxSpeedTimesDistance;
    }

    inline double maxRouteDistance(const StopId stop) const noexcept {
        AssertMsg(isStop(stop), "The id " << stop << " does not represent a stop!");
        double maxDist = 0;
        for (const RouteSegment& route : routesContainingStop(stop)) {
            const double dist = maxRouteDistance(route);
            if (maxDist >= dist) continue;
            maxDist = dist;
        }
        return maxDist;
    }

    inline double maxRouteSpeed(const StopId stop) const noexcept {
        AssertMsg(isStop(stop), "The id " << stop << " does not represent a stop!");
        double maxSpeed = 0;
        for (const RouteSegment& route : routesContainingStop(stop)) {
            const double speed = maxRouteSpeed(route);
            if (maxSpeed >= speed) continue;
            maxSpeed = speed;
        }
        return maxSpeed;
    }

    inline double maxRouteSpeedTimesDistance(const StopId stop) const noexcept {
        AssertMsg(isStop(stop), "The id " << stop << " does not represent a stop!");
        double maxSpeedTimesDistance = 0;
        for (const RouteSegment& route : routesContainingStop(stop)) {
            const double speedTimesDist = maxRouteSpeedTimesDistance(route);
            if (maxSpeedTimesDistance >= speedTimesDist) continue;
            maxSpeedTimesDistance = speedTimesDist;
        }
        return maxSpeedTimesDistance;
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

    inline std::vector<std::string> journeyToText(const Journey& journey) const noexcept {
        std::vector<std::string> text;
        for (const JourneyLeg& leg : journey) {
            std::stringstream line;
            if (leg.usesRoute) {
                line << "Take " << GTFS::TypeNames[routeData[leg.routeId].type];
                line << ": " << routeData[leg.routeId].name << " [" << leg.routeId << "] ";
                line << "from " << stopData[leg.from].name << " [" << leg.from << "] ";
                line << "departing at " << String::secToTime(leg.departureTime) << " [" << leg.departureTime << "], ";
                line << "to " << stopData[leg.to].name << " [" << leg.to << "] ";
                line << "arrive at " << String::secToTime(leg.arrivalTime) << " [" << leg.arrivalTime << "] ";
            } else if (leg.from == leg.to) {
                line << "Wait at " << stopData[leg.from].name << " [" << leg.from << "], ";
                line << "minimal waiting time: " << String::secToString(leg.arrivalTime - leg.departureTime) << ".";
            } else {
                line << "Walk from " << (isStop(leg.from) ? stopData[leg.from].name : "Vertex") << " [" << leg.from << "] ";
                line << "to " << (isStop(leg.to) ? stopData[leg.to].name : "Vertex") << " [" << leg.to << "], ";
                line << "start at " << String::secToTime(leg.departureTime) << " [" << leg.departureTime << "] ";
                line << "and arrive at " << String::secToTime(leg.arrivalTime) << " [" << leg.arrivalTime << "] ";
                line << "(" << String::secToString(leg.arrivalTime - leg.departureTime) << ").";
            }
            text.emplace_back(line.str());
        }
        return text;
    }

    inline Data reverseNetwork() const noexcept {
        Permutation dummy;
        return reverseNetwork(dummy);
    }

    inline Data reverseNetwork(Permutation& stopEventPermutation) const noexcept {
        stopEventPermutation.resize(stopEvents.size());
        Data result;
        result.firstRouteSegmentOfStop = firstRouteSegmentOfStop;
        result.firstStopIdOfRoute = firstStopIdOfRoute;
        result.firstStopEventOfRoute = firstStopEventOfRoute;
        for (const RouteSegment routeSegment : routeSegments) {
            result.routeSegments.emplace_back(routeSegment.routeId, StopIndex(numberOfStopsInRoute(routeSegment.routeId) - routeSegment.stopIndex - 1));
        }
        for (const RouteId route : routes()) {
            for (size_t i = 0; i < numberOfStopsInRoute(route); i++) {
                result.stopIds.emplace_back(stopIds[firstStopIdOfRoute[route + 1] - i - 1]);
            }
            for (size_t i = 0; i < numberOfStopEventsInRoute(route); i++) {
                result.stopEvents.emplace_back(stopEvents[firstStopEventOfRoute[route + 1] - i - 1].reverseStopEvent());
                stopEventPermutation[result.stopEvents.size() - 1] = firstStopEventOfRoute[route + 1] - i - 1;
            }
        }
        result.stopData = stopData;
        result.routeData = routeData;
        result.transferGraph = transferGraph;
        result.transferGraph.revert();
        result.implicitDepartureBufferTimes = implicitArrivalBufferTimes;
        result.implicitArrivalBufferTimes = implicitDepartureBufferTimes;
        return result;
    }

    inline TransferGraph averageTravelTimeGraph(const int minTime, const int maxTime) const noexcept {
        struct Connection {
            Connection(const int departureTime, const int travelTime) : departureTime(departureTime), travelTime(travelTime) {}
            inline bool operator<(const Connection other) const noexcept {return (departureTime < other.departureTime) || ((departureTime == other.departureTime) && (travelTime < other.travelTime));}
            inline int arrivalTime() const noexcept {return departureTime + travelTime;}
            int departureTime;
            int travelTime;
        };
        Intermediate::TransferGraph topology;
        topology.addVertices(transferGraph.numVertices());
        for (const RouteId route : routes()) {
            for (size_t stopIndex = firstStopIdOfRoute[route] + 1; stopIndex < firstStopIdOfRoute[route + 1]; stopIndex++) {
                topology.findOrAddEdge(stopIds[stopIndex - 1], stopIds[stopIndex]).set(TravelTime, intMax);
            }
        }
        for (Vertex vertex : transferGraph.vertices()) {
            topology.set(Coordinates, vertex, transferGraph.get(Coordinates, vertex));
            for (Edge edge : transferGraph.edgesFrom(vertex)) {
                topology.findOrAddEdge(vertex, transferGraph.get(ToVertex, edge)).set(TravelTime, transferGraph.get(TravelTime, edge));
            }
        }
        topology.packEdges();
        std::vector<std::vector<Connection>> connectionsByEdgeId(topology.numEdges());
        for (const RouteId route : routes()) {
            const StopId* stops = stopArrayOfRoute(route);
            const size_t tripSize = numberOfStopsInRoute(route);
            for (size_t stopIndex = 1; stopIndex < tripSize; stopIndex++) {
                const Edge edge = topology.findEdge(stops[stopIndex - 1], stops[stopIndex]);
                for (const StopEvent* trip = firstTripOfRoute(route); trip <= lastTripOfRoute(route); trip += tripSize) {
                    if (trip[stopIndex - 1].departureTime < minTime) continue;
                    const int travelTime = trip[stopIndex].arrivalTime - trip[stopIndex - 1].departureTime;
                    if (travelTime >= topology.get(TravelTime, edge)) continue;
                    if (edge >= connectionsByEdgeId.size()) connectionsByEdgeId.resize(edge + 1);
                    connectionsByEdgeId[edge].emplace_back(trip[stopIndex - 1].departureTime, travelTime);
                }
            }
        }
        for (Edge edge : topology.edges()) {
            if (connectionsByEdgeId[edge].empty()) continue;
            int lastDeparture = minTime;
            sort(connectionsByEdgeId[edge]);
            int64_t travelTimeSum = 0;
            for (size_t i = 0; i < connectionsByEdgeId[edge].size(); i++) {
                const Connection& c = connectionsByEdgeId[edge][i];
                if ((i + 1 < connectionsByEdgeId[edge].size()) && (connectionsByEdgeId[edge][i + 1].arrivalTime() <= c.arrivalTime())) continue;
                int64_t delta = c.departureTime - lastDeparture;
                if (delta <= 0) continue;
                if (topology.get(TravelTime, edge) < c.travelTime + delta) {
                    int64_t constTime = c.travelTime + delta - topology.get(TravelTime, edge);
                    travelTimeSum += topology.get(TravelTime, edge) * constTime;
                    delta -= constTime;
                }
                travelTimeSum += (c.travelTime + (0.5 * delta)) * delta;
                lastDeparture = c.departureTime;
                if (lastDeparture >= maxTime) break;
            }
            if (lastDeparture >= maxTime) {
                topology.set(TravelTime, edge, travelTimeSum / (maxTime - minTime));
            }
            AssertMsg(topology.get(TravelTime, edge) >= 0, "Edge " << edge << " has negative travel time!");
        }
        topology.deleteEdges([&](Edge edge){return topology.get(TravelTime, edge) >= intMax;});
        TransferGraph result;
        Graph::move(std::move(topology), result);
        return result;
    }

    inline TransferGraph minTravelTimeGraph() const noexcept {
        Intermediate::TransferGraph topology;
        topology.addVertices(transferGraph.numVertices());
        for (const RouteId route : routes()) {
            for (size_t stopIndex = firstStopIdOfRoute[route] + 1; stopIndex < firstStopIdOfRoute[route + 1]; stopIndex++) {
                if (stopIds[stopIndex - 1] == stopIds[stopIndex]) continue;
                topology.findOrAddEdge(stopIds[stopIndex - 1], stopIds[stopIndex]).set(TravelTime, intMax);
            }
        }
        for (Vertex vertex : transferGraph.vertices()) {
            topology.set(Coordinates, vertex, transferGraph.get(Coordinates, vertex));
            for (Edge edge : transferGraph.edgesFrom(vertex)) {
                if (vertex == transferGraph.get(ToVertex, edge)) continue;
                topology.findOrAddEdge(vertex, transferGraph.get(ToVertex, edge)).set(TravelTime, transferGraph.get(TravelTime, edge));
            }
        }
        topology.packEdges();
        for (const RouteId route : routes()) {
            const StopId* stops = stopArrayOfRoute(route);
            const size_t tripSize = numberOfStopsInRoute(route);
            for (size_t stopIndex = 1; stopIndex < tripSize; stopIndex++) {
                if (stops[stopIndex - 1] == stops[stopIndex]) continue;
                const Edge edge = topology.findEdge(stops[stopIndex - 1], stops[stopIndex]);
                for (const StopEvent* trip = firstTripOfRoute(route); trip <= lastTripOfRoute(route); trip += tripSize) {
                    const int travelTime = trip[stopIndex].arrivalTime - trip[stopIndex - 1].departureTime;
                    if (travelTime >= topology.get(TravelTime, edge)) continue;
                    topology.set(TravelTime, edge, travelTime);
                }
            }
        }
        topology.deleteEdges([&](Edge edge){return topology.get(TravelTime, edge) >= intMax;});
        TransferGraph result;
        Graph::move(std::move(topology), result);
        return result;
    }

    inline TransferGraph minTravelTimeTransitiveGraph() const noexcept {
        Intermediate::TransferGraph topology;
        topology.addVertices(transferGraph.numVertices());
        for (const RouteId route : routes()) {
            for (size_t toStopIndex = firstStopIdOfRoute[route] + 1; toStopIndex < firstStopIdOfRoute[route + 1]; toStopIndex++) {
                for (size_t fromStopIndex = firstStopIdOfRoute[route]; fromStopIndex < toStopIndex; fromStopIndex++) {
                    topology.findOrAddEdge(stopIds[fromStopIndex], stopIds[toStopIndex]).set(TravelTime, intMax);
                }
            }
        }
        for (Vertex vertex : transferGraph.vertices()) {
            topology.set(Coordinates, vertex, transferGraph.get(Coordinates, vertex));
            for (Edge edge : transferGraph.edgesFrom(vertex)) {
                topology.findOrAddEdge(vertex, transferGraph.get(ToVertex, edge)).set(TravelTime, transferGraph.get(TravelTime, edge));
            }
        }
        topology.packEdges();
        for (const RouteId route : routes()) {
            const StopId* stops = stopArrayOfRoute(route);
            const size_t tripSize = numberOfStopsInRoute(route);
            for (size_t toStopIndex = 1; toStopIndex < tripSize; toStopIndex++) {
                for (size_t fromStopIndex = 0; fromStopIndex < toStopIndex; fromStopIndex++) {
                    const Edge edge = topology.findEdge(stops[fromStopIndex], stops[toStopIndex]);
                    for (const StopEvent* trip = firstTripOfRoute(route); trip <= lastTripOfRoute(route); trip += tripSize) {
                        const int travelTime = trip[toStopIndex].arrivalTime - trip[fromStopIndex].departureTime;
                        if (travelTime >= topology.get(TravelTime, edge)) continue;
                        topology.set(TravelTime, edge, travelTime);
                    }
                }
            }
        }
        topology.deleteEdges([&](Edge edge){return topology.get(TravelTime, edge) >= intMax;});
        TransferGraph result;
        Graph::move(std::move(topology), result);
        return result;
    }

    inline TransferGraph minChangeTimeGraph() const noexcept {
        Intermediate::TransferGraph graph;
        graph.addVertices(transferGraph.numVertices());
        Dijkstra<TransferGraph, false> dijkstra(transferGraph, transferGraph[TravelTime]);
        for (const StopId stop : stops()) {
            const int range = stopData[stop].minTransferTime / 2;
            dijkstra.run(stop, noVertex, [&](const Vertex u) {
                if (u == stop || dijkstra.getParent(u) == stop) return;
                const int travelTime = dijkstra.getDistance(u);
                if (travelTime > range || isStop(u)) {
                    graph.addEdge(stop, u).set(TravelTime, travelTime);
                    graph.addEdge(u, stop).set(TravelTime, travelTime);
                }
            }, [&]() {
                return false;
            }, [&](const Vertex from, const Edge) {
                return dijkstra.getDistance(from) > range;
            });
        }
        graph.reduceMultiEdgesBy(TravelTime);
        TransferGraph result;
        Graph::move(std::move(graph), result);
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

    inline void applyVertexPermutation(const Permutation& permutation, const bool permutateStops = true) noexcept {
        Permutation splitPermutation = permutation.splitAt(numberOfStops());
        if (!permutateStops) {
            for (size_t i = 0; i < numberOfStops(); i++) {
                splitPermutation[i] = i;
            }
        }
        Permutation stopPermutation = splitPermutation;
        stopPermutation.resize(numberOfStops());
        permutate(splitPermutation, stopPermutation);
    }

    inline void applyVertexOrder(const Order& order, const bool permutateStops = true) noexcept {
        applyVertexPermutation(Permutation(Construct::Invert, order), permutateStops);
    }

    inline void applyStopPermutation(const Permutation& permutation) noexcept {
        permutate(permutation.extend(transferGraph.numVertices()), permutation);
    }

    inline void applyStopOrder(const Order& order) noexcept {
        applyStopPermutation(Permutation(Construct::Invert, order));
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
        std::vector<RouteId> illFormedRoutes = checkForIllFormedRoutes();
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
        if (!illFormedRoutes.empty()) {
            Enumeration text;
            for (size_t i = 0; i < illFormedRoutes.size(); i++) {
                text << illFormedRoutes[i];
                if (i % 15 == 14) {
                    text << Sep("\n                             ");
                } else {
                    text << Sep(", ");
                }
            }
            std::cout << "   Ill Formed Routes:        " << text << std::endl;
        }
    }

    inline void printRoute(const RouteId route) const noexcept {
        AssertMsg(isRoute(route), "Id = " << route << " does not represent a route!");
        std::cout << "Route " << route << ":" << std::endl << "    Index: ";
        for (size_t i = 0; i < numberOfStopsInRoute(route); i++) {
            std::cout << std::setw(10) << i;
        }
        std::cout << std::endl << "  Stop Id: ";
        for (const StopId stop : stopsOfRoute(route)) {
            std::cout << std::setw(10) << stop;
        }
        std::cout << std::endl;
        const size_t tripSize = numberOfStopsInRoute(route);
        const StopEvent* trip = firstTripOfRoute(route);
        for (size_t i = 0; i < numberOfTripsInRoute(route); i++) {
            std::cout << "Trip " << std::setw(4) << i << ": ";
            for (size_t j = 0; j < tripSize; j++) {
                std::cout << std::setw(10) << trip[j].arrivalTime;
            }
            std::cout << std::endl << "           ";
            for (size_t j = 0; j < tripSize; j++) {
                std::cout << std::setw(10) << trip[j].departureTime;
            }
            std::cout << std::endl;
            trip += tripSize;
        }
    }

    inline std::vector<RouteId> checkForIllFormedRoutes() const noexcept {
        std::vector<RouteId> illFormedRoutes;
        for (const RouteId route : routes()) {
            const StopEvent* tripA = firstTripOfRoute(route);
            const StopEvent* tripB = tripA + numberOfStopsInRoute(route);
            const StopEvent* end = tripA + numberOfStopEventsInRoute(route);
            while (tripB < end) {
                bool isEqual = true;
                bool isGreater = false;
                for (size_t i = 0; i < numberOfStopsInRoute(route); i++, tripA++, tripB++) {
                    if (tripA->arrivalTime != tripB->arrivalTime) isEqual = false;
                    if (tripA->departureTime != tripB->departureTime) isEqual = false;
                    if (tripA->arrivalTime > tripB->arrivalTime) isGreater = true;
                    if (tripA->departureTime > tripB->departureTime) isGreater = true;
                }
                if (isEqual || isGreater) {
                    illFormedRoutes.emplace_back(route);
                    break;
                }
            }
        }
        return illFormedRoutes;
    }

    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, firstRouteSegmentOfStop, firstStopIdOfRoute, firstStopEventOfRoute, routeSegments, stopIds, stopEvents, stopData, routeData, implicitDepartureBufferTimes, implicitArrivalBufferTimes);
        transferGraph.writeBinary(fileName + ".graph");
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, firstRouteSegmentOfStop, firstStopIdOfRoute, firstStopEventOfRoute, routeSegments, stopIds, stopEvents, stopData, routeData, implicitDepartureBufferTimes, implicitArrivalBufferTimes);
        transferGraph.readBinary(fileName + ".graph");
    }

    inline void writeCSV(const std::string& fileBaseName) const noexcept {
        writeStopCSV(fileBaseName + "stops.csv");
        writeLineCSV(fileBaseName + "lines.csv");
        writeTripCSV(fileBaseName + "trips.csv");
        writeFootpathCSV(fileBaseName + "footpaths.csv");
    }

    inline void writeStopCSV(const std::string& fileName) const noexcept {
        std::ofstream file(fileName);
        Assert(file);
        Assert(file.is_open());
        file << "StopId,Latitude,Longitude,MinChangeTime\n";
        if (hasImplicitBufferTimes()) {
            for (const StopId stop : stops()) {
                file << stop.value() << "," << stopData[stop].coordinates.latitude << "," << stopData[stop].coordinates.longitude << ",0\n";
            }
        } else {
            for (const StopId stop : stops()) {
                file << stop.value() << "," << stopData[stop].coordinates.latitude << "," << stopData[stop].coordinates.longitude << "," << stopData[stop].minTransferTime << "\n";
            }
        }
        file.close();
    }

    inline void writeLineCSV(const std::string& fileName) const noexcept {
        std::ofstream file(fileName);
        Assert(file);
        Assert(file.is_open());
        file << "LineId,StopIndex,StopId\n";
        for (const RouteId route : routes()) {
            const StopId* stops = stopArrayOfRoute(route);
            for (size_t i = 0; i < numberOfStopsInRoute(route); i++) {
                file << route.value() << "," << i << "," << stops[i].value() << "\n";
            }
        }
        file.close();
    }

    inline void writeTripCSV(const std::string& fileName) const noexcept {
        std::ofstream file(fileName);
        Assert(file);
        Assert(file.is_open());
        file << "LineId,TripId,StopIndex,ArrivalTime,DepartureTime\n";
        for (const RouteId route : routes()) {
            const StopEvent* stopEvents = firstTripOfRoute(route);
            const size_t tripLength = numberOfStopsInRoute(route);
            for (size_t i = 0; i < numberOfTripsInRoute(route); i++) {
                for (size_t j = 0; j < tripLength; j++) {
                    const StopEvent& stopEvent = stopEvents[(i * tripLength) + j];
                    file << route.value() << "," << i << "," << j << "," << stopEvent.arrivalTime << "," << stopEvent.departureTime << "\n";
                }
            }
        }
        file.close();
    }

    inline void writeFootpathCSV(const std::string& fileName) const noexcept {
        std::ofstream file(fileName);
        Assert(file);
        Assert(file.is_open());
        file << "FromStopId,ToStopId,TravelTime\n";
        for (const StopId from : stops()) {
            for (const Edge edge : transferGraph.edgesFrom(from)) {
                file << from.value() << "," << transferGraph.get(ToVertex, edge).value() << "," << transferGraph.get(TravelTime, edge) << "\n";
            }
        }
        file.close();
    }

    inline std::vector<TripId> mapStopEventsToTripId() const noexcept {
        std::vector<TripId> result;
        TripId currentTrip = TripId(0);
        for (const RouteId route : routes()) {
            const size_t routeLength = numberOfStopsInRoute(route);
            for (size_t i = 0; i < numberOfTripsInRoute(route); i++) {
                result.insert(result.end(), routeLength, currentTrip);
                currentTrip++;
            }
        }
        return result;
    }

    inline std::vector<RouteId> mapStopEventsToRouteId() const noexcept {
        std::vector<RouteId> result;
        for (const RouteId route : routes()) {
            result.insert(result.end(), numberOfStopEventsInRoute(route), route);
        }
        return result;
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

    inline Order rebuildRoutes() noexcept {
        Order stopEventOrder;
        stopEventOrder.reserve(stopEvents.size());
        std::vector<std::vector<std::vector<size_t>>> newRoutes(numberOfRoutes());
        std::vector<size_t> newFirstStopEventOfRoute;
        newFirstStopEventOfRoute.reserve(firstStopEventOfRoute.size());
        std::vector<std::vector<RouteSegment>> routeSegmentsOfStop(numberOfStops());

        for (const RouteId route : routes()) {
            const size_t tripLength = numberOfStopsInRoute(route);
            for (StopIndex stopIndex(0); stopIndex < tripLength; stopIndex++) {
                const StopId stop = stopsOfRoute(route)[stopIndex];
                routeSegmentsOfStop[stop].emplace_back(route, stopIndex);
            }

            newFirstStopEventOfRoute.emplace_back(stopEventOrder.size());
            const size_t numTrips = numberOfTripsInRoute(route);
            for (size_t i = 0; i < numTrips; i++) {
                const SubRange<std::vector<StopEvent>> trip = stopEventsOfTrip(route, i);
                if (i == numTrips - 1 || !overtakes(stopEventsOfTrip(route, i+1), trip)) {
                    const size_t firstStopEvent = firstStopEventOfRoute[route] + i * tripLength;
                    for (size_t e = firstStopEvent; e < firstStopEvent + tripLength; e++) {
                        stopEventOrder.emplace_back(e);
                    }
                    continue;
                }
                bool added = false;
                for (std::vector<size_t>& newRoute : newRoutes[route]) {
                    const SubRange<std::vector<StopEvent>> lastTrip = stopEventsOfTrip(route, newRoute.back());
                    if (!overtakes(trip, lastTrip)) {
                        newRoute.emplace_back(i);
                        added = true;
                        break;
                    }
                }
                if (!added) {
                    newRoutes[route].resize(newRoutes[route].size() + 1);
                    newRoutes[route].back().emplace_back(i);
                }
            }
        }

        for (RouteId route(0); route < newRoutes.size(); route++) {
            if (newRoutes[route].empty()) continue;
            const size_t tripLength = numberOfStopsInRoute(route);
            for (StopIndex stopIndex(0); stopIndex < tripLength; stopIndex++) {
                const StopId stop = stopsOfRoute(route)[stopIndex];
                for (size_t i = 0; i < newRoutes[route].size(); i++) {
                    const RouteId newRouteId(routeData.size() + i);
                    routeSegmentsOfStop[stop].emplace_back(newRouteId, stopIndex);
                }
            }
            for (const std::vector<size_t>& newRoute : newRoutes[route]) {
                newFirstStopEventOfRoute.emplace_back(stopEventOrder.size());
                routeData.emplace_back(routeData[route]);
                for (const StopId stop : stopsOfRoute(route)) {
                    stopIds.emplace_back(stop);
                }
                firstStopIdOfRoute.emplace_back(stopIds.size());
                for (const size_t i : newRoute) {
                    const size_t firstStopEvent = firstStopEventOfRoute[route] + i * tripLength;
                    for (size_t e = firstStopEvent; e < firstStopEvent + tripLength; e++) {
                        stopEventOrder.emplace_back(e);
                    }
                }
            }
        }

        newFirstStopEventOfRoute.emplace_back(stopEventOrder.size());

        firstRouteSegmentOfStop.clear();
        routeSegments.clear();
        for (const std::vector<RouteSegment>& routeSegmentList : routeSegmentsOfStop) {
            firstRouteSegmentOfStop.emplace_back(routeSegments.size());
            for (const RouteSegment& routeSegment : routeSegmentList) {
                routeSegments.emplace_back(routeSegment);
            }
        }
        firstRouteSegmentOfStop.emplace_back(routeSegments.size());

        newFirstStopEventOfRoute.swap(firstStopEventOfRoute);
        stopEventOrder.order(stopEvents);

        return stopEventOrder;
    }

private:
    inline bool overtakes(const SubRange<std::vector<StopEvent>>& tripA, const SubRange<std::vector<StopEvent>>& tripB) const noexcept {
        AssertMsg(tripA.size() == tripB.size(), "Compared trips have different lengths!");
        for (size_t i = 0; i < tripA.size(); i++) {
            if (tripA[i].arrivalTime < tripB[i].arrivalTime) return true;
            if (tripA[i].departureTime < tripB[i].departureTime) return true;
        }
        return false;
    }

    inline void permutate(const Permutation& fullPermutation, const Permutation& stopPermutation) noexcept {
        AssertMsg(fullPermutation.size() == transferGraph.numVertices(), "Full permutation size (" << fullPermutation.size() << ") must be the same as number of vertices (" << transferGraph.numVertices() << ")!");
        AssertMsg(stopPermutation.size() == numberOfStops(), "Stop permutation size (" << stopPermutation.size() << ") must be the same as number of stops (" << numberOfStops() << ")!");

        Order order(Construct::Invert, stopPermutation);
        std::vector<RouteSegment> newRouteSegments;
        std::vector<size_t> newFirstRouteSegmentOfStop;
        for (const size_t i : order) {
            newFirstRouteSegmentOfStop.emplace_back(newRouteSegments.size());
            for (const RouteSegment& segment : routesContainingStop(StopId(i))) {
                newRouteSegments.emplace_back(segment);
            }
        }
        newFirstRouteSegmentOfStop.emplace_back(newRouteSegments.size());
        firstRouteSegmentOfStop = newFirstRouteSegmentOfStop;
        routeSegments = newRouteSegments;

        stopPermutation.mapPermutation(stopIds);
        stopPermutation.permutate(stopData);

        transferGraph.applyVertexPermutation(fullPermutation);
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
