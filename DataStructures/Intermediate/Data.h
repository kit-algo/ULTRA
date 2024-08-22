#pragma once

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <map>

#include "Entities/Stop.h"
#include "Entities/StopEvent.h"
#include "Entities/Trip.h"

#include "../GTFS/Data.h"
#include "../Container/Map.h"
#include "../Container/IndexedSet.h"
#include "../Graph/Graph.h"
#include "../Geometry/Point.h"
#include "../Geometry/Rectangle.h"
#include "../Geometry/CoordinateTree.h"

#include "../../Helpers/Assert.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/IO/Serialization.h"
#include "../../Helpers/Ranges/Range.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Vector/Vector.h"
#include "../../Helpers/Vector/Permutation.h"
#include "../../Helpers/Console/Progress.h"
#include "../../Algorithms/Dijkstra/Dijkstra.h"

namespace Intermediate {

using TransferGraph = DynamicTransferGraph;

class Data {

public:
    Data() {}

    Data(const std::string& fileName) {
        deserialize(fileName);
    }

    inline static Data FromBinary(const std::string& fileName) noexcept {
        Data data;
        data.deserialize(fileName);
        return data;
    }

    inline static Data FromGTFS(const GTFS::Data& gtfs, const int startDate, const int endDate, const bool ignoreDaysOfOperation = false, const bool ignoreFrequencies = false) noexcept {
        std::cout << "Extracting timetable from " << dayToString(startDate) << " to " << dayToString(endDate) << ", " << (ignoreDaysOfOperation ? "ignoring" : ", applying") << " days of operation" << std::endl;
        Data data;
        Map<std::string, std::vector<int>> calendars = gtfs.unrollCalendarDates(startDate, endDate, ignoreDaysOfOperation);
        Map<std::string, int> routeIds = gtfs.routeIds();
        Map<std::string, int> stopIds = gtfs.stopIds();
        Map<std::string, int> tripIds = gtfs.tripIds();
        Map<std::string, std::vector<int>> frequencyIds = gtfs.frequencyIds();
        Map<std::string, std::vector<GTFS::StopTime>> stopTimeTrips;
        for (const GTFS::Trip& trip : gtfs.trips) {
            if (stopTimeTrips.contains(trip.tripId)) continue;
            if (!routeIds.contains(trip.routeId)) continue;
            if (!calendars.contains(trip.serviceId)) continue;
            stopTimeTrips.insert(trip.tripId, std::vector<GTFS::StopTime>());
        }
        for (const GTFS::StopTime& stopTime : gtfs.stopTimes) {
            if (!stopIds.contains(stopTime.stopId)) continue;
            if (!stopTimeTrips.contains(stopTime.tripId)) continue;
            stopTimeTrips[stopTime.tripId].emplace_back(stopTime);
        }
        int timeTravelTrips = 0;
        int emptyTrips = 0;
        for (const auto& stopTimeTrip : stopTimeTrips) {
            const std::string& tripId = stopTimeTrip.first;
            std::vector<GTFS::StopTime>& stopTimes = stopTimeTrips[tripId];
            std::sort(stopTimes.begin(), stopTimes.end());
            int offset = 0;
            for (size_t i = 1; i < stopTimes.size(); i++) {
                if (stopTimes[i - 1].departureTime == stopTimes[i].arrivalTime + offset) offset++;
                stopTimes[i].arrivalTime += offset;
                stopTimes[i].departureTime += offset;
                if (stopTimes[i - 1].departureTime >= stopTimes[i].arrivalTime) {
                    timeTravelTrips++;
                    stopTimes.clear();
                }
            }
            if (stopTimes.empty()) {
                emptyTrips++;
                continue;
            }
            const GTFS::Trip& trip = gtfs.trips[tripIds[tripId]];
            const GTFS::Route& route = gtfs.routes[routeIds[trip.routeId]];
            for (const int day : calendars[trip.serviceId]) {
                const int seconds = day * 24 * 60 * 60;
                if (frequencyIds.contains(tripId)) {
                    if (ignoreFrequencies) continue;
                    for (const int i : frequencyIds[tripId]) {
                        const GTFS::Frequency& frequency = gtfs.frequencies[i];
                        for (int time = frequency.startTime; time <= frequency.endTime; time += frequency.headwaySecs) {
                            data.buildTrip(gtfs, stopIds, stopTimes, seconds - stopTimes[0].departureTime + time, trip.name, route.name, route.type);
                        }
                    }
                } else {
                    data.buildTrip(gtfs, stopIds, stopTimes, seconds, trip.name, route.name, route.type);
                }
            }
        }
        emptyTrips -= timeTravelTrips;
        std::cout << "Removed " << String::prettyInt(timeTravelTrips) << " Trips, due to negative travel times." << std::endl;
        std::cout << "Removed " << String::prettyInt(emptyTrips) << " Trips, because they were empty." << std::endl;
        data.transferGraph.addVertices(data.stops.size());
        for (const StopId stop : data.stopIds()) {
            data.transferGraph.set(Coordinates, stop, data.stops[stop].coordinates);
        }
        for (const GTFS::Transfer& transfer : gtfs.transfers) {
            if (!stopIds.contains(transfer.fromStopId)) continue;
            if (!stopIds.contains(transfer.toStopId)) continue;
            const StopId fromStopId = StopId(-(stopIds[transfer.fromStopId] + 1));
            const StopId toStopId = StopId(-(stopIds[transfer.toStopId] + 1));
            if (!data.transferGraph.isVertex(fromStopId)) continue;
            if (!data.transferGraph.isVertex(toStopId)) continue;
            if (fromStopId == toStopId) {
                data.stops[fromStopId].minTransferTime = std::max(data.stops[fromStopId].minTransferTime, transfer.minTransferTime);
            } else {
                data.transferGraph.addEdge(fromStopId, toStopId).set(TravelTime, transfer.minTransferTime);
                data.transferGraph.addEdge(toStopId, fromStopId).set(TravelTime, transfer.minTransferTime);
            }
        }
        data.transferGraph.reduceMultiEdgesBy(TravelTime);
        for (const Vertex vertex : data.transferGraph.vertices()) {
            for (const Edge a : data.transferGraph.edgesFrom(vertex)) {
                const Edge b = data.transferGraph.findReverseEdge(a);
                if (data.transferGraph.get(TravelTime, a) <  data.transferGraph.get(TravelTime, b)) {
                    data.transferGraph.set(TravelTime, b, data.transferGraph.get(TravelTime, a));
                } else if (data.transferGraph.get(TravelTime, b) <  data.transferGraph.get(TravelTime, a)) {
                    data.transferGraph.set(TravelTime, a, data.transferGraph.get(TravelTime, b));
                }
            }
        }
        data.connectIsolatedStops();
        data.transferGraph.packEdges();
        data.validate();
        return data;
    }

    template<typename CSA>
    inline static Data FromCSA(const CSA& csa, const bool validate = false) noexcept {
        Intermediate::Data data;
        for (const auto& stop : csa.stopData) {
            data.stops.emplace_back(stop);
        }
        for (const auto& trip : csa.tripData) {
            data.trips.emplace_back(trip);
        }
        for (const auto& connection : csa.connections) {
            Trip& trip = data.trips[connection.tripId];
            if (trip.stopEvents.empty()) {
                trip.stopEvents.emplace_back(connection.departureStopId, connection.departureTime, connection.departureTime);
            }
            trip.stopEvents.emplace_back(connection.arrivalStopId, connection.arrivalTime, connection.arrivalTime);
            StopEvent& a = trip.stopEvents[trip.stopEvents.size() - 2];
            StopEvent& b = trip.stopEvents[trip.stopEvents.size() - 1];
            a.departureTime = std::max(a.arrivalTime, connection.departureTime);
            b.arrivalTime = std::max(a.departureTime + 1, b.arrivalTime);
            b.departureTime = std::max(b.arrivalTime, b.departureTime);
        }
        auto graph = csa.transferGraph;
        Graph::move(std::move(graph), data.transferGraph);
        if (validate) data.validate();
        return data;
    }

    template<typename RAPTOR>
    inline static Data FromRAPTOR(const RAPTOR& raptor, const bool validate = false) noexcept {
        Intermediate::Data data;
        RAPTOR raptorCopy = raptor;
        raptorCopy.dontUseImplicitDepartureBufferTimes();
        raptorCopy.dontUseImplicitArrivalBufferTimes();
        for (const auto& stop : raptorCopy.stopData) {
            data.stops.emplace_back(stop);
        }
        for (const auto& route : raptorCopy.routes()) {
            for (size_t i = 0; i < raptorCopy.numberOfTripsInRoute(route); i++) {
                data.trips.emplace_back("", raptorCopy.routeData[route].name, raptorCopy.routeData[route].type);
                for (StopIndex stopIndex(0); stopIndex < raptorCopy.numberOfStopsInRoute(route); stopIndex++) {
                    const StopId stop = raptorCopy.stopArrayOfRoute(route)[stopIndex];
                    const auto& stopEvent = raptorCopy.tripOfRoute(route, i)[stopIndex];
                    data.trips.back().stopEvents.emplace_back(stop, stopEvent.arrivalTime, stopEvent.departureTime);
                }
            }
        }
        Graph::move(std::move(raptorCopy.transferGraph), data.transferGraph);
        if (validate) data.validate();
        return data;
    }

protected:
    inline void buildTrip(const GTFS::Data& gtfs, Map<std::string, int>& stopIds, const std::vector<GTFS::StopTime>& stopTimes, const int offset, const std::string& tripName, const std::string& routeName, const int type) {
        trips.emplace_back(tripName, routeName, type);
        Trip& trip = trips.back();
        for (const GTFS::StopTime& stopTime : stopTimes) {
            int& stopId = stopIds[stopTime.stopId];
            if (stopId >= 0) {
                stops.emplace_back(gtfs.stops[stopId]);
                stopId = -stops.size();
            }
            trip.stopEvents.emplace_back(StopId(-(stopId + 1)), stopTime.arrivalTime + offset, stopTime.departureTime + offset);
        }
        if (trip.stopEvents.empty()) {
            trips.pop_back();
        } else {
            trip.stopEvents.front().arrivalTime = trip.stopEvents.front().departureTime - 1;
            trip.stopEvents.back().departureTime = trip.stopEvents.back().arrivalTime + 1;
        }
    }

public:
    inline size_t numberOfStops() const noexcept {return stops.size();}
    inline bool isStop(const Vertex stop) const noexcept {return stop < numberOfStops();}
    inline Range<StopId> stopIds() const noexcept {return Range<StopId>(StopId(0), StopId(numberOfStops()));}

    inline size_t numberOfTrips() const noexcept {return trips.size();}
    inline bool isTrip(const TripId trip) const noexcept {return trip < numberOfTrips();}
    inline Range<TripId> tripIds() const noexcept {return Range<TripId>(TripId(0), TripId(numberOfTrips()));}

    inline void connectIsolatedStops(const double maxConnectingDistanceInCM = 1000, const double speedInKMH = 4.5) noexcept {
        std::cout << "Connecting Isolated Stops." << std::endl;
        size_t edgeCount = 0;
        CoordinateTree<Geometry::GeoMetric> ct(Geometry::GeoMetric(), transferGraph[Coordinates]);
        for (const StopId stop : stopIds()) {
            if (transferGraph.outDegree(stop) > 0) continue;
            for (const Vertex other : ct.getNeighbors(transferGraph.get(Coordinates, stop), maxConnectingDistanceInCM)) {
                if (other == stop) continue;
                const double distance = std::max(1.0, Geometry::geoDistanceInCM(transferGraph.get(Coordinates, stop), transferGraph.get(Coordinates, other)));
                Assert(distance <= maxConnectingDistanceInCM, "CoordinateTree returned a neighbor with distance " << distance << " > " << maxConnectingDistanceInCM << "!");
                const double travelTime = (distance / speedInKMH) * 0.036;
                transferGraph.addEdge(other, stop).set(TravelTime, travelTime);
                transferGraph.addEdge(stop, other).set(TravelTime, travelTime);
                edgeCount++;
            }
        }
        transferGraph.packEdges();
        std::cout << "   added " << String::prettyInt(edgeCount) << " edges." << std::endl;
    }

    inline void makeImpassableVertices() noexcept {
        TransferGraph newGraph;
        size_t isolatedVertices = 0;
        for (const Vertex vertex : transferGraph.vertices()) {
            if (isStop(vertex)) {
                newGraph.set(Coordinates, newGraph.addVertex(), transferGraph.get(Coordinates, vertex));
            } else {
                newGraph.set(Coordinates, newGraph.addVertex(), transferGraph.get(Coordinates, vertex));
                newGraph.set(Coordinates, newGraph.addVertex(), transferGraph.get(Coordinates, vertex));
                if (transferGraph.inDegree(vertex) == 0 || transferGraph.outDegree(vertex) == 0) {
                    isolatedVertices++;
                }
            }
        }
        std::cout << "   Number of isolated vertices: " << String::prettyInt(isolatedVertices) << std::endl;
        for (const Vertex vertex : transferGraph.vertices()) {
            const Vertex from = (isStop(vertex)) ? (vertex) : (Vertex((vertex * 2) - numberOfStops()));
            for (const Edge edge : transferGraph.edgesFrom(vertex)) {
                Vertex to = transferGraph.get(ToVertex, edge);
                to = (isStop(to)) ? (to) : (Vertex((to * 2) - numberOfStops() + 1));
                newGraph.addEdge(from, to).set(TravelTime, transferGraph.get(TravelTime, edge));
            }
        }
        transferGraph = std::move(newGraph);
    }

    inline void addTransferGraph(const TransferGraph& graph, const double maxUnificationDistanceInCM = 500, const double maxConnectingDistanceInCM = 10000, const double speedInKMH = 4.5, const bool connectStopsOnly = false) noexcept {
        std::cout << "Matching stops to nearest graph vertices." << std::endl;
        Geometry::Rectangle boundingBox = Geometry::Rectangle::BoundingBox(graph[Coordinates]);
        Geometry::GeoMetricAproximation metric = Geometry::GeoMetricAproximation::ComputeCorrection(boundingBox.center());
        CoordinateTree<Geometry::GeoMetricAproximation> ct(metric, graph[Coordinates]);
        std::vector<std::vector<Vertex>> nearestOldVerticesOfNewVertex = std::vector<std::vector<Vertex>>(graph.numVertices());
        std::vector<double> distanceToNewVertex(transferGraph.numVertices(), maxConnectingDistanceInCM);
        for (const Vertex oldVertex : transferGraph.vertices()) {
            if (connectStopsOnly && !isStop(oldVertex)) break;
            const Vertex newVertex = ct.getNearestNeighbor(transferGraph.get(Coordinates, oldVertex));
            const double distance = Geometry::geoDistanceInCM(transferGraph.get(Coordinates, oldVertex), graph.get(Coordinates, newVertex));
            if (distance <= maxConnectingDistanceInCM) {
                nearestOldVerticesOfNewVertex[newVertex].emplace_back(oldVertex);
                distanceToNewVertex[oldVertex] = distance;
            }
        }
        ct.clear();
        std::cout << "Adding edges to the transfer graph." << std::endl;
        std::vector<Vertex> oldVertexOfNewVertex(graph.numVertices());
        for (const Vertex newVertex : graph.vertices()) {
            sort(nearestOldVerticesOfNewVertex[newVertex], [&](const Vertex a, const Vertex b) {
               return distanceToNewVertex[a] < distanceToNewVertex[b];
            });
            Vertex from;
            if (nearestOldVerticesOfNewVertex[newVertex].empty() || distanceToNewVertex[nearestOldVerticesOfNewVertex[newVertex][0]] > maxUnificationDistanceInCM) {
                from = transferGraph.addVertex();
                transferGraph.set(Coordinates, from, graph.get(Coordinates, newVertex));
            } else {
                from = nearestOldVerticesOfNewVertex[newVertex][0];
            }
            oldVertexOfNewVertex[newVertex] = from;
            for (size_t i = 0; i < nearestOldVerticesOfNewVertex[newVertex].size(); i++) {
                const Vertex oldVertex = nearestOldVerticesOfNewVertex[newVertex][i];
                if (i > 0 || distanceToNewVertex[oldVertex] > maxUnificationDistanceInCM) {
                    if (distanceToNewVertex[oldVertex] <= maxConnectingDistanceInCM) {
                        const double travelTime = (distanceToNewVertex[oldVertex] / speedInKMH) * 0.036;
                        transferGraph.addEdge(from, oldVertex).set(TravelTime, travelTime);
                        transferGraph.addEdge(oldVertex, from).set(TravelTime, travelTime);
                    }
                }
            }
        }
        for (const Vertex newVertex : graph.vertices()) {
            for (const Edge edge : graph.edgesFrom(newVertex)) {
                const Vertex from = oldVertexOfNewVertex[newVertex];
                const Vertex to = oldVertexOfNewVertex[graph.get(ToVertex, edge)];
                const double travelTime = graph.get(TravelTime, edge);
                transferGraph.addEdge(from, to).set(TravelTime, travelTime);
                transferGraph.addEdge(to, from).set(TravelTime, travelTime);
            }
        }
        std::cout << "Reducing multi edges." << std::endl;
        connectIsolatedStops(maxConnectingDistanceInCM, speedInKMH);
        transferGraph.reduceMultiEdgesBy(TravelTime);
        transferGraph.packEdges();
        if (!connectStopsOnly) validate();
    }

    inline size_t contractDegreeTwoVertices() noexcept {
        std::cout << "Reducing multi edges." << std::endl;
        transferGraph.reduceMultiEdgesBy(TravelTime);
        std::cout << "Contracting vertices with degree less than three." << std::endl;
        std::vector<bool> deleteVertex(transferGraph.numVertices(), false);
        std::vector<Vertex> vertices;
        for (const Vertex vertex : transferGraph.vertices()) {
            if (isStop(vertex)) continue;
            if (transferGraph.outDegree(vertex) > 2) continue;
            vertices.emplace_back(vertex);
        }
        size_t deletionCount = 0;
        for (size_t i = 0; i < vertices.size(); i++) {
            const Vertex vertex = vertices[i];
            if (isStop(vertex)) continue;
            if (transferGraph.outDegree(vertex) == 0) {
                deleteVertex[vertex] = true;
                deletionCount++;
            } else if (transferGraph.outDegree(vertex) == 1) {
                const Vertex n = transferGraph.get(ToVertex, transferGraph.edgesFrom(vertex)[0]);
                transferGraph.isolateVertex(vertex);
                if (transferGraph.outDegree(n) <= 2) vertices.emplace_back(n);
                deleteVertex[vertex] = true;
                deletionCount++;
            } else if (transferGraph.outDegree(vertex) == 2) {
                const Edge a = transferGraph.edgesFrom(vertex)[0];
                const Edge b = transferGraph.edgesFrom(vertex)[1];
                const Vertex from = transferGraph.get(ToVertex, a);
                const Vertex to = transferGraph.get(ToVertex, b);
                if (transferGraph.hasEdge(from, to)) {
                    const Edge c = transferGraph.findEdge(from, to);
                    const Edge d = transferGraph.findEdge(to, from);
                    const double travelTime = std::min(transferGraph.get(TravelTime, c), transferGraph.get(TravelTime, a) + transferGraph.get(TravelTime, b));
                    transferGraph.set(TravelTime, c, travelTime);
                    transferGraph.set(TravelTime, d, travelTime);
                } else {
                    const double travelTime = transferGraph.get(TravelTime, a) + transferGraph.get(TravelTime, b);
                    transferGraph.addEdge(from, to).set(TravelTime, travelTime);
                    transferGraph.addEdge(to, from).set(TravelTime, travelTime);
                }
                transferGraph.isolateVertex(vertex);
                if (transferGraph.outDegree(from) <= 2) vertices.emplace_back(from);
                if (transferGraph.outDegree(to) <= 2) vertices.emplace_back(to);
                deleteVertex[vertex] = true;
                deletionCount++;
            }
        }
        transferGraph.deleteVertices(deleteVertex, true);
        transferGraph.packEdges();
        validate();
        std::cout << "   removed " << String::prettyInt(deletionCount) << " vertices." << std::endl;
        return deletionCount;
    }

    inline void applyMaxSpeed(const double speedInKMH = 4.5) noexcept {
        Graph::computeTravelTimes(transferGraph, speedInKMH, true);
    }

    inline void applySpeed(const double speedInKMH) noexcept {
        Graph::computeTravelTimes(transferGraph, speedInKMH, false);
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

    inline void applyBoundingBox(const Geometry::Rectangle& boundingBox) noexcept {
        deleteVertices([&](const Vertex vertex){
            return !boundingBox.contains(transferGraph.get(Coordinates, vertex));
        });
    }

    template<typename DELETE_VERTEX>
    inline void deleteVertices(const DELETE_VERTEX& deleteVertex) noexcept {
        for (size_t i = 0; i < trips.size(); i++) {
            bool deleteTrip = false;
            for (const StopEvent& stopEvent : trips[i].stopEvents) {
                if (deleteVertex(stopEvent.stopId)) {
                    deleteTrip = true;
                    break;
                }
            }
            if (deleteTrip) {
                trips[i] = trips.back();
                trips.pop_back();
                i--;
            }
        }
        for (const Vertex vertex : transferGraph.vertices()) {
            if (deleteVertex(vertex)) {
                transferGraph.isolateVertex(vertex);
            }
        }
        validate();
    }

    inline void makeDirectTransfers(const double maxTransferTravelTime, const bool verbose = false) noexcept {
        TransferGraph graph;
        graph.addVertices(stops.size());
        Dijkstra<TransferGraph, false> dijkstra(transferGraph, transferGraph[TravelTime]);
        Progress progress(stops.size(), verbose);
        for (const StopId stop : stopIds()) {
            graph.set(Coordinates, stop, stops[stop].coordinates);
            dijkstra.run(stop, noVertex, [&](const Vertex u) {
                if (u >= stop) return;
                const int travelTime = dijkstra.getDistance(u);
                graph.addEdge(stop, u).set(TravelTime, travelTime);
                graph.addEdge(u, stop).set(TravelTime, travelTime);
            }, [&]() {
                return dijkstra.getDistance(dijkstra.getQFront()) > maxTransferTravelTime;
            });
            progress++;
        }
        graph.packEdges();
        Graph::move(std::move(graph), transferGraph);
        validate();
        if (verbose) std::cout << " done." << std::endl;
    }

    inline void makeDirectTransfersByGeoDistance(const double maxDistance, const double speedInKMH, const bool verbose = false) noexcept {
        TransferGraph graph;
        graph.addVertices(stops.size());
        Progress progress(stops.size(), verbose);
        for (const StopId from : stopIds()) {
            graph.set(Coordinates, from, stops[from].coordinates);
            for (const StopId to : stopIds()) {
                const Geometry::Point& a = stops[from].coordinates;
                const Geometry::Point& b = stops[to].coordinates;
                const double distance = Geometry::geoDistanceInCM(a, b);
                if (distance <= maxDistance) {
                    const int travelTime = (distance / speedInKMH) * 0.036;
                    graph.addEdge(from, to).set(TravelTime, travelTime);
                }
            }
            progress++;
        }
        graph.packEdges();
        Graph::move(std::move(graph), transferGraph);
        validate();
        if (verbose) std::cout << " done." << std::endl;
    }

    inline void duplicateTrips(const int timeOffset = 24 * 60 * 60) noexcept {
        const size_t oldTripCount = trips.size();
        for (size_t i = 0; i < oldTripCount; i++) {
            trips.emplace_back(trips[i], timeOffset);
        }
    }

    inline void validate() noexcept {
        std::vector<bool> deleteVertex(transferGraph.numVertices(), true);
        std::vector<bool> hasStopEvents(numberOfStops(), false);
        int invalidTrips = 0;
        for (size_t i = 0; i < trips.size(); i++) {
            bool deleteTrip = trips[i].stopEvents.size() < 2;
            for (const StopEvent& stopEvent : trips[i].stopEvents) {
                if (stopEvent.stopId >= stops.size()) {
                    deleteTrip = true;
                    break;
                }
            }
            if (deleteTrip) {
                trips[i] = trips.back();
                trips.pop_back();
                invalidTrips++;
                i--;
            } else {
                for (const StopEvent& stopEvent : trips[i].stopEvents) {
                    deleteVertex[stopEvent.stopId] = false;
                    hasStopEvents[stopEvent.stopId] = true;
                }
            }
        }
        std::sort(trips.begin(), trips.end());
        std::vector<size_t> duplicateTrips;
        for (size_t i = 1; i < trips.size(); i++) {
            if (trips[i].dominates(trips[i - 1])) {
                duplicateTrips.emplace_back(i - 1);
            }
        }
        for (size_t i = duplicateTrips.size() - 1; i < duplicateTrips.size(); i--) {
            trips[duplicateTrips[i]] = trips.back();
            trips.pop_back();
        }
        for (const Edge edge : transferGraph.edges()) {
            if (transferGraph.get(FromVertex, edge) == transferGraph.get(ToVertex, edge)) continue;
            deleteVertex[transferGraph.get(FromVertex, edge)] = false;
            deleteVertex[transferGraph.get(ToVertex, edge)] = false;
        }
        std::vector<size_t> newStops;
        std::vector<size_t> oldStops;
        std::vector<size_t> newVertices;
        std::vector<size_t> deletions;
        for (const StopId stop : stopIds()) {
            if (hasStopEvents[stop]) {
                newStops.emplace_back(stop);
            } else {
                oldStops.emplace_back(stop);
                if (deleteVertex[stop]) {
                    deletions.emplace_back(stop);
                } else {
                    newVertices.emplace_back(stop);
                }
            }
        }
        for (Vertex vertex = Vertex(numberOfStops()); vertex < transferGraph.numVertices(); vertex++) {
            if (deleteVertex[vertex]) {
                deletions.emplace_back(vertex);
            } else {
                newVertices.emplace_back(vertex);
            }
        }
        Order order = Order(newStops + newVertices + deletions);
        Permutation permutation(Construct::Invert, order);
        transferGraph.applyVertexPermutation(permutation);
        transferGraph.deleteVertices([&](Vertex vertex){return vertex >= newStops.size() + newVertices.size();});
        if (invalidTrips > 0) std::cout << "Deleting " << invalidTrips << " trips, because they contain invalid stops or stop events!" << std::endl;
        if (duplicateTrips.size() > 0) std::cout << "Deleting " << duplicateTrips.size() << " trips, because they are duplicates!" << std::endl;
        if (oldStops.size() > 0) std::cout << "Deleting " << oldStops.size() << " stops, because they are not used by any Trip/Connection!" << std::endl;
        if (deletions.size() > 0) std::cout << "Deleting " << deletions.size() << " vertices, because they are isolated!" << std::endl;
        order = Order(newStops + oldStops);
        permutation = Permutation(Construct::Invert, order);
        permutation.permutate(stops);
        stops.resize(newStops.size());
        for (size_t i = 0; i < trips.size(); i++) {
            for (StopEvent& stopEvent : trips[i].stopEvents) {
                stopEvent.stopId = permutation.permutate(stopEvent.stopId);
            }
        }
        transferGraph.packEdges();
        for (const StopId stop : stopIds()) {
            Assert(stops[stop].coordinates == transferGraph.get(Coordinates, stop), "Transit stop " << stop << " does no match with its transfer vertex!");
        }
    }

private:
    template<typename SAME_ROUTE>
    inline void appendRoutes(std::vector<std::vector<Intermediate::Trip>>& routes, const std::vector<Intermediate::Trip>& trips, const SAME_ROUTE& sameRoute) const noexcept {
        size_t j = routes.size();
        routes.emplace_back(std::vector<Intermediate::Trip>{trips[0]});
        for (size_t i = 1; i < trips.size(); i++) {
            if (equals(trips[i].stopEvents, trips[i - 1].stopEvents)) {
                bool added = false;
                for (size_t k = j; k < routes.size(); k++) {
                    if (sameRoute(routes[k].back(), trips[i])) {
                        routes[k].emplace_back(trips[i]);
                        added = true;
                        break;
                    }
                }
                if (!added) {
                    routes.emplace_back(std::vector<Intermediate::Trip>{trips[i]});
                }
            } else {
                j = routes.size();
                routes.emplace_back(std::vector<Intermediate::Trip>{trips[i]});
            }
        }
    }

public:
    inline std::vector<std::vector<Intermediate::Trip>> fifoRoutes() const noexcept {
        std::vector<Intermediate::Trip> sortedTrips = trips;
        std::sort(sortedTrips.begin(), sortedTrips.end());
        std::vector<std::vector<Intermediate::Trip>> routes;
        appendRoutes(routes, sortedTrips, [](const Trip& a, const Trip& b){return isFiFo(a, b);});
        return routes;
    }

    inline TransferGraph minTravelTimeGraph() const noexcept {
        TransferGraph result;
        result.addVertices(transferGraph.numVertices());
        for (const Trip& trip : trips) {
            for (size_t i = 1; i < trip.stopEvents.size(); i++) {
                if (trip.stopEvents[i - 1].stopId == trip.stopEvents[i].stopId) continue;
                const size_t numEdges = result.numEdges();
                const Edge newEdge = result.findOrAddEdge(trip.stopEvents[i - 1].stopId, trip.stopEvents[i].stopId);
                if (result.numEdges() != numEdges) {
                    result.set(TravelTime, newEdge, intMax);
                }
                result.set(TravelTime, newEdge, std::min(result.get(TravelTime, newEdge), trip.stopEvents[i].arrivalTime - trip.stopEvents[i - 1].departureTime));
            }
        }
        for (Vertex from : transferGraph.vertices()) {
            result.set(Coordinates, from, transferGraph.get(Coordinates, from));
            for (Edge edge : transferGraph.edgesFrom(from)) {
                const Vertex to = transferGraph.get(ToVertex, edge);
                if (from == to) continue;
                const size_t numEdges = result.numEdges();
                const Edge newEdge = result.findOrAddEdge(from, to);
                if (result.numEdges() != numEdges) {
                    result.set(TravelTime, newEdge, intMax);
                }
                result.set(TravelTime, newEdge, std::min(result.get(TravelTime, newEdge), transferGraph.get(TravelTime, edge)));
            }
        }
        result.deleteEdges([&](Edge edge){return result.get(TravelTime, edge) >= intMax;});
        result.packEdges();
        return result;
    }

    inline void scaleTimes(const double factor) noexcept {
        for (const Edge edge : transferGraph.edges()) {
            transferGraph.set(TravelTime, edge, transferGraph.get(TravelTime, edge) * factor);
        }
        for (Trip& trip : trips) {
            for (StopEvent& stopEvent : trip.stopEvents) {
                stopEvent.departureTime *= factor;
                stopEvent.arrivalTime *= factor;
            }
        }
        for (Stop& stop : stops) {
            stop.minTransferTime *= factor;
        }

    }

    inline const std::vector<Geometry::Point>& getCoordinates() const noexcept {
        return transferGraph[Coordinates];
    }

    inline Geometry::Rectangle boundingBox() const noexcept {
        Geometry::Rectangle result = Geometry::Rectangle::Empty();
        for (const Stop& stop : stops) {
            result.extend(stop.coordinates);
        }
        return result;
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

    inline void printInfo() const noexcept {
        size_t numberOfStopEvents = 0;
        size_t numberOfEmptyTrips = 0;
        int firstDay = std::numeric_limits<int>::max();
        int lastDay = std::numeric_limits<int>::min();
        std::vector<size_t> stopEventsPerStop(numberOfStops(), 0);
        std::vector<size_t> stopEventsPerDay;
        for (const Trip& trip : trips) {
            Assert(trip.stopEvents.size() >= 2, "Trip contains an insufficient number of stops!");
            numberOfStopEvents += trip.stopEvents.size();
            if (firstDay > trip.stopEvents.front().departureTime) firstDay = trip.stopEvents.front().departureTime;
            if (lastDay < trip.stopEvents.back().arrivalTime) lastDay = trip.stopEvents.back().arrivalTime;
            if (trip.stopEvents.empty()) numberOfEmptyTrips++;
            for (const StopEvent& event : trip.stopEvents) {
                stopEventsPerStop[event.stopId]++;
                const size_t day = event.departureTime / (60 * 60 * 24);
                if (day >= 14) continue;
                if (stopEventsPerDay.size() <= day) stopEventsPerDay.resize(day + 1);
                stopEventsPerDay[day]++;
            }
        }
        std::cout << "Intermediate public transit data:" << std::endl;
        std::cout << "   Number of Stops:          " << std::setw(12) << String::prettyInt(stops.size()) << std::endl;
        std::cout << "   Number of Trips:          " << std::setw(12) << String::prettyInt(trips.size()) << std::endl;
        std::cout << "   Number of Stop Events:    " << std::setw(12) << String::prettyInt(numberOfStopEvents) << std::endl;
        std::cout << "   Number of Connections:    " << std::setw(12) << String::prettyInt(numberOfStopEvents - trips.size()) << std::endl;
        std::cout << "   Number of Vertices:       " << std::setw(12) << String::prettyInt(transferGraph.numVertices()) << std::endl;
        std::cout << "   Number of Edges:          " << std::setw(12) << String::prettyInt(transferGraph.numEdges()) << std::endl;
        std::cout << "   First Day:                " << std::setw(12) << String::prettyInt(firstDay / (60 * 60 * 24)) << std::endl;
        std::cout << "   Last Day:                 " << std::setw(12) << String::prettyInt(lastDay / (60 * 60 * 24)) << std::endl;
        for (size_t day = 0; day < stopEventsPerDay.size(); day++) {
            std::cout << "      Stop Events at day " << std::setw(4) << std::left << (std::to_string(day) + ":") << std::right << std::setw(12) << String::prettyInt(stopEventsPerDay[day]) << std::endl;
        }
        std::cout << "   Min Stop Events per Stop: " << std::setw(12) << String::prettyInt(Vector::min(stopEventsPerStop)) << std::endl;
        std::cout << "   Number of empty Stops:    " << std::setw(12) << String::prettyInt(Vector::count(stopEventsPerStop, size_t(0))) << std::endl;
        std::cout << "   Number of empty Trips:    " << std::setw(12) << String::prettyInt(numberOfEmptyTrips) << std::endl;
        std::cout << "   Bounding Box:             " << std::setw(12) << boundingBox() << std::endl;
        std::vector<int> componentOfStop(numberOfStops(), -1);
        std::vector<int> sizeOfComponent;
        int numberOfIsolatedStops = 0;
        int maxComponentSize = 0;
        Dijkstra<TransferGraph> dijkstra(transferGraph, transferGraph[TravelTime]);
        for (const StopId stop : stopIds()) {
            if (componentOfStop[stop] == -1) {
                const int component = sizeOfComponent.size();
                sizeOfComponent.emplace_back(0);
                dijkstra.run(stop, noVertex, [&](const Vertex u) {
                    if (isStop(u)) {
                        componentOfStop[u] = component;
                        sizeOfComponent[component]++;
                    }
                });
                if (sizeOfComponent[component] == 1) {
                    numberOfIsolatedStops++;
                }
                if (maxComponentSize < sizeOfComponent[component]) {
                    maxComponentSize = sizeOfComponent[component];
                }
            }
        }
        std::cout << "Transfer graph statistics:" << std::endl;
        std::cout << "   Number of Components:     " << std::setw(12) << String::prettyInt(sizeOfComponent.size()) << std::endl;
        std::cout << "   Maximum Component Size:   " << std::setw(12) << String::prettyInt(maxComponentSize) << std::endl;
        std::cout << "   Number of isolated Stops: " << std::setw(12) << String::prettyInt(numberOfIsolatedStops) << std::endl;
        Graph::printInfo(transferGraph);
        transferGraph.printAnalysis();
    }

    inline void printTrip(const TripId trip) const noexcept {
        Assert(trip < trips.size(), "Trip id = " << trip << "is out of bounds (0, " << trips.size() << ")!");
        std::cout << trips[trip] << std::endl;
        for (const StopEvent& stopEvent : trips[trip].stopEvents) {
            std::cout << "   " << stopEvent << std::endl;
        }
    }

    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, stops, trips);
        transferGraph.writeBinary(fileName + ".graph");
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, stops, trips);
        transferGraph.readBinary(fileName + ".graph");
    }

private:
    inline void permutate(const Permutation& fullPermutation, const Permutation& stopPermutation) noexcept {
        Assert(fullPermutation.size() == transferGraph.numVertices(), "Full permutation size (" << fullPermutation.size() << ") must be the same as number of vertices (" << transferGraph.numVertices() << ")!");
        Assert(stopPermutation.size() == numberOfStops(), "Stop permutation size (" << stopPermutation.size() << ") must be the same as number of stops (" << numberOfStops() << ")!");

        stopPermutation.permutate(stops);
        for (Trip& trip : trips) {
            trip.applyStopPermutation(stopPermutation);
        }

        transferGraph.applyVertexPermutation(fullPermutation);
    }

public:
    std::vector<Stop> stops;
    std::vector<Trip> trips;

    TransferGraph transferGraph;

};

}
