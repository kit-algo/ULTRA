#pragma once

#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <set>

#include "Entities/Connection.h"
#include "Entities/Stop.h"
#include "Entities/Trip.h"
#include "Entities/Journey.h"

#include "../Intermediate/Data.h"
#include "../Graph/Graph.h"
#include "../Geometry/Rectangle.h"

#include "../../Helpers/IO/Serialization.h"
#include "../../Helpers/IO/ParserCSV.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Vector/Permutation.h"
#include "../../Helpers/Console/Progress.h"
#include "../../Helpers/Console/ProgressBar.h"
#include "../../Helpers/FileSystem/FileSystem.h"
#include "../../Helpers/Ranges/Range.h"
#include "../../DataStructures/Container/Map.h"
#include "../../Algorithms/Dijkstra/Dijkstra.h"

namespace CSA {

using TransferGraph = ::TransferGraph;

class Data {

private:
    Data() { }

public:
    Data(const std::string& fileName) {
        deserialize(fileName);
    }

    inline static Data FromBinary(const std::string& fileName) noexcept {
        Data data;
        data.deserialize(fileName);
        return data;
    }

    inline static Data FromIntermediate(const Intermediate::Data& inter) noexcept {
        Data data;
        for (const Intermediate::Stop& stop : inter.stops) {
            data.stopData.emplace_back(stop);
        }
        for (const Intermediate::Trip& trip : inter.trips) {
            Assert(!trip.stopEvents.empty(), "Intermediate data contains trip without any stop event!");
            for (size_t i = 1; i < trip.stopEvents.size(); i++) {
                const Intermediate::StopEvent& from = trip.stopEvents[i - 1];
                const Intermediate::StopEvent& to = trip.stopEvents[i];
                data.connections.emplace_back(from.stopId, to.stopId, from.departureTime, to.arrivalTime, TripId(data.tripData.size()));
            }
            data.tripData.emplace_back(trip);
        }
        std::sort(data.connections.begin(), data.connections.end());
        Intermediate::TransferGraph transferGraph = inter.transferGraph;
        Graph::printInfo(transferGraph);
        transferGraph.printAnalysis();
        Graph::move(std::move(transferGraph), data.transferGraph);
        return data;
    }

    template<bool MAKE_BIDIRECTIONAL = true, typename GRAPH_TYPE>
    inline static Data FromInput(const std::vector<Stop>& stops, const std::vector<Connection>& connections, const std::vector<Trip>& trips, GRAPH_TYPE transferGraph) noexcept {
        Assert(transferGraph.numVertices() >= stops.size(), "Network containes " << stops.size() << " stops, but transfer graph has only " << transferGraph.numVertices() << " vertices!");
        Data data;
        data.stopData = stops;
        data.tripData = trips;
        for (const Connection con : connections) {
            if (con.departureStopId >= data.stopData.size() || data.stopData[con.departureStopId].minTransferTime < 0) continue;
            if (con.arrivalStopId >= data.stopData.size() || data.stopData[con.arrivalStopId].minTransferTime < 0) continue;
            if (con.departureTime > con.arrivalTime) continue;
            if (con.tripId >= data.tripData.size()) continue;
            data.connections.emplace_back(con);
        }
        Intermediate::TransferGraph graph;
        Graph::move(std::move(transferGraph), graph);
        if constexpr (MAKE_BIDIRECTIONAL) graph.makeBidirectional();
        graph.reduceMultiEdgesBy(TravelTime);
        graph.packEdges();
        Graph::move(std::move(graph), data.transferGraph);
        return data;
    }

public:
    inline size_t numberOfStops() const noexcept {return stopData.size();}
    inline bool isStop(const Vertex stop) const noexcept {return stop < numberOfStops();}
    inline Range<StopId> stops() const noexcept {return Range<StopId>(StopId(0), StopId(numberOfStops()));}

    inline size_t numberOfTrips() const noexcept {return tripData.size();}
    inline bool isTrip(const TripId tripId) const noexcept {return tripId < numberOfTrips();}
    inline Range<TripId> tripIds() const noexcept {return Range<TripId>(TripId(0), TripId(numberOfTrips()));}

    inline size_t numberOfConnections() const noexcept {return connections.size();}
    inline bool isConnection(const ConnectionId connectionId) const noexcept {return connectionId < numberOfConnections();}
    inline Range<ConnectionId> connectionIds() const noexcept {return Range<ConnectionId>(ConnectionId(0), ConnectionId(numberOfConnections()));}

    inline int minTransferTime(const StopId stop) const noexcept {return stopData[stop].minTransferTime;}

    inline Geometry::Rectangle boundingBox() const noexcept {
        Geometry::Rectangle result = Geometry::Rectangle::Empty();
        for (const Stop& stop : stopData) {
            result.extend(stop.coordinates);
        }
        return result;
    }

    inline void sortConnectionsAscending() noexcept {
        std::stable_sort(connections.begin(), connections.end());
    }

    inline void sortConnectionsDescending() noexcept {
        std::stable_sort(connections.begin(), connections.end(), [](const Connection& a, const Connection& b){
            return b < a;
        });
    }

    inline void sortConnectionsAscendingByDepartureTime() noexcept {
        std::stable_sort(connections.begin(), connections.end(), [](const Connection& a, const Connection& b){
            return a.departureTime < b.departureTime;
        });
    }

    inline void sortConnectionsAscendingByArrivalTime() noexcept {
        std::stable_sort(connections.begin(), connections.end(), [](const Connection& a, const Connection& b){
            return a.arrivalTime < b.arrivalTime;
        });
    }

    inline void sortConnectionsDescendingByDepartureTime() noexcept {
        std::stable_sort(connections.begin(), connections.end(), [](const Connection& a, const Connection& b){
            return a.departureTime > b.departureTime;
        });
    }

    inline void sortConnectionsDescendingByArrivalTime() noexcept {
        std::stable_sort(connections.begin(), connections.end(), [](const Connection& a, const Connection& b){
            return a.arrivalTime > b.arrivalTime;
        });
    }

    inline void sortUnique() noexcept {
        std::sort(connections.begin(), connections.end(), [](const Connection& a, const Connection& b){
            return std::make_tuple(a.departureTime, a.arrivalTime, a.departureStopId, a.arrivalStopId, a.tripId) < std::make_tuple(b.departureTime, b.arrivalTime, b.departureStopId, b.arrivalStopId, b.tripId);
        });
    }

    inline bool isCombinable(const Vertex source, const int departureTime, const Vertex target, const int arrivalTime = intMax) const noexcept {
        Assert(transferGraph.isVertex(source), "Source vertex id " << source << " does not represent a vertex!");
        Assert(transferGraph.isVertex(target), "Target vertex id " << target << " does not represent a vertex!");
        if (source == target) {
            return departureTime <= arrivalTime;
        } else {
            Edge transferEdge = transferGraph.findEdge(source, target);
            if (!transferGraph.isEdge(transferEdge)) return false;
            return departureTime + transferGraph.get(TravelTime, transferEdge) <= arrivalTime;
        }
    }

    template<bool APPLY_MIN_TRANSFER_TIME>
    inline bool isCombinable(const StopId source, const int departureTime, const StopId target, const int arrivalTime = intMax) const noexcept {
        Assert(isStop(source), "Source vertex id " << source << " does not represent a stop!");
        Assert(isStop(target), "Target vertex id " << target << " does not represent a stop!");
        if constexpr (APPLY_MIN_TRANSFER_TIME) {
            if (source == target) {
                return departureTime + minTransferTime(source) <= arrivalTime;
            }
        }
        return isCombinable(source, departureTime, target, arrivalTime);
    }

    inline bool isCombinable(const Connection& first, const Connection& second) const noexcept {
        if (first.arrivalTime > second.departureTime) return false;
        if (first.tripId == second.tripId) return true;
        return isCombinable<true>(first.arrivalStopId, first.arrivalTime, second.departureStopId, second.departureTime);
    }

    inline bool isCombinable(const Vertex source, const int departureTime, const Connection& second) const noexcept {
        return isCombinable(source, departureTime, second.departureStopId, second.departureTime);
    }

    template<bool APPLY_MIN_TRANSFER_TIME>
    inline bool isCombinable(const StopId source, const int departureTime, const Connection& second) const noexcept {
        return isCombinable<APPLY_MIN_TRANSFER_TIME>(source, departureTime, second.departureStopId, second.departureTime);
    }

    inline bool isCombinable(const Connection& first, const StopId target, const int arrivalTime = intMax) const noexcept {
        return isCombinable<true>(first.arrivalStopId, first.arrivalTime, target, arrivalTime);
    }

    inline bool isCombinable(const Connection& first, const Vertex target, const int arrivalTime = intMax) const noexcept {
        return isCombinable(first.arrivalStopId, first.arrivalTime, target, arrivalTime);
    }


    inline void makeUndirectedTransitiveStopGraph(const bool verbose = false) noexcept {
        Intermediate::TransferGraph graph;
        graph.addVertices(transferGraph.numVertices());
        for (const Vertex from : transferGraph.vertices()) {
            graph.set(Coordinates, from, transferGraph.get(Coordinates, from));
            for (const Edge edge : transferGraph.edgesFrom(from)) {
                const Vertex to = transferGraph.get(ToVertex, edge);
                if (to >= stopData.size()) continue;
                graph.addEdge(from, to).set(TravelTime, transferGraph.get(TravelTime, edge));
            }
        }
        graph.packEdges();
        Graph::move(std::move(graph), transferGraph);
        graph.clear();
        graph.addVertices(transferGraph.numVertices());
        Dijkstra<TransferGraph, false> dijkstra(transferGraph, transferGraph[TravelTime]);
        Progress progress(transferGraph.numVertices(), verbose);
        for (const Vertex v : transferGraph.vertices()) {
            graph.set(Coordinates, v, transferGraph.get(Coordinates, v));
            dijkstra.run(v, noVertex, [&](const Vertex u) {
                if (u >= v) return;
                const int travelTime = dijkstra.getDistance(u);
                graph.addEdge(v, u).set(TravelTime, travelTime);
                graph.addEdge(u, v).set(TravelTime, travelTime);
            });
            progress++;
        }
        graph.packEdges();
        Graph::move(std::move(graph), transferGraph);
    }

    inline void makeDirectedTransitiveStopGraph(const bool verbose = false) noexcept {
        Intermediate::TransferGraph toZones;
        Intermediate::TransferGraph fromZones;
        Intermediate::TransferGraph newTransferGraph;
        toZones.addVertices(transferGraph.numVertices());
        fromZones.addVertices(transferGraph.numVertices());
        newTransferGraph.addVertices(transferGraph.numVertices());
        for (const Vertex from : transferGraph.vertices()) {
            newTransferGraph.set(Coordinates, from, transferGraph.get(Coordinates, from));
            for (const Edge edge : transferGraph.edgesFrom(from)) {
                const Vertex to = transferGraph.get(ToVertex, edge);
                const int travelTime = transferGraph.get(TravelTime, edge);
                if (from < stopData.size()) toZones.addEdge(from, to).set(TravelTime, travelTime);
                if (to < stopData.size()) fromZones.addEdge(from, to).set(TravelTime, travelTime);
            }
        }
        if (stopData.size() > 0) {
            toZones.packEdges();
            Dijkstra<Intermediate::TransferGraph, false> dijkstra(toZones, toZones[TravelTime]);
            Progress progress(stopData.size(), verbose);
            for (const StopId stop : stops()) {
                dijkstra.run(stop, noVertex, [&](const Vertex u) {
                    if (u != stop) newTransferGraph.addEdge(stop, u).set(TravelTime, dijkstra.getDistance(u));
                });
                progress++;
            }
        }
        if (transferGraph.numVertices() > stopData.size()) {
            fromZones.packEdges();
            Dijkstra<Intermediate::TransferGraph, false> dijkstra(fromZones, fromZones[TravelTime]);
            Progress progress(transferGraph.numVertices() - stopData.size(), verbose);
            for (Vertex v = Vertex(stopData.size()); v < transferGraph.numVertices(); v++) {
                dijkstra.run(v, noVertex, [&](const Vertex u) {
                    if (u != v) newTransferGraph.addEdge(v, u).set(TravelTime, dijkstra.getDistance(u));
                });
                progress++;
            }
        }
        Dijkstra<TransferGraph, false> dijkstraToZones(transferGraph, transferGraph[TravelTime]);
        newTransferGraph.packEdges();
        Graph::move(std::move(newTransferGraph), transferGraph);
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
    inline const std::vector<Geometry::Point>& getCoordinates() const noexcept {
        return transferGraph[Coordinates];
    }

    inline TransferGraph minTravelTimeGraph() const noexcept {
        Intermediate::TransferGraph topology;
        topology.addVertices(transferGraph.numVertices());
        for (const Connection& connection : connections) {
            if (connection.departureStopId == connection.arrivalStopId) continue;
            const size_t numEdges = topology.numEdges();
            const Edge newEdge = topology.findOrAddEdge(connection.departureStopId, connection.arrivalStopId);
            if (topology.numEdges() != numEdges) {
                topology.set(TravelTime, newEdge, intMax);
            }
            topology.set(TravelTime, newEdge, std::min(topology.get(TravelTime, newEdge), connection.arrivalTime - connection.departureTime));
        }
        for (Vertex vertex : transferGraph.vertices()) {
            topology.set(Coordinates, vertex, transferGraph.get(Coordinates, vertex));
            for (Edge edge : transferGraph.edgesFrom(vertex)) {
                if (vertex == transferGraph.get(ToVertex, edge)) continue;
                const size_t numEdges = topology.numEdges();
                const Edge newEdge = topology.findOrAddEdge(vertex, transferGraph.get(ToVertex, edge));
                if (topology.numEdges() != numEdges) {
                    topology.set(TravelTime, newEdge, intMax);
                }
                topology.set(TravelTime, newEdge, std::min(topology.get(TravelTime, newEdge), transferGraph.get(TravelTime, edge)));
            }
        }
        topology.deleteEdges([&](Edge edge){return topology.get(TravelTime, edge) >= intMax;});
        topology.packEdges();
        TransferGraph result;
        Graph::move(std::move(topology), result);
        return result;
    }

    inline void printInfo() const noexcept {
        int firstDay = std::numeric_limits<int>::max();
        int lastDay = std::numeric_limits<int>::min();
        std::vector<int> departuresByStop(numberOfStops(), 0);
        std::vector<int> arrivalsByStop(numberOfStops(), 0);
        std::vector<int> connectionsByStop(numberOfStops(), 0);
        for (const Connection& connection : connections) {
            if (firstDay > connection.departureTime) firstDay = connection.departureTime;
            if (lastDay < connection.arrivalTime) lastDay = connection.arrivalTime;
            departuresByStop[connection.departureStopId]++;
            arrivalsByStop[connection.arrivalStopId]++;
            connectionsByStop[connection.departureStopId]++;
            connectionsByStop[connection.arrivalStopId]++;
        }
        size_t numberOfIsolatedStops = 0;
        for (const StopId stop : stops()) {
            if (transferGraph.outDegree(stop) == 0) numberOfIsolatedStops++;
        }
        std::cout << "CSA public transit data:" << std::endl;
        std::cout << "   Number of Stops:           " << std::setw(12) << String::prettyInt(numberOfStops()) << std::endl;
        std::cout << "   Number of Isolated Stops:  " << std::setw(12) << String::prettyInt(numberOfIsolatedStops) << std::endl;
        std::cout << "   Number of Trips:           " << std::setw(12) << String::prettyInt(numberOfTrips()) << std::endl;
        std::cout << "   Number of Stop Events:     " << std::setw(12) << String::prettyInt(numberOfConnections() + numberOfTrips()) << std::endl;
        std::cout << "   Number of Connections:     " << std::setw(12) << String::prettyInt(numberOfConnections()) << std::endl;
        std::cout << "   Number of Vertices:        " << std::setw(12) << String::prettyInt(transferGraph.numVertices()) << std::endl;
        std::cout << "   Number of Edges:           " << std::setw(12) << String::prettyInt(transferGraph.numEdges()) << std::endl;
        std::cout << "   Stops without departures:  " << std::setw(12) << String::prettyInt(Vector::count(departuresByStop, 0)) << std::endl;
        std::cout << "   Stops without arrivals:    " << std::setw(12) << String::prettyInt(Vector::count(arrivalsByStop, 0)) << std::endl;
        std::cout << "   Stops without connections: " << std::setw(12) << String::prettyInt(Vector::count(connectionsByStop, 0)) << std::endl;
        std::cout << "   First Day:                 " << std::setw(12) << String::prettyInt(firstDay / (60 * 60 * 24)) << std::endl;
        std::cout << "   Last Day:                  " << std::setw(12) << String::prettyInt(lastDay / (60 * 60 * 24)) << std::endl;
        std::cout << "   First Departure:           " << std::setw(12) << String::secToTime(firstDay) << std::endl;
        std::cout << "   Last Arrival:              " << std::setw(12) << String::secToTime(lastDay) << std::endl;
        std::cout << "   Bounding Box:              " << std::setw(12) << boundingBox() << std::endl;
    }

    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, connections, stopData, tripData);
        transferGraph.writeBinary(fileName + ".graph");
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, connections, stopData, tripData);
        transferGraph.readBinary(fileName + ".graph");
    }

private:
    inline void permutate(const Permutation& fullPermutation, const Permutation& stopPermutation) noexcept {
        Assert(fullPermutation.size() == transferGraph.numVertices(), "Full permutation size (" << fullPermutation.size() << ") must be the same as number of vertices (" << transferGraph.numVertices() << ")!");
        Assert(stopPermutation.size() == numberOfStops(), "Stop permutation size (" << stopPermutation.size() << ") must be the same as number of stops (" << numberOfStops() << ")!");

        for (Connection& connection : connections) {
            connection.applyStopPermutation(stopPermutation);
        }
        stopPermutation.permutate(stopData);

        transferGraph.applyVertexPermutation(fullPermutation);
    }

public:
    std::vector<Connection> connections;
    std::vector<Stop> stopData;
    std::vector<Trip> tripData;

    TransferGraph transferGraph;

};

}
