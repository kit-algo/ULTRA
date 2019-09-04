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

#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <set>

#include "Entities/Connection.h"
#include "Entities/Stop.h"
#include "Entities/Trip.h"

#include "../Intermediate/Data.h"
#include "../Graph/Graph.h"
#include "../Geometry/Rectangle.h"

#include "../../Helpers/IO/Serialization.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Vector/Permutation.h"
#include "../../Helpers/Console/Progress.h"
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
            AssertMsg(!trip.stopEvents.empty(), "Intermediate data contains trip without any stop event!");
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
        AssertMsg(transferGraph.numVertices() >= stops.size(), "Network containes " << stops.size() << " stops, but transfer graph has only " << transferGraph.numVertices() << " vertices!");
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
    inline const std::vector<Geometry::Point>& getCoordinates() const noexcept {
        return transferGraph[Coordinates];
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
        if (transferGraph.numVertices() > numberOfStops()) {
            DynamicTransferGraph stopGraph;
            Graph::copy(transferGraph, stopGraph);
            stopGraph.deleteVertices([&](const Vertex v){return v >= numberOfStops();});
        }
    }

    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, connections, stopData, tripData);
        transferGraph.writeBinary(fileName + ".graph");
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, connections, stopData, tripData);
        transferGraph.readBinary(fileName + ".graph");
    }

public:
    std::vector<Connection> connections;
    std::vector<Stop> stopData;
    std::vector<Trip> tripData;

    TransferGraph transferGraph;

};

}
