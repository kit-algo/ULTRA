#pragma once

#include <vector>

#include "../Classes/GraphInterface.h"

#include "../Classes/DynamicGraph.h"
#include "../Classes/StaticGraph.h"
#include "../Classes/EdgeList.h"

#include "Conversion.h"

#include "../../Geometry/Point.h"

#include "../../../Helpers/Assert.h"
#include "../../../Helpers/IO/Serialization.h"

namespace Graph {

    template<typename GRAPH>
    inline void fromDimacs(const std::string& fileBaseName, GRAPH& graph) noexcept {
        EdgeList<typename GRAPH::ListOfVertexAttributes, typename GRAPH::ListOfEdgeAttributes> edgeList;
        edgeList.fromDimacs(fileBaseName);
        move(std::move(edgeList), graph);
    }

    template<typename GRAPH, typename WEIGHT_TYPE>
    inline void toDimacs(const std::string& fileBaseName, const GRAPH& graph, const std::vector<WEIGHT_TYPE>& weight) noexcept {
        std::ofstream grOs(fileBaseName + ".gr");
        AssertMsg(grOs, "Cannot create output stream for " << fileBaseName << ".gr");
        AssertMsg(grOs.is_open(), "Cannot open output stream for " << fileBaseName << ".gr");
        grOs << "p sp " << graph.numVertices() << " " << graph.numEdges() << std::endl;
        for (const auto [edge, from] : graph.edgesWithFromVertex()) {
            grOs << "a " << (from + 1) << " " << (graph.get(ToVertex, edge) + 1) << " " << weight[edge] << std::endl;
        }
        grOs.close();
        if constexpr (GRAPH::HasVertexAttribute(Coordinates)) {
            std::ofstream coOs(fileBaseName + ".co");
            AssertMsg(coOs, "Cannot create output stream for " << fileBaseName << ".co");
            AssertMsg(coOs.is_open(), "Cannot open output stream for " << fileBaseName << ".co");
            coOs << "p aux sp co " << graph.numVertices() << std::endl;
            for (const Vertex vertex : graph.vertices()) {
                coOs << "v " << (vertex + 1) << " " << graph.get(Coordinates, vertex).x << " " << graph.get(Coordinates, vertex).y << std::endl;
            }
            coOs.close();
        }
    }

    template<typename GRAPH>
    inline void toDimacs(const std::string& fileBaseName, const GRAPH& graph) noexcept {
        toDimacs(fileBaseName, graph, graph.get(Weight));
    }

    template<typename GRAPH>
    inline void toGML(const std::string& fileBaseName, const GRAPH& graph) noexcept {
        std::ofstream gml(fileBaseName + ".gml");
        AssertMsg(gml, "Cannot create output stream for " << fileBaseName << ".gml");
        AssertMsg(gml.is_open(), "Cannot open output stream for " << fileBaseName << ".gml");
        gml << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        gml << "<graphml xmlns=\"http://graphml.graphdrawing.org/xmlns\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd\">\n";
        gml << "    <graph id=\"G\" edgedefault=\"directed\">\n";
        for (const Vertex vertex : graph.vertices()) {
            gml << "        <node id=\"" << size_t(vertex) << "\"/>\n";
        }
        for (const auto [edge, from] : graph.edgesWithFromVertex()) {
            gml << "        <edge source=\"" << size_t(from) << "\" target=\"" << size_t(graph.get(ToVertex, edge)) << "\"/>\n";
        }
        gml << "    </graph>\n";
        gml << "</graphml>" << std::endl;
        gml.close();
    }

    template<typename GRAPH>
    inline void fromStrasserBinary(const std::string& fileName, GRAPH& graph, const double timeFactor = 1, const double distanceFactor = 1, const double coordinateFactor = 1) noexcept {
        std::vector<float> latitude;
        std::vector<float> longitude;
        std::vector<int> weight;
        std::vector<int> geo_distance;
        std::vector<int> travel_time;
        std::vector<int> first_out;
        std::vector<int> head;
        IO::deserialize(fileName + "/first_out", first_out);
        IO::deserialize(fileName + "/head", head);
        if constexpr (GRAPH::HasVertexAttribute(Coordinates)) IO::deserialize(fileName + "/latitude", latitude);
        if constexpr (GRAPH::HasVertexAttribute(Coordinates)) IO::deserialize(fileName + "/longitude", longitude);
        if constexpr (GRAPH::HasEdgeAttribute(Weight)) IO::deserialize(fileName + "/weight", weight);
        if constexpr (GRAPH::HasEdgeAttribute(Distance)) IO::deserialize(fileName + "/geo_distance", geo_distance);
        if constexpr (GRAPH::HasEdgeAttribute(TravelTime)) IO::deserialize(fileName + "/travel_time", travel_time);
        AssertMsg(latitude.size() == longitude.size(), "Latitude and longitude vector have different sizes! (" << latitude.size() << " vs. " << longitude.size() << ")");
        SimpleDynamicGraph temp;
        temp.addVertices(first_out.size() - 1);
        for (const Vertex vertex : temp.vertices()) {
            for (int i = first_out[vertex]; i < first_out[vertex + 1]; i++) {
                temp.addEdge(vertex, Vertex(head[i]));
            }
        }
        Graph::move(std::move(temp), graph);
        for (const Vertex vertex : graph.vertices()) {
            if constexpr (GRAPH::HasVertexAttribute(Coordinates)) {
                graph.set(Coordinates, vertex, Geometry::Point(Construct::LatLong, latitude[vertex] * coordinateFactor, longitude[vertex] * coordinateFactor));
            }
            for (int i = first_out[vertex]; i < first_out[vertex + 1]; i++) {
                const Edge edge = graph.findEdge(vertex, Vertex(head[i]));
                if constexpr (GRAPH::HasEdgeAttribute(Weight)) {
                    graph.set(Weight, edge, weight[i]);
                }
                if constexpr (GRAPH::HasEdgeAttribute(Distance)) {
                    graph.set(Distance, edge, static_cast<int>((geo_distance[i] * distanceFactor * 10) + 5) / 10);
                }
                if constexpr (GRAPH::HasEdgeAttribute(TravelTime)) {
                    graph.set(TravelTime, edge, static_cast<int>((travel_time[i] * timeFactor * 10) + 5) / 10);
                }
                (void)edge;
            }
        }
        Assert(graph.satisfiesInvariants());
    }

}
