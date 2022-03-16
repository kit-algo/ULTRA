#pragma once

#include "../../Geometry/Rectangle.h"

namespace Graph {

    template<typename GRAPH>
    inline bool hasLoops(const GRAPH& graph) noexcept {
        for (const Vertex vertex : graph.vertices()) {
            for (const Edge edge : graph.edgesFrom(vertex)) {
                if (graph.get(ToVertex, edge) == vertex) return true;
            }
        }
        return false;
    }

    template<typename GRAPH>
    inline void computeTravelTimes(GRAPH& graph, const double speedInKMH, const bool maximize = false, const size_t timeFactor = 1) noexcept {
        static_assert(GRAPH::HasVertexAttribute(Coordinates), "GRAPH has no coordinates!");
        static_assert(GRAPH::HasEdgeAttribute(TravelTime), "GRAPH has no travel time!");
        for (const auto [edge, from] : graph.edgesWithFromVertex()) {
            const Geometry::Point& a = graph.get(Coordinates, from);
            const Geometry::Point& b = graph.get(Coordinates, graph.get(ToVertex, edge));
            const double distance = Geometry::geoDistanceInCM(a, b);
            const int travelTime = (distance / speedInKMH) * timeFactor * 0.036;
            if (maximize) {
                graph.set(TravelTime, edge, std::max(graph.get(TravelTime, edge), travelTime));
            } else {
                graph.set(TravelTime, edge, travelTime);
            }
        }
    }

    template<typename GRAPH>
    inline void applyBoundingBox(GRAPH& graph, Geometry::Rectangle boundingBox) noexcept {
        static_assert(GRAPH::HasVertexAttribute(Coordinates), "GRAPH has no coordinates!");
        const std::vector<Geometry::Point>& coordinates = graph[Coordinates];
        graph.deleteVertices([&](const Vertex vertex){return !boundingBox.contains(coordinates[vertex]);});
    }

    template<typename GRAPH>
    inline void printInfo(const GRAPH& graph, std::ostream& out = std::cout) noexcept {
        const std::string typeString = graphType(graph);
        const std::string vertexData = cleanGraphType(attributeListToString<typename GRAPH::ListOfRecordVertexAttributes>());
        const std::string edgeData = cleanGraphType(attributeListToString<typename GRAPH::ListOfRecordEdgeAttributes>());
        out << typeString.substr(0, typeString.find_first_of('<')) << " with " << String::prettyInt(graph.numVertices()) << " vertices and " << String::prettyInt(graph.numEdges()) << " edges"
            << " (" << String::bytesToString(graph.byteSize()) << " on disk)." << std::endl;
        if (!vertexData.empty()) out << "    Vertices contain: " << vertexData << "." << std::endl;
        if (!edgeData.empty()) out << "    Edges contain: " << edgeData << "." << std::endl;
        if (vertexData.empty() && edgeData.empty()) out << "      no additional data exists." << std::endl;
    }

    template<typename GRAPH>
    inline void writeStatisticsFile(const GRAPH& graph, const std::string& fileNameBase, const std::string& separator = ".") noexcept {
        const std::string fileName = fileNameBase + separator + "statistics.txt";
        std::ofstream statistics(fileName);
        AssertMsg(statistics, "Cannot create output stream for: " << fileName);
        AssertMsg(statistics.is_open(), "Cannot open output stream for: " << fileName);
        printInfo(graph, statistics);
        graph.printAnalysis(statistics);
        statistics.close();
    }

}
