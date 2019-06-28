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

#include "../../Geometry/Rectangle.h"

namespace Graph {

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
