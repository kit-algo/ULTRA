#pragma once

#include <vector>
#include <string>
#include <sstream>
#include <utility>
#include <algorithm>

#include "../Classes/GraphInterface.h"

#include "../../Geometry/Point.h"
#include "../../Geometry/Rectangle.h"

#include "../../../Helpers/Assert.h"
#include "../../../Helpers/String/String.h"
#include "../../../Helpers/Vector/Permutation.h"

#include "../../../Algorithms/DepthFirstSearch.h"
#include "../../../Algorithms/Dijkstra/Dijkstra.h"
#include "../../../Algorithms/StronglyConnectedComponents.h"

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
    inline bool isAcyclic(const GRAPH& graph) noexcept {
        bool result = true;
        std::vector<bool> active(graph.numVertices(), false);
        dfs(graph, [&](const Vertex vertex){
            active[vertex] = true;
        }, [&](const Vertex vertex){
            if (active[vertex]) result = false;
        }, [&](const Vertex vertex){
            active[vertex] = false;
        });
        return result;
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

    template<typename GRAPH_A, typename GRAPH_B>
    inline void incorporateGraph(GRAPH_A& graphA, const GRAPH_B& graphB) noexcept {
        for (const Vertex vertex : graphB.vertices()) {
            if (!graphA.isVertex(vertex)) {
                graphA.addVertex(graphB.vertexRecord(vertex));
            }
            if constexpr (GRAPH_A::HasVertexAttribute(Coordinates) && GRAPH_B::HasVertexAttribute(Coordinates)) {
                AssertMsg(graphA.get(Coordinates, vertex) == graphB.get(Coordinates, vertex), "Vertex " << vertex << " cannot be merged, because the coordinates differ!");
            }
        }
        for (const auto [edge, from] : graphB.edgesWithFromVertex()) {
            graphA.addEdge(from, graphB.get(ToVertex, edge), graphB.edgeRecord(edge));
        }
    }

    template<typename GRAPH>
    inline size_t numberOfMultiEdges(const GRAPH& graph) noexcept {
        std::vector<std::pair<Vertex, Vertex>> edges;
        for (const auto [edge, from] : graph.edgesWithFromVertex()) {
            const Vertex to = graph.get(ToVertex, edge);
            edges.emplace_back(from, to);
        }
        std::sort(edges.begin(), edges.end());
        size_t result = 0;
        for (size_t i = 1; i < edges.size(); i++) {
            if (edges[i - 1] == edges[i]) {
                result++;
            }
        }
        return result;
    }

    template<typename GRAPH, AttributeNameType ATTRIBUTE_NAME>
    inline bool hasTriangleInequality(const GRAPH& graph, const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName) noexcept {
        static_assert(GRAPH::HasEdgeAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()), "GRAPH does not have the required edge attribute!");
        for (const Vertex from : graph.vertices()) {
            for (const Edge edge : graph.edgesFrom(from)) {
                const Vertex to = graph.get(ToVertex, edge);
                for (const Edge first : graph.edgesFrom(from)) {
                    for (const Edge second : graph.edgesFrom(graph.get(ToVertex, first))) {
                        if (graph.get(ToVertex, second) != to) continue;
                        if (graph.get(attributeName, first) + graph.get(attributeName, second) < graph.get(attributeName, edge)) return false;
                    }
                }
            }
        }
        return true;
    }

    template<typename GRAPH, AttributeNameType ATTRIBUTE_NAME = TravelTime>
    inline std::string characterize(const GRAPH& graph, const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName = TravelTime) noexcept {
        if (isClusterGraph(graph)) {
            if (hasTriangleInequality(graph, attributeName)) {
                return "Transitively closed";
            } else {
                return "Cluster graph without triangle inequality";
            }
        } else {
            if (hasTriangleInequality(graph, attributeName)) {
                return "Transitive but not closed";
            } else {
                return "Not transitive";
            }
        }
    }

    template<typename GRAPH>
    inline Geometry::Rectangle boundingBox(const GRAPH& graph) noexcept {
        static_assert(GRAPH::HasVertexAttribute(Coordinates), "GRAPH has no coordinates!");
        return Geometry::Rectangle::BoundingBox(graph[Coordinates]);
    }

    template<typename GRAPH>
    inline void applyBoundingBox(GRAPH& graph, Geometry::Rectangle boundingBox) noexcept {
        static_assert(GRAPH::HasVertexAttribute(Coordinates), "GRAPH has no coordinates!");
        const std::vector<Geometry::Point>& coordinates = graph[Coordinates];
        graph.deleteVertices([&](const Vertex vertex){return !boundingBox.contains(coordinates[vertex]);});
    }

    template<typename GRAPH, bool DEBUG = true>
    inline void reduceToBiggestStronglyConnectedComponent(GRAPH& graph) noexcept {
        StronglyConnectedComponents<GRAPH, DEBUG> scc(graph);
        scc.run();
        const int maxComponent = scc.maxComponent();
        const std::vector<int>& component = scc.getComponent();
        graph.deleteVertices([&](const Vertex vertex){return component[vertex] != maxComponent;});
    }

    template<typename GRAPH, typename WEIGHT_TYPE>
    inline void makeTransitivelyClosed(GRAPH& graph, const WEIGHT_TYPE weight) noexcept {
        static_assert(GRAPH::HasEdgeAttribute(weight), "GRAPH has no weight!");
        Dijkstra<GRAPH> dijkstra(graph);
        for (const Vertex v : graph.vertices()) {
            dijkstra.run(v, noVertex, [&](const Vertex u) {
                if (u == v) return;
                const int distance = dijkstra.getDistance(u);
                const Edge edge = graph.findOrAddEdge(v, u);
                if (distance < graph.get(weight, edge)) {
                    graph.set(weight, edge, distance);
                }
            });
        }
        graph.packEdges();
    }

    template<typename GRAPH>
    inline std::string vertexToString(const GRAPH& graph, const Vertex vertex) noexcept {
        AssertMsg(graph.isVertex(vertex), vertex << " is not a valid vertex!");
        std::stringstream result;
        result << "id: " << vertex;
        const std::string attributeString = graph.vertexRecord(vertex).toString();
        if (!attributeString.empty()) {
            result << " | " << attributeString;
        }
        return result.str();
    }

    template<typename GRAPH>
    inline std::string edgeToString(const GRAPH& graph, const Edge edge) noexcept {
        AssertMsg(graph.isEdge(edge), edge << " is not a valid edge!");
        std::stringstream result;
        result << "id: " << edge << ", to: " << graph.get(ToVertex, edge);
        const std::string attributeString = graph.edgeRecord(edge).toString();
        if (!attributeString.empty()) {
            result << " | " << attributeString;
        }
        return result.str();
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
