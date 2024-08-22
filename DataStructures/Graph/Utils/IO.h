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
        Assert(grOs, "Cannot create output stream for " << fileBaseName << ".gr");
        Assert(grOs.is_open(), "Cannot open output stream for " << fileBaseName << ".gr");
        grOs << "p sp " << graph.numVertices() << " " << graph.numEdges() << std::endl;
        for (const auto [edge, from] : graph.edgesWithFromVertex()) {
            grOs << "a " << (from + 1) << " " << (graph.get(ToVertex, edge) + 1) << " " << weight[edge] << std::endl;
        }
        grOs.close();
        if constexpr (GRAPH::HasVertexAttribute(Coordinates)) {
            std::ofstream coOs(fileBaseName + ".co");
            Assert(coOs, "Cannot create output stream for " << fileBaseName << ".co");
            Assert(coOs.is_open(), "Cannot open output stream for " << fileBaseName << ".co");
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
        Assert(gml, "Cannot create output stream for " << fileBaseName << ".gml");
        Assert(gml.is_open(), "Cannot open output stream for " << fileBaseName << ".gml");
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
}
