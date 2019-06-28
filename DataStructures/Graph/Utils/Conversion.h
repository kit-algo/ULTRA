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

#include <vector>

#include "../Classes/GraphInterface.h"

#include "../Classes/DynamicGraph.h"
#include "../Classes/StaticGraph.h"
#include "../Classes/EdgeList.h"

#include "../../../Helpers/Assert.h"
#include "../../../Helpers/Vector/Permutation.h"

namespace Graph {

    template<typename GRAPH>
    inline void computeReverseEdgePointers(GRAPH& graph) noexcept {
        if constexpr (GRAPH::HasEdgeAttribute(ReverseEdge)) {
            for (const auto [edge, from] : graph.edgesWithFromVertex()) {
                if (graph.isEdge(graph.get(ReverseEdge, edge))) continue;
                const Edge reverse = graph.findEdge(graph.get(ToVertex, edge), from);
                if (!graph.isEdge(reverse)) continue;
                if (graph.isEdge(graph.get(ReverseEdge, reverse))) continue;
                graph.set(ReverseEdge, edge, reverse);
                graph.set(ReverseEdge, reverse, edge);
            }
        }
    }

    // StaticGraph --> StaticGraph
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    inline void move(StaticGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, StaticGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges) {
        using FromType = StaticGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>;
        using ToType = StaticGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>;
        to.clear();
        to.vertexAttributes.assign(from.vertexAttributes, attributeNameChanges...);
        to.edgeAttributes.assign(from.edgeAttributes, attributeNameChanges...);
        to.beginOut.swap(from.beginOut);
        if constexpr (ToType::HasEdgeAttribute(FromVertex) && !FromType::HasEdgeAttribute(FromVertex)) {
            for (const Vertex vertex : to.vertices()) {
                for (const Edge edge : to.edgesFrom(vertex)) {
                    to.set(FromVertex, edge, vertex);
                }
            }
        }
        if constexpr (!FromType::HasEdgeAttribute(ReverseEdge)) {
            computeReverseEdgePointers(to);
        }
        to.checkVectorSize();
        Assert(to.satisfiesInvariants());
        from.clear();
    }

    // StaticGraph --> DynamicGraph
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    inline void move(StaticGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, DynamicGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges) {
        using FromType = StaticGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>;
        to.clear();
        std::vector<size_t> outDegree;
        outDegree.reserve(from.numVertices() * 1.2);
        for (const Vertex vertex : from.vertices()) {
            outDegree.emplace_back(from.outDegree(vertex));
        }
        from.beginOut.pop_back();
        to.edgeCount = from.numEdges();
        to.vertexAttributes.assign(from.vertexAttributes, attributeNameChanges...);
        to.edgeAttributes.assign(from.edgeAttributes, attributeNameChanges...);
        to.vertexAttributes.get(BeginOut).swap(from.beginOut);
        to.vertexAttributes.get(OutDegree).swap(outDegree);
        for (const Vertex vertex : to.vertices()) {
            for (const Edge edge : to.edgesFrom(vertex)) {
                to[Valid][edge] = true;
                to.get(IncomingEdgePointer, edge) = to.get(IncomingEdges, to.get(ToVertex, edge)).size();
                to.get(IncomingEdges, to.get(ToVertex, edge)).push_back(edge);
                to.set(FromVertex, edge, vertex);
            }
        }
        if constexpr (!FromType::HasEdgeAttribute(ReverseEdge)) {
            computeReverseEdgePointers(to);
        }
        AssertMsg(to.satisfiesInvariants(), "Invariants not satisfied!");
        from.clear();
    }

    // StaticGraph --> EdgeList
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    inline void move(StaticGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, EdgeListImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges) {
        using FromType = StaticGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>;
        to.clear();
        std::vector<Vertex> fromVertex;
        if constexpr (FromType::HasEdgeAttribute(FromVertex)) {
            fromVertex = from.get(FromVertex);
        } else {
            fromVertex.resize(from.numEdges());
            for (const Vertex vertex : from.vertices()) {
                for (const Edge edge : from.edgesFrom(vertex)) {
                    fromVertex[edge] = vertex;
                }
            }
        }
        to.vertexAttributes.assign(from.vertexAttributes, attributeNameChanges...);
        to.edgeAttributes.assign(from.edgeAttributes, attributeNameChanges...);
        to.get(FromVertex).swap(fromVertex);
        if constexpr (!FromType::HasEdgeAttribute(ReverseEdge)) {
            computeReverseEdgePointers(to);
        }
        from.clear();
    }

    // DynamicGraph --> StaticGraph
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    inline void move(DynamicGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, StaticGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges) {
        using FromType = DynamicGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>;
        to.clear();
        from.packEdges();
        to.beginOut = from.get(BeginOut);
        to.beginOut.emplace_back(from.numEdges());
        for (Vertex v = Vertex(to.beginOut.size() - 2); v < to.beginOut.size(); v--) {
            const Vertex u = Vertex(v + 1);
            if (from[OutDegree][v] == 0) to.beginOut[v] = to.beginOut[u];
            AssertMsg(to.beginOut[v] <= to.beginOut[u], "Adjacency data structure is out of order!");
        }
        to.vertexAttributes.assign(from.vertexAttributes, attributeNameChanges...);
        to.edgeAttributes.assign(from.edgeAttributes, attributeNameChanges...);
        if constexpr (!FromType::HasEdgeAttribute(ReverseEdge)) {
            computeReverseEdgePointers(to);
        }
        to.checkVectorSize();
        AssertMsg(to.satisfiesInvariants(), "Invariants not satisfied!");
        from.clear();
    }

    // DynamicGraph --> DynamicGraph
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    inline void move(DynamicGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, DynamicGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges) {
        using FromType = DynamicGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>;
        to.clear();
        to.vertexAttributes.assign(from.vertexAttributes, attributeNameChanges...);
        to.edgeAttributes.assign(from.edgeAttributes, attributeNameChanges...);
        to.edgeCount = from.edgeCount;
        if constexpr (!FromType::HasEdgeAttribute(ReverseEdge)) {
            computeReverseEdgePointers(to);
        }
        to.checkVectorSize();
        AssertMsg(to.satisfiesInvariants(), "Invariants not satisfied!");
        from.clear();
    }

    // DynamicGraph --> EdgeList
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    inline void move(DynamicGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, EdgeListImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges) {
        using FromType = DynamicGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>;
        to.clear();
        from.packEdges();
        to.vertexAttributes.assign(from.vertexAttributes, attributeNameChanges...);
        to.edgeAttributes.assign(from.edgeAttributes, attributeNameChanges...);
        if constexpr (!FromType::HasEdgeAttribute(ReverseEdge)) {
            computeReverseEdgePointers(to);
        }
        from.clear();
    }

    // EdgeList --> EdgeList
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    inline void move(EdgeListImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, EdgeListImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges) {
        using FromType = EdgeListImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>;
        to.clear();
        to.vertexAttributes.assign(from.vertexAttributes, attributeNameChanges...);
        to.edgeAttributes.assign(from.edgeAttributes, attributeNameChanges...);
        if constexpr (!FromType::HasEdgeAttribute(ReverseEdge)) {
            computeReverseEdgePointers(to);
        }
        from.clear();
    }

    // EdgeList --> StaticGraph
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    inline void move(EdgeListImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, StaticGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges) {
        using FromType = EdgeListImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>;
        to.clear();
        const Order order(Construct::Sort, from.get(FromVertex));
        from.applyEdgeOrder(order);
        for (const Edge edge : from.edges()) {
            while (to.beginOut.size() <= from.get(FromVertex, edge)) {
                to.beginOut.emplace_back(edge);
            }
        }
        while (to.beginOut.size() <= from.numVertices()) {
            to.beginOut.emplace_back(from.numEdges());
        }
        to.vertexAttributes.assign(from.vertexAttributes, attributeNameChanges...);
        to.edgeAttributes.assign(from.edgeAttributes, attributeNameChanges...);
        if constexpr (!FromType::HasEdgeAttribute(ReverseEdge)) {
            computeReverseEdgePointers(to);
        }
        to.checkVectorSize();
        AssertMsg(to.satisfiesInvariants(), "Invariants not satisfied!");
        from.clear();
    }

    // EdgeList --> DynamicGraph
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    inline void move(EdgeListImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, DynamicGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges) {
        StaticGraph<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM> tempGraph;
        move(std::move(from), tempGraph, attributeNameChanges...);
        move(std::move(tempGraph), to, attributeNameChanges...);
    }

    // Copy graph data instead of moving it
    template<typename FROM_GRAPH_TYPE, typename TO_GRAPH_TYPE, typename... ATTRIBUTE_NAME_CHANGES>
    inline void copy(const FROM_GRAPH_TYPE& from, TO_GRAPH_TYPE& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges) noexcept {
        FROM_GRAPH_TYPE fromCopy = from;
        move(std::move(fromCopy), to, attributeNameChanges...);
    }

}
