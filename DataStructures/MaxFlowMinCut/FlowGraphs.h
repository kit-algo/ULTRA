#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../Graph/Graph.h"

#include "../../Helpers/Assert.h"
#include "../../Helpers/Types.h"

namespace Graph {

template<typename GRAPH>
static inline DynamicFlowGraph generateFlowGraph(const GRAPH& graph, const std::vector<int>& capacities, const bool undirectedEdges = false) noexcept {
    DynamicFlowGraph flowGraph;
    flowGraph.addVertices(graph.numVertices());
    for (const auto [edge, from] : graph.edgesWithFromVertex()) {
        const Vertex to = graph.get(ToVertex, edge);
        if (from == to) continue;
        const Edge forwardEdge = flowGraph.findOrAddEdge(from, to);
        flowGraph.set(Capacity, forwardEdge, flowGraph.get(Capacity, forwardEdge) + capacities[edge]);
        if (undirectedEdges) {
            const Edge backwardEdge = flowGraph.findOrAddEdge(to, from);
            flowGraph.set(Capacity, backwardEdge, flowGraph.get(Capacity, backwardEdge) + capacities[edge]);
        } else {
            flowGraph.findOrAddEdge(to, from);
        }
    }
    return flowGraph;
}

template<typename GRAPH>
static inline DynamicFlowGraph generateFlowGraph(const GRAPH& graph, const bool undirectedEdges = false) noexcept {
    DynamicFlowGraph flowGraph;
    flowGraph.addVertices(graph.numVertices());
    for (const auto [edge, from] : graph.edgesWithFromVertex()) {
        const Vertex to = graph.get(ToVertex, edge);
        if (from == to) continue;
        flowGraph.set(Capacity, flowGraph.findOrAddEdge(from, to), 1);
        if (undirectedEdges) {
            flowGraph.set(Capacity, flowGraph.findOrAddEdge(to, from), 1);
        } else {
            flowGraph.findOrAddEdge(to, from);
        }
    }
    return flowGraph;
}

template<typename GRAPH>
inline DynamicFlowGraph generateVertexFlowGraph(const GRAPH& graph) noexcept {
    DynamicFlowGraph flowGraph;
    const Vertex offset = Vertex(graph.numVertices());
    flowGraph.addVertices(offset * 2);
    for (const Vertex from : graph.vertices()) {
        for (const Edge edge : graph.edgesFrom(from)) {
            const Vertex to = graph.get(ToVertex, edge);
            if (from == to) continue;
            flowGraph.set(Capacity, flowGraph.findOrAddEdge(from, to + offset), 0);
            flowGraph.set(Capacity, flowGraph.findOrAddEdge(to + offset, from), 1);
            flowGraph.set(Capacity, flowGraph.findOrAddEdge(to, from + offset), 0);
            flowGraph.set(Capacity, flowGraph.findOrAddEdge(from + offset, to), 1);
        }
        flowGraph.set(Capacity, flowGraph.findOrAddEdge(from, from + offset), 1);
        flowGraph.set(Capacity, flowGraph.findOrAddEdge(from + offset, from), 0);
    }
    return flowGraph;
}

}
