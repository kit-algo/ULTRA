#pragma once

#include <vector>

#include "../../Algorithms/CH/CH.h"
#include "../Container/Set.h"
#include "../Graph/Graph.h"
#include "../../Helpers/Vector/Permutation.h"
#include "../../Helpers/Vector/Vector.h"

namespace CH {

template<typename GRAPH_TYPE, typename EXCLUDE>
inline std::vector<bool> targetSelection(const GRAPH_TYPE& graph, const std::vector<Vertex>& targets, const EXCLUDE& exclude) noexcept {
    std::vector<bool> isRequired(graph.numVertices(), false);
    std::vector<Vertex> stack;
    for (const Vertex v : targets) {
        if (exclude(v)) continue;
        stack.emplace_back(v);
        isRequired[v] = true;
    }
    while (!stack.empty()) {
        const Vertex u = stack.back();
        stack.pop_back();
        for (const Edge e : graph.edgesFrom(u)) {
            const Vertex v = graph.get(ToVertex, e);
            if (isRequired[v]) continue;
            isRequired[v] = true;
            if (!exclude(v)) stack.emplace_back(v);
        }
    }
    return isRequired;
}

template<typename GRAPH_TYPE>
inline std::vector<bool> targetSelection(const GRAPH_TYPE& graph, const std::vector<Vertex>& targets) noexcept {
    return targetSelection(graph, targets, [&](const Vertex) {
        return false;
    });
}

struct SweepGraph {
    SweepGraph() { }

    template<typename GRAPH_TYPE>
    inline void build(const GRAPH_TYPE& originalGraph, const std::vector<Vertex>& targets, const bool invertOrder, const bool revertEdges) noexcept {
        std::vector<bool> isRequired = targetSelection(originalGraph, targets);
        Order nonRequiredOrder;
        for (const Vertex vertex : originalGraph.vertices()) {
            if (isRequired[vertex]) {
                order.emplace_back(vertex);
            } else {
                nonRequiredOrder.emplace_back(vertex);
            }
        }
        if (invertOrder) {
            Vector::reverse(order);
            Vector::reverse(nonRequiredOrder);
        }
        const size_t numRequiredVertices = order.size();
        order += nonRequiredOrder;
        permutation = Permutation(Construct::Invert, order);

        CHConstructionGraph constructionGraph;
        constructionGraph.addVertices(numRequiredVertices);
        for (const Vertex fromVertex : constructionGraph.vertices()) {
            const Vertex originalFromVertex(order[fromVertex]);
            for (const Edge edge : originalGraph.edgesFrom(originalFromVertex)) {
                const Vertex originalToVertex = originalGraph.get(ToVertex, edge);
                if (!isRequired[originalToVertex]) continue;
                const Vertex toVertex(permutation[originalToVertex]);
                constructionGraph.addEdge(revertEdges ? toVertex : fromVertex, revertEdges ? fromVertex : toVertex, originalGraph.edgeRecord(edge));
            }
        }
        Graph::move(std::move(constructionGraph), graph);
        graph.sortEdges(ToVertex);

        toVertex.resize(graph.numEdges());
        for (const Edge edge : graph.edges()) {
            toVertex[edge] = internalToExternal(graph.get(ToVertex, edge));
        }
    }

    inline Vertex externalToInternal(const Vertex vertex) const noexcept {
        return permutation.permutate(vertex);
    }

    inline Vertex internalToExternal(const Vertex vertex) const noexcept {
        return Vertex(order[vertex]);
    }

    CHGraph graph;
    Order order;
    Permutation permutation;
    std::vector<Vertex> toVertex;
};

struct CompactSweepGraph {
    CompactSweepGraph() { }

    template<typename GRAPH_TYPE>
    inline void build(const GRAPH_TYPE& originalGraph, const std::vector<Vertex>& targets, const bool invertOrder, const bool revertEdges, const size_t numExcludedVertices) noexcept {
        std::vector<bool> isRequired = targetSelection(originalGraph, targets, [&](const Vertex v) {
            return v < numExcludedVertices;
        });

        for (const Vertex vertex : originalGraph.vertices()) {
            if (!isRequired[vertex]) continue;
            order.emplace_back(vertex);
        }
        if (invertOrder) Vector::reverse(order);
        std::vector<Vertex> permutation(originalGraph.numVertices(), noVertex);
        for (size_t i = 0; i < order.size(); i++) {
            permutation[order[i]] = Vertex(i);
        }

        CHConstructionGraph constructionGraph;
        constructionGraph.addVertices(order.size());
        for (const Vertex fromVertex : constructionGraph.vertices()) {
            const Vertex originalFromVertex = order[fromVertex];
            for (const Edge edge : originalGraph.edgesFrom(originalFromVertex)) {
                const Vertex originalToVertex = originalGraph.get(ToVertex, edge);
                if (!isRequired[originalToVertex]) continue;
                const Vertex toVertex = permutation[originalToVertex];
                constructionGraph.addEdge(revertEdges ? toVertex : fromVertex, revertEdges ? fromVertex : toVertex, originalGraph.edgeRecord(edge));
            }
        }
        Graph::copy(constructionGraph, graph);
        graph.sortEdges(ToVertex);

        toVertex.resize(graph.numEdges());
        for (const Edge edge : graph.edges()) {
            toVertex[edge] = internalToExternal(graph.get(ToVertex, edge));
        }
    }


    inline Vertex internalToExternal(const Vertex vertex) const noexcept {
        return order[vertex];
    }

    CHGraph graph;
    std::vector<Vertex> order;
    std::vector<Vertex> toVertex;
};

struct BucketGraph {
    BucketGraph() { }

    inline void initialize(CHGraph&& initGraph) noexcept {
        ::Graph::move(std::move(initGraph), graph);
        graph.sortEdges(Weight);
    }

    CHGraph graph;
};
}
