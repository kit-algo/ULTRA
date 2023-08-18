#pragma once

#include "Dinic.h"

#include "../BipartiteGraphAlgorithms.h"

#include "../../DataStructures/MaxFlowMinCut/FlowGraphs.h"

class VertexSeparatorUsingVertexCover {

public:
    template<typename GRAPH>
    VertexSeparatorUsingVertexCover(const GRAPH& graph) :
        dinic(Graph::generateFlowGraph(graph, true)),
        usedVertices(graph.numVertices(), false) {
    }

public:
    inline void run(const std::vector<Vertex>& sources, const std::vector<Vertex>& targets) noexcept {
        dinic.run(sources, targets);
        warning("flow: ", dinic.getFlow());
        std::vector<Vertex> sourceSeparator = getVertexSeparator(dinic.sourceCut());
        std::vector<Vertex> targetSeparator = getVertexSeparator(dinic.targetCut());
        if (sourceSeparator.size() < targetSeparator.size()) {
            separator = sourceSeparator;
        } else {
            separator = targetSeparator;
        }
    }

    inline void run(const Vertex source, const Vertex target) noexcept {
        run(std::vector<Vertex>(1, source), std::vector<Vertex>(1, target));
    }

    inline const std::vector<Vertex>& getSeparator() const noexcept {
        return separator;
    }

    inline std::vector<Vertex> getSide(const std::vector<Vertex>& vertices) noexcept {
        const StaticFlowGraph& graph = dinic.getFlowGraph();
        std::vector<Vertex> side;
        for (const Vertex vertex : separator) {
            usedVertices[vertex] = true;
        }
        for (const Vertex vertex : vertices) {
            side.emplace_back(vertex);
            usedVertices[vertex] = true;
        }
        for (size_t i = 0; i < side.size(); i++) {
            const Vertex from = side[i];
            for (const Edge edge : graph.edgesFrom(from)) {
                if (graph.get(Capacity, edge) == 0) continue;
                const Vertex to = graph.get(ToVertex, edge);
                if (usedVertices[to]) continue;
                side.emplace_back(to);
                usedVertices[to] = true;
            }
        }
        for (const Vertex vertex : side) {
            usedVertices[vertex] = false;
        }
        for (const Vertex vertex : separator) {
            usedVertices[vertex] = false;
        }
        return side;
    }

    inline std::vector<Vertex> getSide(const Vertex vertex) noexcept {
        return getSide(std::vector<Vertex>(1, vertex));
    }

    inline void isolateVertex(const Vertex vertex) noexcept {
        dinic.isolateVertex(vertex);
    }

private:
    inline std::vector<Vertex> getVertexSeparator(const std::vector<Dinic::CutEdge>& edgeCut) const noexcept {
        std::vector<Vertex> newVertexId(dinic.getFlowGraph().numVertices(), noVertex);
        std::vector<Vertex> oldVertexId;
        for (const Dinic::CutEdge& edge : edgeCut) {
            if (newVertexId[edge.from] == noVertex) {
                newVertexId[edge.from] = Vertex(oldVertexId.size());
                oldVertexId.emplace_back(edge.from);
            }
            if (newVertexId[edge.to] == noVertex) {
                newVertexId[edge.to] = Vertex(oldVertexId.size());
                oldVertexId.emplace_back(edge.to);
            }
        }
        SimpleEdgeList cutGraph;
        cutGraph.addVertices(oldVertexId.size());
        for (const Dinic::CutEdge& edge : edgeCut) {
            cutGraph.addEdge(newVertexId[edge.from], newVertexId[edge.to]);
        }
        std::vector<Vertex> separator = minimumBipartiteVertexCover(cutGraph);
        for (Vertex& vertex : separator) {
            vertex = oldVertexId[vertex];
        }
        return separator;
    }

private:
    Dinic dinic;
    std::vector<bool> usedVertices;
    std::vector<Vertex> separator;

};

class VertexSeparatorUsingGraphExpansion {

public:
    template<typename GRAPH>
    VertexSeparatorUsingGraphExpansion(const GRAPH& graph) :
        offset(graph.numVertices()),
        dinic(Graph::generateVertexFlowGraph(graph)),
        usedVertices(graph.numVertices(), false) {
    }

    inline void run(const std::vector<Vertex>& sources, const std::vector<Vertex>& targets) noexcept {
        std::vector<Vertex> offsetSources = sources;
        for (Vertex& vertex : offsetSources) vertex += offset;
        dinic.run(offsetSources, targets);
        separator.clear();
        for (const Dinic::CutEdge& edge : dinic.sourceCut()) {
            if (edge.from < edge.to) {
                if (usedVertices[edge.from]) continue;
                separator.emplace_back(edge.from);
                usedVertices[edge.from] = true;
            } else {
                if (usedVertices[edge.to]) continue;
                separator.emplace_back(edge.to);
                usedVertices[edge.to] = true;
            }
        }
        for (const Vertex vertex : separator) {
            usedVertices[vertex] = false;
        }
    }

    inline void run(const Vertex source, const Vertex target) noexcept {
        run(std::vector<Vertex>(1, source), std::vector<Vertex>(1, target));
    }

    inline const std::vector<Vertex>& getSeparator() const noexcept {
        return separator;
    }

    inline std::vector<Vertex> getSide(const std::vector<Vertex>& vertices) noexcept {
        const StaticFlowGraph& graph = dinic.getFlowGraph();
        std::vector<Vertex> side;
        for (const Vertex vertex : separator) {
            usedVertices[vertex] = true;
        }
        for (const Vertex vertex : vertices) {
            side.emplace_back(vertex);
            usedVertices[vertex] = true;
        }
        for (size_t i = 0; i < side.size(); i++) {
            const Vertex from = side[i] + offset;
            for (const Edge edge : graph.edgesFrom(from)) {
                if (graph.get(Capacity, edge) == 0) continue;
                const Vertex to = graph.get(ToVertex, edge);
                if (usedVertices[to]) continue;
                side.emplace_back(to);
                usedVertices[to] = true;
            }
        }
        for (const Vertex vertex : side) {
            usedVertices[vertex] = false;
        }
        for (const Vertex vertex : separator) {
            usedVertices[vertex] = false;
        }
        return side;
    }

    inline std::vector<Vertex> getSide(const Vertex vertex) noexcept {
        return getSide(std::vector<Vertex>(1, vertex));
    }

    inline void isolateVertex(const Vertex vertex) noexcept {
        dinic.isolateVertex(vertex);
        dinic.isolateVertex(vertex + offset);
    }

    inline const StaticFlowGraph& getFlowGraph() const noexcept {
        return dinic.getFlowGraph();
    }

private:
    const Vertex offset;
    Dinic dinic;
    std::vector<bool> usedVertices;
    std::vector<Vertex> separator;

};
