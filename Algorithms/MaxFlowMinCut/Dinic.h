#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../DataStructures/Container/Set.h"
#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/MaxFlowMinCut/FlowGraphs.h"

#include "../../Helpers/Assert.h"
#include "../../Helpers/Types.h"
#include "../../Helpers/Vector/Vector.h"

class Dinic {

public:
    struct CutEdge {
        CutEdge(const Vertex from, const Vertex to, const Edge edge) :
            from(from),
            to(to),
            edge(edge) {
        }
        Vertex from;
        Vertex to;
        Edge edge;
    };

    struct Cut {
        std::vector<CutEdge> edges;
        std::vector<Vertex> side;
    };

public:
    Dinic(DynamicFlowGraph&& dynamicGraph) :
        residualFlows(std::vector<int>(dynamicGraph.numEdges(), 0)),
        distances(std::vector<int>(dynamicGraph.numVertices(), intMax)),
        edges(dynamicGraph.numEdges(), noEdge),
        activeVertices(dynamicGraph.numVertices()),
        sourceVertices(dynamicGraph.numVertices()),
        targetVertices(dynamicGraph.numVertices()),
        capacityLimit(1 << 16),
        flow(0) {
        Graph::move(std::move(dynamicGraph), graph);
        for (const Vertex vertex : graph.vertices()) {
            edges[vertex] = graph.endEdgeFrom(vertex);
        }
        residualFlows = graph.get(Capacity);
    }

    Dinic(DynamicFlowGraph& dynamicGraph) :
        Dinic(std::move(dynamicGraph)) {
    }

    template<typename GRAPH>
    Dinic(const GRAPH& originalGraph) :
        Dinic(generateFlowGraph(originalGraph)) {
    }

    template<typename GRAPH>
    Dinic(const GRAPH& originalGraph, const std::vector<int>& capacities) :
        Dinic(generateFlowGraph(originalGraph, capacities)) {
    }

    Dinic(const Dinic&) = default;
    Dinic(Dinic&&) = default;

    Dinic& operator=(const Dinic&) = default;
    Dinic& operator=(Dinic&&) = default;

public:
    inline void run(const Vertex source, const Vertex target) noexcept {
        clear();
        sourceVertices.insert(source);
        targetVertices.insert(target);
        run();
    }

    inline void run(const std::vector<Vertex>& sources, const std::vector<Vertex>& targets) noexcept {
        clear();
        sourceVertices.insert(sources);
        targetVertices.insert(targets);
        run();
    }

    inline void clear() noexcept {
        for (const Vertex vertex : activeVertices) {
            distances[vertex] = intMax;
            edges[vertex] = graph.endEdgeFrom(vertex);
            for (const Edge edge : graph.edgesFrom(vertex)) {
                residualFlows[edge] = graph.get(Capacity, edge);
            }
        }
        activeVertices.clear();
        sourceVertices.clear();
        targetVertices.clear();
        capacityLimit = 1 << 16;
        flow = 0;
    }

    inline size_t getFlow() const noexcept {
        return flow;
    }

    inline size_t getFlow(const Edge edge) const noexcept {
        return graph.get(Capacity, edge) - residualFlows[edge];
    }

    inline std::vector<CutEdge> sourceCut() noexcept {
        return getCut<true>().edges;
    }

    inline std::vector<CutEdge> targetCut() noexcept {
        return getCut<false>().edges;
    }

    inline Cut sourceCutAndSide() noexcept {
        return getCut<true>();
    }

    inline Cut targetCutAndSide() noexcept {
        return getCut<false>();
    }

    inline StaticFlowGraph& getFlowGraph() noexcept {
        return graph;
    }

    inline const StaticFlowGraph& getFlowGraph() const noexcept {
        return graph;
    }

    inline std::vector<int>& getCapacities() noexcept {
        return graph.get(Capacity);
    }

    inline const std::vector<int>& getCapacities() const noexcept {
        return graph.get(Capacity);
    }

    inline void isolateVertex(const Vertex vertex) noexcept {
        for (const Edge edge : graph.edgesFrom(vertex)) {
            graph.set(Capacity, edge, 0);
            residualFlows[edge] = 0;
            graph.set(Capacity, graph.get(ReverseEdge, edge), 0);
            residualFlows[graph.get(ReverseEdge, edge)] = 0;
        }
    }

    inline void removeCut(const std::vector<CutEdge>& cut) noexcept {
        for (const CutEdge& edge : cut) {
            graph.set(Capacity, edge.edge, 0);
            graph.set(Capacity, graph.get(ReverseEdge, edge.edge), 0);
        }
    }

private:
    inline void run() noexcept {
        while (capacityLimit > 0) {
            capacityLimit = capacityLimit >> 1;
            while (computeDistances<false>()) {
                for (const Vertex vertex : activeVertices) {
                    edges[vertex] = graph.endEdgeFrom(vertex);
                }
                size_t newFlow = findBlockingPath<false>();
                while (newFlow > 0) {
                    flow += newFlow;
                    newFlow = findBlockingPath<false>();
                }
            }
        }
    }

    template<bool FORWARD>
    inline bool computeDistances() noexcept {
        for (const Vertex vertex : activeVertices) {
            distances[vertex] = intMax;
        }
        currentQueue.clear();
        nextQueue.clear();
        for (const Vertex vertex : (FORWARD ? sourceVertices : targetVertices)) {
            distances[vertex] = 0;
            nextQueue.emplace_back(vertex);
            activeVertices.insert(vertex);
        }
        int distance = 0;
        bool connected = false;
        while (!nextQueue.empty()) {
            nextQueue.swap(currentQueue);
            distance++;
            while (!currentQueue.empty()) {
                const Vertex from = currentQueue.back();
                currentQueue.pop_back();
                for (const Edge edge : graph.edgesFrom(from)) {
                    if (FORWARD) {
                        if (residualFlows[edge] <= capacityLimit) continue;
                    } else {
                        if (residualFlows[graph.get(ReverseEdge, edge)] <= capacityLimit) continue;
                    }
                    const Vertex to = graph.get(ToVertex, edge);
                    if (distances[to] != intMax) continue;
                    distances[to] = distance;
                    nextQueue.emplace_back(to);
                    activeVertices.insert(to);
                    if (FORWARD) {
                        if (targetVertices.contains(to)) connected = true;
                    } else {
                        if (sourceVertices.contains(to)) connected = true;
                    }
                }
            }
        }
        return connected;
    }

    template<bool INCREASING_DISTANCE>
    inline size_t findBlockingPath() noexcept {
        std::vector<Edge> usedEdges;
        std::vector<Vertex> openVertices;
        for (const Vertex source : sourceVertices) {
            if (edges[source] == graph.beginEdgeFrom(source)) continue;
            openVertices.emplace_back(source);
        }
        while (!openVertices.empty()) {
            const Vertex from = openVertices.back();
            if (targetVertices.contains(from)) break;
            if (edges[from] == graph.beginEdgeFrom(from)) {
                openVertices.pop_back();
                if (!usedEdges.empty()) usedEdges.pop_back();
            } else {
                edges[from]--;
                if (residualFlows[edges[from]] <= capacityLimit) continue;
                const Vertex to = graph.get(ToVertex, edges[from]);
                if (INCREASING_DISTANCE) {
                    if (distances[from] + 1 != distances[to]) continue;
                } else {
                    if (distances[from] - 1 != distances[to]) continue;
                }
                if (edges[to] == graph.beginEdgeFrom(to)) continue;
                usedEdges.emplace_back(edges[from]);
                openVertices.emplace_back(to);
            }
        }
        AssertMsg(usedEdges.empty() || targetVertices.contains(graph.get(ToVertex, usedEdges.back())), "The blocking path does not end with a target vertex!");
        if (usedEdges.empty()) return 0;
        size_t newFlow = residualFlows[usedEdges.back()];
        for (const Edge edge : usedEdges) {
            AssertMsg(residualFlows[edge] > 0, "An edge with residual flow " << residualFlows[edge] << " was used!");
            newFlow = std::min<size_t>(newFlow, residualFlows[edge]);
        }
        for (const Edge edge : usedEdges) {
            residualFlows[edge] -= newFlow;
            residualFlows[graph.get(ReverseEdge, edge)] += newFlow;
        }
        return newFlow;
    }

    template<bool FORWARD>
    inline Cut getCut() noexcept {
        Cut cut;
        size_t cutSize = 0;
        computeDistances<FORWARD>();
        for (const Vertex from : activeVertices) {
            if (distances[from] == intMax) continue;
            cut.side.emplace_back(from);
            for (const Edge edge : graph.edgesFrom(from)) {
                const Vertex to = graph.get(ToVertex, edge);
                if (distances[to] != intMax) continue;
                if (FORWARD) {
                    cut.edges.emplace_back(from, to, edge);
                } else {
                    cut.edges.emplace_back(to, from, graph.get(ReverseEdge, edge));
                }
                if (graph.get(Capacity, cut.edges.back().edge) == 0) {
                    cut.edges.pop_back();
                } else {
                    cutSize += graph.get(Capacity, cut.edges.back().edge);
                    AssertMsg(residualFlows[cut.edges.back().edge] == 0, "Non saturated edge " << cut.edges.back().edge << " from " << cut.edges.back().from << " to " << cut.edges.back().to << " is part of the cut!");
                }
            }
        }
        AssertMsg(cutSize == flow, "Maximum flow is " << flow << " but cut size is " << cutSize << "!");
        return cut;
    }

private:
    StaticFlowGraph graph;
    std::vector<int> residualFlows;
    std::vector<int> distances;
    std::vector<Edge> edges;

    IndexedSet<false, Vertex> activeVertices;
    IndexedSet<false, Vertex> sourceVertices;
    IndexedSet<false, Vertex> targetVertices;

    std::vector<Vertex> currentQueue;
    std::vector<Vertex> nextQueue;

    int capacityLimit;
    size_t flow;

};
