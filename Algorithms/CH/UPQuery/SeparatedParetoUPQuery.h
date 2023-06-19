#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../../DataStructures/CH/UPGraphs.h"
#include "../CH.h"
#include "../CHUtils.h"

#include "../../../Helpers/Helpers.h"
#include "../../../Helpers/Types.h"
#include "../../../Helpers/Timer.h"
#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/String/String.h"
#include "../../../Helpers/Vector/Vector.h"

#include "../../../DataStructures/Container/ExternalKHeap.h"
#include "../../../DataStructures/Container/Set.h"
#include "BucketBuilder.h"

namespace CH {

template<bool STALL_ON_DEMAND = true, bool DEBUG = false>
class SeparatedParetoUPQuery {

public:
    constexpr static bool StallOnDemand = STALL_ON_DEMAND;
    constexpr static bool Debug = DEBUG;
    using Type = SeparatedParetoUPQuery<StallOnDemand, Debug>;
    using BucketBuilderType = BucketBuilder<StallOnDemand, Debug>;

private:
    struct DijkstraLabel : public ExternalKHeapElement {
        DijkstraLabel(int* const distance) :
            ExternalKHeapElement(),
            distance(distance) {
        }

        inline int getDistance() const noexcept {
            return *distance;
        }

        inline void setDistance(const int newDistance) noexcept {
            *distance = newDistance;
        }

        inline bool hasSmallerKey(const DijkstraLabel* other) const noexcept {return getDistance() < other->getDistance();}

        int* const distance;
    };

    struct TargetLabel {
        TargetLabel(const int distance = never, const Vertex parent = noVertex) :
            distance(distance),
            parent(parent) {
        }

        int distance;
        Vertex parent;
    };

public:
    SeparatedParetoUPQuery(const TransferGraph&, const CHGraph& forward, const CHGraph& backward, const Order&& order, const Vertex::ValueType numberOfStops, const IndexedSet<false, Vertex>& originalTargets) :
        graph {forward, backward},
        contractionOrder(std::move(order)),
        positionInOrder(Construct::Invert, contractionOrder),
        sweepStart(noVertex),
        stops(graph[FORWARD].numVertices(), Vector::id<Vertex>(numberOfStops)),
        targets(originalTargets),
        targetId(graph[FORWARD].numVertices(), -1),
        Q(graph[FORWARD].numVertices()),
        distance(graph[FORWARD].numVertices(), never),
        parent(graph[FORWARD].numVertices(), noVertex),
        timestamp(graph[FORWARD].numVertices(), 0),
        currentTimestamp(0),
        queryStartTimestamp(0) {
        reorderVertices();
        upwardSweepGraph.build(graph[FORWARD], stops, true, true);
        reverseUpwardSweepGraph.build(graph[FORWARD], stops, true, false);
        IndexedSet<false, Vertex> stopsAndTargets = stops;
        for (const Vertex target : targets) {
            stopsAndTargets.insert(target);
        }
        stopGraph.build(graph[BACKWARD], stopsAndTargets, false, false);
        size_t id = 0;
        for (const Vertex target : targets) {
            targetId[target] = id++;
        }
        targetGraph.build(graph[BACKWARD], targets, false, false);
        reverseTargetGraph.build(graph[BACKWARD], targets, false, true);
        for (const Vertex vertex : graph[FORWARD].vertices()) {
            dijkstraLabel.emplace_back(&distance[vertex]);
        }
    }

    SeparatedParetoUPQuery(const TransferGraph& transferGraph, const CH& ch, const Order&& order, const Vertex::ValueType numberOfStops, const IndexedSet<false, Vertex>& targets, const int direction = FORWARD) :
        SeparatedParetoUPQuery(transferGraph, ch.getGraph(direction), ch.getGraph(!direction), std::move(order), numberOfStops, targets) {
    }

    inline void initialize() noexcept {
        clear();
    }

    inline void clear() noexcept {
        Q.clear();
        currentTimestamp++;
        queryStartTimestamp = currentTimestamp;
        sweepStart = noVertex;
        targetLabels.clear();
    }

    template<bool FOR_SWEEP>
    inline void addSource(const Vertex vertex, const int initialDistance, const Vertex parentVertex) noexcept {
        addSourceInternal<FOR_SWEEP>(originalToInternal(vertex), initialDistance, parentVertex);
    }

    inline void startNewRound() noexcept {
        currentTimestamp++;
        targetLabels.emplace_back(targets.size());
        sweepStart = noVertex;
    }

    inline void initialUpwardSearch() noexcept {
        if constexpr (Debug) {
            std::cout << "Running upward search with " << Q.size() << " queue elements" << std::endl;
            timer.restart();
        }

        while (!Q.empty()) {
            settle();
        }

        if constexpr (Debug) {
            std::cout << "Time: " << String::musToString(timer.elapsedMicroseconds()) << std::endl;
        }
    }

    inline void relaxFinalTransfers(const bool restricted = true) noexcept {
        upwardSweep(restricted);
        downwardSearchToTargets(restricted);
    }

    inline void upwardSweep(const bool restricted = true) noexcept {
        if (restricted) {
            restrictedUpwardSweep();
        } else {
            fullUpwardSweep();
        }
    }

    inline void downwardSearchToTargets(const bool restricted = true) noexcept {
        if (restricted) {
            restrictedDownwardSweepToTargets();
        } else {
            fullDownwardSweepToTargets();
        }
    }

    inline void downwardSearchToStopsAndTargets() noexcept {
        if constexpr (Debug) {
            std::cout << "Running downward sweep to stops and targets" << std::endl;
            timer.restart();
        }

        for (const Vertex sweepV : stopGraph.graph.vertices()) {
            const Vertex v = stopGraph.internalToExternal(sweepV);
            check(v);
            for (const Edge edge : stopGraph.graph.edgesFrom(sweepV)) {
                const Vertex u = stopGraph.toVertex[edge];
                const int weight = stopGraph.graph.get(Weight, edge);
                const int newDistance = distance[u] + weight;
                const bool update = newDistance < distance[v];
                distance[v] = branchlessConditional(update, newDistance, distance[v]);
                parent[v] = branchlessConditional(update, parent[u], parent[v]);
            }
        }

        updateTargetLabels();

        if constexpr (Debug) {
            std::cout << "Time: " << String::musToString(timer.elapsedMicroseconds()) << std::endl;
        }
    }

    inline int getDistance(const size_t round, const Vertex vertex) noexcept {
        suppressUnusedParameterWarning(round);
        const Vertex internalVertex = originalToInternal(vertex);
        check(internalVertex);
        return distance[internalVertex];
    }

    inline Vertex getParent(const Vertex vertex) noexcept {
        const Vertex internalVertex = originalToInternal(vertex);
        check(internalVertex);
        return Vertex(parent[internalVertex]);
    }

    inline const TargetLabel& getTargetLabel(const size_t round, const Vertex vertex) const noexcept {
        const Vertex internalVertex = originalToInternal(vertex);
        return targetLabels[round][targetId[internalVertex]];
    }

    inline int getTargetDistance(const size_t round, const Vertex vertex) const noexcept {
        return getTargetLabel(round, vertex).distance;
    }

    inline Vertex getTargetParent(const size_t round, const Vertex vertex) const noexcept {
        return getTargetLabel(round, vertex).parent;
    }

    inline size_t numRounds() const noexcept {
        return targetLabels.size();
    }

    inline long long getUpwardSweepGraphVertices() const noexcept {
        return upwardSweepGraph.graph.numVertices();
    }

    inline long long getUpwardSweepGraphEdges() const noexcept {
        return upwardSweepGraph.graph.numEdges();
    }

    inline long long getStopGraphVertices() const noexcept {
        return stopGraph.graph.numVertices();
    }

    inline long long getStopGraphEdges() const noexcept {
        return stopGraph.graph.numEdges();
    }

    inline long long getTargetGraphVertices() const noexcept {
        return targetGraph.graph.numVertices();
    }

    inline long long getTargetGraphEdges() const noexcept {
        return targetGraph.graph.numEdges();
    }

private:
    inline void reorderVertices() noexcept {
        reorder(graph[FORWARD]);
        reorder(graph[BACKWARD]);
        stops.applyPermutation(positionInOrder);
        targets.applyPermutation(positionInOrder);
    }

    inline void reorder(CHGraph& graph) noexcept {
        graph.applyVertexPermutation(positionInOrder);
        graph.sortEdges(ToVertex);
    }

    template<bool FOR_SWEEP>
    inline void addSourceInternal(const Vertex vertex, const int initialDistance, const Vertex parentVertex) noexcept {
        check(vertex);
        if (initialDistance >= distance[vertex]) return;
        distance[vertex] = initialDistance;
        parent[vertex] = parentVertex;
        if constexpr (FOR_SWEEP) {
            AssertMsg(upwardSweepGraph.externalToInternal(vertex) < upwardSweepGraph.graph.numVertices(), "Vertex is not in sweep graph! (original: "<< internalToOriginal(vertex) << ", CH: " << vertex << ", sweep: " << upwardSweepGraph.externalToInternal(vertex) << ")");
            sweepStart = std::min(sweepStart, upwardSweepGraph.externalToInternal(vertex));
            timestamp[vertex] = currentTimestamp;
        } else {
            Q.update(&dijkstraLabel[vertex]);
        }
    }

    inline void settle() noexcept {
        const Vertex u = Vertex(Q.extractFront() - &(dijkstraLabel[0]));
        AssertMsg(u < graph[FORWARD].numVertices(), u << " is not a valid vertex!");
        if constexpr (StallOnDemand) {
            for (Edge edge : graph[BACKWARD].edgesFrom(u)) {
                const Vertex v = graph[BACKWARD].get(ToVertex, edge);
                check(v);
                if (distance[v] < distance[u] - graph[BACKWARD].get(Weight, edge)) return;
            }
        }
        for (Edge edge : graph[FORWARD].edgesFrom(u)) {
            const Vertex v = graph[FORWARD].get(ToVertex, edge);
            const int newDistance = distance[u] + graph[FORWARD].get(Weight, edge);
            check(v);
            if (distance[v] > newDistance) {
                Q.update(&dijkstraLabel[v]);
                distance[v] = newDistance;
                AssertMsg(parent[u] != int(noVertex), "Invalid parent!");
                parent[v] = parent[u];
            }
        }
    }

    inline void fullUpwardSweep() noexcept {
        if constexpr (Debug) {
            std::cout << "Running full upward sweep from " << sweepStart << "/" << Vertex(upwardSweepGraph.graph.numVertices()) << std::endl;
            timer.restart();
        }

        for (Vertex sweepV = sweepStart; sweepV < upwardSweepGraph.graph.numVertices(); sweepV++) {
            const Vertex v = upwardSweepGraph.internalToExternal(sweepV);
            check(v);
            for (const Edge edge : upwardSweepGraph.graph.edgesFrom(sweepV)) {
                const Vertex u = upwardSweepGraph.toVertex[edge];
                check(u);
                const int weight = upwardSweepGraph.graph.get(Weight, edge);
                const int newDistance = distance[u] + weight;
                const bool update = newDistance < distance[v];
                distance[v] = branchlessConditional(update, newDistance, distance[v]);
                parent[v] = branchlessConditional(update, parent[u], parent[v]);
            }
        }

        if constexpr (Debug) {
            std::cout << "Time: " << String::musToString(timer.elapsedMicroseconds()) << std::endl;
        }
    }

    inline void restrictedUpwardSweep() noexcept {
        if constexpr (Debug) {
            std::cout << "Running restricted upward sweep from " << sweepStart << "/" << Vertex(reverseUpwardSweepGraph.graph.numVertices()) << std::endl;
            timer.restart();
        }

        for (Vertex sweepV = sweepStart; sweepV < reverseUpwardSweepGraph.graph.numVertices(); sweepV++) {
            const Vertex v = reverseUpwardSweepGraph.internalToExternal(sweepV);
            if (timestamp[v] != currentTimestamp) continue;
            for (const Edge edge : reverseUpwardSweepGraph.graph.edgesFrom(sweepV)) {
                const Vertex u = reverseUpwardSweepGraph.toVertex[edge];
                check(u);
                const int weight = reverseUpwardSweepGraph.graph.get(Weight, edge);
                const int newDistance = distance[v] + weight;
                const bool update = newDistance < distance[u];
                distance[u] = branchlessConditional(update, newDistance, distance[u]);
                parent[u] = branchlessConditional(update, parent[v], parent[u]);
                timestamp[u] = branchlessConditional(update, currentTimestamp, timestamp[u]);
            }
        }

        if constexpr (Debug) {
            std::cout << "Time: " << String::musToString(timer.elapsedMicroseconds()) << std::endl;
        }
    }

    inline void fullDownwardSweepToTargets() noexcept {
        if constexpr (Debug) {
            std::cout << "Running full downward sweep to targets" << std::endl;
            timer.restart();
        }

        for (const Vertex sweepV : targetGraph.graph.vertices()) {
            const Vertex v = targetGraph.internalToExternal(sweepV);
            check(v);
            for (const Edge edge : targetGraph.graph.edgesFrom(sweepV)) {
                const Vertex u = targetGraph.toVertex[edge];
                const int weight = targetGraph.graph.get(Weight, edge);
                const int newDistance = distance[u] + weight;
                const bool update = newDistance < distance[v];
                distance[v] = branchlessConditional(update, newDistance, distance[v]);
                parent[v] = branchlessConditional(update, parent[u], parent[v]);
            }
        }

        updateTargetLabels();

        if constexpr (Debug) {
            std::cout << "Time: " << String::musToString(timer.elapsedMicroseconds()) << std::endl;
        }
    }

    inline void restrictedDownwardSweepToTargets() noexcept {
        if constexpr (Debug) {
            std::cout << "Running restricted downward sweep to targets" << std::endl;
            timer.restart();
        }

        for (const Vertex sweepV : reverseTargetGraph.graph.vertices()) {
            const Vertex v = reverseTargetGraph.internalToExternal(sweepV);
            if (timestamp[v] != currentTimestamp) continue;
            for (const Edge edge : reverseTargetGraph.graph.edgesFrom(sweepV)) {
                const Vertex u = reverseTargetGraph.toVertex[edge];
                check(u);
                const int weight = reverseTargetGraph.graph.get(Weight, edge);
                const int newDistance = distance[v] + weight;
                const bool update = newDistance < distance[u];
                distance[u] = branchlessConditional(update, newDistance, distance[u]);
                parent[u] = branchlessConditional(update, parent[v], parent[u]);
                timestamp[u] = branchlessConditional(update, currentTimestamp, timestamp[u]);
            }
        }

        updateTargetLabels();

        if constexpr (Debug) {
            std::cout << "Time: " << String::musToString(timer.elapsedMicroseconds()) << std::endl;
        }
    }

    inline void updateTargetLabels() noexcept {
        size_t id = 0;
        for (const Vertex target : targets) {
            check(target);
            TargetLabel& targetLabel = targetLabels.back()[id++];
            targetLabel.distance = distance[target];
            targetLabel.parent = Vertex(parent[target]);
        }
    }

    inline Vertex originalToInternal(const Vertex vertex) const noexcept {
        return positionInOrder.permutate(vertex);
    }

    inline Vertex internalToOriginal(const Vertex vertex) const noexcept {
        return Vertex(contractionOrder[vertex]);
    }

    inline void check(const Vertex vertex) noexcept {
        if (timestamp[vertex] < queryStartTimestamp) {
            distance[vertex] = never;
            parent[vertex] = int(noVertex);
            timestamp[vertex] = queryStartTimestamp;
        }
    }

private:
    CHGraph graph[2];
    SweepGraph upwardSweepGraph;
    SweepGraph reverseUpwardSweepGraph;
    SweepGraph stopGraph;
    SweepGraph targetGraph;
    SweepGraph reverseTargetGraph;

    const Order contractionOrder;
    const Permutation positionInOrder;

    Vertex sweepStart;
    IndexedSet<false, Vertex> stops;
    IndexedSet<false, Vertex> targets;
    std::vector<size_t> targetId;

    ExternalKHeap<2, DijkstraLabel> Q;
    std::vector<DijkstraLabel> dijkstraLabel;
    std::vector<int> distance;
    std::vector<int> parent;
    std::vector<int> timestamp;
    std::vector<std::vector<TargetLabel>> targetLabels;
    int currentTimestamp;
    int queryStartTimestamp;

    Timer timer;
};

}
