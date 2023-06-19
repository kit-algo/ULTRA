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

template<bool USE_STOP_BUCKETS, bool USE_TARGET_BUCKETS, bool STALL_ON_DEMAND = true, bool DEBUG = false>
class UPQuery {

public:
    constexpr static bool UseStopBuckets = USE_STOP_BUCKETS;
    constexpr static bool UseTargetBuckets = USE_TARGET_BUCKETS;
    constexpr static bool StallOnDemand = STALL_ON_DEMAND;
    constexpr static bool Debug = DEBUG;
    using StopGraph = Meta::IF<UseStopBuckets, BucketGraph, SweepGraph>;
    using TargetGraph = Meta::IF<UseTargetBuckets, BucketGraph, SweepGraph>;
    using Type = UPQuery<UseStopBuckets, UseTargetBuckets, StallOnDemand, Debug>;
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

public:
    UPQuery(const CHGraph& forward, const CHGraph& backward, const Order& order, const Vertex::ValueType numberOfStops, const IndexedSet<false, Vertex>& originalTargets) :
        graph {forward, backward},
        contractionOrder(order),
        positionInOrder(Construct::Invert, contractionOrder),
        sweepStart(noVertex),
        stops(graph[FORWARD].numVertices(), Vector::id<Vertex>(numberOfStops)),
        targets(originalTargets),
        Q(graph[FORWARD].numVertices()),
        distance(graph[FORWARD].numVertices(), never),
        parent(graph[FORWARD].numVertices(), noVertex),
        timestamp(graph[FORWARD].numVertices(), 0),
        currentTimestamp(0),
        bucketSources(graph[FORWARD].numVertices()) {

        std::cout << "Reordering vertices... " << std::endl;
        timer.restart();
        reorderVertices();
        std::cout << String::musToString(timer.elapsedMicroseconds()) << std::endl;

        std::cout << "Building upward sweep graph... " << std::endl;
        timer.restart();
        buildUpwardSweepGraph();
        std::cout << String::musToString(timer.elapsedMicroseconds()) << std::endl;

        std::cout << "Creating bucket builder... " << std::endl;
        timer.restart();
        BucketBuilderType bucketBuilder(graph[FORWARD], graph[BACKWARD]);
        std::cout << String::musToString(timer.elapsedMicroseconds()) << std::endl;

        std::cout << "Building stop graph... " << std::endl;
        timer.restart();
        if constexpr (UseStopBuckets) {
            stopGraph.initialize(bucketBuilder.build(stops));
        } else {
            stopGraph.build(graph[BACKWARD], stops, false, false);
        }
        std::cout << String::musToString(timer.elapsedMicroseconds()) << std::endl;

        std::cout << "Building target graph... " << std::endl;
        timer.restart();
        if constexpr (UseTargetBuckets) {
            targetGraph.initialize(bucketBuilder.build(targets));
        } else {
            targetGraph.build(graph[BACKWARD], targets, false, false);
        }
        std::cout << String::musToString(timer.elapsedMicroseconds()) << std::endl;

        for (const Vertex vertex : graph[FORWARD].vertices()) {
            dijkstraLabel.emplace_back(&distance[vertex]);
        }
    }

    UPQuery(const CH& ch, const Order& order, const Vertex::ValueType numberOfStops, const IndexedSet<false, Vertex>& targets, const int direction = FORWARD) :
        UPQuery(ch.getGraph(direction), ch.getGraph(!direction), order, numberOfStops, targets) {
    }

    inline void initialize() noexcept {
        clear();
    }

    inline void clear() noexcept {
        Q.clear();
        currentTimestamp++;
        bucketSources.clear();
        sweepStart = noVertex;
    }

    template<bool FOR_SWEEP>
    inline void addSource(const Vertex vertex, const int initialDistance, const Vertex parentVertex) noexcept {
        addSourceInternal<FOR_SWEEP>(originalToInternal(vertex), initialDistance, parentVertex);
    }

    inline void upwardSearch() noexcept {
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

    inline void upwardSweep() noexcept {
        if constexpr (Debug) {
            std::cout << "Running upward sweep from " << sweepStart << "/" << Vertex(upwardSweepGraph.graph.numVertices()) << std::endl;
            timer.restart();
        }

        for (Vertex sweepV = sweepStart; sweepV < upwardSweepGraph.graph.numVertices(); sweepV++) {
            const Vertex v = upwardSweepGraph.internalToExternal(sweepV);
            check(v);
            const int oldDistance = distance[v];
            for (const Edge edge : upwardSweepGraph.graph.edgesFrom(sweepV)) {
                const Vertex u = upwardSweepGraph.toVertex[edge];
                check(u);
                const int weight = upwardSweepGraph.graph.get(Weight, edge);
                const int newDistance = distance[u] + weight;
                const bool update = newDistance < distance[v];
                distance[v] = branchlessConditional(update, newDistance, distance[v]);
                parent[v] = branchlessConditional(update, parent[u], parent[v]);
            }
            if constexpr (UseTargetBuckets) {
                if (distance[v] < oldDistance) bucketSources.insert(v);
            } else {
                suppressUnusedParameterWarning(oldDistance);
            }
        }

        if constexpr (Debug) {
            std::cout << "Time: " << String::musToString(timer.elapsedMicroseconds()) << std::endl;
        }
    }

    inline void downwardSearchToTargets() noexcept {
        if constexpr (UseTargetBuckets) {
            evaluateTargetBuckets();
        } else {
            downwardSweepToTargets();
        }
    }

    inline void downwardSearchToStops() noexcept {
        if constexpr (UseStopBuckets) {
            evaluateStopBuckets();
        } else {
            downwardSweepToStops();
        }
    }

    inline int getDistance(const Vertex vertex) noexcept {
        const Vertex internalVertex = originalToInternal(vertex);
        check(internalVertex);
        return distance[internalVertex];
    }

    inline Vertex getParent(const Vertex vertex) noexcept {
        const Vertex internalVertex = originalToInternal(vertex);
        check(internalVertex);
        return Vertex(parent[internalVertex]);
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

    inline void buildUpwardSweepGraph() noexcept {
        upwardSweepGraph.build(graph[FORWARD], stops, true, true);
        sweepStartOf.resize(upwardSweepGraph.graph.numVertices(), noVertex);
        for (const Vertex to : upwardSweepGraph.graph.vertices()) {
            for (const Edge edge : upwardSweepGraph.graph.edgesFrom(to)) {
                const Vertex from = upwardSweepGraph.graph.get(ToVertex, edge);
                if (sweepStartOf[from] != noVertex) continue;
                sweepStartOf[from] = to;
            }
        }
    }

    template<bool FOR_SWEEP>
    inline void addSourceInternal(const Vertex vertex, const int initialDistance, const Vertex parentVertex) noexcept {
        check(vertex);
        if (initialDistance >= distance[vertex]) return;
        distance[vertex] = initialDistance;
        parent[vertex] = parentVertex;
        if constexpr (FOR_SWEEP) {
            AssertMsg(upwardSweepGraph.externalToInternal(vertex) < upwardSweepGraph.graph.numVertices(), "Vertex is not in sweep graph! (original: "<< internalToOriginal(vertex) << ", CH: " << vertex << ", sweep: " << upwardSweepGraph.externalToInternal(vertex) << ")");
            sweepStart = std::min(sweepStart, sweepStartOf[upwardSweepGraph.externalToInternal(vertex)]);
            if constexpr (UseTargetBuckets) {
                bucketSources.insert(vertex);
            }
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
        if constexpr (UseStopBuckets) {
            bucketSources.insert(u);
        }
    }

    template<bool T = UseStopBuckets, typename = std::enable_if_t<T == UseStopBuckets && !T>>
    inline void downwardSweepToStops() noexcept {
        if constexpr (Debug) {
            std::cout << "Running downward sweep to stops" << std::endl;
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

        if constexpr (Debug) {
            std::cout << "Time: " << String::musToString(timer.elapsedMicroseconds()) << std::endl;
        }
    }

    template<bool T = UseStopBuckets, typename = std::enable_if_t<T == UseStopBuckets && T>>
    inline void evaluateStopBuckets() noexcept {
        if constexpr (Debug) {
            std::cout << "Evaluating stop buckets" << std::endl;
            timer.restart();
        }

        for (const Vertex vertex : bucketSources) {
            AssertMsg(dijkstraLabel[vertex].distance[0] != never, "Reached vertex " << vertex << " has no distance set!");
            for (const Edge edge : stopGraph.graph.edgesFrom(vertex)) {
                const int newDistance = distance[vertex] + stopGraph.graph.get(Weight, edge);
                const Vertex poi = stopGraph.graph.get(ToVertex, edge);
                AssertMsg(stops.contains(poi), "POI " << poi << " is not a stop!");
                check(poi);
                if (newDistance < distance[poi]) {
                    distance[poi] = newDistance;
                    parent[poi] = parent[vertex];
                }
            }
        }

        if constexpr (Debug) {
            std::cout << "Time: " << String::musToString(timer.elapsedMicroseconds()) << std::endl;
        }
    }

    template<bool T = UseTargetBuckets, typename = std::enable_if_t<T == UseTargetBuckets && !T>>
    inline void downwardSweepToTargets() noexcept {
        if constexpr (Debug) {
            std::cout << "Running downward sweep to targets" << std::endl;
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

        if constexpr (Debug) {
            std::cout << "Time: " << String::musToString(timer.elapsedMicroseconds()) << std::endl;
        }
    }

    template<bool T = UseTargetBuckets, typename = std::enable_if_t<T == UseTargetBuckets && T>>
    inline void evaluateTargetBuckets() noexcept {
        if constexpr (Debug) {
            std::cout << "Evaluating target buckets" << std::endl;
            timer.restart();
        }

        for (const Vertex vertex : bucketSources) {
            AssertMsg(parent[vertex] != int(noVertex), "Invalid parent!");
            for (const Edge edge : targetGraph.graph.edgesFrom(vertex)) {
                const int newDistance = distance[vertex] + targetGraph.graph.get(Weight, edge);
                const Vertex poi = targetGraph.graph.get(ToVertex, edge);
                check(poi);
                if (newDistance < distance[poi]) {
                    distance[poi] = newDistance;
                    parent[poi] = parent[vertex];
                }
            }
        }

        if constexpr (Debug) {
            std::cout << "Time: " << String::musToString(timer.elapsedMicroseconds()) << std::endl;
        }
    }

    inline Vertex originalToInternal(const Vertex vertex) const noexcept {
        return positionInOrder.permutate(vertex);
    }

    inline Vertex internalToOriginal(const Vertex vertex) const noexcept {
        return Vertex(contractionOrder[vertex]);
    }

    inline void check(const Vertex vertex) noexcept {
        if (timestamp[vertex] != currentTimestamp) {
            distance[vertex] = never;
            parent[vertex] = int(noVertex);
            timestamp[vertex] = currentTimestamp;
        }
    }

private:
    CHGraph graph[2];
    SweepGraph upwardSweepGraph;
    StopGraph stopGraph;
    TargetGraph targetGraph;

    const Order contractionOrder;
    const Permutation positionInOrder;

    Vertex sweepStart;
    std::vector<Vertex> sweepStartOf;
    IndexedSet<false, Vertex> stops;
    IndexedSet<false, Vertex> targets;

    ExternalKHeap<2, DijkstraLabel> Q;
    std::vector<DijkstraLabel> dijkstraLabel;
    std::vector<int> distance;
    std::vector<int> parent;
    std::vector<int> timestamp;
    int currentTimestamp;

    IndexedSet<false, Vertex> bucketSources;

    Timer timer;
};

}
