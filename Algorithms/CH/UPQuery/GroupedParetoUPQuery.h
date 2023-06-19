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

namespace CH {


template<bool STALL_ON_DEMAND = true, bool DEBUG = false, size_t GROUPED_ROUNDS = 6>
class GroupedParetoUPQuery {

public:
    constexpr static bool StallOnDemand = STALL_ON_DEMAND;
    constexpr static bool Debug = DEBUG;
    constexpr static size_t GroupedRounds = GROUPED_ROUNDS;
    using Type = GroupedParetoUPQuery<StallOnDemand, Debug, GroupedRounds>;

private:
    struct GroupedLabel {
        GroupedLabel() {
            clear();
        }

        inline void clear() noexcept {
            std::fill(distance, distance + GroupedRounds, never);
            std::fill(parent, parent + GroupedRounds, noVertex);
        }

        inline int getMinDistance() noexcept {
            return *std::min_element(distance, distance + GroupedRounds);
        }

        int distance[GroupedRounds];
        int parent[GroupedRounds];
        int timestamp;
    };

    struct Distance : public ExternalKHeapElement {
        Distance() : ExternalKHeapElement(), distance(never), parent(noVertex), timestamp(0) {}
        inline bool hasSmallerKey(const Distance* other) const noexcept {return distance < other->distance;}
        int distance;
        Vertex parent;
        int timestamp;
    };

    struct TargetLabel {
        TargetLabel(const int distance = never, const Vertex parent = noVertex) :
            distance(distance),
            parent(parent),
            timestamp(0) {
        }

        int distance;
        Vertex parent;
        int timestamp;
    };

public:
    GroupedParetoUPQuery(const TransferGraph& transferGraph, const CHGraph& forward, const CHGraph& backward, const Order&& order, const Vertex::ValueType numberOfStops, const IndexedSet<false, Vertex>& originalTargets) :
        graph {forward, backward},
        searchGraph(transferGraph),
        contractionOrder(std::move(order)),
        positionInOrder(Construct::Invert, contractionOrder),
        sweepStart(noVertex),
        stops(graph[FORWARD].numVertices(), Vector::id<Vertex>(numberOfStops)),
        targets(originalTargets),
        Q(graph[FORWARD].numVertices()),
        distance(graph[FORWARD].numVertices()),
        groupedLabel(graph[FORWARD].numVertices()),
        round(-1),
        timestamp(0),
        targetId(graph[FORWARD].numVertices(), -1) {
        reorderVertices();
        buildUpwardSweepGraph();
        stopGraph.build(graph[BACKWARD], stops, false, false);
        size_t id = 0;
        for (const Vertex target : targets) {
            targetId[target] = id++;
        }
        targetGraph.build(graph[BACKWARD], targets, false, false);
    }

    GroupedParetoUPQuery(const TransferGraph& transferGraph, const CH& ch, const Order&& order, const Vertex::ValueType numberOfStops, const IndexedSet<false, Vertex>& targets, const int direction = FORWARD) :
        GroupedParetoUPQuery(transferGraph, ch.getGraph(direction), ch.getGraph(!direction), std::move(order), numberOfStops, targets) {
    }

    inline void initialize() noexcept {
        clear();
    }

    inline void clear() noexcept {
        Q.clear();
        timestamp++;
        queryStartTimestamp = timestamp;
        round = -1;
        sweepStart = noVertex;
    }

    inline void startNewRound() noexcept {
        AssertMsg(Q.empty(), "Queue should be empty!");
        round++;
        if (round == GroupedRounds) {
            timestamp++;
        }
        if (round >= targetLabel.size() + GroupedRounds) {
            targetLabel.emplace_back(targets.size());
        }
    }

    template<bool FOR_SWEEP>
    inline void addSource(const Vertex vertex, const int initialDistance, const Vertex parentVertex) noexcept {
        addSource<FOR_SWEEP>(vertex, initialDistance, parentVertex, round);
    }

    template<bool FOR_SWEEP>
    inline void addSource(const Vertex vertex, const int initialDistance, const Vertex parentVertex, const size_t numTrips) noexcept {
        const Vertex originalVertex = originalToInternal(vertex);
        if (round >= GroupedRounds) {
            addDijkstraSourceInternal(originalVertex, initialDistance, parentVertex);
        } else {
            addGroupedSourceInternal<FOR_SWEEP>(originalVertex, initialDistance, parentVertex, numTrips);
        }
    }

    inline void relaxFinalTransfers() noexcept {
        if (round == GroupedRounds - 1) {
            upwardSweep();
            downwardSearchToTargets();
        } else if (round >= GroupedRounds) {
            dijkstraSearch();
        }
    }

    inline void finalize() noexcept {
        if (round < GroupedRounds) {
            upwardSweep();
            downwardSearchToTargets();
        }
    }

    inline void dijkstraSearch() noexcept {
        if constexpr (Debug) {
            std::cout << "Running Dijkstra search with " << Q.size() << " queue elements" << std::endl;
            timer.restart();
        }

        while (!Q.empty()) {
            Distance* distanceU = Q.extractFront();
            const Vertex u = Vertex(distanceU - &(distance[0]));
            AssertMsg(u < searchGraph.numVertices(), u << " is not a valid vertex!");
            for (Edge edge : searchGraph.edgesFrom(u)) {
                const Vertex v = searchGraph.get(ToVertex, edge);
                const int newDistance = distanceU->distance + searchGraph.get(TravelTime, edge);
                updateDistance(v);
                if (distance[v].distance > newDistance) {
                    distance[v].distance = newDistance;
                    distance[v].parent = distance[u].parent;
                    Q.update(&distance[v]);
                }
            }
            if (targets.contains(u)) {
                TargetLabel& uLabel = getTargetLabel(u, round);
                uLabel.distance = distance[u].distance;
                uLabel.parent = distance[u].parent;
            }
        }

        if constexpr (Debug) {
            std::cout << "Time: " << String::musToString(timer.elapsedMicroseconds()) << std::endl;
        }
    }

    inline void initialUpwardSearch() noexcept {
        if constexpr (Debug) {
            std::cout << "Running upward search with " << Q.size() << " queue elements" << std::endl;
            timer.restart();
        }

        while (!Q.empty()) {
            settleInitial();
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
            GroupedLabel& vLabel = getGroupedLabel(v);
            for (const Edge edge : upwardSweepGraph.graph.edgesFrom(sweepV)) {
                const Vertex u = upwardSweepGraph.toVertex[edge];
                const GroupedLabel& uLabel = getGroupedLabel(u);
                const int weight = upwardSweepGraph.graph.get(Weight, edge);
                for (size_t i = 1; i <= round; i++) {
                    const int newDistance = uLabel.distance[i] + weight;
                    const bool update = newDistance < vLabel.distance[i];
                    vLabel.distance[i] = branchlessConditional(update, newDistance, vLabel.distance[i]);
                    vLabel.parent[i] = branchlessConditional(update, uLabel.parent[i], vLabel.parent[i]);
                }
            }
        }

        if constexpr (Debug) {
            std::cout << "Time: " << String::musToString(timer.elapsedMicroseconds()) << std::endl;
        }
    }

    inline void downwardSearchToTargets() noexcept {
        if constexpr (Debug) {
            std::cout << "Running downward sweep" << std::endl;
            timer.restart();
        }

        for (const Vertex sweepV : targetGraph.graph.vertices()) {
            const Vertex v = targetGraph.internalToExternal(sweepV);
            GroupedLabel& vLabel = getGroupedLabel(v);
            for (const Edge edge : targetGraph.graph.edgesFrom(sweepV)) {
                const Vertex u = targetGraph.toVertex[edge];
                const GroupedLabel& uLabel = groupedLabel[u]; //Already known to be up to date
                const int weight = targetGraph.graph.get(Weight, edge);
                for (size_t i = 0; i <= round; i++) {
                    const int newDistance = uLabel.distance[i] + weight;
                    const bool update = newDistance < vLabel.distance[i];
                    vLabel.distance[i] = branchlessConditional(update, newDistance, vLabel.distance[i]);
                    vLabel.parent[i] = branchlessConditional(update, uLabel.parent[i], vLabel.parent[i]);
                }
            }
        }

        if constexpr (Debug) {
            std::cout << "Time: " << String::musToString(timer.elapsedMicroseconds()) << std::endl;
        }
    }

    inline void downwardSearchToStops() noexcept {
        if constexpr (Debug) {
            std::cout << "Running downward sweep to stops" << std::endl;
            timer.restart();
        }

        for (const Vertex sweepV : stopGraph.graph.vertices()) {
            const Vertex v = stopGraph.internalToExternal(sweepV);
            GroupedLabel& vLabel = getGroupedLabel(v);
            for (const Edge edge : stopGraph.graph.edgesFrom(sweepV)) {
                const Vertex u = stopGraph.toVertex[edge];
                const int weight = stopGraph.graph.get(Weight, edge);
                GroupedLabel& uLabel = groupedLabel[u]; //Already known to be up to date
                const int newDistance = uLabel.distance[0] + weight;
                const bool update = newDistance < vLabel.distance[0];
                vLabel.distance[0] = branchlessConditional(update, newDistance, vLabel.distance[0]);
                vLabel.parent[0] = branchlessConditional(update, uLabel.parent[0], vLabel.parent[0]);
            }
        }

        if constexpr (Debug) {
            std::cout << "Time: " << String::musToString(timer.elapsedMicroseconds()) << std::endl;
        }
    }

    inline int getDistance(const size_t numTrips, const Vertex vertex) noexcept {
        const Vertex originalVertex = originalToInternal(vertex);
        if (numTrips >= GroupedRounds) {
            return getTargetLabel(originalVertex, numTrips).distance;
        } else {
            return getGroupedLabel(originalVertex).distance[numTrips];
        }
    }

    inline Vertex getParent(const size_t numTrips, const Vertex vertex) noexcept {
        const Vertex originalVertex = originalToInternal(vertex);
        if (numTrips >= GroupedRounds) {
            return getTargetLabel(originalVertex, numTrips).parent;
        } else {
            return Vertex(getGroupedLabel(originalVertex).parent[numTrips]);
        }
    }

    inline int getTargetDistance(const size_t numTrips, const Vertex vertex) noexcept {
        return getDistance(numTrips, vertex);
    }

    inline Vertex getTargetParent(const size_t numTrips, const Vertex vertex) noexcept {
        return getParent(numTrips, vertex);
    }

    inline size_t numRounds() const noexcept {
        return round + 1;
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
        reorder(searchGraph);
        stops.applyPermutation(positionInOrder);
        targets.applyPermutation(positionInOrder);
    }

    template<typename GRAPH>
    inline void reorder(GRAPH& graph) noexcept {
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
    inline void addGroupedSourceInternal(const Vertex vertex, const int initialDistance, const Vertex parentVertex, const size_t numTrips) noexcept {
        GroupedLabel& vertexLabel = getGroupedLabel(vertex);
        if (initialDistance >= vertexLabel.distance[numTrips]) return;
        vertexLabel.distance[numTrips] = initialDistance;
        vertexLabel.parent[numTrips] = parentVertex;
        if constexpr (FOR_SWEEP) {
            AssertMsg(upwardSweepGraph.externalToInternal(vertex) < upwardSweepGraph.graph.numVertices(), "Vertex is not in sweep graph! (original: "<< internalToOriginal(vertex) << ", CH: " << vertex << ", sweep: " << upwardSweepGraph.externalToInternal(vertex) << ")");
            sweepStart = std::min(sweepStart, sweepStartOf[upwardSweepGraph.externalToInternal(vertex)]);
        } else {
            distance[vertex].distance = initialDistance;
            distance[vertex].timestamp = timestamp;
            Q.update(&distance[vertex]);
        }
    }

    inline void addDijkstraSourceInternal(const Vertex vertex, const int initialDistance, const Vertex parentVertex) noexcept {
        updateDistance(vertex);
        if (initialDistance >= distance[vertex].distance) return;
        distance[vertex].distance = initialDistance;
        distance[vertex].parent = parentVertex;
        distance[vertex].timestamp = timestamp;
        Q.update(&distance[vertex]);
    }

    inline void settleInitial() noexcept {
        Distance* distanceU = Q.extractFront();
        const Vertex u = Vertex(distanceU - &(distance[0]));
        AssertMsg(u < graph[FORWARD].numVertices(), u << " is not a valid vertex!");
        if constexpr (StallOnDemand) {
            for (Edge edge : graph[BACKWARD].edgesFrom(u)) {
                const Vertex v = graph[BACKWARD].get(ToVertex, edge);
                updateDistanceInitial(v);
                if (distance[v].distance < distance[u].distance - graph[BACKWARD].get(Weight, edge)) return;
            }
        }
        for (Edge edge : graph[FORWARD].edgesFrom(u)) {
            const Vertex v = graph[FORWARD].get(ToVertex, edge);
            const int newDistance = distanceU->distance + graph[FORWARD].get(Weight, edge);
            updateDistanceInitial(v);
            if (distance[v].distance > newDistance) {
                distance[v].distance = newDistance;
                Q.update(&distance[v]);
                GroupedLabel& vLabel = getGroupedLabel(v);
                vLabel.distance[round] = newDistance;
                AssertMsg(groupedLabel[u].parent[round] != int(noVertex), "Invalid parent!");
                vLabel.parent[round] = groupedLabel[u].parent[round];
            }
        }
    }

    inline Vertex originalToInternal(const Vertex vertex) const noexcept {
        return positionInOrder.permutate(vertex);
    }

    inline Vertex internalToOriginal(const Vertex vertex) const noexcept {
        return Vertex(contractionOrder[vertex]);
    }

    inline GroupedLabel& getGroupedLabel(const Vertex vertex) noexcept {
        GroupedLabel& label = groupedLabel[vertex];
        if (label.timestamp < queryStartTimestamp) {
            label.clear();
            label.timestamp = queryStartTimestamp;
        }
        return label;
    }

    inline TargetLabel& getTargetLabel(const Vertex vertex, const size_t round) noexcept {
        TargetLabel& label = targetLabel[round - GroupedRounds][targetId[vertex]];
        if (label.timestamp < queryStartTimestamp) {
            label.distance = never;
            label.parent = noVertex;
            label.timestamp = queryStartTimestamp;
        }
        return label;
    }

    inline void updateDistanceInitial(const Vertex vertex) noexcept {
        if (distance[vertex].timestamp == timestamp) return;
        distance[vertex].distance = never;
        distance[vertex].timestamp = timestamp;
    }

    inline void updateDistance(const Vertex vertex) noexcept {
        if (distance[vertex].timestamp == timestamp) return;
        GroupedLabel& label = groupedLabel[vertex];
        distance[vertex].distance = (label.timestamp < queryStartTimestamp) ? never : label.getMinDistance();
        distance[vertex].timestamp = timestamp;
    }

private:
    CHGraph graph[2];
    SweepGraph upwardSweepGraph;
    SweepGraph stopGraph;
    SweepGraph targetGraph;
    TransferGraph searchGraph;

    const Order contractionOrder;
    const Permutation positionInOrder;

    Vertex sweepStart;
    std::vector<Vertex> sweepStartOf;
    IndexedSet<false, Vertex> stops;
    IndexedSet<false, Vertex> targets;

    ExternalKHeap<2, Distance> Q;
    std::vector<Distance> distance;
    std::vector<GroupedLabel> groupedLabel;
    size_t round;
    int queryStartTimestamp;
    int timestamp;

    std::vector<size_t> targetId;
    std::vector<std::vector<TargetLabel>> targetLabel;

    Timer timer;
};

}
