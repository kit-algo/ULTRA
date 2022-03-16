#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../CH.h"

#include "../../../Helpers/Types.h"
#include "../../../Helpers/Timer.h"
#include "../../../Helpers/String/String.h"
#include "../../../Helpers/Vector/Vector.h"

#include "../../../DataStructures/Container/ExternalKHeap.h"

namespace CH {

template<typename GRAPH = CHGraph, bool STALL_ON_DEMAND = true, bool DEBUG = false, size_t COLLECT_POIS = false>
class Query {

public:
    using Graph = GRAPH;
    constexpr static bool StallOnDemand = STALL_ON_DEMAND;
    constexpr static bool Debug = DEBUG;
    constexpr static bool CollectPOIs = COLLECT_POIS;
    using Type = Query<Graph, StallOnDemand, Debug, CollectPOIs>;

private:
    struct Distance : public ExternalKHeapElement {
        Distance() : ExternalKHeapElement(), distance(INFTY) {}
        inline bool hasSmallerKey(const Distance* other) const noexcept {return distance < other->distance;}
        int distance;
    };

public:
    Query(const Graph& forward, const Graph& backward, const std::vector<int>& forwardWeight, const std::vector<int>& backwardWeight, const Vertex::ValueType endOfPOIs = 0) :
        graph {&forward, &backward},
        weight {&forwardWeight, &backwardWeight},
        root {noVertex, noVertex},
        Q {ExternalKHeap<2, Distance>(forward.numVertices()), ExternalKHeap<2, Distance>(backward.numVertices())},
        distance {std::vector<Distance>(forward.numVertices()), std::vector<Distance>(backward.numVertices())},
        parent {std::vector<Vertex>(forward.numVertices()), std::vector<Vertex>(backward.numVertices())},
        timeStamp(forward.numVertices()),
        time(0),
        tentativeDistance(INFTY),
        intersectingVertex(noVertex),
        settleCount(0),
        stallCount(0),
        endOfPOIs(endOfPOIs),
        reachedPOIs {std::vector<Vertex>(), std::vector<Vertex>()} {
        Assert(forward.numVertices() == backward.numVertices());
    }

    template<typename ATTRIBUTE>
    Query(const Graph& forward, const Graph& backward, const Vertex::ValueType endOfPOIs = 0, const ATTRIBUTE attribute = Weight) :
        Query(forward, backward, forward[attribute], backward[attribute], endOfPOIs) {
    }

    Query(const CH& ch, const int direction = FORWARD, const Vertex::ValueType endOfPOIs = 0) :
        Query(ch.getGraph(direction), ch.getGraph(!direction), endOfPOIs, Weight) {
    }

    template<bool TARGET_PRUNING = true>
    inline void run(const Vertex from, const Vertex to, const double targetPruningFactor = 1) noexcept {
        if (root[FORWARD] == from && root[BACKWARD] == to) return;
        if constexpr (Debug) {
            std::cout << "Starting " <<  ((CollectPOIs) ? ("CH-POI") : ("CH")) << " query" << std::endl;
            std::cout << "   Source vertex: " << from << std::endl;
            std::cout << "   Target vertex: " << to << std::endl;
        }
        clear();
        addSource(from);
        addTarget(to);
        run<TARGET_PRUNING>(targetPruningFactor);
    }

    template<int I, int J, bool TARGET_PRUNING = true>
    inline void run(const Vertex origin) noexcept {
        if (root[I] == origin && root[J] == noVertex) return;
        if constexpr (Debug) {
            std::cout << "Starting unidirectional " <<  ((CollectPOIs) ? ("CH-POI") : ("CH")) << " query" << std::endl;
            std::cout << "   Origin vertex: " << origin << std::endl;
        }
        clear<I>();
        addOrigin<I>(origin);
        root[J] = noVertex;
        run<I, J, TARGET_PRUNING>();
    }

    inline void clear() noexcept {
        clearGeneral();
        clearDirection<FORWARD>();
        clearDirection<BACKWARD>();
    }

    template<int I>
    inline void clear() noexcept {
        clearGeneral();
        clearDirection<I>();
    }

    template<int I>
    inline void addOrigin(const Vertex vertex, const int initialDistance = 0) noexcept {
        root[I] = vertex;
        cleanLabel(vertex);
        distance[I][vertex].distance = initialDistance;
        Q[I].update(&distance[I][vertex]);
    }

    inline void addSource(const Vertex vertex, const int initialDistance = 0) noexcept {
        addOrigin<FORWARD>(vertex, initialDistance);
    }

    inline void addTarget(const Vertex vertex, const int initialDistance = 0) noexcept {
        addOrigin<BACKWARD>(vertex, initialDistance);
    }

    template<bool TARGET_PRUNING = true>
    inline void run(const double targetPruningFactor = 1) noexcept {
        if constexpr (Debug) std::cout << "Running " << ((CollectPOIs) ? ("CH-POI") : ("CH")) << " query" << std::endl;

        if (root[FORWARD] == root[BACKWARD]) {
            tentativeDistance = 0;
            intersectingVertex = root[FORWARD];
        }

        while ((!Q[FORWARD].empty()) && (!Q[BACKWARD].empty())) {
            settle<FORWARD, BACKWARD, TARGET_PRUNING>(targetPruningFactor);
            settle<BACKWARD, FORWARD, TARGET_PRUNING>(targetPruningFactor);
        }

        while (!Q[FORWARD].empty()) {
            settle<FORWARD, BACKWARD, TARGET_PRUNING>(targetPruningFactor);
        }

        while (!Q[BACKWARD].empty()) {
            settle<BACKWARD, FORWARD, TARGET_PRUNING>(targetPruningFactor);
        }

        if constexpr (Debug) printStatistics();
    }

    template<int I, int J, bool TARGET_PRUNING = true>
    inline void run() noexcept {
        if constexpr (Debug) std::cout << "Running unidirectional " << ((CollectPOIs) ? ("CH-POI") : ("CH")) << " query" << std::endl;

        if (root[FORWARD] == root[BACKWARD]) {
            tentativeDistance = 0;
            intersectingVertex = root[FORWARD];
        }

        while (!Q[I].empty()) {
            settle<I, J, TARGET_PRUNING>();
        }

        if constexpr (Debug) printStatistics();
    }

    inline void setTentativeDistance(const int distance) noexcept {
        tentativeDistance = distance;
    }

    inline bool reachable() const noexcept {
        return intersectingVertex != noVertex;
    }

    inline bool visited(const Vertex vertex) const noexcept {
        return timeStamp[vertex] == time;
    }

    inline int getDistance(const Vertex = noVertex) const noexcept {
        return tentativeDistance;
    }

    inline Vertex getIntersectingVertex() const noexcept {
        return intersectingVertex;
    }

    inline int getForwardDistance(const Vertex vertex) noexcept {
        cleanLabel(vertex);
        return distance[FORWARD][vertex].distance;
    }

    inline int getBackwardDistance(const Vertex vertex) noexcept {
        cleanLabel(vertex);
        return distance[BACKWARD][vertex].distance;
    }

    template<int DIRECTION>
    inline int getDistanceToPOI(const Vertex vertex) noexcept {
        cleanLabel(vertex);
        return distance[DIRECTION][vertex].distance;
    }

    inline const std::vector<Vertex>& getForwardPOIs() const noexcept {
        return reachedPOIs[FORWARD];
    }

    inline const std::vector<Vertex>& getBackwardPOIs() const noexcept {
        return reachedPOIs[BACKWARD];
    }

    template<int DIRECTION>
    inline const std::vector<Vertex>& getPOIs() const noexcept {
        return reachedPOIs[DIRECTION];
    }

    inline std::vector<Vertex> getReversePath(const Vertex = noVertex) const noexcept {
        return Vector::reverse(backwardLeg<true>()) + forwardLeg<true>();
    }

    inline std::vector<Vertex> getPath(const Vertex = noVertex) const noexcept {
        return Vector::reverse(forwardLeg<true>()) + backwardLeg<true>();
    }

    inline std::vector<Vertex> getPackedForwardLeg() const noexcept {
        return forwardLeg<false>();
    }

    inline std::vector<Vertex> getPackedBackwardLeg() const noexcept {
        return backwardLeg<false>();
    }

    inline int getSettleCount() const noexcept {
        return settleCount;
    }

    inline int getStallCount() const noexcept {
        return stallCount;
    }

private:
    inline void clearGeneral() noexcept {
        if constexpr (Debug) {
            timer.restart();
            settleCount = 0;
            stallCount = 0;
        }
        time++;
        tentativeDistance = INFTY;
        intersectingVertex = noVertex;
    }

    template<int I>
    inline void clearDirection() noexcept {
        Q[I].clear();
        if constexpr (CollectPOIs) {
            reachedPOIs[I].clear();
        }
    }

    inline void cleanLabel(const Vertex vertex) noexcept {
        if (timeStamp[vertex] != time) {
            distance[FORWARD][vertex].distance = INFTY;
            distance[BACKWARD][vertex].distance = INFTY;
            parent[FORWARD][vertex] = noVertex;
            parent[BACKWARD][vertex] = noVertex;
            timeStamp[vertex] = time;
        }
    }

    inline void printStatistics() const noexcept {
        if constexpr (StallOnDemand) std::cout << "   Stalled Vertices = " << String::prettyInt(stallCount) << std::endl;
        std::cout << "   Settled Vertices = " << String::prettyInt(settleCount) << std::endl;
        std::cout << "   Time = " << String::msToString(timer.elapsedMilliseconds()) << std::endl;
    }

    template<bool UNPACK>
    inline std::vector<Vertex> forwardLeg() const noexcept {
        return getLeg<FORWARD, BACKWARD, UNPACK>();
    }

    template<bool UNPACK>
    inline std::vector<Vertex> backwardLeg() const noexcept {
        return getLeg<BACKWARD, FORWARD, UNPACK>();
    }

    template<int I, int J, bool UNPACK>
    inline std::vector<Vertex> getLeg() const noexcept {
        std::vector<Vertex> path;
        if (!graph[I]->isVertex(intersectingVertex)) return path;
        if (!visited(intersectingVertex)) return path;
        path.push_back(intersectingVertex);
        Vertex p = parent[I][path.back()];
        while (p != noVertex) {
            Edge edge = graph[I]->findEdge(p, path.back());
            Assert(graph[I]->isEdge(edge));
            const Vertex via = graph[I]->get(ViaVertex, edge);
            if (UNPACK && graph[I]->isVertex(via)) {
                path += unpackShortcut(*(graph[J]), *(graph[I]), path.back(), via, p);
            } else {
                path.push_back(p);
            }
            p = parent[I][path.back()];
        }
        return path;
    }

    template<int I, int J, bool TARGET_PRUNING>
    inline void settle(const double targetPruningFactor = 1) noexcept {
        Distance* distanceU = Q[I].extractFront();
        if constexpr (TARGET_PRUNING) {
            if (distanceU->distance > tentativeDistance * targetPruningFactor) {
                Q[I].clear();
                return;
            }
        }
        const Vertex u = Vertex(distanceU - &(distance[I][0]));
        if constexpr (StallOnDemand) {
            for (Edge edge : graph[J]->edgesFrom(u)) {
                const Vertex v = graph[J]->get(ToVertex, edge);
                cleanLabel(v);
                if (distance[I][v].distance < distance[I][u].distance - (*(weight[J]))[edge]) {
                    if constexpr (Debug) stallCount++;
                    return;
                }
            }
        }
        for (Edge edge : graph[I]->edgesFrom(u)) {
            const Vertex v = graph[I]->get(ToVertex, edge);
            cleanLabel(v);
            const int newDistance = distanceU->distance + (*(weight[I]))[edge];
            if (distance[I][v].distance > newDistance) {
                distance[I][v].distance = newDistance;
                parent[I][v] = u;
                Q[I].update(&distance[I][v]);
                const int newTentativeDistance = newDistance + distance[J][v].distance;
                if (tentativeDistance > newTentativeDistance) {
                    tentativeDistance = newTentativeDistance;
                    intersectingVertex = v;
                }
            }
        }
        if constexpr (CollectPOIs) {
            if (u < endOfPOIs) {
                reachedPOIs[I].emplace_back(u);
            }
        }
        if constexpr (Debug) settleCount++;
    }

private:
    const Graph* graph[2];
    const std::vector<int>* weight[2];

    Vertex root[2];

    ExternalKHeap<2, Distance> Q[2];
    std::vector<Distance> distance[2];
    std::vector<Vertex> parent[2];
    std::vector<int> timeStamp;
    int time;

    int tentativeDistance;
    Vertex intersectingVertex;

    int settleCount;
    int stallCount;
    Timer timer;

    Vertex::ValueType endOfPOIs;
    std::vector<Vertex> reachedPOIs[2];

};

}
