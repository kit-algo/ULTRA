#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "CHQuery.h"

#include "../../../Helpers/Console/Progress.h"

namespace CH {

template<typename GRAPH = CHGraph, bool STALL_ON_DEMAND = true, bool DEBUG = false>
class BucketQuery {

public:
    using Graph = GRAPH;
    constexpr static bool StallOnDemand = STALL_ON_DEMAND;
    constexpr static bool Debug = DEBUG;
    using Type = BucketQuery<Graph, StallOnDemand, Debug>;

    using BaseQuery = Query<Graph, StallOnDemand, false, true>;

public:
    BucketQuery(const Graph& forward, const Graph& backward, const std::vector<int>& forwardWeight, const std::vector<int>& backwardWeight, const Vertex::ValueType endOfPOIs) :
        baseQuery(forward, backward, forwardWeight, backwardWeight, forward.numVertices()),
        bucketGraph {CHGraph(), CHGraph()},
        distance {std::vector<int>(forward.numVertices(), INFTY), std::vector<int>(backward.numVertices(), INFTY)},
        root {noVertex, noVertex},
        endOfPOIs(endOfPOIs),
        reachedPOIs {std::vector<Vertex>(), std::vector<Vertex>()} {
        buildBucketGraph<FORWARD, BACKWARD>();
        buildBucketGraph<BACKWARD, FORWARD>();
    }

    template<typename ATTRIBUTE>
    BucketQuery(const Graph& forward, const Graph& backward, const Vertex::ValueType endOfPOIs, const ATTRIBUTE attribute = Weight) :
        BucketQuery(forward, backward, forward[attribute], backward[attribute], endOfPOIs) {
    }

    BucketQuery(const CH& ch, const int direction = FORWARD, const Vertex::ValueType endOfPOIs = 0) :
        BucketQuery(ch.getGraph(direction), ch.getGraph(!direction), endOfPOIs, Weight) {
    }

    template<bool TARGET_PRUNING = true>
    inline void run(const Vertex from, const Vertex to, const double targetPruningFactor = 1) noexcept {
        if (root[FORWARD] == from && root[BACKWARD] == to) return;
        if constexpr (Debug) {
            std::cout << "Starting bucket query" << std::endl;
            timer.restart();
        }

        root[FORWARD] = from;
        root[BACKWARD] = to;

        clear<FORWARD>();
        clear<BACKWARD>();

        baseQuery.template run<TARGET_PRUNING>(from, to, targetPruningFactor);

        collectPOIs<FORWARD>(targetPruningFactor);
        collectPOIs<BACKWARD>(targetPruningFactor);

        if constexpr (Debug) std::cout << "   Time = " << String::msToString(timer.elapsedMilliseconds()) << std::endl;
    }

    template<int I, int J, bool TARGET_PRUNING = true>
    inline void run(const Vertex origin) noexcept {
        if (root[I] == origin && root[J] == noVertex) return;
        if constexpr (Debug) {
            std::cout << "Starting unidirectional bucket query" << std::endl;
            timer.restart();
        }

        root[I] = origin;
        root[J] = noVertex;

        clear<I>();
        baseQuery.template run<I, J, TARGET_PRUNING>(origin);
        collectPOIs<I>();

        if constexpr (Debug) std::cout << "   Time = " << String::msToString(timer.elapsedMilliseconds()) << std::endl;
    }

    inline void clear() noexcept {
        if constexpr (Debug) timer.restart();
        clear<FORWARD>();
        clear<BACKWARD>();
        baseQuery.clear();
    }

    inline void addSource(const Vertex vertex, const int distance = 0) noexcept {
        baseQuery.addSource(vertex, distance);
    }

    inline void addTarget(const Vertex vertex, const int distance = 0) noexcept {
        baseQuery.addTarget(vertex, distance);
    }

    inline void setTentativeDistance(const int distance) noexcept {
        baseQuery.setTentativeDistance(distance);
    }

    inline void run() noexcept {
        if constexpr (Debug) std::cout << "Running bucket query" << std::endl;
        baseQuery.run();
        collectPOIs<FORWARD>();
        collectPOIs<BACKWARD>();
        if constexpr (Debug) std::cout << "   Time = " << String::msToString(timer.elapsedMilliseconds()) << std::endl;
    }

    inline bool reachable() const noexcept {
        return baseQuery.reachable();
    }

    inline bool visited(const Vertex vertex) const noexcept {
        return baseQuery.visited(vertex);
    }

    inline int getDistance(const Vertex = noVertex) const noexcept {
        return baseQuery.getDistance();
    }

    inline const std::vector<int>& getForwardDistance() const noexcept {
        return distance[FORWARD];
    }

    inline int getForwardDistance(const Vertex vertex) const noexcept {
        return distance[FORWARD][vertex];
    }

    inline const std::vector<int>& getBackwardDistance() const noexcept {
        return distance[BACKWARD];
    }

    inline int getBackwardDistance(const Vertex vertex) const noexcept {
        return distance[BACKWARD][vertex];
    }

    inline const std::vector<Vertex>& getForwardPOIs() const noexcept {
        return reachedPOIs[FORWARD];
    }

    inline const std::vector<Vertex>& getBackwardPOIs() const noexcept {
        return reachedPOIs[BACKWARD];
    }

    inline std::vector<Vertex> getReversePath(const Vertex = noVertex) const noexcept {
        return baseQuery.getReversePath();
    }

    inline std::vector<Vertex> getPath(const Vertex = noVertex) const noexcept {
        return baseQuery.getPath();
    }

    inline int getSettleCount() const noexcept {
        return baseQuery.getSettleCount();
    }

    inline int getStallCount() const noexcept {
        return baseQuery.getStallCount();
    }

private:
    template<int I, int J>
    inline void buildBucketGraph() noexcept {
        if constexpr (Debug) std::cout << "Building " << ((I == FORWARD) ? ("forward") : ("backward")) << " bucket graph" << std::endl;
        CHConstructionGraph temp;
        temp.addVertices(distance[I].size());
        Progress progress(endOfPOIs, Debug);
        for (Vertex vertex = Vertex(0); vertex < endOfPOIs; vertex++) {
            baseQuery.template run<J, I>(vertex);
            for (const Vertex bucket : baseQuery.template getPOIs<J>()) {
                AssertMsg(!temp.hasEdge(bucket, vertex), "Bucket graph contains already an edge from " << bucket << " to " << vertex << "!");
                temp.addEdge(bucket, vertex).set(Weight, baseQuery.template getDistanceToPOI<J>(bucket));
            }
            progress++;
        }
        ::Graph::move(std::move(temp), bucketGraph[I]);
        bucketGraph[I].sortEdges(Weight);
        if constexpr (Debug) {
            std::cout << std::endl;
            ::Graph::printInfo(bucketGraph[I]);
            bucketGraph[I].printAnalysis();
        }
    }

    template<int DIRECTION>
    inline void clear() noexcept {
        for (const Vertex vertex : reachedPOIs[DIRECTION]) {
            distance[DIRECTION][vertex] = INFTY;
        }
        reachedPOIs[DIRECTION].clear();
    }

    template<int DIRECTION>
    inline void collectPOIs(const double targetPruningFactor = 1) noexcept {
        const int maxDistance = baseQuery.getDistance() * targetPruningFactor;
        for (const Vertex vertex : baseQuery.template getPOIs<DIRECTION>()) {
            if (baseQuery.template getDistanceToPOI<DIRECTION>(vertex) > maxDistance) break;
            for (const Edge edge : bucketGraph[DIRECTION].edgesFrom(vertex)) {
                const int newDistance = baseQuery.template getDistanceToPOI<DIRECTION>(vertex) + bucketGraph[DIRECTION].get(Weight, edge);
                if (newDistance > maxDistance) break;
                const Vertex poi = bucketGraph[DIRECTION].get(ToVertex, edge);
                if (distance[DIRECTION][poi] == INFTY) {
                    reachedPOIs[DIRECTION].emplace_back(poi);
                    distance[DIRECTION][poi] = newDistance;
                } else {
                    distance[DIRECTION][poi] = std::min(distance[DIRECTION][poi], newDistance);
                }
            }
        }
    }

private:
    BaseQuery baseQuery;

    CHGraph bucketGraph[2];
    std::vector<int> distance[2];

    Vertex root[2];

    Vertex endOfPOIs;
    std::vector<Vertex> reachedPOIs[2];

    Timer timer;

};

}
