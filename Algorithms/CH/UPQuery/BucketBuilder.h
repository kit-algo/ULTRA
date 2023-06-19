#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../CH.h"
#include "../CHUtils.h"

#include "../../../Helpers/Types.h"
#include "../../../Helpers/Timer.h"
#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/String/String.h"
#include "../../../Helpers/Vector/Vector.h"

#include "../../../DataStructures/Container/ExternalKHeap.h"
#include "../../../DataStructures/Container/Set.h"

namespace CH {

template<bool STALL_ON_DEMAND = true, bool DEBUG = false>
class BucketBuilder {

public:
    constexpr static bool StallOnDemand = STALL_ON_DEMAND;
    constexpr static bool Debug = DEBUG;
    using Type = BucketBuilder<StallOnDemand, Debug>;

private:
    struct Distance : public ExternalKHeapElement {
        Distance() : ExternalKHeapElement(), distance(INFTY) {}
        inline bool hasSmallerKey(const Distance* other) const noexcept {return distance < other->distance;}
        int distance;
    };

public:
    BucketBuilder(const CHGraph& forward, const CHGraph& backward) :
        graph {&forward, &backward},
        Q(graph[FORWARD]->numVertices()),
        distance(graph[FORWARD]->numVertices()),
        timestamp(graph[FORWARD]->numVertices()),
        currentTimestamp(0),
        reachedVertices(graph[FORWARD]->numVertices()) {
    }

    BucketBuilder(const CH& ch, const int direction = FORWARD) :
        BucketBuilder(ch.getGraph(direction), ch.getGraph(!direction)) {
    }

    inline CHGraph build(const IndexedSet<false, Vertex>& targets) noexcept {
        if constexpr (Debug) std::cout << "Building bucket graph" << std::endl;
        CHConstructionGraph temp;
        temp.addVertices(graph[FORWARD]->numVertices());
        Progress progress(targets.size(), Debug);
        for (const Vertex vertex : targets) {
            clear();
            cleanLabel(vertex);
            distance[vertex].distance = 0;
            Q.update(&distance[vertex]);
            while (!Q.empty()) {
                settle();
            }
            for (const Vertex bucket : reachedVertices) {
                AssertMsg(!temp.hasEdge(bucket, vertex), "Bucket graph contains already an edge from " << bucket << " to " << vertex << "!");
                temp.addEdge(bucket, vertex).set(Weight, distance[bucket].distance);
            }
            progress++;
        }
        CHGraph result;
        ::Graph::move(std::move(temp), result);
        result.sortEdges(Weight);
        if constexpr (Debug) {
            std::cout << std::endl;
            ::Graph::printInfo(result);
            result.printAnalysis();
        }
        return result;
    }

private:
    inline void settle() noexcept {
        Distance* distanceU = Q.extractFront();
        const Vertex u = Vertex(distanceU - &(distance[0]));
        AssertMsg(u < graph[BACKWARD]->numVertices(), u << " is not a valid vertex!");
        if constexpr (StallOnDemand) {
            for (Edge edge : graph[FORWARD]->edgesFrom(u)) {
                const Vertex v = graph[FORWARD]->get(ToVertex, edge);
                cleanLabel(v);
                if (distance[v].distance < distance[u].distance - graph[FORWARD]->get(Weight, edge)) return;
            }
        }
        for (Edge edge : graph[BACKWARD]->edgesFrom(u)) {
            const Vertex v = graph[BACKWARD]->get(ToVertex, edge);
            cleanLabel(v);
            const int newDistance = distanceU->distance + graph[BACKWARD]->get(Weight, edge);
            if (distance[v].distance > newDistance) {
                distance[v].distance = newDistance;
                Q.update(&distance[v]);
            }
        }
        reachedVertices.insert(u);
    }

    inline void clear() noexcept {
        Q.clear();
        currentTimestamp++;
        reachedVertices.clear();
    }

    inline void cleanLabel(const Vertex vertex) noexcept {
        if (timestamp[vertex] != currentTimestamp) {
            distance[vertex].distance = INFTY;
            timestamp[vertex] = currentTimestamp;
        }
    }

private:
    const CHGraph* graph[2];

    ExternalKHeap<2, Distance> Q;
    std::vector<Distance> distance;
    std::vector<int> timestamp;
    int currentTimestamp;
    IndexedSet<false, Vertex> reachedVertices;

    Timer timer;
};

}
