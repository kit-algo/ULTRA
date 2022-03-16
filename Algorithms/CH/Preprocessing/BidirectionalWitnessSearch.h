#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/Types.h"
#include "../../../Helpers/Timer.h"
#include "../../../Helpers/String/String.h"
#include "../../../Helpers/Vector/Vector.h"

#include "../../../DataStructures/Container/ExternalKHeap.h"

namespace CH {

template<typename GRAPH, typename PROFILER, int Q_POP_LIMIT = -1, bool ONE_HOP_HEURISTIC = true>
class BidirectionalWitnessSearch {

public:
    using Graph = GRAPH;
    using Profiler = PROFILER;
    constexpr static int QPopLimit = Q_POP_LIMIT;
    constexpr static bool OneHopHeuristic = ONE_HOP_HEURISTIC;
    using Type = BidirectionalWitnessSearch<Graph, Profiler, QPopLimit, OneHopHeuristic>;

private:
    struct Distance : public ExternalKHeapElement {
        Distance() : ExternalKHeapElement(), distance(intMax / 2) {}
        inline bool hasSmallerKey(const Distance* other) const noexcept {return distance < other->distance;}
        int distance;
    };

public:
    BidirectionalWitnessSearch() :
        graph(0),
        weight(0),
        Q {ExternalKHeap<2, Distance>(), ExternalKHeap<2, Distance>()},
        distance {std::vector<Distance>(), std::vector<Distance>()},
        settled {std::vector<Vertex>(), std::vector<Vertex>()},
        profiler(0) {
    }

    inline void initialize(const Graph* graph, const std::vector<int>* weight, Profiler* profiler) {
        this->graph = graph;
        this->weight = weight;
        this->profiler = profiler;
        Q[0].reserve(graph->numVertices());
        Q[1].reserve(graph->numVertices());
        std::vector<Distance>(graph->numVertices()).swap(distance[0]);
        std::vector<Distance>(graph->numVertices()).swap(distance[1]);
    }

    inline bool shortcutIsNecessary(const Vertex from, const Vertex to, const Vertex via, const int shortcutDistance) noexcept {
        if (graph->outDegree(from) == 1) return true;
        if (graph->inDegree(to) == 1) return true;
        if (OneHopHeuristic) {
            const Edge edge = graph->findEdge(from, to);
            if (graph->isEdge(edge)) {
                return (*weight)[edge] > shortcutDistance;
            }
        }

        profiler->startWitnessSearch();
        foundWitness = false;
        int qPops = QPopLimit;
        clear<BACKWARD>();
        if ((currentFrom != from) || (currentVia != via)) {
            currentFrom = from;
            currentVia = via;
            clear<FORWARD>();
            distance[FORWARD][from].distance = 0;
            Q[FORWARD].update(&distance[FORWARD][from]);
        } else if (distance[FORWARD][to].distance <= shortcutDistance) {
            return false;
        }
        distance[BACKWARD][to].distance = 0;
        Q[BACKWARD].update(&distance[BACKWARD][to]);

        while(!Q[FORWARD].empty() && !Q[BACKWARD].empty()) {
            if (Q[FORWARD].min().distance + Q[BACKWARD].min().distance > shortcutDistance) return true;

            settle<FORWARD>(via, shortcutDistance);
            if (foundWitness) return false;

            settle<BACKWARD>(via, shortcutDistance);
            if (foundWitness) return false;

            if constexpr (QPopLimit > 0) {
                qPops -= 2;
                if (qPops < 0) break;
            }
        }

        profiler->doneWitnessSearch();
        return true;
    }

private:
    template<int DIRECTION>
    inline void clear() noexcept {
        for (const Vertex vertex : settled[DIRECTION]) {
            distance[DIRECTION][vertex].distance = INFTY;
        }
        settled[DIRECTION].clear();
        for (Distance* const label : Q[DIRECTION].data()) {
            label->distance = INFTY;
        }
        Q[DIRECTION].clear();
    }

    template<int DIRECTION>
    inline void settle(const Vertex via, const int shortcutDistance) noexcept {
        Distance* label = Q[DIRECTION].extractFront();
        const Vertex u = Vertex(label - &(distance[DIRECTION][0]));
        settled[DIRECTION].emplace_back(u);
        if constexpr (DIRECTION == FORWARD) {
            for (Edge edge : graph->edgesFrom(u)) {
                const Vertex v = graph->get(ToVertex, edge);
                if (v == via) continue;
                relax<DIRECTION>(v, label->distance + (*weight)[edge], shortcutDistance);
            }
        } else {
            for (Edge edge : graph->edgesTo(u)) {
                const Vertex v = graph->get(FromVertex, edge);
                if (v == via) continue;
                relax<DIRECTION>(v, label->distance + (*weight)[edge], shortcutDistance);
            }
        }
        profiler->settledVertex();
    }

    template<int DIRECTION>
    inline void relax(const Vertex v, const int newDistance, const int shortcutDistance) noexcept {
        if (distance[DIRECTION][v].distance > newDistance) {
            distance[DIRECTION][v].distance = newDistance;
            Q[DIRECTION].update(&distance[DIRECTION][v]);
            if (distance[FORWARD][v].distance + distance[BACKWARD][v].distance <= shortcutDistance) foundWitness = true;
        }
    }

private:
    const Graph* graph;
    const std::vector<int>* weight;

    ExternalKHeap<2, Distance> Q[2];
    std::vector<Distance> distance[2];
    std::vector<Vertex> settled[2];

    Vertex currentFrom;
    Vertex currentVia;
    bool foundWitness;

    Profiler* profiler;

};

}
