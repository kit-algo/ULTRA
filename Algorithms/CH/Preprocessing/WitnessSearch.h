#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../../Helpers/Types.h"
#include "../../../Helpers/Timer.h"
#include "../../../Helpers/String/String.h"
#include "../../../Helpers/Vector/Vector.h"

#include "../../../DataStructures/Container/ExternalKHeap.h"
#include "../../../DataStructures/Graph/Graph.h"

namespace CH {

template<typename GRAPH, typename PROFILER, int Q_POP_LIMIT = -1, bool ADAPTIVE_Q_POP_LIMIT = true>
class NoWitnessSearch {

public:
    using Graph = GRAPH;
    using Profiler = PROFILER;
    constexpr static int QPopLimit = Q_POP_LIMIT;
    constexpr static bool AdaptiveQPopLimit = ADAPTIVE_Q_POP_LIMIT;
    using Type = NoWitnessSearch<Graph, Profiler, QPopLimit, AdaptiveQPopLimit>;

public:
    inline void initialize(const GRAPH*, const std::vector<int>*, Profiler*) noexcept {}
    inline bool shortcutIsNecessary(const Vertex, const Vertex, const Vertex, const int) noexcept {return true;}
    inline void reset() {}

};

template<typename GRAPH, typename PROFILER, int Q_POP_LIMIT = -1, bool ADAPTIVE_Q_POP_LIMIT = true>
class WitnessSearch {

public:
    using Graph = GRAPH;
    using Profiler = PROFILER;
    constexpr static int QPopLimit = Q_POP_LIMIT;
    constexpr static bool AdaptiveQPopLimit = ADAPTIVE_Q_POP_LIMIT;
    using Type = WitnessSearch<Graph, Profiler, QPopLimit, AdaptiveQPopLimit>;

private:
    struct VertexLabel : public ExternalKHeapElement {
        VertexLabel() : ExternalKHeapElement(), distance(intMax), timeStamp(-1) {}
        inline void reset(int time) {
            distance = intMax;
            timeStamp = time;
        }
        inline bool hasSmallerKey(const VertexLabel* other) const {
            return distance < other->distance;
        }
        int distance;
        int timeStamp;
    };

public:
    WitnessSearch() :
        graph(0),
        weight(0),
        timeStamp(0),
        currentFrom(noVertex),
        currentVia(noVertex),
        qPops(0),
        qPopLimit(0),
        profiler(0) {
    }

    inline void initialize(const Graph* graph, const std::vector<int>* weight, Profiler* profiler) noexcept {
        this->graph = graph;
        this->weight = weight;
        this->profiler = profiler;
        Q.reserve(graph->numVertices());
        std::vector<VertexLabel>(graph->numVertices()).swap(label);
        reset();
    }

    inline bool shortcutIsNecessary(const Vertex from, const Vertex to, const Vertex via, const int shortcutDistance, const bool = false) noexcept {
        profiler->startWitnessSearch();
        if ((currentFrom != from) || (currentVia != via)) {
            currentFrom = from;
            currentVia = via;
            Q.clear();
            timeStamp++;
            VertexLabel& source = getLabel(from);
            source.distance = 0;
            Q.update(&source);
            if constexpr (QPopLimit > 0) {
                qPops = 0;
                if constexpr (AdaptiveQPopLimit) {
                    qPopLimit = QPopLimit * graph->outDegree(via);
                }
            }
        }

        while(!Q.empty()) {
            VertexLabel* uLabel = Q.front();
            if (uLabel->distance > shortcutDistance) break;
            const Vertex u = Vertex(uLabel - &(label[0]));
            if (u == to) break;
            Q.extractFront();
            for (Edge edge : graph->edgesFrom(u)) {
                const Vertex v = graph->get(ToVertex, edge);
                if (v == via) continue;
                VertexLabel& vLabel = getLabel(v);
                const int distance = uLabel->distance + (*weight)[edge];
                if (vLabel.distance > distance) {
                    vLabel.distance = distance;
                    Q.update(&vLabel);
                }
            }
            if constexpr (QPopLimit > 0) {
                qPops++;
                if constexpr (AdaptiveQPopLimit) {
                    if (qPops > qPopLimit) break;
                } else {
                    if (qPops > QPopLimit) break;
                }
            }
            profiler->settledVertex();
        }

        profiler->doneWitnessSearch();
        return getLabel(to).distance > shortcutDistance;
    }

    inline void reset() noexcept {
        currentFrom = noVertex;
        currentVia = noVertex;
    }

private:
    inline VertexLabel& getLabel(const Vertex vertex) noexcept {
        VertexLabel& result = label[vertex];
        if (result.timeStamp != timeStamp) result.reset(timeStamp);
        return result;
    }

private:
    const Graph* graph;
    const std::vector<int>* weight;

    ExternalKHeap<2, VertexLabel> Q;
    std::vector<VertexLabel> label;
    int timeStamp;
    Vertex currentFrom;
    Vertex currentVia;
    int qPops;
    int qPopLimit;

    Profiler* profiler;

};

}
