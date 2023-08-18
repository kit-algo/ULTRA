#pragma once

#include <queue>
#include <vector>

#include "../../DataStructures/Container/ExternalKHeap.h"
#include "../../DataStructures/Container/Set.h"
#include "../../DataStructures/Graph/Classes/GraphInterface.h"
#include "../../Helpers/Vector/Vector.h"

template<typename GRAPH>
class IsoDijkstra {

private:
    struct VertexLabel : public ExternalKHeapElement {
        VertexLabel() : distance(INFTY) {}
        int distance;

        inline bool hasSmallerKey(const VertexLabel* other) const {
            return distance < other->distance;
        }
    };

public:
    IsoDijkstra(const GRAPH& graph) :
        graph(graph),
        label(graph.numVertices()),
        sourceVertex(noVertex),
        range(INFTY),
        isochrone(graph.numVertices()) {
    }

    template<typename ATTRIBUTE>
    inline void run(const Vertex source, const int isochroneRange, const ATTRIBUTE weight) noexcept {
        clear();
        sourceVertex = source;
        range = isochroneRange;
        label[source].distance = 0;
        queue.update(&label[source]);
        isochrone.insert(source);

        while (!queue.empty()) {
            VertexLabel* uLabel = queue.extractFront();
            const Vertex u = Vertex(uLabel - &(label[0]));
            isochrone.insert(u);
            const int uDistance = uLabel->distance;
            for (const Edge e : graph.edgesFrom(u)) {
                const Vertex v = graph.get(ToVertex, e);
                const int vDistance = uDistance + graph.get(weight, e);
                if (vDistance < label[v].distance && vDistance <= range) {
                    label[v].distance = vDistance;
                    queue.update(&label[v]);
                }
            }
        }
    }

    inline int getDistance(const Vertex vertex) const noexcept {
        return label[vertex].distance;
    }

    inline bool reached(const Vertex vertex) const noexcept {
        return label[vertex].distance != INFTY;
    }

    inline std::vector<Vertex> getIsochrone() const noexcept {
        return isochrone;
    }

private:
    inline void clear() noexcept {
        std::vector<VertexLabel>(label.size()).swap(label);
        queue.clear();
        sourceVertex = noVertex;
        isochrone.clear();
    }

    const GRAPH& graph;
    std::vector<VertexLabel> label;
    ExternalKHeap<2, VertexLabel> queue;
    Vertex sourceVertex;
    int range;
    IndexedSet<false, Vertex> isochrone;
};
