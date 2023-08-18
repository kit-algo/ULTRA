#pragma once

#include <vector>

#include "../Helpers/Types.h"

template<typename GRAPH>
inline std::vector<int> greedyVertexColors(const GRAPH& graph) noexcept {
    std::vector<int> color(graph.numVertices(), -1);
    for (const Vertex vertex : graph.vertices()) {
        bool notColored = true;
        while (notColored) {
            color[vertex]++;
            notColored = false;
            for (const Edge edge : graph.edgesFrom(vertex)) {
                if (color[vertex] != color[graph.get(ToVertex, edge)]) continue;
                notColored = true;
                break;
            }
        }
    }
    return color;
}
