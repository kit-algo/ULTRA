#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "CH.h"
#include "Preprocessing/CHData.h"

#include "../DepthFirstSearch.h"

#include "../../Helpers/Helpers.h"
#include "../../Helpers/Vector/Vector.h"
#include "../../Helpers/Vector/Permutation.h"
#include "../../Helpers/String/String.h"

namespace CH {

inline std::vector<Vertex> getOrder(const CH& ch) noexcept {
    std::vector<Vertex> order;
    order.reserve(ch.numVertices());
    depthFirstSearch(ch, NoOperation, NoOperation, [&](const Edge edge, const Vertex) {
        order.push_back(ch.get(ToVertex, edge));
    }, NoOperation, [&](const Vertex root) {
        order.push_back(root);
    });
    size_t front = 0;
    size_t back = order.size() - 1;
    std::vector<Vertex> coreOrder(order.size());
    while (!order.empty()) {
        if (ch.isCoreVertex(order.back())) {
            coreOrder[back] = order.back();
            back--;
        } else {
            coreOrder[front] = order.back();
            front++;
        }
        order.pop_back();
    }
    return coreOrder;
}

inline std::vector<uint16_t> vertexLevelTopDown(const CHGraph& ch, const std::vector<Vertex>& order) noexcept {
    std::vector<uint16_t> level(order.size(), 0);
    for (Vertex vertex : descending(order)) {
        for (Edge edge : ch.edgesFrom(vertex)) {
            level[vertex] = std::max<uint16_t>(level[vertex], level[ch.get(ToVertex, edge)] + 1);
        }
    }
    return level;
}

inline std::vector<uint16_t> vertexLevelTopDown(const CH& ch, const std::vector<Vertex>& order) noexcept {
    std::vector<uint16_t> level(order.size(), 0);
    for (Vertex vertex : descending(order)) {
        if (ch.isCoreVertex(vertex)) continue;
        for (Edge edge : ch.forward.edgesFrom(vertex)) {
            level[vertex] = std::max<uint16_t>(level[vertex], level[ch.forward.get(ToVertex, edge)] + 1);
        }
        for (Edge edge : ch.backward.edgesFrom(vertex)) {
            level[vertex] = std::max<uint16_t>(level[vertex], level[ch.backward.get(ToVertex, edge)] + 1);
        }
    }
    return level;
}

inline std::vector<uint16_t> vertexLevelBottomUp(const CH& ch, const std::vector<Vertex>& order) noexcept {
    std::vector<uint16_t> level(order.size(), 0);
    for (Vertex vertex : order) {
        if (ch.isCoreVertex(vertex)) continue;
        for (Edge edge : ch.forward.edgesFrom(vertex)) {
            level[ch.forward.get(ToVertex, edge)] = std::max<uint16_t>(level[vertex] + 1, level[ch.forward.get(ToVertex, edge)]);
        }
        for (Edge edge : ch.backward.edgesFrom(vertex)) {
            level[ch.backward.get(ToVertex, edge)] = std::max<uint16_t>(level[vertex] + 1, level[ch.backward.get(ToVertex, edge)]);
        }
    }
    return level;
}

inline std::vector<std::vector<Vertex>> verticesByLevel(const CH& ch, const std::vector<uint16_t>& level) noexcept {
    std::vector<std::vector<Vertex>> verticesByLevel;
    for (Vertex vertex : ch.vertices()) {
        if (verticesByLevel.size() <= level[vertex]) verticesByLevel.resize(level[vertex] + 1);
        verticesByLevel[level[vertex]].push_back(vertex);
    }
    return verticesByLevel;
}

inline std::vector<size_t> verticesPerLevel(const CH& ch, const std::vector<uint16_t>& level) noexcept {
    std::vector<size_t> verticesPerLevel;
    for (Vertex vertex : ch.vertices()) {
        if (verticesPerLevel.size() <= level[vertex]) verticesPerLevel.resize(level[vertex] + 1, 0);
        verticesPerLevel[level[vertex]]++;
    }
    return verticesPerLevel;
}

inline std::vector<std::vector<Vertex>> verticesByLevelTopDown(const CH& ch, const std::vector<Vertex>& order) noexcept {
    return verticesByLevel(ch, vertexLevelTopDown(ch, order));
}

inline std::vector<size_t> verticesPerLevelTopDown(const CH& ch, const std::vector<Vertex>& order) noexcept {
    return verticesPerLevel(ch, vertexLevelTopDown(ch, order));
}

inline std::vector<Vertex> getLevelOrderTopDown(const CH& ch) noexcept {
    return Vector::flatten(verticesByLevel(ch, vertexLevelTopDown(ch, getOrder(ch))));
}

inline std::vector<std::vector<Vertex>> verticesByLevelBottomUp(const CH& ch, const std::vector<Vertex>& order) noexcept {
    return verticesByLevel(ch, vertexLevelTopDown(ch, order));
}

inline std::vector<size_t> verticesPerLevelBottomUp(const CH& ch, const std::vector<Vertex>& order) noexcept {
    return verticesPerLevel(ch, vertexLevelBottomUp(ch, order));
}

inline std::vector<Vertex> getLevelOrderBottomUp(const CH& ch) noexcept {
    return Vector::flatten(verticesByLevel(ch, vertexLevelBottomUp(ch, getOrder(ch))));
}

inline Data expandData(const CH& ch) noexcept {
    Data data;
    data.numVertices = ch.numVertices();
    data.order = getOrder(ch);
    data.level = vertexLevelBottomUp(ch, data.order);
    data.core.addVertices(data.numVertices);
    data.forwardCH.addVertices(data.numVertices);
    data.backwardCH.addVertices(data.numVertices);
    while (!data.order.empty() && ch.isCoreVertex(data.order.back())) {
        for (const Edge edge : ch.forward.edgesFrom(data.order.back())) {
            data.core.addEdge(data.order.back(), ch.forward.get(ToVertex, edge)).set(ViaVertex, ch.forward.get(ViaVertex, edge)).set(Weight, ch.forward.get(Weight, edge));
        }
        data.order.pop_back();
    }
    for (const Vertex vertex : data.order) {
        AssertMsg(!ch.isCoreVertex(vertex), "Vertex " << vertex << " is a core vertex!");
        for (const Edge edge : ch.forward.edgesFrom(vertex)) {
            data.forwardCH.addEdge(vertex, ch.forward.get(ToVertex, edge)).set(ViaVertex, ch.forward.get(ViaVertex, edge)).set(Weight, ch.forward.get(Weight, edge));
        }
        for (const Edge edge : ch.backward.edgesFrom(vertex)) {
            data.backwardCH.addEdge(vertex, ch.backward.get(ToVertex, edge)).set(ViaVertex, ch.backward.get(ViaVertex, edge)).set(Weight, ch.backward.get(Weight, edge));
        }
    }
    return data;
}

template<typename GRAPH>
inline std::vector<Vertex> unpackShortcut(const GRAPH& forward, const GRAPH& backward, const Vertex from, const Vertex via, const Vertex to) noexcept {
    struct Shortcut {const Vertex via; const Vertex to;};
    std::vector<Shortcut> unprocessedShortCuts;
    std::vector<Vertex> path;
    unprocessedShortCuts.push_back(Shortcut({via, to}));
    path.push_back(from);
    while (!unprocessedShortCuts.empty()) {
        const Vertex from = path.back();
        const Vertex via = unprocessedShortCuts.back().via;
        const Vertex to = unprocessedShortCuts.back().to;
        unprocessedShortCuts.pop_back();
        if (forward.isVertex(via)) {
            const Edge first = backward.findEdge(via, from);
            const Edge second = forward.findEdge(via, to);
            Assert(backward.isEdge(first));
            Assert(forward.isEdge(second));
            unprocessedShortCuts.push_back(Shortcut({forward.get(ViaVertex, second), to}));
            unprocessedShortCuts.push_back(Shortcut({backward.get(ViaVertex, first), via}));
        } else {
            path.push_back(to);
        }
    }
    return path;
}

inline std::vector<Vertex> unpackShortcut(const CH& ch, const Vertex from, const Vertex via, const Vertex to) noexcept {
    return unpackShortcut(ch.forward, ch.backward, from, via, to);
}

inline void analyze(const CH& ch, const std::vector<Vertex>& order, std::ostream& out = std::cout) noexcept {
    Assert(ch.numVertices() == order.size());
    Assert(Order(order).isValid());
    out << "Forward Graph: " << std::endl << std::flush;
    Graph::printInfo(ch.forward);
    ch.forward.printAnalysis();
    out << "Backward Graph: " << std::endl << std::flush;
    Graph::printInfo(ch.backward);
    ch.backward.printAnalysis();
    out << "CH: " << std::endl << std::flush;
    size_t vertexCount = 0;
    size_t isolatedVertexCount = 0;
    size_t sourceCount = 0;
    size_t sinkCount = 0;
    size_t minInDegree = intMax;
    size_t maxInDegree = 0;
    size_t minOutDegree = intMax;
    size_t maxOutDegree = 0;
    size_t edgeCount = 0;
    size_t levelCount = verticesPerLevelTopDown(ch, order).size();
    std::vector<size_t> inDegree(ch.numVertices(), 0);
    std::vector<size_t> outDegree(ch.numVertices(), 0);
    for (Vertex vertex : ch.vertices()) {
        vertexCount++;
        for (Edge edge : ch.edgesFrom(vertex)) {
            Vertex to = ch.get(ToVertex, edge);
            inDegree[to]++;
            outDegree[vertex]++;
            edgeCount++;
        }
    }
    for (size_t v = 0; v < inDegree.size(); v++) {
        if (inDegree[v] == 0 && outDegree[v] == 0) isolatedVertexCount++;
        if (inDegree[v] == 0) sourceCount++;
        if (outDegree[v] == 0) sinkCount++;
        if (inDegree[v] < minInDegree) minInDegree = inDegree[v];
        if (inDegree[v] > maxInDegree) maxInDegree = inDegree[v];
        if (outDegree[v] < minOutDegree) minOutDegree = outDegree[v];
        if (outDegree[v] > maxOutDegree) maxOutDegree = outDegree[v];
    }
    int tabSize = 18;
    out << std::right;
    out << "                  #Vertices : " << std::setw(tabSize) << String::prettyInt(vertexCount) << std::endl;
    out << "          #IsolatedVertices : " << std::setw(tabSize) << String::prettyInt(isolatedVertexCount) << "  (" << String::percent(isolatedVertexCount / (double) vertexCount) << ")" << std::endl;
    out << "                   #Sources : " << std::setw(tabSize) << String::prettyInt(sourceCount) << "  (" << String::percent(sourceCount / (double) vertexCount) << ")" << std::endl;
    out << "                     #Sinks : " << std::setw(tabSize) << String::prettyInt(sinkCount) << "  (" << String::percent(sinkCount / (double) vertexCount) << ")" << std::endl;
    out << "                minInDegree : " << std::setw(tabSize) << String::prettyInt(minInDegree) << std::endl;
    out << "                maxInDegree : " << std::setw(tabSize) << String::prettyInt(maxInDegree) << std::endl;
    out << "               minOutDegree : " << std::setw(tabSize) << String::prettyInt(minOutDegree) << std::endl;
    out << "               maxOutDegree : " << std::setw(tabSize) << String::prettyInt(maxOutDegree) << std::endl;
    out << "                     #Edges : " << std::setw(tabSize) << String::prettyInt(edgeCount) << std::endl;
    out << "                     #Level : " << std::setw(tabSize) << String::prettyInt(levelCount) << std::endl;
}

inline void analyze(const CH& ch, std::ostream& out = std::cout) noexcept {
    analyze(ch, getOrder(ch), out);
}

}
