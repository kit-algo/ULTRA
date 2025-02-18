#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../DataStructures/Graph/Classes/GraphInterface.h"

#include "../Helpers/Assert.h"
#include "../Helpers/Vector/Permutation.h"

template<typename GRAPH, typename TRAVERSE_TREE_EDGE = NO_OPERATION, typename TRAVERSE_NON_TREE_EDGE = NO_OPERATION, typename BACKTRACK = NO_OPERATION, typename ROOT = NO_OPERATION, typename FINALIZE = NO_OPERATION>
class DepthFirstSearch {

private:
    struct Item {
        Edge edge;
        Vertex from;
        bool backtrack;
    };

public:

    DepthFirstSearch(const GRAPH& graph, const TRAVERSE_TREE_EDGE& traverseTreeEdge = TRAVERSE_TREE_EDGE(), const TRAVERSE_NON_TREE_EDGE& traverseNonTreeEdge = TRAVERSE_NON_TREE_EDGE(), const BACKTRACK& backtrack = BACKTRACK(), const ROOT& root = ROOT(), const FINALIZE& finalize = FINALIZE()) :
        traverseTreeEdge(traverseTreeEdge),
        traverseNonTreeEdge(traverseNonTreeEdge),
        backtrack(backtrack),
        root(root),
        finalize(finalize),
        graph(graph) {
    }

    inline void initialize() noexcept {
        std::vector<bool>(graph.numVertices(), false).swap(settled);
        stack.clear();
    }

    inline void run() noexcept {
        initialize();
        for (const Vertex vertex : graph.vertices()) {
            if (!settled[vertex]) {
                run(vertex);
            }
        }
    }

    inline void run(const Vertex u) noexcept {
        root(u);
        settle(u);
        while (!stack.empty()) {
            const Edge edge = topEdge();
            const Vertex v = topVertex();
            if (topBacktrack()) {
                backtrack(edge, v);
                pop();
            } else {
                const Vertex w = graph.get(ToVertex, edge);
                if (settled[w]) {
                    traverseNonTreeEdge(edge, v);
                    pop();
                } else {
                    traverseTreeEdge(edge, v);
                    settle(w);
                }
            }
        }
        finalize(u);
    }

    inline bool hasSettled(const Vertex vertex) const noexcept {
        return settled[vertex];
    }

private:
    inline void settle(const Vertex v) noexcept {
        Assert(!settled[v], "Vertex " << v << " is already settled!");
        settled[v] = true;
        for (const Edge edge : graph.edgesFrom(v)) {
            push(edge, v);
        }
    }

    inline void push(const Edge edge, const Vertex from) noexcept {
        stack.push_back(Item({edge, from, false}));
    }

    inline Edge topEdge() const noexcept {
        return stack.back().edge;
    }

    inline Vertex topVertex() const noexcept {
        return stack.back().from;
    }

    inline bool topBacktrack() noexcept {
        const bool backtrack = stack.back().backtrack;
        stack.back().backtrack = true;
        return backtrack;
    }

    inline void pop() noexcept {
        stack.pop_back();
    }

private:
    TRAVERSE_TREE_EDGE traverseTreeEdge;
    TRAVERSE_NON_TREE_EDGE traverseNonTreeEdge;
    BACKTRACK backtrack;
    ROOT root;
    FINALIZE finalize;

    const GRAPH& graph;
    std::vector<bool> settled;
    std::vector<Item> stack;

};

template<typename GRAPH, typename OPERATION>
struct SimpleOperation {
    SimpleOperation(const GRAPH& graph, const OPERATION& operation) : operation(operation), graph(graph) {}
    inline void operator()(const Edge edge, const Vertex) {operation(graph.get(ToVertex, edge));}
    OPERATION operation;
    const GRAPH& graph;
};

template<typename GRAPH, typename TRAVERSE_TREE_EDGE = NO_OPERATION, typename TRAVERSE_NON_TREE_EDGE = NO_OPERATION, typename BACKTRACK = NO_OPERATION>
class DFS : public DepthFirstSearch<GRAPH, SimpleOperation<GRAPH, TRAVERSE_TREE_EDGE>, SimpleOperation<GRAPH, TRAVERSE_NON_TREE_EDGE>, SimpleOperation<GRAPH, BACKTRACK>, TRAVERSE_TREE_EDGE, BACKTRACK> {

public:
    using DFSType = DepthFirstSearch<GRAPH, SimpleOperation<GRAPH, TRAVERSE_TREE_EDGE>, SimpleOperation<GRAPH, TRAVERSE_NON_TREE_EDGE>, SimpleOperation<GRAPH, BACKTRACK>, TRAVERSE_TREE_EDGE, BACKTRACK>;
    DFS(const GRAPH& graph, const TRAVERSE_TREE_EDGE& traverseTreeEdge = TRAVERSE_TREE_EDGE(), const TRAVERSE_NON_TREE_EDGE& traverseNonTreeEdge = TRAVERSE_NON_TREE_EDGE(), const BACKTRACK& backtrack = BACKTRACK()) :
        DFSType(graph, SimpleOperation<GRAPH, TRAVERSE_TREE_EDGE>(graph, traverseTreeEdge), SimpleOperation<GRAPH, TRAVERSE_NON_TREE_EDGE>(graph, traverseNonTreeEdge), SimpleOperation<GRAPH, BACKTRACK>(graph, backtrack), traverseTreeEdge, backtrack) {
    }

};

template<typename GRAPH, typename TRAVERSE_TREE_EDGE = NO_OPERATION, typename TRAVERSE_NON_TREE_EDGE = NO_OPERATION, typename BACKTRACK = NO_OPERATION, typename ROOT = NO_OPERATION, typename FINALIZE = NO_OPERATION>
inline void depthFirstSearch(const GRAPH& graph, const TRAVERSE_TREE_EDGE& traverseTreeEdge = TRAVERSE_TREE_EDGE(), const TRAVERSE_NON_TREE_EDGE& traverseNonTreeEdge = TRAVERSE_NON_TREE_EDGE(), const BACKTRACK& backtrack = BACKTRACK(), const ROOT& root = ROOT(), const FINALIZE& finalize = FINALIZE()) {
    DepthFirstSearch<GRAPH, TRAVERSE_TREE_EDGE, TRAVERSE_NON_TREE_EDGE, BACKTRACK, ROOT, FINALIZE> dfs(graph, traverseTreeEdge, traverseNonTreeEdge, backtrack, root, finalize);
    dfs.run();
}

template<typename GRAPH, typename TRAVERSE_TREE_EDGE = NO_OPERATION, typename TRAVERSE_NON_TREE_EDGE = NO_OPERATION, typename BACKTRACK = NO_OPERATION>
inline void dfs(const GRAPH& graph, const TRAVERSE_TREE_EDGE& traverseTreeEdge = TRAVERSE_TREE_EDGE(), const TRAVERSE_NON_TREE_EDGE& traverseNonTreeEdge = TRAVERSE_NON_TREE_EDGE(), const BACKTRACK& backtrack = BACKTRACK()) {
    DFS<GRAPH, TRAVERSE_TREE_EDGE, TRAVERSE_NON_TREE_EDGE, BACKTRACK> dfs(graph, traverseTreeEdge, traverseNonTreeEdge, backtrack);
    dfs.run();
}

template<typename GRAPH>
inline Order getDFSVertexOrder(const GRAPH& graph) noexcept {
    Order order;
    depthFirstSearch(graph, [&](const Edge e, const Vertex) {
        const Vertex v = graph.get(ToVertex, e);
        order.emplace_back(v);
    }, NoOperation, NoOperation, [&](const Vertex v) {
        order.emplace_back(v);
    }, NoOperation);
    return order;
}
