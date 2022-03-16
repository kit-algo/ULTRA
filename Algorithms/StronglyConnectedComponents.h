#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "DepthFirstSearch.h"

#include "../Helpers/Types.h"
#include "../Helpers/Assert.h"
#include "../Helpers/Meta.h"

template<typename GRAPH, bool DEBUG = true, bool IGNORE_REVERSE_EDGES = false>
class StronglyConnectedComponents {

public:
    using Graph = GRAPH;
    constexpr static bool Debug = DEBUG;
    constexpr static bool IgnoreReverseEdges = IGNORE_REVERSE_EDGES;
    using Type = StronglyConnectedComponents<Graph, Debug, IgnoreReverseEdges>;

public:
    struct VertexLabelWithoutParent {
        VertexLabelWithoutParent() : index(-1), lowlink(-1), onStack(false) {}
        inline void setParent(const Vertex) const noexcept {}
        inline bool hasParent(const Vertex) const noexcept {return false;}
        int index;
        int lowlink;
        bool onStack;
    };

    struct VertexLabelWithParent {
        VertexLabelWithParent() : index(-1), lowlink(-1), onStack(false), parent(noVertex) {}
        inline void setParent(const Vertex vertex) noexcept {parent = vertex;}
        inline bool hasParent(const Vertex vertex) const noexcept {return parent == vertex;}
        int index;
        int lowlink;
        bool onStack;
        Vertex parent;
    };

    using VertexLabel = Meta::IF<IgnoreReverseEdges, VertexLabelWithParent, VertexLabelWithoutParent>;

public:
    StronglyConnectedComponents(const Graph& graph) :
        graph(graph),
        index(0),
        componentCount(0) {
    }

    inline int getComponent(const Vertex vertex) const noexcept {
        AssertMsg(vertex < component.size(), "vertex " << vertex << " is out of range (0, " << component.size() << ")!");
        return component[vertex];
    }

    inline size_t numComponents() const noexcept {
        return componentCount;
    }

    inline const std::vector<int>& getComponent() const noexcept {
        return component;
    }

    inline std::vector<int>& getComponent() noexcept {
        return component;
    }

    inline std::vector<size_t> getComponentSizes() const noexcept {
        std::vector<size_t> sizes(componentCount, 0);
        for (const Vertex vertex : graph.vertices()) {
            AssertMsg(vertex < component.size(), "vertex " << vertex << " is out of range (0, " << component.size() << ")!");
            AssertMsg(component[vertex] >= 0 && size_t(component[vertex]) < sizes.size(), "Component of vertex " << vertex << " is " << component[vertex] << ", but should be less than " << sizes.size() << "!");
            sizes[component[vertex]]++;
        }
        return sizes;
    }

    inline size_t getComponentSize(const int component) const noexcept {
        return getComponentSizes()[component];
    }

    inline int maxComponent() const noexcept {
        std::vector<size_t> size = getComponentSizes();
        int maxComponent = 0;
        size_t maxSize = size[maxComponent];
        for (size_t i = 1; i < componentCount; i++) {
            if (maxSize < size[i]) {
                maxComponent = i;
                maxSize = size[maxComponent];
            }
        }
        return maxComponent;
    }

    inline int maxComponentSize() const noexcept {
        const std::vector<size_t> size = getComponentSizes();
        size_t maxSize = size[0];
        for (size_t i = 1; i < componentCount; i++) {
            if (maxSize < size[i]) {
                maxSize = size[i];
            }
        }
        return maxSize;
    }

    inline void run() noexcept {
        if (Debug) std::cout << "Computing strongly connected components... " << std::flush;
        initialize();
        tarjanDFS();
        if (Debug) std::cout << "done (" << numComponents() << " components)." << std::endl;
    }

    inline void runRecursive() noexcept {
        if (Debug) std::cout << "Computing strongly connected components... " << std::flush;
        initialize();

        std::cout << "Start" << std::endl << std::flush;
        for (const Vertex v : graph.vertices()) {
            if (label[v].index == -1) {
                tarjan(v);
            }
        }

        if (Debug) std::cout << "done (" << numComponents() << " components)." << std::endl;
    }

private:
    inline void initialize() noexcept {
        std::vector<VertexLabel>(graph.numVertices()).swap(label);
        std::vector<int>(graph.numVertices(), -1).swap(component);
        stack.clear();
        index = 0;
        componentCount = 0;
    }

    void tarjan(Vertex v) noexcept {
        label[v].index = index;
        label[v].lowlink = index;
        index++;
        push(v);

        for (const Edge edge : graph.edgesFrom(v)) {
            Vertex w = graph.get(ToVertex, edge);
            if (label[w].index == -1) {
                tarjan(w);
                label[v].lowlink = std::min(label[v].lowlink, label[w].lowlink);
            } else if (contains(w)) {
                label[v].lowlink = std::min(label[v].lowlink, label[w].index);
            }
        }

        if (label[v].lowlink == label[v].index) {
            Vertex w;
            do {
                w = pop();
                component[w] = componentCount;
            } while(w != v);
            componentCount++;
        }
    }

    inline void tarjanDFS() noexcept {
        depthFirstSearch(graph, [&](const Edge edge, const Vertex v){
            // Traverse tree Edge
            const Vertex w = graph.get(ToVertex, edge);
            label[w].index = index;
            label[w].lowlink = index;
            if (IgnoreReverseEdges) label[w].setParent(v);
            index++;
            push(w);
        }, [&](const Edge edge, const Vertex v){
            // Traverse non tree Edge
            const Vertex w = graph.get(ToVertex, edge);
            if (IgnoreReverseEdges) {if (label[v].hasParent(w)) return;}
            if (!contains(w)) return;
            label[v].lowlink = std::min(label[v].lowlink, label[w].index);
        }, [&](const Edge edge, const Vertex v){
            // Backtrack
            Vertex w = graph.get(ToVertex, edge);
            if (label[w].lowlink == label[w].index) {
                Vertex u;
                do {
                    u = pop();
                    component[u] = componentCount;
                } while(u != w);
                componentCount++;
            }
            label[v].lowlink = std::min(label[v].lowlink, label[w].lowlink);
        }, [&](const Vertex v){
            // Root
            label[v].index = index;
            label[v].lowlink = index;
            index++;
            push(v);
        }, [&](const Vertex v){
            // Finalize
            if (label[v].lowlink == label[v].index) {
                Vertex w;
                do {
                    w = pop();
                    component[w] = componentCount;
                } while(w != v);
                componentCount++;
            }
        });
    }

    inline void push(const Vertex v) noexcept {
        AssertMsg(!contains(v), "Vertex " << v << " is already contained in the stack!");
        stack.push_back(v);
        label[v].onStack = true;
    }

    inline bool contains(const Vertex v) const noexcept {
        return label[v].onStack;
    }

    inline Vertex pop() noexcept {
        AssertMsg(!stack.empty(), "pop() cannot be called on an empty stack!");
        Vertex v = stack.back();
        stack.pop_back();
        label[v].onStack = false;
        return v;
    }

private:
    const Graph& graph;

    std::vector<VertexLabel> label;
    std::vector<Vertex> stack;
    int index;

    std::vector<int> component;
    size_t componentCount;

};

template<typename GRAPH, bool DEBUG = true>
using TwoConnectedComponents = StronglyConnectedComponents<GRAPH, DEBUG, true>;
