#pragma once

#include "../../DataStructures/Graph/Classes/GraphInterface.h"

template<typename GRAPH>
class DirectEdgeRange {

public:
    using Graph = GRAPH;
    using Type = DirectEdgeRange<Graph>;

private:
    using EdgeRange = decltype(std::declval<Graph>().edges());
    using EdgeIterator = decltype(std::declval<EdgeRange>().begin());

public:
    class Iterator {
    public:
        Iterator(const Graph* const graph, const EdgeIterator& currentEdge) : graph(graph), currentEdge(currentEdge) {}
        inline bool operator!=(const Iterator& other) const noexcept  {return currentEdge != other.currentEdge;}
        inline EdgeWithFromVertex operator*() const noexcept  {return EdgeWithFromVertex(*currentEdge, graph->get(FromVertex, *currentEdge));}
        inline Iterator& operator++() noexcept {++currentEdge; return *this;}
        inline Iterator& operator+=(const size_t n) noexcept  {currentEdge += n; return *this;}
        inline Iterator operator+(const size_t n) const noexcept {return Iterator(graph, currentEdge + n);}
        inline EdgeWithFromVertex operator[](const size_t n) const noexcept {return *(*this + n);}
    private:
        const Graph* graph;
        EdgeIterator currentEdge;
    };

    DirectEdgeRange() :
        graph(nullptr) {
    }

    DirectEdgeRange(const Graph& graph) :
        graph(&graph),
        edges(graph.edges()) {
    }

    DirectEdgeRange(const Graph&&) = delete;

    inline Iterator begin() const noexcept  {
        return Iterator(graph, edges.begin());
    }

    inline Iterator end() const noexcept  {
        return Iterator(graph, edges.end());
    }

    inline bool empty() const noexcept {
        return edges.empty();
    }

    inline size_t size() const noexcept  {
        return edges.size();
    }

    inline EdgeWithFromVertex operator[](const size_t i) const noexcept  {
        AssertMsg(i < size(), "Index " << i << " is out of range!");
        return begin()[i];
    }

    inline EdgeWithFromVertex front() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        return begin()[0];
    }

    inline EdgeWithFromVertex back() const noexcept {
        AssertMsg(!empty(), "Range is empty!");
        return begin()[size() - 1];
    }

private:
    const Graph* graph;
    EdgeRange edges;

};
