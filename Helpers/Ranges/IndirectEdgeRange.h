#pragma once

#include "../../DataStructures/Graph/Classes/GraphInterface.h"

template<typename GRAPH>
class IndirectEdgeRange {

public:
    using Graph = GRAPH;
    using Type = IndirectEdgeRange<Graph>;

private:
    using VertexRange = decltype(std::declval<Graph>().vertices());
    using VertexIterator = decltype(std::declval<VertexRange>().begin());

    using EdgeRange = decltype(std::declval<Graph>().edgesFrom(std::declval<Vertex>()));
    using EdgeIterator = decltype(std::declval<EdgeRange>().begin());

public:
    class Iterator {
    private:
        struct InnerIerator {
            EdgeRange currentEdgeRange;
            EdgeIterator currentEdge;
            EdgeIterator endEdge;
        };
    public:
        Iterator(const IndirectEdgeRange* const indirectEdgeRange, const VertexIterator& currentVertex) :
            indirectEdgeRange(indirectEdgeRange),
            currentVertex(currentVertex),
            undefined(true) {
            if (currentVertex != indirectEdgeRange->endVertex) {
                inner.currentEdgeRange = indirectEdgeRange->graph->edgesFrom(*currentVertex);
                inner.currentEdge = inner.currentEdgeRange.begin();
                inner.endEdge = inner.currentEdgeRange.end();
                checkCurrentEdge();
            }
        }
        inline bool operator!=(const Iterator& other) const noexcept  {
            return currentVertex != other.currentVertex;
        }
        inline EdgeWithFromVertex operator*() const noexcept  {
            return EdgeWithFromVertex(*inner.currentEdge, *currentVertex);
        }
        inline Iterator& operator++() noexcept {
            ++inner.currentEdge;
            checkCurrentEdge();
            return *this;
        }
        inline Iterator& operator+=(const size_t n) noexcept  {
            for (size_t i = 0; i < n; i++) {
                ++(*this);
            }
            return *this;
        }
        inline Iterator operator+(const size_t n) const noexcept {
            return Iterator(*this) += n;;
        }
        inline EdgeWithFromVertex operator[](const size_t n) const noexcept {
            return *(*this + n);
        }
    private:
        inline void checkCurrentEdge() noexcept {
            if (!(inner.currentEdge != inner.endEdge)) {
                ++currentVertex;
                while (currentVertex != indirectEdgeRange->endVertex) {
                    inner.currentEdgeRange = indirectEdgeRange->graph->edgesFrom(*currentVertex);
                    if (!inner.currentEdgeRange.empty()) {
                        inner.currentEdge = inner.currentEdgeRange.begin();
                        inner.endEdge = inner.currentEdgeRange.end();
                        break;
                    }
                    ++currentVertex;
                }
            }
        }
    private:
        const IndirectEdgeRange* indirectEdgeRange;
        VertexIterator currentVertex;
        union {
            bool undefined;
            InnerIerator inner;
        };
    };

    IndirectEdgeRange() :
        graph(nullptr),
        vertices(),
        endVertex(vertices.end()) {
    }

    IndirectEdgeRange(const Graph& graph) :
        graph(&graph),
        vertices(graph.vertices()),
        endVertex(vertices.end()) {
    }

    IndirectEdgeRange(const Graph&&) = delete;

    inline Iterator begin() const noexcept  {
        return Iterator(this, vertices.begin());
    }

    inline Iterator end() const noexcept  {
        return Iterator(this, endVertex);
    }

    inline bool empty() const noexcept {
        return graph->numEdges() == 0;
    }

    inline size_t size() const noexcept  {
        return graph->numEdges();
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
    VertexRange vertices;
    VertexIterator endVertex;

};
