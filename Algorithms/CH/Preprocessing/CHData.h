#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../../DataStructures/Graph/Graph.h"

namespace CH {

class Data {

public:
    Data() : numVertices(0) {}

    Data(const Data&) = default;
    Data(Data&&) = default;

    Data& operator=(const Data&) = default;
    Data& operator=(Data&&) = default;

    template<typename GRAPH>
    Data(GRAPH&& graph, std::vector<int> weight) :
        numVertices(graph.numVertices()) {
        Graph::move(std::move(graph), core);
        core[Weight].swap(weight);
        forwardCH.addVertices(numVertices);
        backwardCH.addVertices(numVertices);
        std::vector<uint16_t>(numVertices, 0).swap(level);
    }

    template<typename GRAPH, AttributeNameType ATTRIBUTE_NAME>
    Data(GRAPH&& graph, const AttributeNameWrapper<ATTRIBUTE_NAME> weight) :
        numVertices(graph.numVertices()) {
        Graph::move(std::move(graph), core, Weight << weight);
        forwardCH.addVertices(numVertices);
        backwardCH.addVertices(numVertices);
        std::vector<uint16_t>(numVertices, 0).swap(level);
    }

    Data(CHCoreGraph&& graph) :
        numVertices(graph.numVertices()),
        core(std::move(graph)) {
        forwardCH.addVertices(numVertices);
        backwardCH.addVertices(numVertices);
        std::vector<uint16_t>(numVertices, 0).swap(level);
    }

    inline size_t coreSize() const noexcept {
        return level.size() - order.size();
    }

    inline size_t numContracted() const noexcept {
        return order.size();
    }

    inline void applyVertexPermutation(const Permutation& permutation) noexcept {
        core.applyVertexPermutation(permutation);
        forwardCH.applyVertexPermutation(permutation);
        backwardCH.applyVertexPermutation(permutation);
        permutation.permutate(level);
        permutation.mapPermutation(order);
    }

    inline void applyVertexOrder(const Order& vertexOrder) noexcept {
        applyVertexPermutation(Permutation(Construct::Invert, vertexOrder));
    }

public:
    size_t numVertices;
    CHCoreGraph core;
    CHConstructionGraph forwardCH;
    CHConstructionGraph backwardCH;
    std::vector<Vertex> order;
    std::vector<uint16_t> level;

};

}
