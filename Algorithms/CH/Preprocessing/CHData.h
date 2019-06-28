/**********************************************************************************

 Copyright (c) 2019 Tobias Zündorf

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
 modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************************/

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

public:
    size_t numVertices;
    CHCoreGraph core;
    CHConstructionGraph forwardCH;
    CHConstructionGraph backwardCH;
    std::vector<Vertex> order;
    std::vector<uint16_t> level;

};

}
