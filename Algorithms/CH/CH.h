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

#include "Preprocessing/CHData.h"
#include "Preprocessing/CHBuilder.h"

#include "../../DataStructures/Graph/Graph.h"
#include "../../Helpers/Ranges/IndirectEdgeRange.h"
#include "../../Helpers/Ranges/ConcatenatedRange.h"
#include "../../Helpers/Ranges/Range.h"

namespace CH {

class CH {

public:
    using EdgeRange = ConcatenatedRange<Range<Edge>, Edge>;

    template<AttributeNameType ATTRIBUTE_NAME>
    using AttributeConstReferenceType = typename CHGraph::AttributeConstReferenceType<ATTRIBUTE_NAME>;

public:
    CH() {}

    CH(const CH&) = default;
    CH(CH&&) = default;

    CH& operator=(const CH&) = default;
    CH& operator=(CH&&) = default;

    CH(CHConstructionGraph&& forwardCH, CHConstructionGraph&& backwardCH) {
        Graph::move(std::move(forwardCH), forward);
        Graph::move(std::move(backwardCH), backward);
    }

    template<typename WITNESS_SEARCH, typename KEY_FUNCTION, typename STOP_CRITERION, bool BUILD_Q_LINEAR, bool BREAK_KEY_TIES_BY_ID>
    CH(Builder<WITNESS_SEARCH, KEY_FUNCTION, STOP_CRITERION, BUILD_Q_LINEAR, BREAK_KEY_TIES_BY_ID>&& builder) :
        CH(std::move(builder.getData().forwardCH), std::move(builder.getData().backwardCH)) {
    }

    CH(const std::string& fileName, const std::string& separator = ".") {
        readBinary(fileName, separator);
    }

    // Access
    inline const CHGraph& getGraph(const int direction) const noexcept {
        return (direction == FORWARD) ? forward : backward;
    }

    inline size_t numVertices() const noexcept {
        return forward.numVertices();
    }

    inline size_t numEdges() const noexcept {
        return forward.numEdges() + backward.numEdges();
    }

    inline size_t edgeLimit() const noexcept {
        return numEdges();
    }

    inline bool isVertex(const Vertex vertex) const noexcept {
        return vertex < numVertices();
    }

    inline bool isEdge(const Edge edge) const noexcept {
        return forward.isEdge(edge) || backward.isEdge(Edge(edge - forward.numEdges()));
    }

    inline Range<Vertex> vertices() const noexcept {
        return Range<Vertex>(Vertex(0), Vertex(numVertices()));
    }

    inline EdgeRange edgesFrom(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return EdgeRange(forward.edgesFrom(vertex), backward.edgesFrom(vertex), Edge(forward.numEdges()));
    }

    inline EdgeRange edges() const noexcept {
        return EdgeRange(forward.edges(), backward.edges(), Edge(forward.numEdges()));
    }

    inline IndirectEdgeRange<CH> edgesWithFromVertex() const noexcept {
        return IndirectEdgeRange<CH>(*this);
    }

    template<AttributeNameType ATTRIBUTE_NAME>
    inline AttributeConstReferenceType<ATTRIBUTE_NAME> get(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName, const Edge edge) const noexcept {
        return (edge < forward.numEdges()) ? forward.get(attributeName, edge) : backward.get(attributeName, Edge(edge - forward.numEdges()));
    }

    inline bool isCoreVertex(const Vertex vertex) const noexcept {
        for (const Edge forwardEdge : forward.edgesFrom(vertex)) {
            for (const Edge backwardEdge : backward.edgesFrom(forward.get(ToVertex, forwardEdge))) {
                if (backward.get(ToVertex, backwardEdge) == vertex) return true;
            }
        }
        return false;
    }

    // IO:
    inline void writeBinary(const std::string& fileName, const std::string& separator = ".") const noexcept {
        forward.writeBinary(fileName + separator + "forward", separator);
        backward.writeBinary(fileName + separator + "backward", separator);
    }

    inline void readBinary(const std::string& fileName, const std::string& separator = ".") noexcept {
        forward.readBinary(fileName + separator + "forward", separator);
        backward.readBinary(fileName + separator + "backward", separator);
    }

public:
    CHGraph forward;
    CHGraph backward;

};

}
