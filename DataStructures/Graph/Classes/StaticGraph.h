#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <type_traits>
#include <iomanip>
#include <algorithm>

#include "GraphInterface.h"

#include "../Utils/Utils.h"

#include "../../Container/Set.h"
#include "../../Geometry/Point.h"
#include "../../Geometry/Rectangle.h"

#include "../../../Helpers/FileSystem/FileSystem.h"
#include "../../../Helpers/IO/Serialization.h"
#include "../../../Helpers/Vector/Permutation.h"
#include "../../../Helpers/Ranges/Range.h"
#include "../../../Helpers/Ranges/SubRange.h"
#include "../../../Helpers/Ranges/IndirectEdgeRange.h"

template<typename LIST_OF_VERTEX_ATTRIBUTES, typename LIST_OF_EDGE_ATTRIBUTES>
class StaticGraphImplementation {
    static_assert(Meta::Equals<Vertex, Meta::FindAttributeType<ToVertex, LIST_OF_EDGE_ATTRIBUTES>>(), "A static graph requires an edge attribute named ToVertex of type Vertex!");

public:
    using ListOfVertexAttributes = LIST_OF_VERTEX_ATTRIBUTES;
    using ListOfEdgeAttributes = LIST_OF_EDGE_ATTRIBUTES;
    using Type = StaticGraphImplementation<ListOfVertexAttributes, ListOfEdgeAttributes>;

    using VertexAttributes = Attributes<ListOfVertexAttributes>;
    using EdgeAttributes = Attributes<ListOfEdgeAttributes>;

    using VertexHandle = AttributeHandle<VertexAttributes, Vertex>;
    using EdgeHandle = AttributeHandle<EdgeAttributes, Edge>;

    using ListOfRecordVertexAttributes = ListOfVertexAttributes;
    using ListOfRecordEdgeAttributes = Meta::RemoveAttribute<FromVertex, Meta::RemoveAttribute<ToVertex, Meta::RemoveAttribute<ReverseEdge, ListOfEdgeAttributes>>>;
    using VertexRecord = AttributeRecord<ListOfRecordVertexAttributes>;
    using EdgeRecord = AttributeRecord<ListOfRecordEdgeAttributes>;

    using ListOfAllAttributes = Meta::Concat<ListOfVertexAttributes, ListOfEdgeAttributes>;

    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    friend inline void Graph::move(StaticGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, StaticGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    friend inline void Graph::move(StaticGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, DynamicGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    friend inline void Graph::move(StaticGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, EdgeListImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    friend inline void Graph::move(DynamicGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, StaticGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    friend inline void Graph::move(EdgeListImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, StaticGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);

    template<AttributeNameType ATTRIBUTE_NAME>
    inline constexpr static bool HasVertexAttribute(const AttributeNameWrapper<ATTRIBUTE_NAME>) noexcept {return VertexAttributes::HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>());}

    template<AttributeNameType ATTRIBUTE_NAME>
    using VertexAttributeType = typename VertexAttributes::template AttributeType<ATTRIBUTE_NAME>;

    template<AttributeNameType ATTRIBUTE_NAME>
    inline constexpr static bool HasEdgeAttribute(const AttributeNameWrapper<ATTRIBUTE_NAME>) noexcept {return EdgeAttributes::HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>());}

    template<AttributeNameType ATTRIBUTE_NAME>
    using EdgeAttributeType = typename EdgeAttributes::template AttributeType<ATTRIBUTE_NAME>;

    template<AttributeNameType ATTRIBUTE_NAME>
    using AttributeType = typename Meta::FindAttributeType<ATTRIBUTE_NAME, ListOfAllAttributes>;

    template<AttributeNameType ATTRIBUTE_NAME>
    using AttributeReferenceType = typename std::vector<Meta::FindAttributeType<ATTRIBUTE_NAME, ListOfAllAttributes>>::reference;

    template<AttributeNameType ATTRIBUTE_NAME>
    using AttributeConstReferenceType = typename std::vector<Meta::FindAttributeType<ATTRIBUTE_NAME, ListOfAllAttributes>>::const_reference;

public:
    StaticGraphImplementation() {
        beginOut.emplace_back(Edge(0));
    }

    StaticGraphImplementation(const std::string& fileName, const std::string& separator = ".", const bool debug = true) {
        readBinary(fileName, separator, debug);
    }

    StaticGraphImplementation(const Type&) = default;
    StaticGraphImplementation(Type&&) = default;

    Type& operator=(const Type&) = default;
    Type& operator=(Type&&) = default;

    // Access
    inline size_t numVertices() const noexcept {
        return beginOut.size() - 1;
    }

    inline size_t numEdges() const noexcept {
        return edgeAttributes.size();
    }

    inline size_t edgeLimit() const noexcept {
        return edgeAttributes.size();
    }

    inline bool isVertex(const Vertex vertex) const noexcept {
        return vertex < numVertices();
    }

    inline bool isEdge(const Edge edge) const noexcept {
        return edge < numEdges();
    }

    inline size_t outDegree(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return beginOut[vertex + 1] - beginOut[vertex];
    }

    inline Range<Vertex> vertices() const noexcept {
        return Range<Vertex>(Vertex(0), Vertex(numVertices()));
    }

    inline Range<Edge> edgesFrom(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return Range<Edge>(beginOut[vertex], beginOut[vertex + 1]);
    }

    inline Edge beginEdgeFrom(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex) || vertex == numVertices(), vertex << " is not a valid vertex!");
        return beginOut[vertex];
    }

    inline Edge endEdgeFrom(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return beginOut[vertex + 1];
    }

    inline Range<Edge> edges() const noexcept {
        return Range<Edge>(Edge(0), Edge(numEdges()));
    }

    inline IndirectEdgeRange<Type> edgesWithFromVertex() const noexcept {
        return IndirectEdgeRange<Type>(*this);
    }

    inline SubRange<std::vector<Vertex>> outgoingNeighbors(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return SubRange<std::vector<Vertex>>(get(ToVertex), beginOut[vertex], beginOut[vertex + 1]);
    }

    inline Edge findEdge(const Vertex from, const Vertex to) const noexcept {
        if (!isVertex(from)) return noEdge;
        for (const Edge edge : edgesFrom(from)) {
            if (get(ToVertex, edge) == to) return edge;
        }
        return noEdge;
    }

    inline bool hasEdge(const Vertex from, const Vertex to) const noexcept {
        return isEdge(findEdge(from, to));
    }

    inline bool empty() const noexcept {
        return numVertices() == 0;
    }

    inline long long byteSize() const noexcept {
        long long result = vertexAttributes.byteSize();
        result += edgeAttributes.byteSize();
        return result;
    }

    inline long long memoryUsageInBytes() const noexcept {
        long long result = vertexAttributes.memoryUsageInBytes();
        result += edgeAttributes.memoryUsageInBytes();
        return result;
    }

    // Manipulation:
    inline void clear() noexcept {
        vertexAttributes.clear();
        edgeAttributes.clear();
    }

    inline void reserve(const size_t numVertices, const size_t numEdges) noexcept {
        beginOut.reserve(numVertices + 1);
        vertexAttributes.reserve(numVertices);
        edgeAttributes.reserve(numEdges);
    }

    inline Vertex addVertex() noexcept {
        addVertices();
        return Vertex(numVertices() - 1);
    }

    inline void addVertices(const size_t n = 1)  noexcept {
        beginOut.resize(beginOut.size() + n, beginOut.back());
        vertexAttributes.resize(vertexAttributes.size() + n);
    }

    inline Vertex addVertex(const VertexRecord& record) noexcept {
        addVertices(1, record);
        return Vertex(numVertices() - 1);
    }

    inline void addVertices(const size_t n, const VertexRecord& record) noexcept {
        beginOut.resize(beginOut.size() + n, beginOut.back());
        vertexAttributes.resize(vertexAttributes.size() + n, record);
    }

    inline EdgeHandle addEdge(const Vertex from, const Vertex to) noexcept {
        AssertMsg(from == numVertices() - 1, "Can only add outgoing edges to last vertex!");
        if (from != numVertices() - 1) return EdgeHandle(edgeAttributes, noEdge);
        beginOut.back()++;
        const Edge newEdge(edgeAttributes.size());
        edgeAttributes.emplaceBack();
        set(ToVertex, newEdge, to);
        if constexpr (HasVertexAttribute(FromVertex)) {
            set(FromVertex, newEdge, from);
        }
        if constexpr (HasEdgeAttribute(ReverseEdge)) {
            const Edge reverse = findEdge(to, from);
            if (isEdge(reverse) && !isEdge(get(ReverseEdge, reverse))) {
                set(ReverseEdge, newEdge, reverse);
                set(ReverseEdge, reverse, newEdge);
            }
        }
        return EdgeHandle(edgeAttributes, newEdge);
    }

    inline EdgeHandle addEdge(const Vertex from, const Vertex to, const EdgeRecord& record) noexcept {
        return addEdge(from, to).set(record);
    }

    inline void redirectEdge(const Edge edge, const Vertex oldFrom, const Vertex newTo) noexcept {
        AssertMsg(isVertex(oldFrom), oldFrom << " is not a valid vertex!");
        AssertMsg(isVertex(newTo), newTo << " is not a valid vertex!");
        AssertMsg(isEdge(edge), edge << " is not a valid edge!");
        AssertMsg(beginOut[oldFrom] <= edge && beginOut[oldFrom + 1] > edge, edge << " is not an outgoing edge of " << oldFrom);
        if constexpr (HasEdgeAttribute(ReverseEdge)) {
            if (isEdge(get(ReverseEdge, edge))) {
                set(ReverseEdge, get(ReverseEdge, edge), noEdge);
            }
            set(ReverseEdge, edge, noEdge);
            const Edge reverse = findEdge(newTo, oldFrom);
            if (isEdge(reverse) && !isEdge(get(ReverseEdge, reverse))) {
                set(ReverseEdge, edge, reverse);
                set(ReverseEdge, reverse, edge);
            }
        }
        set(ToVertex, edge, newTo);
    }

    template<typename DELETE_VERTEX>
    inline void deleteVertices(const DELETE_VERTEX& deleteVertex) noexcept {
        size_t vertexCount = 0;
        size_t edgeCount = 0;
        Permutation vertexPerm(numVertices());
        Permutation edgePerm(numEdges());
        std::vector<Edge> newBeginOut;
        newBeginOut.emplace_back(Edge(0));
        for (const Vertex vertex : vertices()) {
            if (deleteVertex(vertex)) {
                vertexPerm[vertex] = numVertices() + vertexCount - vertex - 1;
                for (const Edge edge : edgesFrom(vertex)) {
                    edgePerm[edge] = numEdges() + edgeCount - edge - 1;
                }
            } else {
                vertexPerm[vertex] = vertexCount++;
                for (const Edge edge : edgesFrom(vertex)) {
                    if (deleteVertex(get(ToVertex, edge))) {
                        edgePerm[edge] = numEdges() + edgeCount - edge - 1;
                    } else {
                        edgePerm[edge] = edgeCount++;
                    }
                }
                newBeginOut.emplace_back(Edge(edgeCount));
            }
        }

        vertexAttributes.forEach([&](std::vector<Vertex>& values) {
            vertexPerm.mapPermutation(values);
        });
        vertexAttributes.forEach([&](std::vector<Edge>& values) {
            edgePerm.mapPermutation(values);
        });
        edgeAttributes.forEach([&](std::vector<Vertex>& values) {
            vertexPerm.mapPermutation(values);
        });
        edgeAttributes.forEach([&](std::vector<Edge>& values) {
            edgePerm.mapPermutation(values);
        });
        vertexAttributes.forEach([&](auto& values) {
            vertexPerm.permutate(values);
        });
        edgeAttributes.forEach([&](auto& values) {
            edgePerm.permutate(values);
        });

        vertexAttributes.resize(vertexCount);
        edgeAttributes.resize(edgeCount);
        beginOut.swap(newBeginOut);
    }

    template<typename T>
    inline void deleteVertices(const std::vector<T>& vertexMap, const T& deleteValue) noexcept {
        deleteVertices([&](const Vertex vertex){return vertexMap[vertex] == deleteValue;});
    }

    inline void deleteVertices(const std::vector<Vertex>& vertexList) noexcept {
        std::vector<bool> deleteVertex(numVertices(), false);
        for (const Vertex vertex : vertexList) deleteVertex[vertex] = true;
        deleteVertices([&](const Vertex vertex){return deleteVertex[vertex];});
    }

    template<typename DELETE_EDGE>
    inline void deleteEdges(const DELETE_EDGE& deleteEdge) noexcept {
        size_t edgeCount = 0;
        Permutation edgePerm(numEdges());
        std::vector<Edge> newBeginOut;
        for (const Vertex vertex : vertices()) {
            newBeginOut.emplace_back(Edge(edgeCount));
            for (const Edge edge : edgesFrom(vertex)) {
                if (deleteEdge(edge)) continue;
                edgePerm[edge] = edgeCount++;
            }
        }
        newBeginOut.emplace_back(Edge(edgeCount));

        vertexAttributes.forEach([&](std::vector<Edge>& values) {
            edgePerm.mapPermutation(values);
        });
        edgeAttributes.forEach([&](std::vector<Edge>& values) {
            edgePerm.mapPermutation(values);
        });
        edgeAttributes.forEach([&](auto& values) {
            edgePerm.permutate(values);
        });

        edgeAttributes.resize(edgeCount);
        beginOut.swap(newBeginOut);

    }

    inline void applyVertexPermutation(const Permutation& permutation) noexcept {
        changeVertexIds(Order(Construct::Invert, permutation), permutation);
    }

    inline void applyVertexOrder(const Order& order) noexcept {
        changeVertexIds(order, Permutation(Construct::Invert, order));
    }

    inline void revert() noexcept {
        if (numEdges() == 0) return;
        std::vector<Edge> newBeginOut(beginOut.size(), Edge(0));
        std::vector<Vertex> fromVertex(numEdges());
        for (const auto [edge, from] : edgesWithFromVertex()) {
            newBeginOut[get(ToVertex, edge) + 1]++;
            fromVertex[edge] = from;
        }
        for (size_t i = 1; i < newBeginOut.size(); i++) {
            newBeginOut[i] += newBeginOut[i - 1];
        }
        std::vector<Edge> firstEdge = newBeginOut;
        Permutation edgePermutation(numEdges());
        for (Edge edge(0); edge < numEdges(); edge++) {
            const Vertex toVertex = get(ToVertex, edge);
            edgePermutation[edge] = firstEdge[toVertex]++;
        }
        if constexpr (HasEdgeAttribute(FromVertex)) {
            get(ToVertex).swap(get(FromVertex));
        } else {
            get(ToVertex).swap(fromVertex);
        }
        vertexAttributes.forEach([&](std::vector<Edge>& values) {
            edgePermutation.mapPermutation(values);
        });
        edgeAttributes.forEach([&](std::vector<Edge>& values) {
            edgePermutation.mapPermutation(values);
        });
        edgeAttributes.forEach([&](auto& values) {
            edgePermutation.permutate(values);
        });
        beginOut.swap(newBeginOut);
        checkVectorSize();
        AssertMsg(satisfiesInvariants(), "Invariants not satisfied!");
    }

    template<AttributeNameType ATTRIBUTE_NAME>
    inline void sortEdges(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName) noexcept {
        std::vector<Edge> edgeOrder = Vector::id<Edge>(numEdges());
        for (const Vertex vertex : vertices()) {
            if (beginOut[vertex] >= beginOut[vertex + 1]) continue;
            std::stable_sort(edgeOrder.begin() + beginOut[vertex], edgeOrder.begin() + beginOut[vertex + 1], [&](const Edge a, const Edge b){
                return get(attributeName, a) < get(attributeName, b);
            });
        }
        Permutation edgePermutation(Construct::Invert, Order(edgeOrder));
        vertexAttributes.forEach([&](std::vector<Edge>& values) {
            edgePermutation.mapPermutation(values);
        });
        edgeAttributes.forEach([&](std::vector<Edge>& values) {
            edgePermutation.mapPermutation(values);
        });
        edgeAttributes.forEach([&](auto& values) {
            edgePermutation.permutate(values);
        });
        AssertMsg(satisfiesInvariants(), "Invariants not satisfied!");
    }

    // Attributes:
    template<AttributeNameType ATTRIBUTE_NAME>
    inline std::vector<AttributeType<ATTRIBUTE_NAME>>& operator[](const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName) noexcept {
        return get(attributeName);
    }

    template<AttributeNameType ATTRIBUTE_NAME>
    inline const std::vector<AttributeType<ATTRIBUTE_NAME>>& operator[](const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName) const noexcept {
        return get(attributeName);
    }

    template<AttributeNameType ATTRIBUTE_NAME>
    inline std::vector<AttributeType<ATTRIBUTE_NAME>>& get(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName) noexcept {
        if constexpr (HasVertexAttribute(attributeName)) {
            return vertexAttributes.get(attributeName);
        } else {
            return edgeAttributes.get(attributeName);
        }
    }

    template<AttributeNameType ATTRIBUTE_NAME>
    inline const std::vector<AttributeType<ATTRIBUTE_NAME>>& get(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName) const noexcept {
        if constexpr (HasVertexAttribute(attributeName)) {
            return vertexAttributes.get(attributeName);
        } else {
            return edgeAttributes.get(attributeName);
        }
    }

    template<AttributeNameType ATTRIBUTE_NAME>
    inline AttributeReferenceType<ATTRIBUTE_NAME> get(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName, const Vertex vertex) noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return vertexAttributes.get(attributeName, vertex);
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline AttributeReferenceType<ATTRIBUTE_NAME> get(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName, const Edge edge) noexcept {
        AssertMsg(isEdge(edge), edge << " is not a valid edge!");
        return edgeAttributes.get(attributeName, edge);
    }

    template<AttributeNameType ATTRIBUTE_NAME>
    inline AttributeConstReferenceType<ATTRIBUTE_NAME> get(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName, const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return vertexAttributes.get(attributeName, vertex);
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline AttributeConstReferenceType<ATTRIBUTE_NAME> get(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName, const Edge edge) const noexcept {
        AssertMsg(isEdge(edge), edge << " is not a valid edge!");
        return edgeAttributes.get(attributeName, edge);
    }

    template<AttributeNameType ATTRIBUTE_NAME>
    inline void set(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName, const std::vector<VertexAttributeType<ATTRIBUTE_NAME>>& values) noexcept {
        if constexpr (HasVertexAttribute(attributeName)) {
            return vertexAttributes.set(attributeName, values);
        } else {
            return edgeAttributes.set(attributeName, values);
        }
    }

    template<AttributeNameType ATTRIBUTE_NAME>
    inline void set(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName, const Vertex vertex, const VertexAttributeType<ATTRIBUTE_NAME>& value) noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return vertexAttributes.set(attributeName, vertex, value);
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline void set(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName, const Edge edge, const EdgeAttributeType<ATTRIBUTE_NAME>& value) noexcept {
        AssertMsg(isEdge(edge), edge << " is not a valid edge!");
        return edgeAttributes.set(attributeName, edge, value);
    }

    inline VertexRecord vertexRecord(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return VertexRecord(vertexAttributes, vertex);
    }

    inline EdgeRecord edgeRecord(const Edge edge) const noexcept {
        AssertMsg(isEdge(edge), edge << " is not a valid edge!");
        return EdgeRecord(edgeAttributes, edge);
    }

    inline void setVertexAttributes(const Vertex vertex, const VertexRecord& record) noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        vertexAttributes.set(vertex, record);
    }

    inline void setEdgeAttributes(const Edge edge, const EdgeRecord& record) noexcept {
        AssertMsg(isEdge(edge), edge << " is not a valid edge!");
        edgeAttributes.set(edge, record);
    }

    inline EdgeHandle edge(const Edge edge) noexcept {
        AssertMsg(isEdge(edge), edge << " is not a valid edge!");
        return EdgeHandle(edgeAttributes, edge);
    }

    inline VertexHandle vertex(const Vertex vertex) noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return VertexHandle(vertexAttributes, vertex);
    }

    // IO:
    inline void writeBinary(const std::string& fileName, const std::string& separator = ".") const noexcept {
        IO::serialize(fileName + separator + "beginOut", beginOut);
        vertexAttributes.serialize(fileName, separator);
        edgeAttributes.serialize(fileName, separator);
        Graph::writeStatisticsFile(*this, fileName, separator);
    }

    inline void readBinary(const std::string& fileName, const std::string& separator = ".", const bool debug = true) noexcept {
        clear();
        if (debug) std::cout << "Loading static graph from " << fileName << std::endl;
        IO::deserialize(fileName + separator + "beginOut", beginOut);
        vertexAttributes.deserialize(fileName, separator, beginOut.size() - 1);
        edgeAttributes.deserialize(fileName, separator);
        AssertMsg(satisfiesInvariants(), "Invariants not satisfied!");
    }

    inline void printAnalysis(std::ostream& out = std::cout) const noexcept {
        AssertMsg(satisfiesInvariants(), "Invariants not satisfied!");
        size_t vertexCount = 0;
        size_t isolatedVertexCount = 0;
        size_t sourceCount = 0;
        size_t sinkCount = 0;
        size_t minInDegree = numEdges();
        size_t maxInDegree = 0;
        size_t minOutDegree = numEdges();
        size_t maxOutDegree = 0;
        size_t edgeCount = 0;
        size_t loopEdgeCount = 0;
        size_t multiEdgeCount = 0;
        size_t fromVertexErrorCount = 0;
        size_t edgeWithViaVertexCount = 0;
        size_t reverseEdgeEntryCount = 0;
        size_t reverseEdgeErrorCount = 0;
        long long graphDistance = 0;
        double minSpeed = intMax;
        double maxSpeed = -intMax;
        double avgSpeed = 0;
        long long hash = 0;
        Geometry::Rectangle boundingBox;
        if constexpr (HasVertexAttribute(Coordinates)) {
            if (!empty()) {
                boundingBox = Geometry::Rectangle(get(Coordinates, Vertex(0)));
            }
        }
        std::vector<size_t> inDegree(numVertices(), 0);
        IndexedSet<false, Vertex> neighbors(numVertices());
        for (const Vertex v : vertices()) {
            vertexCount++;
            if constexpr (HasVertexAttribute(Coordinates)) {
                boundingBox.extend(get(Coordinates, v));
                hash += get(Coordinates, v).latitude + get(Coordinates, v).longitude;
            }
            neighbors.clear();
            for (const Edge e : edgesFrom(v)) {
                const Vertex u = get(ToVertex, e);
                inDegree[u]++;
                hash += u;
                edgeCount++;
                if (u == v) loopEdgeCount++;
                if (neighbors.contains(u)) multiEdgeCount++;
                neighbors.insert(u);
                if constexpr (HasEdgeAttribute(FromVertex)) {
                    hash += get(FromVertex, e);
                    if (get(FromVertex, e) != v) fromVertexErrorCount++;
                }
                if constexpr (HasEdgeAttribute(ViaVertex)) {
                    if (isVertex(get(ViaVertex, e))) edgeWithViaVertexCount++;
                    hash += get(ViaVertex, e);
                }
                if constexpr (HasEdgeAttribute(TravelTime) && HasVertexAttribute(Coordinates)) {
                    const double dist = Geometry::geoDistanceInCM(get(Coordinates, v), get(Coordinates, u));
                    if (dist > 50000 || get(TravelTime, e) > 10) {
                        const double speed = (dist * 0.036) / get(TravelTime, e);
                        graphDistance += dist;
                        avgSpeed += (dist * speed);
                        if (minSpeed > speed) minSpeed = speed;
                        if (maxSpeed < speed) maxSpeed = speed;
                    }
                }
                if constexpr (HasEdgeAttribute(ReverseEdge)) {
                    if (isEdge(get(ReverseEdge, e))) {
                        reverseEdgeEntryCount++;
                        const Edge f = get(ReverseEdge, e);
                        if (get(ReverseEdge, f) != e) reverseEdgeErrorCount++;
                        if constexpr (HasEdgeAttribute(FromVertex)) {
                            if (get(FromVertex, f) != u) reverseEdgeErrorCount++;
                        }
                        if (get(ToVertex, f) != v) reverseEdgeErrorCount++;
                    }
                }
            }
        }
        for (const Vertex v : vertices()) {
            const size_t inDeg = inDegree[v];
            const size_t outDeg = outDegree(v);
            const size_t deg = inDeg + outDeg;
            hash += deg;
            if (deg == 0) isolatedVertexCount++;
            if (outDeg == 0 && inDeg > 0) sinkCount++;
            if (outDeg > 0 && inDeg == 0) sourceCount++;
            if (inDeg < minInDegree) minInDegree = inDeg;
            if (inDeg > maxInDegree) maxInDegree = inDeg;
            if (outDeg < minOutDegree) minOutDegree = outDeg;
            if (outDeg > maxOutDegree) maxOutDegree = outDeg;
        }
        const int tabSize = 18;
        out << std::right;
        out << "                  #Vertices : " << std::setw(tabSize) << String::prettyInt(vertexCount) << "  (" << String::percent(vertexCount / (double) numVertices()) << ")" << std::endl;
        out << "          #IsolatedVertices : " << std::setw(tabSize) << String::prettyInt(isolatedVertexCount) << "  (" << String::percent(isolatedVertexCount / (double) vertexCount) << ")" << std::endl;
        out << "                   #Sources : " << std::setw(tabSize) << String::prettyInt(sourceCount) << "  (" << String::percent(sourceCount / (double) vertexCount) << ")" << std::endl;
        out << "                     #Sinks : " << std::setw(tabSize) << String::prettyInt(sinkCount) << "  (" << String::percent(sinkCount / (double) vertexCount) << ")" << std::endl;
        out << "                minInDegree : " << std::setw(tabSize) << String::prettyInt(minInDegree) << std::endl;
        out << "                maxInDegree : " << std::setw(tabSize) << String::prettyInt(maxInDegree) << std::endl;
        out << "               minOutDegree : " << std::setw(tabSize) << String::prettyInt(minOutDegree) << std::endl;
        out << "               maxOutDegree : " << std::setw(tabSize) << String::prettyInt(maxOutDegree) << std::endl;
        out << "                     #Edges : " << std::setw(tabSize) << String::prettyInt(edgeCount) << "  (" << String::percent(edgeCount / (double) numEdges()) << ")" << std::endl;
        out << "                 #LoopEdges : " << std::setw(tabSize) << String::prettyInt(loopEdgeCount) << "  (" << String::percent(loopEdgeCount / (double) edgeCount) << ")" << std::endl;
        out << "                #MultiEdges : " << std::setw(tabSize) << String::prettyInt(multiEdgeCount) << "  (" << String::percent(multiEdgeCount / (double) edgeCount) << ")" << std::endl;
        edgeAttributes.forEach([&](const auto& values, const AttributeNameType attribute) {
            using ValueType = typename Meta::RemoveReference<decltype(values)>::value_type;
            const std::string attributeName = attributeToString(attribute);
            if constexpr (std::is_arithmetic<ValueType>::value && !Meta::Equals<ValueType, bool>()) {
                ValueType minWeight = std::numeric_limits<ValueType>::max();
                ValueType maxWeight = std::numeric_limits<ValueType>::lowest();
                size_t negativeWeightCount = 0;
                for (size_t e = 0; e < values.size(); e++) {
                    hash += values[e];
                    if (values[e] < minWeight) minWeight = values[e];
                    if (values[e] > maxWeight) maxWeight = values[e];
                    if (values[e] < 0) negativeWeightCount++;
                }
                out << std::setw(27 - attributeName.size()) << "min" << attributeName << " : " << std::setw(tabSize) << minWeight << std::endl;
                out << std::setw(27 - attributeName.size()) << "max" << attributeName << " : " << std::setw(tabSize) << maxWeight << std::endl;
                out << std::setw(27 - attributeName.size()) << "#Negative" << attributeName << " : " << std::setw(tabSize) << String::prettyInt(negativeWeightCount) << "  (" << String::percent(negativeWeightCount / (double) edgeCount) << ")" << std::endl;
            }
        });
        if constexpr (HasEdgeAttribute(ReverseEdge)) {
            out << "        #ReverseEdgeEntries : " << std::setw(tabSize) << String::prettyInt(reverseEdgeEntryCount) << "  (" << String::percent(reverseEdgeEntryCount / (double) edgeCount) << ")" << std::endl;
            out << "         #ReverseEdgeErrors : " << std::setw(tabSize) << String::prettyInt(reverseEdgeErrorCount) << std::endl;
        }
        if constexpr (HasEdgeAttribute(TravelTime) && HasVertexAttribute(Coordinates)) {
            out << "              graphDistance : " << std::setw(tabSize) << String::prettyDouble(graphDistance / 100000.0, 3) << " km" << std::endl;
            out << "                   minSpeed : " << std::setw(tabSize) << ((graphDistance == 0) ? "undefined" : String::prettyDouble(minSpeed)) << " km/h" << std::endl;
            out << "                   maxSpeed : " << std::setw(tabSize) << ((graphDistance == 0) ? "undefined" : String::prettyDouble(maxSpeed)) << " km/h" << std::endl;
            out << "                   avgSpeed : " << std::setw(tabSize) << ((graphDistance == 0) ? "undefined" : String::prettyDouble(avgSpeed / graphDistance)) << " km/h" << std::endl;
        }
        if constexpr (HasEdgeAttribute(FromVertex)) {
            out << "          #FromVertexErrors : " << std::setw(tabSize) << String::prettyInt(fromVertexErrorCount) << std::endl;
        }
        if constexpr (HasEdgeAttribute(ViaVertex)) {
            out << "        #EdgesWithViaVertex : " << std::setw(tabSize) << String::prettyInt(edgeWithViaVertexCount) << "  (" << String::percent(edgeWithViaVertexCount / (double) edgeCount) << ")" << std::endl;
        }
        if constexpr (HasVertexAttribute(Coordinates)) {
            out << "                boundingBox : " << std::setw(tabSize) << boundingBox << std::endl;
        }
        out << "                       hash : " << std::setw(tabSize) << String::prettyInt(hash) << std::endl;
    }

    inline void printAdjacencyList(std::ostream& out = std::cout) const noexcept {
        for (const Vertex vertex : vertices()) {
            out << std::setw(log10(numVertices()) + 1) << std::left << vertex << std::right << " -> ";
            Enumeration edgeList;
            for (const Edge edge : edgesFrom(vertex)) {
                edgeList << "(" << Graph::edgeToString(*this, edge) << ")" << sep;
            }
            out << edgeList << std::endl;
        }
    }

private:
    inline void changeVertexIds(const Order& vertexOrder, const Permutation& vertexPermutation) noexcept {
        Order edgeOrder;
        std::vector<Edge> newBeginOut;
        edgeOrder.reserve(numEdges());
        newBeginOut.reserve(beginOut.capacity());
        newBeginOut.emplace_back(0);
        for (const size_t i : vertexOrder) {
            const Vertex vertex(i);
            for (const Edge edge : edgesFrom(vertex)) {
                edgeOrder.emplace_back(edge);
            }
            newBeginOut.emplace_back(Edge(edgeOrder.size()));
        }

        const Permutation edgePermutation(Construct::Invert, edgeOrder);

        vertexAttributes.forEach([&](std::vector<Vertex>& values) {
            vertexPermutation.mapPermutation(values);
        });
        vertexAttributes.forEach([&](std::vector<Edge>& values) {
            edgePermutation.mapPermutation(values);
        });
        edgeAttributes.forEach([&](std::vector<Vertex>& values) {
            vertexPermutation.mapPermutation(values);
        });
        edgeAttributes.forEach([&](std::vector<Edge>& values) {
            edgePermutation.mapPermutation(values);
        });
        vertexAttributes.forEach([&](auto& values) {
            vertexOrder.order(values);
        });
        edgeAttributes.forEach([&](auto& values) {
            edgeOrder.order(values);
        });

        beginOut.swap(newBeginOut);
        checkVectorSize();
        AssertMsg(satisfiesInvariants(), "Invariants not satisfied!");
    }

public:
    inline void checkVectorSize() const noexcept {
        AssertMsg(beginOut.size() > 0, "Adjacency structure is empty!");
        AssertMsg(vertexAttributes.hasSize(beginOut.size() - 1), "Size of vertex attributes is inconsistent!");
        AssertMsg(edgeAttributes.hasSize(beginOut.back()), "Size of edge attributes is inconsistent!");
    }

    inline bool satisfiesInvariants() const noexcept {
        for (const Vertex v : vertices()) {
            AssertMsg(beginOut[v] <= beginOut[v + 1], "beginOut[" << v << "] (" << beginOut[v] << ") < " << "beginOut[" << v + 1 << "] (" << beginOut[v + 1] << ")");
            if (beginOut[v] > beginOut[v + 1]) return false;
            if constexpr (HasEdgeAttribute(FromVertex)) {
                for (const Edge e : edgesFrom(v)) {
                    AssertMsg(get(FromVertex, e) == v, "From vertex of " << e << " should be " << v << " but is " << get(FromVertex, e));
                    if (get(FromVertex, e) != v) return false;
                }
            }
            if constexpr (HasEdgeAttribute(ReverseEdge)) {
                for (const Edge e : edgesFrom(v)) {
                    if (!isEdge(get(ReverseEdge, e))) continue;
                    AssertMsg(get(ReverseEdge, get(ReverseEdge, e)) == e, "Reverse edge of " << e << " is inconsistent: " << get(ReverseEdge, e) << ", " << get(ReverseEdge, get(ReverseEdge, e)));
                    if (get(ReverseEdge, get(ReverseEdge, e)) != e) return false;
                }
            }
        }
        return true;
    }

private:
    std::vector<Edge> beginOut;
    VertexAttributes vertexAttributes;

    EdgeAttributes edgeAttributes;

};
