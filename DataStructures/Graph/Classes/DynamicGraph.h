#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <type_traits>
#include <iomanip>
#include <set>
#include <algorithm>

#include "GraphInterface.h"

#include "../Utils/Utils.h"

#include "../../Geometry/Point.h"
#include "../../Geometry/Rectangle.h"

#include "../../../Helpers/FileSystem/FileSystem.h"
#include "../../../Helpers/IO/Serialization.h"
#include "../../../Helpers/Vector/Permutation.h"
#include "../../../Helpers/Ranges/Range.h"
#include "../../../Helpers/Ranges/SubRange.h"
#include "../../../Helpers/Ranges/IndexRange.h"
#include "../../../Helpers/Ranges/SparseRange.h"
#include "../../../Helpers/Ranges/IndirectEdgeRange.h"

template<typename LIST_OF_VERTEX_ATTRIBUTES, typename LIST_OF_EDGE_ATTRIBUTES>
class DynamicGraphImplementation {
    static_assert(Meta::Equals<Edge, Meta::FindAttributeType<BeginOut, LIST_OF_VERTEX_ATTRIBUTES>>(), "A dynamic graph requires a vertex attribute named BeginOut of type Edge!");
    static_assert(Meta::Equals<size_t, Meta::FindAttributeType<OutDegree, LIST_OF_VERTEX_ATTRIBUTES>>(), "A dynamic graph requires a vertex attribute named OutDegree of type size_t!");
    static_assert(Meta::Equals<std::vector<Edge>, Meta::FindAttributeType<IncomingEdges, LIST_OF_VERTEX_ATTRIBUTES>>(), "A dynamic graph requires a vertex attribute named IncomingEdges of type vector<Edge>!");
    static_assert(Meta::Equals<bool, Meta::FindAttributeType<Valid, LIST_OF_EDGE_ATTRIBUTES>>(), "A dynamic graph requires an edge attribute named Valid of type bool!");
    static_assert(Meta::Equals<size_t, Meta::FindAttributeType<IncomingEdgePointer, LIST_OF_EDGE_ATTRIBUTES>>(), "A dynamic graph requires an edge attribute named IncomingEdgePointer of type size_t!");
    static_assert(Meta::Equals<Vertex, Meta::FindAttributeType<FromVertex, LIST_OF_EDGE_ATTRIBUTES>>(), "A dynamic graph requires an edge attribute named FromVertex of type Vertex!");
    static_assert(Meta::Equals<Vertex, Meta::FindAttributeType<ToVertex, LIST_OF_EDGE_ATTRIBUTES>>(), "A dynamic graph requires an edge attribute named ToVertex of type Vertex!");

public:
    using ListOfVertexAttributes = LIST_OF_VERTEX_ATTRIBUTES;
    using ListOfEdgeAttributes = LIST_OF_EDGE_ATTRIBUTES;
    using Type = DynamicGraphImplementation<ListOfVertexAttributes, ListOfEdgeAttributes>;

    using VertexAttributes = Attributes<ListOfVertexAttributes>;
    using EdgeAttributes = Attributes<ListOfEdgeAttributes>;

    using VertexHandle = AttributeHandle<VertexAttributes, Vertex>;
    using EdgeHandle = AttributeHandle<EdgeAttributes, Edge>;

    using ListOfRecordVertexAttributes = Meta::RemoveAttribute<BeginOut, Meta::RemoveAttribute<OutDegree, Meta::RemoveAttribute<IncomingEdges, ListOfVertexAttributes>>>;
    using ListOfRecordEdgeAttributes = Meta::RemoveAttribute<Valid, Meta::RemoveAttribute<IncomingEdgePointer, Meta::RemoveAttribute<FromVertex, Meta::RemoveAttribute<ToVertex, Meta::RemoveAttribute<ReverseEdge, ListOfEdgeAttributes>>>>>;
    using VertexRecord = AttributeRecord<ListOfRecordVertexAttributes>;
    using EdgeRecord = AttributeRecord<ListOfRecordEdgeAttributes>;

    using ListOfAllAttributes = Meta::Concat<ListOfVertexAttributes, ListOfEdgeAttributes>;

    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    friend inline void Graph::move(DynamicGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, StaticGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    friend inline void Graph::move(DynamicGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, DynamicGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    friend inline void Graph::move(DynamicGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, EdgeListImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    friend inline void Graph::move(StaticGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, DynamicGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    friend inline void Graph::move(EdgeListImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, DynamicGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);

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
    DynamicGraphImplementation() :
        edgeCount(0) {
    }

    DynamicGraphImplementation(const std::string& fileName, const std::string& separator = ".", const bool debug = true) {
        readBinary(fileName, separator, debug);
    }

    DynamicGraphImplementation(const Type&) = default;
    DynamicGraphImplementation(Type&&) = default;

    Type& operator=(const Type&) = default;
    Type& operator=(Type&&) = default;

    // Access
    inline size_t numVertices() const noexcept {
        return vertexAttributes.size();
    }

    inline size_t numEdges() const noexcept {
        return edgeCount;
    }

    inline size_t edgeLimit() const noexcept {
        return edgeAttributes.size();
    }

    inline bool isVertex(const Vertex vertex) const noexcept {
        return vertex < numVertices();
    }

    inline bool isEdge(const Edge edge) const noexcept {
        return (edge < edgeAttributes.size()) && get(Valid)[edge];
    }

    inline size_t outDegree(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return get(OutDegree, vertex);
    }

    inline size_t inDegree(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return get(IncomingEdges, vertex).size();
    }

    inline size_t degree(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return neighbors(vertex).size();
    }

    inline Range<Vertex> vertices() const noexcept {
        return Range<Vertex>(Vertex(0), Vertex(numVertices()));
    }

    inline Range<Edge> edgesFrom(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return Range<Edge>(get(BeginOut, vertex), Edge(get(BeginOut, vertex) + get(OutDegree, vertex)));
    }

    inline Edge beginEdgeFrom(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex) || vertex == numVertices(), vertex << " is not a valid vertex!");
        return get(BeginOut, vertex);
    }

    inline Edge endEdgeFrom(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return Edge(get(BeginOut, vertex) + get(OutDegree, vertex));
    }

    inline const std::vector<Edge>& edgesTo(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return get(IncomingEdges, vertex);
    }

    inline SparseRange<Edge> edges() const noexcept {
        return SparseRange<Edge>(get(Valid));
    }

    inline IndirectEdgeRange<Type> edgesWithFromVertex() const noexcept {
        return IndirectEdgeRange<Type>(*this);
    }

    inline SubRange<std::vector<Vertex>> outgoingNeighbors(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return SubRange<std::vector<Vertex>>(get(ToVertex), get(BeginOut, vertex), get(BeginOut, vertex) + get(OutDegree, vertex));
    }

    inline IndexRange<std::vector<Vertex>, std::vector<Edge>> incomingNeighbors(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return IndexRange<std::vector<Vertex>, std::vector<Edge>>(get(FromVertex), get(IncomingEdges, vertex));
    }

    inline std::set<Vertex> neighbors(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        std::set<Vertex> neighbors;
        for (const Vertex neighbor : outgoingNeighbors(vertex)) {
            neighbors.insert(neighbor);
        }
        for (const Vertex neighbor : incomingNeighbors(vertex)) {
            neighbors.insert(neighbor);
        }
        return neighbors;
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

    inline Edge findReverseEdge(const Edge edge) const noexcept {
        if (!isEdge(edge)) return noEdge;
        if constexpr (HasEdgeAttribute(ReverseEdge)) return get(ReverseEdge, edge);
        return findEdge(get(ToVertex, edge), get(FromVertex, edge));
    }

    inline bool hasReverseEdge(const Edge edge) const noexcept {
        return isEdge(findReverseEdge(edge));
    }

    inline bool isIsolated(const Vertex vertex) const noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        return (get(OutDegree, vertex) == 0) && (get(IncomingEdges, vertex).empty());
    }

    inline bool empty() const noexcept {
        return numVertices() == 0;
    }

    inline long long byteSize() const noexcept {
        long long result = sizeof(size_t);
        result += vertexAttributes.byteSize();
        result += edgeAttributes.byteSize();
        return result;
    }

    inline long long memoryUsageInBytes() const noexcept {
        long long result = sizeof(size_t);
        result += vertexAttributes.memoryUsageInBytes();
        result += edgeAttributes.memoryUsageInBytes();
        return result;
    }

    // Manipulation:
    inline void clear() noexcept {
        edgeCount = 0;
        vertexAttributes.clear();
        edgeAttributes.clear();
    }

    inline void clearEdges() noexcept {
        edgeCount = 0;
        edgeAttributes.clear();
        Vector::fill(get(BeginOut), Edge(0));
        Vector::fill(get(OutDegree), size_t(0));
    }

    inline void reserve(const size_t numVertices, const size_t numEdges) noexcept {
        vertexAttributes.reserve(numVertices);
        edgeAttributes.reserve(numEdges);
    }

    inline Vertex addVertex() noexcept {
        addVertices();
        return Vertex(numVertices() - 1);
    }

    inline void addVertices(const size_t n = 1) noexcept {
        vertexAttributes.resize(vertexAttributes.size() + n);
    }

    inline Vertex addVertex(const VertexRecord& record) noexcept {
        addVertices(1, record);
        return Vertex(numVertices() - 1);
    }

    inline void addVertices(const size_t n, const VertexRecord& record) noexcept {
        vertexAttributes.resize(vertexAttributes.size() + n, record);
    }

    inline EdgeHandle addEdge(const Vertex from, const Vertex to) noexcept {
        AssertMsg(isVertex(from), from << " is not a valid vertex!");
        AssertMsg(isVertex(to), to << " is not a valid vertex!");
        const Edge newEdge = insertNewEdge(from, to);
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

    inline EdgeHandle addReverseEdge(const Edge edge) noexcept {
        AssertMsg(isEdge(edge), edge << " is not a valid edge!");
        if constexpr (HasEdgeAttribute(ReverseEdge)) {
            AssertMsg(!isEdge(get(ReverseEdge, edge)), edge << " has already a reverse edge!");
        }
        const Edge newEdge = insertNewEdge(get(ToVertex, edge), get(FromVertex, edge));
        edgeAttributes.set(newEdge, edgeRecord(edge));
        if constexpr (HasEdgeAttribute(ReverseEdge)) {
            AssertMsg(!isEdge(get(ReverseEdge, edge)), "Edge " << edge << " already has reverse edge " << get(ReverseEdge, edge));
            set(ReverseEdge, newEdge, edge);
            set(ReverseEdge, edge, newEdge);
        }
        return EdgeHandle(edgeAttributes, newEdge);
    }

    inline void redirectEdge(const Edge edge, const Vertex newTo) noexcept {
        AssertMsg(isEdge(edge), edge << " is not a valid edge!");
        AssertMsg(isVertex(newTo), newTo << " is not a valid vertex!");
        const Vertex oldTo = get(ToVertex, edge);
        if (get(IncomingEdgePointer, edge) != get(IncomingEdges, oldTo).size() - 1) {
            get(IncomingEdgePointer, get(IncomingEdges, oldTo).back()) = get(IncomingEdgePointer, edge);
            get(IncomingEdges, oldTo)[get(IncomingEdgePointer, edge)] = get(IncomingEdges, oldTo).back();
        }
        get(IncomingEdges, oldTo).pop_back();
        set(IncomingEdgePointer, edge, get(IncomingEdges, newTo).size());
        get(IncomingEdges, oldTo).push_back(edge);
        if constexpr (HasEdgeAttribute(ReverseEdge)) {
            const Edge oldReverse = get(ReverseEdge, edge);
            if (isEdge(oldReverse)) set(ReverseEdge, oldReverse, noEdge);
            const Edge newReverse = findEdge(newTo, get(FromVertex, edge));
            if (isEdge(newReverse) && !isEdge(get(ReverseEdge, newReverse))) {
                set(ReverseEdge, edge, newReverse);
                set(ReverseEdge, newReverse, edge);
            }
        }
        set(ToVertex, edge, newTo);
    }

    inline EdgeHandle findOrAddEdge(const Vertex from, const Vertex to) noexcept {
        const Edge edge = findEdge(from, to);
        if (isEdge(edge)) {
            return EdgeHandle(edgeAttributes, edge);
        } else {
            return addEdge(from, to);
        }
    }

    inline void deleteEdge(const Edge edge) noexcept {
        AssertMsg(isEdge(edge), edge << " is not a valid edge!");
        deleteEdge(get(FromVertex, edge), get(ToVertex, edge), edge);
    }

    inline void deleteEdge(const Vertex from, const Edge edge) noexcept {
        AssertMsg(isVertex(from), from << " is not a valid vertex!");
        AssertMsg(isEdge(edge), edge << " is not a valid edge!");
        AssertMsg(from == get(FromVertex, edge), from << " is not the from vertex of edge " << edge << " !");
        deleteEdge(from, get(ToVertex, edge), edge);
    }

    inline void deleteEdge(const Vertex from, const Vertex to) noexcept {
        AssertMsg(isVertex(from), from << " is not a valid vertex!");
        AssertMsg(isVertex(to), to << " is not a valid vertex!");
        deleteEdge(from, to, findEdge(from, to));
    }

    template<typename DELETE_EDGE>
    inline void deleteEdges(const DELETE_EDGE& deleteEdge) {
        for (Edge edge = Edge(edgeAttributes.size() - 1); edge < edgeAttributes.size(); edge--) {
            if (get(Valid)[edge] && deleteEdge(edge)) {
                this->deleteEdge(edge);
            }
        }
    }

    template<typename T>
    inline void deleteEdges(const std::vector<T>& edgeMap, const T& deleteValue) {
        deleteEdges([&](Edge edge){return edgeMap[edge] == deleteValue;});
    }

    template<typename DELETE_EDGE>
    inline void deleteEdgesFrom(const Vertex from, const DELETE_EDGE& deleteEdge) {
        for (size_t i = get(BeginOut, from) + get(OutDegree, from); i > get(BeginOut, from); i--) {
            const Edge edge = Edge(i - 1);
            if (deleteEdge(edge)) {
                this->deleteEdge(from, get(ToVertex, edge), edge);
            }
        }
    }

    inline void applyVertexPermutation(const Permutation& permutation) noexcept {
        changeVertexIds(permutation);
    }

    inline void applyVertexOrder(const Order& order) noexcept {
        changeVertexIds(Permutation(Construct::Invert, order));
    }

    inline void deleteAllOutgoingEdges(const Vertex vertex) noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        const Edge beginEdge = get(BeginOut, vertex);
        const size_t outDegree = get(OutDegree, vertex);
        for (size_t i = outDegree - 1; i < outDegree; i--) {
            const Edge edge = Edge(beginEdge + i);
            deleteEdge(vertex, get(ToVertex, edge), edge);
        }
        AssertMsg(get(OutDegree, vertex) == 0, "OutDegree of " << vertex << " was not reset correctly (" << get(OutDegree, vertex) << ")");
    }

    inline void deleteAllIncomingEdges(const Vertex vertex) noexcept {
        AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
        const size_t edgeCount = get(IncomingEdges, vertex).size();
        for (size_t i = edgeCount - 1; i < edgeCount; i--) {
            Edge edge = get(IncomingEdges, vertex)[i];
            deleteEdge(get(FromVertex, edge), vertex, edge);
        }
    }

    inline void isolateVertex(const Vertex vertex) noexcept {
        deleteAllOutgoingEdges(vertex);
        deleteAllIncomingEdges(vertex);
    }

    template<typename DELETE_VERTEX>
    inline void deleteVertices(const DELETE_VERTEX& deleteVertex) noexcept {
        size_t numDeletions = 0;
        Permutation permutation(numVertices());
        for (const Vertex vertex : vertices()) {
            if (deleteVertex(vertex)) {
                isolateVertex(vertex);
                numDeletions++;
                permutation[vertex] = numVertices() - numDeletions;
            } else {
                permutation[vertex] = vertex - numDeletions;
            }
        }
        applyVertexPermutation(permutation);
        vertexAttributes.resize(vertexAttributes.size() - numDeletions);
    }

    template<typename T>
    inline void deleteVertices(const std::vector<T>& vertexMap, const T& deleteValue) noexcept {
        deleteVertices([&](Vertex vertex){return vertexMap[vertex] == deleteValue;});
    }

    inline void deleteVertices(const std::vector<Vertex>& vertexList) noexcept {
        for (const Vertex vertex : vertexList) {
            AssertMsg(isVertex(vertex), vertex << " is not a valid vertex!");
            isolateVertex(vertex);
        }
        deleteIsolatedVertices();
    }

    inline void packEdges() noexcept {
        Assert(satisfiesInvariants());
        size_t i = 0;
        Permutation permutation(edgeAttributes.size());
        for (const Vertex vertex : vertices()) {
            for (const Edge edge : edgesFrom(vertex)) {
                permutation[edge] = i++;
            }
        }
        for (Edge edge = Edge(0); edge < edgeAttributes.size(); edge++) {
            if (!get(Valid)[edge]) {
                permutation[edge] = i++;
            }
        }
        vertexAttributes.forEach([&](std::vector<Edge>& values) {
            permutation.mapPermutation(values);
        });
        permutation.mapPermutation(get(IncomingEdges));
        edgeAttributes.forEach([&](std::vector<Edge>& values) {
            permutation.mapPermutation(values);
        });
        edgeAttributes.forEach([&](auto& values) {
            permutation.permutate(values);
        });
        edgeAttributes.resize(edgeCount);
        Assert(satisfiesInvariants());
    }

    inline void revert() noexcept {
        if (numEdges() == 0) return;
        packEdges();
        Edge currentEdge = Edge(0);
        std::vector<Edge> newBeginOut;
        std::vector<size_t> newOutDegree;
        newBeginOut.reserve(numEdges() * 1.2);
        newOutDegree.reserve(numEdges() * 1.2);
        for (const Vertex vertex : vertices()) {
            newBeginOut.emplace_back(currentEdge);
            newOutDegree.emplace_back(inDegree(vertex));
            currentEdge += newOutDegree.back();
        }
        const Permutation edgePermutation(Construct::Invert, Order(Construct::Sort, get(ToVertex)));
        get(ToVertex).swap(get(FromVertex));
        vertexAttributes.forEach([&](std::vector<Edge>& values, const AttributeNameType attribute) {
            if (attribute == BeginOut) return;
            edgePermutation.mapPermutation(values);
        });
        edgeAttributes.forEach([&](std::vector<Edge>& values) {
            edgePermutation.mapPermutation(values);
        });
        edgeAttributes.forEach([&](auto& values, const AttributeNameType attribute) {
            if (attribute == Valid) return;
            edgePermutation.permutate(values);
        });
        get(BeginOut).swap(newBeginOut);
        get(OutDegree).swap(newOutDegree);
        for (const Vertex vertex : vertices()) {
            get(IncomingEdges, vertex).clear();
        }
        for (const Edge edge : edges()) {
            set(IncomingEdgePointer, edge, get(IncomingEdges, get(ToVertex, edge)).size());
            get(IncomingEdges, get(ToVertex, edge)).emplace_back(edge);
        }
        checkVectorSize();
        Assert(satisfiesInvariants());
    }

    template<typename LESS>
    inline void sortEdges(const LESS& less) noexcept {
        std::vector<Edge> edgeOrder = Vector::id<Edge>(edgeAttributes.size());
        for (const Vertex vertex : vertices()) {
            if (get(OutDegree, vertex) == 0) continue;
            std::stable_sort(edgeOrder.begin() + get(BeginOut, vertex), edgeOrder.begin() + get(BeginOut, vertex) + get(OutDegree, vertex), less);
        }
        Permutation edgePermutation(Construct::Invert, Order(edgeOrder));
        edgePermutation.mapPermutation(get(IncomingEdges));
        vertexAttributes.forEach([&](std::vector<Edge>& values, const AttributeNameType attribute) {
            if (attribute == BeginOut) return;
            edgePermutation.mapPermutation(values);
        });
        edgeAttributes.forEach([&](std::vector<Edge>& values) {
            edgePermutation.mapPermutation(values);
        });
        edgeAttributes.forEach([&](auto& values, const AttributeNameType attribute) {
            if (attribute == Valid) return;
            edgePermutation.permutate(values);
        });
        Assert(satisfiesInvariants());
    }

    template<AttributeNameType ATTRIBUTE_NAME>
    inline void sortEdges(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName) noexcept {
        sortEdges([&](const Edge a, const Edge b){
            return get(attributeName, a) < get(attributeName, b);
        });
    }

    // Utilities
    inline void deleteIsolatedVertices() noexcept {
        deleteVertices([&](Vertex vertex){ return isIsolated(vertex);});
    }

    inline void makeBidirectional() noexcept {
        for (const Edge edge : edges()) {
            if (!hasReverseEdge(edge)) {
                addReverseEdge(edge);
            }
        }
    }

    inline void removeLoopEdges() noexcept {
        std::vector<bool> isLoopEdge(edgeAttributes.size(), false);
        for (const Vertex vertex : vertices()) {
            for (const Edge edge : edgesFrom(vertex)) {
                isLoopEdge[edge] = get(ToVertex, edge) == vertex;
            }
        }
        deleteEdges([&](Edge edge){return isLoopEdge[edge];});
    }

    template<typename EDGE_MERGE>
    inline void reduceMultiEdges(const EDGE_MERGE& merge) noexcept {
        std::vector<std::vector<EdgeRecord>> edgesByTarget(numVertices());
        std::vector<Edge> edgeByTarget(numVertices());
        std::vector<bool> deleteEdge(edgeAttributes.size(), false);
        for (const Vertex from : vertices()) {
            std::vector<Vertex> neighbors;
            for (const Edge edge : edgesFrom(from)) {
                const Vertex to = get(ToVertex, edge);
                if (edgesByTarget[to].empty()) {
                    neighbors.push_back(to);
                    edgeByTarget[to] = edge;
                } else {
                    deleteEdge[edge] = true;
                }
                edgesByTarget[to].push_back(edgeRecord(edge));
            }
            for (const Vertex to : neighbors) {
                if (edgesByTarget[to].size() > 1) {
                    setEdgeAttributes(edgeByTarget[to], merge(edgesByTarget[to]));
                }
                edgesByTarget[to].clear();
            }
        }
        deleteEdges(deleteEdge, true);
    }

    template<AttributeNameType ATTRIBUTE_NAME>
    inline void reduceMultiEdgesBy(const AttributeNameWrapper<ATTRIBUTE_NAME>) noexcept {
        static_assert(Meta::ContainsAttribute<ATTRIBUTE_NAME, ListOfRecordEdgeAttributes>(), "Graph does not have the given attribute!");
        reduceMultiEdges([](const std::vector<EdgeRecord>& edges){
            return Vector::min(edges, [](const EdgeRecord& a, const EdgeRecord& b){
                return a[AttributeNameWrapper<ATTRIBUTE_NAME>()] < b[AttributeNameWrapper<ATTRIBUTE_NAME>()];
            });
        });
    }

    template<typename EDGE_LINK>
    inline void contractDegreeTwoVertices(const EDGE_LINK& link) noexcept {
        for (const Vertex vertex : vertices()) {
            const std::set<Vertex> n = neighbors(vertex);
            if (n.size() != 2) continue;
            for (const Vertex from : n) {
                for (const Vertex to : n) {
                    if (from == to) continue;
                    const Edge a = findEdge(from, vertex);
                    const Edge b = findEdge(vertex, to);
                    if (isEdge(a) && isEdge(b)) {
                        addEdge(from, to, link(a, b));
                    }
                }
            }
            isolateVertex(vertex);
        }
        deleteIsolatedVertices();
    }

    inline void contractDegreeTwoVertices() noexcept {
        contractDegreeTwoVertices([&](const Edge a, const Edge b){
            EdgeRecord link(edgeRecord(a), edgeRecord(b));
            if constexpr (HasEdgeAttribute(ViaVertex)) link[ViaVertex] = get(ToVertex, a);
            return link;
        });
    }

    inline void isolateDegreeOneVertices() noexcept {
        for (const Vertex vertex : vertices()) {
            Vertex u = vertex;
            std::set<Vertex> n = neighbors(u);
            while (n.size() == 1) {
                isolateVertex(u);
                u = *(n.begin());
                n = neighbors(u);
            }
        }
    }

    inline void removeDegreeOneVertices() noexcept {
        isolateDegreeOneVertices();
        deleteIsolatedVertices();
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
        return vertexAttributes.set(attributeName, values);
    }
    template<AttributeNameType ATTRIBUTE_NAME>
    inline void set(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName, const std::vector<EdgeAttributeType<ATTRIBUTE_NAME>>& values) noexcept {
        return edgeAttributes.set(attributeName, values);
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
        vertexAttributes.serialize(fileName, separator);
        edgeAttributes.serialize(fileName, separator);
        Graph::writeStatisticsFile(*this, fileName, separator);
    }

    inline void readBinary(const std::string& fileName, const std::string& separator = ".", bool debug = true) noexcept {
        clear();
        if (debug) std::cout << "Loading dynamic graph from " << fileName << std::endl;
        vertexAttributes.deserialize(fileName, separator);
        edgeAttributes.deserialize(fileName, separator);
        for (const Vertex vertex : vertices()) edgeCount += outDegree(vertex);
        Assert(satisfiesInvariants());
    }

    inline void printAnalysis(std::ostream& out = std::cout) const noexcept {
        Assert(satisfiesInvariants());
        size_t vertexCount = 0;
        size_t isolatedVertexCount = 0;
        size_t degreeOneVertexCount = 0;
        size_t degreeTwoVertexCount = 0;
        size_t sourceCount = 0;
        size_t sinkCount = 0;
        size_t minInDegree = numEdges();
        size_t maxInDegree = 0;
        size_t minOutDegree = numEdges();
        size_t maxOutDegree = 0;
        size_t edgeCount = 0;
        size_t validEdgeCount = 0;
        size_t invalidEdgeCount = 0;
        size_t edgeWithReverseCount = 0;
        size_t edgeWithoutReverseCount = 0;
        size_t loopEdgeCount = 0;
        size_t multiEdgeCount = 0;
        size_t incomingEdgesErrorCount = 0;
        size_t incomingEdgePointerErrorCount = 0;
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
        for (const Vertex v : vertices()) {
            vertexCount++;
            const size_t outDeg = outDegree(v);
            const size_t inDeg = inDegree(v);
            hash += outDeg + inDeg;
            if (outDeg == 0 && inDeg == 0) isolatedVertexCount++;
            if (outDeg == 0 && inDeg > 0) sinkCount++;
            if (outDeg > 0 && inDeg == 0) sourceCount++;
            if (inDeg < minInDegree) minInDegree = inDeg;
            if (inDeg > maxInDegree) maxInDegree = inDeg;
            if (outDeg < minOutDegree) minOutDegree = outDeg;
            if (outDeg > maxOutDegree) maxOutDegree = outDeg;
            if constexpr (HasVertexAttribute(Coordinates)) {
                boundingBox.extend(get(Coordinates, v));
                hash += get(Coordinates, v).latitude + get(Coordinates, v).longitude;
            }
            std::set<Vertex> neighbors;
            for (const Edge e : edgesFrom(v)) {
                const Vertex u = get(ToVertex, e);
                hash += u + get(FromVertex, e);
                edgeCount++;
                if (hasReverseEdge(e)) {
                    edgeWithReverseCount++;
                } else {
                    edgeWithoutReverseCount++;
                }
                if (u == v) loopEdgeCount++;
                if (neighbors.count(u) > 0) multiEdgeCount++;
                neighbors.insert(u);
                if (get(IncomingEdgePointer, e) >= get(IncomingEdges, u).size()) {
                    incomingEdgePointerErrorCount++;
                } else {
                    if (get(IncomingEdges, u)[get(IncomingEdgePointer, e)] != e) incomingEdgesErrorCount++;
                }
                if (get(FromVertex, e) != v) fromVertexErrorCount++;
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
                        if (get(FromVertex, f) != u) reverseEdgeErrorCount++;
                        if (get(ToVertex, f) != v) reverseEdgeErrorCount++;
                    }
                }
            }
            size_t degr = degree(v);
            if (degr == 1) degreeOneVertexCount++;
            if (degr == 2) degreeTwoVertexCount++;
        }
        for (Edge e = Edge(0); e < edgeAttributes.size(); e++) {
            if (get(Valid)[e]) {
                validEdgeCount++;
            } else {
                invalidEdgeCount++;
            }
        }
        const int tabSize = 18;
        out << std::right;
        out << "                  #Vertices : " << std::setw(tabSize) << String::prettyInt(vertexCount) << "  (" << String::percent(vertexCount / (double) numVertices()) << ")" << std::endl;
        out << "          #IsolatedVertices : " << std::setw(tabSize) << String::prettyInt(isolatedVertexCount) << "  (" << String::percent(isolatedVertexCount / (double) vertexCount) << ")" << std::endl;
        out << "         #DegreeOneVertices : " << std::setw(tabSize) << String::prettyInt(degreeOneVertexCount) << "  (" << String::percent(degreeOneVertexCount / (double) vertexCount) << ")" << std::endl;
        out << "         #DegreeTwoVertices : " << std::setw(tabSize) << String::prettyInt(degreeTwoVertexCount) << "  (" << String::percent(degreeTwoVertexCount / (double) vertexCount) << ")" << std::endl;
        out << "                   #Sources : " << std::setw(tabSize) << String::prettyInt(sourceCount) << "  (" << String::percent(sourceCount / (double) vertexCount) << ")" << std::endl;
        out << "                     #Sinks : " << std::setw(tabSize) << String::prettyInt(sinkCount) << "  (" << String::percent(sinkCount / (double) vertexCount) << ")" << std::endl;
        out << "                minInDegree : " << std::setw(tabSize) << String::prettyInt(minInDegree) << std::endl;
        out << "                maxInDegree : " << std::setw(tabSize) << String::prettyInt(maxInDegree) << std::endl;
        out << "               minOutDegree : " << std::setw(tabSize) << String::prettyInt(minOutDegree) << std::endl;
        out << "               maxOutDegree : " << std::setw(tabSize) << String::prettyInt(maxOutDegree) << std::endl;
        out << "                     #Edges : " << std::setw(tabSize) << String::prettyInt(edgeCount) << "  (" << String::percent(edgeCount / (double) numEdges()) << ")" << std::endl;
        out << "                #ValidEdges : " << std::setw(tabSize) << String::prettyInt(validEdgeCount) << "  (" << String::percent(validEdgeCount / (double) numEdges()) << ")" << std::endl;
        out << "              #InvalidEdges : " << std::setw(tabSize) << String::prettyInt(invalidEdgeCount) << "  (" << String::percent(invalidEdgeCount / (double) numEdges()) << ")" << std::endl;
        out << "          #EdgesWithReverse : " << std::setw(tabSize) << String::prettyInt(edgeWithReverseCount) << "  (" << String::percent(edgeWithReverseCount / (double) edgeCount) << ")" << std::endl;
        out << "       #EdgesWithoutReverse : " << std::setw(tabSize) << String::prettyInt(edgeWithoutReverseCount) << "  (" << String::percent(edgeWithoutReverseCount / (double) edgeCount) << ")" << std::endl;
        out << "                 #LoopEdges : " << std::setw(tabSize) << String::prettyInt(loopEdgeCount) << "  (" << String::percent(loopEdgeCount / (double) edgeCount) << ")" << std::endl;
        out << "                #MultiEdges : " << std::setw(tabSize) << String::prettyInt(multiEdgeCount) << "  (" << String::percent(multiEdgeCount / (double) edgeCount) << ")" << std::endl;
        out << "       #IncomingEdgesErrors : " << std::setw(tabSize) << String::prettyInt(incomingEdgesErrorCount) << std::endl;
        out << " #IncomingEdgePointerErrors : " << std::setw(tabSize) << String::prettyInt(incomingEdgePointerErrorCount) << std::endl;
        out << "          #FromVertexErrors : " << std::setw(tabSize) << String::prettyInt(fromVertexErrorCount) << std::endl;
        edgeAttributes.forEach([&](const auto& values, const AttributeNameType attribute) {
            if (attribute == IncomingEdgePointer) return;
            using ValueType = typename Meta::RemoveReference<decltype (values)>::value_type;
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
    inline void moveEdge(const Edge oldEdge, const Edge newEdge) noexcept {
        AssertMsg(isEdge(oldEdge), oldEdge << " is not a valid edge!");
        AssertMsg(!isEdge(newEdge), "Edge " << newEdge << " already exists!");
        AssertMsg(newEdge <= Edge(edgeAttributes.size()), "New edge " << newEdge << " is out of bounds! (#edges: " << edgeAttributes.size() << ")");
        if (newEdge == Edge(edgeAttributes.size())) {
            edgeAttributes.emplaceBack();
        }
        edgeAttributes.copy(oldEdge, newEdge);
        get(Valid)[oldEdge] = false;
        get(IncomingEdges, get(ToVertex, newEdge))[get(IncomingEdgePointer, newEdge)] = newEdge;
        if constexpr (HasEdgeAttribute(ReverseEdge)) {
            if (get(ReverseEdge, newEdge) == oldEdge) {
                set(ReverseEdge, newEdge, newEdge);
            } else if (isEdge(get(ReverseEdge, newEdge))) {
                AssertMsg(get(ReverseEdge, get(ReverseEdge, newEdge)) == oldEdge, "Reverse edge of reverse edge of " << newEdge << " should be " << oldEdge << " but is " << get(ReverseEdge, get(ReverseEdge, newEdge)));
                set(ReverseEdge, get(ReverseEdge, newEdge), newEdge);
            }
        }
        vertexAttributes.forEach([&](std::vector<Edge>& values, const AttributeNameType attribute) {
            if (attribute == BeginOut) return;
            for (size_t i = 0; i < values.size(); i++) {
                if (values[i] == oldEdge) values[i] = newEdge;
            }
        });
        edgeAttributes.forEach([&](std::vector<Edge>& values, const AttributeNameType attribute) {
            if (attribute == ReverseEdge) return;
            for (size_t i = 0; i < values.size(); i++) {
                if (values[i] == oldEdge) values[i] = newEdge;
            }
        });
    }

    inline Edge insertNewEdge(const Vertex from, const Vertex to) noexcept {
        AssertMsg(isVertex(from), from << " is not a valid vertex!");
        AssertMsg(isVertex(to), to << " is not a valid vertex!");
        if (get(OutDegree, from) == 0) {
            set(BeginOut, from, Edge(edgeAttributes.size()));
        }
        Edge newEdge = Edge(get(BeginOut, from) + get(OutDegree, from));
        if (isEdge(newEdge)) {
            newEdge = Edge(get(BeginOut, from) - 1);
            if (newEdge > edgeAttributes.size() || get(Valid)[newEdge]) {
                const Edge newBeginOut(edgeAttributes.size());
                for (const Edge edge : edgesFrom(from)) {
                    moveEdge(edge, Edge(edgeAttributes.size()));
                }
                set(BeginOut, from, newBeginOut);
                newEdge = Edge(edgeAttributes.size());
            }
        }
        AssertMsg(newEdge <= Edge(edgeAttributes.size()), "New edge " << newEdge << " is out of bounds (#edges: " << edgeAttributes.size() << ")");
        get(IncomingEdges, to).emplace_back(newEdge);
        if (newEdge == get(BeginOut, from) - 1) {
            get(BeginOut, from)--;
        }
        get(OutDegree, from)++;
        edgeCount++;
        if (newEdge == Edge(edgeAttributes.size())) {
            edgeAttributes.emplaceBack();
        } else {
            edgeAttributes.setToDefault(newEdge);
        }
        get(Valid)[newEdge] = true;
        set(IncomingEdgePointer, newEdge, get(IncomingEdges, to).size() - 1);
        set(ToVertex, newEdge, to);
        set(FromVertex, newEdge, from);
        return newEdge;
    }

    inline void deleteEdge(const Vertex from, const Vertex to, const Edge edge) noexcept {
        AssertMsg(isVertex(from), from << " is not a valid vertex!");
        AssertMsg(isVertex(to), to << " is not a valid vertex!");
        AssertMsg(isEdge(edge), edge << " is not a valid edge!");
        AssertMsg(get(ToVertex, edge) == to, "To vertex of " << edge << " should be " << to << " but is " << get(ToVertex, edge));
        AssertMsg(get(FromVertex, edge) == from, "From vertex of " << edge << " should be " << from << " but is " << get(FromVertex, edge));
        if (get(IncomingEdgePointer, edge) != get(IncomingEdges, to).size() - 1) {
            AssertMsg(get(IncomingEdgePointer, get(IncomingEdges, to).back()) == get(IncomingEdges, to).size() - 1, "Vertex " << to << ": Incoming edge pointer of incoming edge #" << get(IncomingEdges, to).size() - 1 << " points to " << get(IncomingEdgePointer, get(IncomingEdges, to).back()));
            get(IncomingEdgePointer, get(IncomingEdges, to).back()) = get(IncomingEdgePointer, edge);
            get(IncomingEdges, to)[get(IncomingEdgePointer, edge)] = get(IncomingEdges, to).back();
        }
        get(IncomingEdges, to).pop_back();
        for (size_t i = 0; i < get(IncomingEdges, to).size(); i++) {
            AssertMsg(get(IncomingEdgePointer, get(IncomingEdges, to)[i]) == i, "Vertex " << to << ": Incoming edge pointer of incoming edge #" << i << " points to " << get(IncomingEdgePointer, get(IncomingEdges, to)[i]));
        }
        if constexpr (HasEdgeAttribute(ReverseEdge)) {
            if (isEdge(get(ReverseEdge, edge))) {
                AssertMsg(get(ReverseEdge, get(ReverseEdge, edge)) == edge, "Reverse edge of reverse edge of " << edge << "  is " << get(ReverseEdge, get(ReverseEdge, edge)));
                set(ReverseEdge, get(ReverseEdge, edge), noEdge);
            }
        }
        set(Valid, edge, false);
        get(OutDegree, from)--;
        if (get(BeginOut, from) + get(OutDegree, from) != edge) {
            moveEdge(Edge(get(BeginOut, from) + get(OutDegree, from)), edge);
        }
        edgeCount--;
    }

    inline void changeVertexIds(const Permutation& permutation) noexcept {
        vertexAttributes.forEach([&](std::vector<Vertex>& values) {
            permutation.mapPermutation(values);
        });
        edgeAttributes.forEach([&](std::vector<Vertex>& values) {
            permutation.mapPermutation(values);
        });
        vertexAttributes.forEach([&](auto& values) {
            permutation.permutate(values);
        });
    }

public:
    inline void checkVectorSize() const noexcept {
        AssertMsg(vertexAttributes.hasSize(vertexAttributes.size()), "Size of vertex attributes is inconsistent!");
        AssertMsg(edgeAttributes.hasSize(edgeAttributes.size()), "Size of edge attributes is inconsistent!");
    }

    inline bool satisfiesInvariants() const noexcept {
        std::vector<bool> hasEdgeFlag(edgeAttributes.size(), false);
        for (const Vertex v : vertices()) {
            for (Edge e = get(BeginOut, v); e < get(BeginOut, v) + get(OutDegree, v); e++) {
                AssertMsg(!hasEdgeFlag[e], "Edge " << e << " was already scanned!");
                if (hasEdgeFlag[e]) return false;
                hasEdgeFlag[e] = true;
                AssertMsg(get(FromVertex, e) == v, "From vertex of edge " << e << " should be " << v << " but is " << get(FromVertex, e));
                if (get(FromVertex, e) != v) return false;
                AssertMsg(get(IncomingEdgePointer, e) < get(IncomingEdges, get(ToVertex, e)).size(), "Incoming edge pointer of edge " << e << " is out of bounds! (size: " << get(IncomingEdges, get(ToVertex, e)).size() << ")");
                if (get(IncomingEdgePointer, e) >= get(IncomingEdges, get(ToVertex, e)).size()) return false;
                AssertMsg(get(IncomingEdges, get(ToVertex, e))[get(IncomingEdgePointer, e)] == e, "IncomingEdges entry for " << e << " points to " << get(IncomingEdges, get(ToVertex, e))[get(IncomingEdgePointer, e)]);
                if (get(IncomingEdges, get(ToVertex, e))[get(IncomingEdgePointer, e)] != e) return false;
            }
            for (size_t i = 0; i < get(IncomingEdges, v).size(); i++) {
                const Edge e = get(IncomingEdges, v)[i];
                AssertMsg(isEdge(e), "Incoming edge #" << i << " of vertex " << v << " (" << e << ") is not a valid edge!");
                if (!isEdge(e)) return false;
                AssertMsg(get(ToVertex, e) == v, "Incoming edge " << e << " of vertex " << v << " does not have " << v << " as its to vertex!");
                if (get(ToVertex, e) != v) return false;
                AssertMsg(get(IncomingEdgePointer, e) == i, "Incoming edge pointer of " << e << " should be " << i << " but is " << get(IncomingEdgePointer, e));
                if (get(IncomingEdgePointer, e) != i) return false;
            }
        }
        for (Edge e = Edge(0); e < edgeAttributes.size(); e++) {
            if (get(Valid)[e]) {
                AssertMsg(hasEdgeFlag[e], "Valid edge " << e << " was not scanned!");
                if (!hasEdgeFlag[e]) return false;
            } else {
                AssertMsg(!hasEdgeFlag[e], "Scanned edge " << e << " is invalid!");
                if (hasEdgeFlag[e]) return false;
            }
        }
        if constexpr (HasEdgeAttribute(ReverseEdge)) {
            for (const Edge edge : edges()) {
                AssertMsg(isEdge(get(ReverseEdge, edge)) || get(ReverseEdge, edge) == noEdge, "Reverse edge " << get(ReverseEdge, edge) << " of edge " << edge << " is invalid but not noEdge!");
                if (isEdge(get(ReverseEdge, edge))) {
                    AssertMsg(get(ReverseEdge, get(ReverseEdge, edge)) == edge, "Reverse edge of reverse edge of " << edge << "  is " << get(ReverseEdge, get(ReverseEdge, edge)));
                    if (get(ReverseEdge, get(ReverseEdge, edge)) != edge) return false;
                    AssertMsg(get(FromVertex, get(ReverseEdge, edge)) == get(ToVertex, edge), "From vertex of reverse edge of " << edge << " (" << get(FromVertex, get(ReverseEdge, edge)) << ") is not to vertex of " << edge << " (" << get(ToVertex, edge) << ")");
                    if (get(FromVertex, get(ReverseEdge, edge)) != get(ToVertex, edge)) return false;
                    AssertMsg(get(ToVertex, get(ReverseEdge, edge)) == get(FromVertex, edge), "To vertex of reverse edge of " << edge << " (" << get(ToVertex, get(ReverseEdge, edge)) << ") is not from vertex of " << edge << " (" << get(FromVertex, edge) << ")");
                    if (get(ToVertex, get(ReverseEdge, edge)) != get(FromVertex, edge)) return false;
                }
            }
        }
        return true;
    }

private:
    size_t edgeCount;

    VertexAttributes vertexAttributes;

    EdgeAttributes edgeAttributes;

};
