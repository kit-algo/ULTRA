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
#include "../../../Helpers/Console/ProgressBar.h"
#include "../../../Helpers/Ranges/Range.h"
#include "../../../Helpers/Ranges/SubRange.h"
#include "../../../Helpers/Ranges/DirectEdgeRange.h"

template<typename LIST_OF_VERTEX_ATTRIBUTES, typename LIST_OF_EDGE_ATTRIBUTES>
class EdgeListImplementation {
    static_assert(Meta::Equals<Vertex, Meta::FindAttributeType<FromVertex, LIST_OF_EDGE_ATTRIBUTES>>(), "An edge list requires an edge attribute named FromVertex of type Vertex!");
    static_assert(Meta::Equals<Vertex, Meta::FindAttributeType<ToVertex, LIST_OF_EDGE_ATTRIBUTES>>(), "An edge list requires an edge attribute named ToVertex of type Vertex!");

public:
    using ListOfVertexAttributes = LIST_OF_VERTEX_ATTRIBUTES;
    using ListOfEdgeAttributes = LIST_OF_EDGE_ATTRIBUTES;
    using Type = EdgeListImplementation<ListOfVertexAttributes, ListOfEdgeAttributes>;

    using VertexAttributes = Attributes<ListOfVertexAttributes>;
    using EdgeAttributes = Attributes<ListOfEdgeAttributes>;

    using VertexHandle = AttributeHandle<VertexAttributes, Vertex>;
    using EdgeHandle = AttributeHandle<EdgeAttributes, Edge>;

    using ListOfRecordVertexAttributes = LIST_OF_VERTEX_ATTRIBUTES;
    using ListOfRecordEdgeAttributes = Meta::RemoveAttribute<FromVertex, Meta::RemoveAttribute<ToVertex, Meta::RemoveAttribute<ReverseEdge, ListOfEdgeAttributes>>>;
    using VertexRecord = AttributeRecord<ListOfRecordVertexAttributes>;
    using EdgeRecord = AttributeRecord<ListOfRecordEdgeAttributes>;

    using ListOfAllAttributes = Meta::Concat<ListOfVertexAttributes, ListOfEdgeAttributes>;

    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    friend inline void Graph::move(EdgeListImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, StaticGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    friend inline void Graph::move(EdgeListImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, DynamicGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    friend inline void Graph::move(EdgeListImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, EdgeListImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    friend inline void Graph::move(DynamicGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, EdgeListImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
    template<typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM, typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO, typename... ATTRIBUTE_NAME_CHANGES>
    friend inline void Graph::move(StaticGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from, EdgeListImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to, const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);

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
    EdgeListImplementation() {}

    EdgeListImplementation(const std::string& fileName, const std::string& separator = ".", const bool debug = true) {
        readBinary(fileName, separator, debug);
    }

    EdgeListImplementation(const Type&) = default;
    EdgeListImplementation(Type&&) = default;

    Type& operator=(const Type&) = default;
    Type& operator=(Type&&) = default;

    // Access
    inline size_t numVertices() const noexcept {
        return vertexAttributes.size();
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

    inline Range<Vertex> vertices() const noexcept {
        return Range<Vertex>(Vertex(0), Vertex(numVertices()));
    }

    inline Range<Edge> edges() const noexcept {
        return Range<Edge>(Edge(0), Edge(numEdges()));
    }

    inline DirectEdgeRange<Type> edgesWithFromVertex() const noexcept {
        return DirectEdgeRange<Type>(*this);
    }

    inline Edge findEdge(const Vertex from, const Vertex to) const noexcept {
        if (!isVertex(from)) return noEdge;
        for (const Edge edge : edges()) {
            if ((get(FromVertex, edge) == from) && (get(ToVertex, edge) == to)) return edge;
        }
        return noEdge;
    }

    inline bool hasEdge(const Vertex from, const Vertex to) const noexcept {
        return isEdge(findEdge(from, to));
    }

    inline Edge findReverseEdge(const Edge edge) const {
        if (!isEdge(edge)) return noEdge;
        if constexpr (HasEdgeAttribute(ReverseEdge)) return get(ReverseEdge, edge);
        return findEdge(get(ToVertex, edge), get(FromVertex, edge));
    }

    inline bool hasReverseEdge(const Edge edge) const noexcept {
        return isEdge(findReverseEdge(edge));
    }

    inline bool empty() const {
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
        vertexAttributes.clear();
        edgeAttributes.clear();
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

    template<typename DELETE_VERTEX>
    inline void deleteVertices(const DELETE_VERTEX& deleteVertex) noexcept {
        for (Edge edge = Edge(numEdges() - 1); edge < numEdges(); edge--) {
            if (deleteVertex(get(ToVertex, edge)) || deleteVertex(get(FromVertex, edge))) {
                this->deleteEdge(edge);
            }
        }
        size_t numDeletions = 0;
        Permutation permutation(numVertices());
        for (const Vertex vertex : vertices()) {
            if (deleteVertex(vertex)) {
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
    inline void deleteVertices(const std::vector<T>& vertexMap, const T deleteValue) noexcept {
        deleteVertices([&](const Vertex vertex){return vertexMap[vertex] == deleteValue;});
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
        if constexpr (HasEdgeAttribute(ReverseEdge)) {
            if (isEdge(get(ReverseEdge, edge))) {
                set(ReverseEdge, get(ReverseEdge, edge), noEdge);
            }
        }
        if (edge != Edge(edgeAttributes.size() - 1)) {
            edgeAttributes.swap(edge, edgeAttributes.size() - 1);
        }
        if constexpr (HasEdgeAttribute(ReverseEdge)) {
            if (isEdge(get(ReverseEdge, edge))) {
                set(ReverseEdge, get(ReverseEdge, edge), edge);
            }
        }
        edgeAttributes.popBack();
    }

    inline void deleteEdge(const Vertex from, const Vertex to) noexcept {
        AssertMsg(isVertex(from), from << " is not a valid vertex!");
        AssertMsg(isVertex(to), to << " is not a valid vertex!");
        deleteEdge(findEdge(from, to));
    }

    template<typename DELETE_EDGE>
    inline void deleteEdges(const DELETE_EDGE& deleteEdge) noexcept {
        for (Edge edge(edgeAttributes.size() - 1); edge < edgeAttributes.size(); edge--) {
            if (deleteEdge(edge)) {
                this->deleteEdge(edge);
            }
        }
    }

    template<typename T>
    inline void deleteEdges(const std::vector<T>& edgeMap, const T& deleteValue) noexcept {
        deleteEdges([&](Edge edge){return edgeMap[edge] == deleteValue;});
    }

    inline void applyVertexPermutation(const Permutation& permutation) noexcept {
        AssertMsg(permutation.size() == numVertices(), "Permutation has the wrong size! (permutation.size(): " << permutation.size() << ", numVertices: " << numVertices() << ")");
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

    inline void applyVertexOrder(const Order& order) noexcept {
        AssertMsg(order.size() == numVertices(), "Order has the wrong size! (order.size(): " << order.size() << ", numVertices: " << numVertices() << ")");
        applyVertexPermutation(Permutation(Construct::Invert, order));
    }

    inline void applyEdgePermutation(const Permutation& permutation) noexcept {
        AssertMsg(permutation.size() == numEdges(), "Permutation has the wrong size! (permutation.size(): " << permutation.size() << ", numEdges: " << numEdges() << ")");
        vertexAttributes.forEach([&](std::vector<Edge>& values) {
            permutation.mapPermutation(values);
        });
        edgeAttributes.forEach([&](std::vector<Edge>& values) {
            permutation.mapPermutation(values);
        });
        edgeAttributes.forEach([&](auto& values) {
            permutation.permutate(values);
        });
    }

    inline void applyEdgeOrder(const Order& order) noexcept {
        AssertMsg(order.size() == numEdges(), "Order has the wrong size! (order.size(): " << order.size() << ", numEdges: " << numEdges() << ")");
        applyEdgePermutation(Permutation(Construct::Invert, order));
    }

    inline void revert() noexcept {
        get(ToVertex).swap(get(FromVertex));
        checkVectorSize();
        Assert(satisfiesInvariants());
    }

    template<AttributeNameType ATTRIBUTE_NAME>
    inline void sortEdges(const AttributeNameWrapper<ATTRIBUTE_NAME> attributeName) noexcept {
        std::vector<Edge> edgeOrder = Vector::id<Edge>(numEdges());
        std::stable_sort(edgeOrder.begin(), edgeOrder.end(), [&](const Edge a, const Edge b){
            return get(attributeName, a) < get(attributeName, b);
        });
        applyEdgeOrder(Order(edgeOrder));
    }

    // Utilities
    inline void deleteIsolatedVertices() noexcept {
        std::vector<bool> isIsolated(numVertices(), true);
        for (Edge edge : edges()) {
            isIsolated[get(FromVertex, edge)] = false;
            isIsolated[get(ToVertex, edge)] = false;
        }
        deleteVertices([&](Vertex vertex){return isIsolated[vertex];});
    }

    inline void removeLoopEdges() noexcept {
        deleteEdges([&](Edge edge){return get(FromVertex, edge) == get(ToVertex, edge);});
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
        if (debug) std::cout << "Loading edge list from " << fileName << std::endl;
        vertexAttributes.deserialize(fileName, separator);
        edgeAttributes.deserialize(fileName, separator);
    }

// TODO Utils
    template<bool VERBOSE = false>
    inline void fromDimacs(const std::string& fileBaseName, const double coordinateFactor = 1) noexcept {
        clear();
        const std::string grFilename = FileSystem::ensureExtension(fileBaseName, ".gr");
        if (VERBOSE) std::cout << "Reading dimacs graph from: " << grFilename << std::endl << std::flush;
        std::ifstream grIs(grFilename);
        AssertMsg(grIs.is_open(), "cannot open file: " << grFilename);
        size_t vertexCount = -1;
        size_t edgeCount = -1;
        ProgressBar bar(1);
        std::cout << "\r                     \r" << std::flush;
        while (!grIs.eof()) {
            std::string line;
            getline(grIs, line);
            line = String::trim(line);
            if (line.empty() || line[0] == 'c') continue;
            const std::vector<std::string> tokens = String::split(line, ' ');
            if (vertexCount == size_t(-1)) {
                if (tokens.size() != 4 || tokens[0] != "p" || tokens[1] != "sp") {
                    std::cout << "ERROR, invalid DIMACS .gr-file header: " << line << std::endl;
                    break;
                } else {
                    vertexCount = String::lexicalCast<size_t>(tokens[2]);
                    edgeCount = String::lexicalCast<size_t>(tokens[3]);
                    addVertices(vertexCount);
                    if (VERBOSE) bar.init(edgeCount);
                }
            } else {
                if (tokens.size() != 4 || tokens[0] != "a") {
                    std::cout << "WARNING, ignoring line in .gr-file: " << line << std::endl;
                    continue;
                } else {
                    const Vertex from(String::lexicalCast<size_t>(tokens[1]) - 1);
                    const Vertex to(String::lexicalCast<size_t>(tokens[2]) - 1);
                    const double weight = String::lexicalCast<double>(tokens[3]);
                    if (!isVertex(from)) {
                        std::cout << "ERROR, " << tokens[1] << " does not name a vertex!" << std::endl;
                        break;
                    } else if (!isVertex(to)) {
                        std::cout << "ERROR, " << tokens[2] << " does not name a vertex!" << std::endl;
                        break;
                    } else {
                        const Edge edge = addEdge(from, to);
                        //ToDo
                        //addEdge(from, to, AnyAttribute(weight));
                        edgeAttributes.forEach([&](auto& values) {
                            using ValueType = typename Meta::RemoveReference<decltype (values)>::value_type;
                            if constexpr (std::is_arithmetic<ValueType>::value && !Meta::Equals<ValueType, bool>()) {
                                values[edge] = weight;
                            }
                        });
                        if (VERBOSE) bar++;
                    }
                }
            }
        }
        grIs.close();
        if (VERBOSE) std::cout << std::endl;
        if (numEdges() != edgeCount) {
            std::cout << "WARNING, found " << numEdges() << " edges, but " << edgeCount << " edges were declared." << std::endl;
        }
        if constexpr (HasVertexAttribute(Coordinates)) {
            const std::string coFilename = FileSystem::ensureExtension(fileBaseName, ".co");
            if (VERBOSE) std::cout << "Reading dimacs coordinates from: " << coFilename << std::endl << std::flush;
            if (VERBOSE) bar.init(vertexCount);
            std::ifstream coIs(coFilename);
            AssertMsg(coIs.is_open(), "cannot open file: " << coFilename);
            bool header = false;
            while (!coIs.eof()) {
                std::string line;
                getline(coIs, line);
                line = String::trim(line);
                if (line.empty() || line[0] == 'c') continue;
                const std::vector<std::string> tokens = String::split(line, ' ');
                if (!header) {
                    if (tokens.size() != 5 || tokens[0] != "p" || tokens[1] != "aux" || tokens[2] != "sp" || tokens[3] != "co") {
                        std::cout << "ERROR, invalid DIMACS .co-file header: " << line << std::endl;
                        break;
                    } else {
                        const size_t n = String::lexicalCast<size_t>(tokens[4]);
                        if (n != numVertices()) {
                            std::cout << "ERROR, .co-file header declares " << n << " vertices, but graph contains " << numVertices() << " vertices." << std::endl;
                            break;
                        }
                        header = true;
                    }
                } else {
                    if (tokens.size() != 4 || tokens[0] != "v") {
                        std::cout << "WARNING, ignoring line in .co-file: " << line << std::endl;
                        continue;
                    } else {
                        const Vertex v(String::lexicalCast<size_t>(tokens[1]) - 1);
                        const double x = String::lexicalCast<double>(tokens[2]) * coordinateFactor;
                        const double y = String::lexicalCast<double>(tokens[3]) * coordinateFactor;
                        if (!isVertex(v)) {
                            std::cout << "ERROR, " << tokens[1] << " does not name a vertex!" << std::endl;
                            break;
                        } else {
                            get(Coordinates, v).x = x;
                            get(Coordinates, v).y = y;
                        }
                        if (VERBOSE) bar++;
                    }
                }
            }
            std::cout << std::endl;
        }
    }

    inline void printAnalysis(std::ostream& out = std::cout) const noexcept {
        size_t isolatedVertexCount = 0;
        size_t sourceCount = 0;
        size_t sinkCount = 0;
        size_t minInDegree = numEdges();
        size_t maxInDegree = 0;
        size_t minOutDegree = numEdges();
        size_t maxOutDegree = 0;
        size_t edgeCount = 0;
        size_t loopEdgeCount = 0;
        size_t edgeWithViaVertexCount = 0;
        size_t reverseEdgeEntryCount = 0;
        size_t reverseEdgeErrorCount = 0;
        long long graphDistance = 0;
        double minSpeed = intMax;
        double maxSpeed = -intMax;
        double avgSpeed = 0;
        long long hash = 0;
        Geometry::Rectangle boundingBox;
        std::vector<size_t> inDegree(numVertices(), 0);
        std::vector<size_t> outDegree(numVertices(), 0);
        if constexpr (HasVertexAttribute(Coordinates)) {
            if (!empty()) {
                boundingBox = Geometry::Rectangle(get(Coordinates, Vertex(0)));
            }
        }
        for (Edge edge : edges()) {
            const Vertex fromVertex = get(FromVertex, edge);
            const Vertex toVertex = get(ToVertex, edge);
            inDegree[toVertex]++;
            outDegree[fromVertex]++;
            edgeCount++;
            if (fromVertex == toVertex) loopEdgeCount++;
            hash += fromVertex + toVertex;
            if constexpr (HasEdgeAttribute(ViaVertex)) {
                if (isVertex(get(ViaVertex, edge))) edgeWithViaVertexCount++;
                hash += get(ViaVertex, edge);
            }
            if constexpr (HasEdgeAttribute(TravelTime) && HasVertexAttribute(Coordinates)) {
                const double dist = Geometry::geoDistanceInCM( get(Coordinates, fromVertex), get(Coordinates, toVertex));
                if (dist > 50000 || get(TravelTime, edge) > 10) {
                    const double speed = (dist * 0.036) / get(TravelTime, edge);
                    graphDistance += dist;
                    avgSpeed += (dist * speed);
                    if (minSpeed > speed) minSpeed = speed;
                    if (maxSpeed < speed) maxSpeed = speed;
                }
            }
            if constexpr (HasEdgeAttribute(ReverseEdge)) {
                if (isEdge(get(ReverseEdge, edge))) {
                    reverseEdgeEntryCount++;
                    const Edge f = get(ReverseEdge, edge);
                    if (get(ReverseEdge, f) != edge) reverseEdgeErrorCount++;
                    if (get(FromVertex, f) != toVertex) reverseEdgeErrorCount++;
                    if (get(ToVertex, f) != fromVertex) reverseEdgeErrorCount++;
                }
            }
        }
        for (Vertex v : vertices()) {
            if (inDegree[v] == 0 && outDegree[v] == 0) isolatedVertexCount++;
            if (inDegree[v] == 0 && outDegree[v] > 0) sourceCount++;
            if (inDegree[v] > 0 && outDegree[v] == 0) sinkCount++;
            if (inDegree[v] < minInDegree) minInDegree = inDegree[v];
            if (inDegree[v] > maxInDegree) maxInDegree = inDegree[v];
            if (outDegree[v] < minOutDegree) minOutDegree = outDegree[v];
            if (outDegree[v] > maxOutDegree) maxOutDegree = outDegree[v];
            if constexpr (HasVertexAttribute(Coordinates)) boundingBox.extend(get(Coordinates, v));
        }
        const int tabSize = 18;
        out << std::right;
        out << "                  #Vertices : " << std::setw(tabSize) << String::prettyInt(numVertices()) << std::endl;
        out << "          #IsolatedVertices : " << std::setw(tabSize) << String::prettyInt(isolatedVertexCount) << "  (" << String::percent(isolatedVertexCount / (double) numVertices()) << ")" << std::endl;
        out << "                   #Sources : " << std::setw(tabSize) << String::prettyInt(sourceCount) << "  (" << String::percent(sourceCount / (double) numVertices()) << ")" << std::endl;
        out << "                     #Sinks : " << std::setw(tabSize) << String::prettyInt(sinkCount) << "  (" << String::percent(sinkCount / (double) numVertices()) << ")" << std::endl;
        out << "                minInDegree : " << std::setw(tabSize) << String::prettyInt(minInDegree) << std::endl;
        out << "                maxInDegree : " << std::setw(tabSize) << String::prettyInt(maxInDegree) << std::endl;
        out << "               minOutDegree : " << std::setw(tabSize) << String::prettyInt(minOutDegree) << std::endl;
        out << "               maxOutDegree : " << std::setw(tabSize) << String::prettyInt(maxOutDegree) << std::endl;
        out << "                     #Edges : " << std::setw(tabSize) << String::prettyInt(edgeCount) << "  (" << String::percent(edgeCount / (double) numEdges()) << ")" << std::endl;
        out << "                 #LoopEdges : " << std::setw(tabSize) << String::prettyInt(loopEdgeCount) << "  (" << String::percent(loopEdgeCount / (double) edgeCount) << ")" << std::endl;
        edgeAttributes.forEach([&](const auto& values, const AttributeNameType attribute) {
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

private:
    inline Edge insertNewEdge(const Vertex from, const Vertex to) noexcept {
        AssertMsg(isVertex(from), from << " is not a valid vertex!");
        AssertMsg(isVertex(to), to << " is not a valid vertex!");
        const Edge newEdge(edgeAttributes.size());
        edgeAttributes.emplaceBack();
        set(ToVertex, newEdge, to);
        set(FromVertex, newEdge, from);
        return newEdge;
    }

public:
    inline void checkVectorSize() const noexcept {
        AssertMsg(vertexAttributes.hasSize(vertexAttributes.size()), "Size of vertex attributes is inconsistent!");
        AssertMsg(edgeAttributes.hasSize(edgeAttributes.size()), "Size of edge attributes is inconsistent!");
    }

    inline bool satisfiesInvariants() const noexcept {
        return true;
    }

private:
    VertexAttributes vertexAttributes;
    EdgeAttributes edgeAttributes;

};
