#pragma once

#include <vector>
#include <string>
#include <iostream>

#include "../Container/Set.h"

#include "../../Helpers/Assert.h"
#include "../../Helpers/Types.h"
#include "../../Helpers/IO/Serialization.h"
#include "../../Helpers/Vector/Vector.h"
#include "../../Helpers/Vector/Permutation.h"
#include "../../Helpers/Ranges/Range.h"
#include "../../Helpers/Ranges/SubRange.h"

#include "../../DataStructures/Container/Set.h"
#include "../../DataStructures/MaxFlowMinCut/FlowGraphs.h"

class NestedDissection {

public:
    NestedDissection(const size_t numberOfVertices = 0) :
        cellOfVertex(numberOfVertices, 0),
        beginOfCell(1, 0),
        endOfCell(1, numberOfVertices) {
        for (const Vertex vertex : range(Vertex(numberOfVertices))) {
            vertexData.emplace_back(vertex);
        }
    }
    NestedDissection(const std::string& fileName) {
        deserialize(fileName);
    }

    inline size_t numberOfVertices() const noexcept {
        return vertexData.size();
    }

    inline size_t numberOfCells() const noexcept {
        return beginOfCell.size();
    }

    inline bool isCell(const size_t cell) const noexcept {
        return cell < numberOfCells();
    }

    inline size_t numberOfLevels() const noexcept {
        return ceil(log2(numberOfCells() + 1));
    }

    inline size_t levelOfCell(const size_t cell) const noexcept {
        return ceil(log2(cell + 2)) - 1;
    }

    inline size_t parentOfCell(const size_t cell) const noexcept {
        return ((cell + 1) / 2) - 1;
    }

    inline size_t firstChildOfCell(const size_t cell) const noexcept {
        return (cell * 2) + 1;
    }

    inline size_t secondChildOfCell(const size_t cell) const noexcept {
        return (cell * 2) + 2;
    }

    inline size_t firstCellOfLevel(const size_t level) const noexcept {
        return (1 << level) - 1;
    }

    inline size_t lastCellOfLevel(const size_t level) const noexcept {
        return firstCellOfLevel(level + 1) - 1;
    }

    inline size_t getLevelSpecificCellId(const size_t cell, const size_t level) const noexcept {
        return cell - firstCellOfLevel(level);
    }

    inline size_t getLevelSpecificCellId(const size_t cell) const noexcept {
        return getLevelSpecificCellId(cell, levelOfCell(cell));
    }

    inline size_t getGlobalCellId(const size_t cell, const size_t level) const noexcept {
        return cell + firstCellOfLevel(level);
    }

    inline Range<Vertex> vertices() const noexcept {
        return Range<Vertex>(Vertex(0), Vertex(numberOfVertices()));
    }

    inline Range<size_t> cells() const noexcept {
        return Range<size_t>(0, numberOfCells());
    }

    inline Range<size_t> cellsOfLevel(const size_t level) const noexcept {
        return Range<size_t>(firstCellOfLevel(level), firstCellOfLevel(level + 1));
    }

    inline size_t sizeOfCell(const size_t cell) const noexcept {
        AssertMsg(isCell(cell), "Cannot access cell " << cell << " as only " << numberOfCells() << " cells exist!");
        return endOfCell[cell] - beginOfCell[cell];
    }

    inline SubRange<std::vector<Vertex>> getCell(const size_t cell) const noexcept {
        AssertMsg(isCell(cell), "Cannot access cell " << cell << " as only " << numberOfCells() << " cells exist!");
        return SubRange<std::vector<Vertex>>(vertexData, beginOfCell[cell], endOfCell[cell]);
    }

    inline std::vector<Vertex> getVerticesOfCell(const size_t cell) const noexcept {
        AssertMsg(isCell(cell), "Cannot access cell " << cell << " as only " << numberOfCells() << " cells exist!");
        std::vector<Vertex> vertices;
        vertices.reserve(sizeOfCell(cell));
        for (const Vertex vertex : getCell(cell)) {
            vertices.emplace_back(vertex);
        }
        return vertices;
    }

    inline SubRange<std::vector<Vertex>> getSeparatorOfCell(const size_t cell) const noexcept {
        AssertMsg(isCell(cell), "Cannot access cell " << cell << " as only " << numberOfCells() << " cells exist!");
        return SubRange<std::vector<Vertex>>(vertexData, endOfCell[firstChildOfCell(cell)], beginOfCell[secondChildOfCell(cell)]);
    }

    inline std::vector<Vertex> getSeparatorOfLevel(const size_t level) const noexcept {
        std::vector<Vertex> separator;
        for (const size_t l : range(level)) {
            for (const size_t cell : cellsOfLevel(l)) {
                for (const Vertex vertex : getSeparatorOfCell(cell)) {
                    separator.emplace_back(vertex);
                }
            }
        }
        return separator;
    }

    inline size_t getCellIdOfVertex(const Vertex vertex) const noexcept {
        return cellOfVertex[vertex];
    }

    inline size_t getCellIdOfVertex(const Vertex vertex, const size_t level) const noexcept {
        return getCellIdOfCell(cellOfVertex[vertex], level);
    }

    inline size_t getCellIdOfCell(const size_t cell, const size_t level) const noexcept {
        const int levelDifferency = static_cast<int>(levelOfCell(cell)) - static_cast<int>(level);
        if (levelDifferency <= 0) return cell;
        return ((cell + 1) >> levelDifferency) - 1;
    }

    inline SubRange<std::vector<Vertex>> getCellOfVertex(const Vertex vertex) const noexcept {
        return getCell(getCellIdOfVertex(vertex));
    }

    inline SubRange<std::vector<Vertex>> getCellOfVertex(const Vertex vertex, const size_t level) const noexcept {
        return getCell(getCellIdOfVertex(vertex, level));
    }

    inline std::vector<Vertex> getContractionOrder() const noexcept {
        std::vector<Vertex> order;
        for (const size_t cell : cellsOfLevel(numberOfLevels() - 1)) {
            for (const Vertex vertex : getCell(cell)) {
                order.emplace_back(vertex);
            }
        }
        for (size_t level = numberOfLevels() - 2; level < numberOfLevels(); level--) {
            for (const size_t cell : cellsOfLevel(level)) {
                for (const Vertex vertex : getSeparatorOfCell(cell)) {
                    order.emplace_back(vertex);
                }
            }
        }
        AssertMsg(order.size() == numberOfVertices(), "Order has " << order.size() << " entries, but should have " << numberOfVertices() << " entries!");
        return order;
    }

    template<typename GRAPH>
    inline void computeIncidentCells(const GRAPH& graph) noexcept {
        std::vector<std::vector<size_t>>(numberOfVertices()).swap(incidentCellsOfVertex);
        const size_t minCellId = firstCellOfLevel(numberOfLevels() - 1);
        for (const Vertex vertex : vertices()) {
            const size_t cellId = getCellIdOfVertex(vertex);
            if (cellId < minCellId) {
                if (!graph.isVertex(vertex)) continue;
                for (const Edge edge : graph.edgesFrom(vertex)) {
                    const size_t incidentCellId = getCellIdOfVertex(graph.get(ToVertex, edge));
                    if (incidentCellId < minCellId) continue;
                    Vector::insertSorted(incidentCellsOfVertex[vertex], incidentCellId);
                }
            } else {
                Vector::insertSorted(incidentCellsOfVertex[vertex], cellId);
                if (!graph.isVertex(vertex)) continue;
                for (const Edge edge : graph.edgesFrom(vertex)) {
                    Vector::insertSorted(incidentCellsOfVertex[graph.get(ToVertex, edge)], cellId);
                }
            }
        }
    }

    inline std::vector<std::vector<size_t>> getIncidentCells(const size_t level, const bool useLevelSpecificCellIds = false) const noexcept {
        std::vector<std::vector<size_t>> result(numberOfVertices());
        for (const Vertex vertex : vertices()) {
            for (const size_t cell : incidentCellsOfVertex[vertex]) {
                if (useLevelSpecificCellIds) {
                    Vector::insertSorted(result[vertex], getLevelSpecificCellId(getCellIdOfCell(cell, level), level));
                } else {
                    Vector::insertSorted(result[vertex], getCellIdOfCell(cell, level));
                }
            }
        }
        return result;
    }

    inline void divideCell(const size_t cell, const std::vector<Vertex>& separator, const std::vector<Vertex>& firstChild, const std::vector<Vertex>& secondChild) noexcept {
        AssertMsg(sizeOfCell(cell) == separator.size() + firstChild.size() + secondChild.size(), "The union of the sub cells does not match with the original cell (Size of cell " << cell << " is " << sizeOfCell(cell) << ", size of union is " << (separator.size() + firstChild.size() + secondChild.size()) << ")!");
        const size_t level = levelOfCell(cell);
        for (const Vertex vertex : firstChild) {
            AssertMsg(getCellIdOfVertex(vertex, level) == cell, "Vertex " << vertex << " is not part of cell " << cell << " (it is part of cell << " << getCellIdOfVertex(vertex, level) << ")!");
        }
        for (const Vertex vertex : separator) {
            AssertMsg(getCellIdOfVertex(vertex, level) == cell, "Vertex " << vertex << " is not part of cell " << cell << " (it is part of cell << " << getCellIdOfVertex(vertex, level) << ")!");
        }
        for (const Vertex vertex : secondChild) {
            AssertMsg(getCellIdOfVertex(vertex, level) == cell, "Vertex " << vertex << " is not part of cell " << cell << " (it is part of cell << " << getCellIdOfVertex(vertex, level) << ")!");
        }
        while (level >= numberOfLevels() - 1) {
            addLevel();
        }
        AssertMsg(level == numberOfLevels() - 2, "Cannot divide cell " << cell << " in level " << level << " because level " << (numberOfLevels() - 1) << " already exists!");
        size_t index = beginOfCell[cell];
        const size_t firstChildIndex = firstChildOfCell(cell);
        for (const Vertex vertex : firstChild) {
            vertexData[index] = vertex;
            cellOfVertex[vertex] = firstChildIndex;
            index++;
        }
        endOfCell[firstChildIndex] = index;
        for (const Vertex vertex : separator) {
            vertexData[index] = vertex;
            cellOfVertex[vertex] = cell;
            index++;
        }
        const size_t secondChildIndex = secondChildOfCell(cell);
        beginOfCell[secondChildIndex] = index;
        for (const Vertex vertex : secondChild) {
            vertexData[index] = vertex;
            cellOfVertex[vertex] = secondChildIndex;
            index++;
        }
    }

    inline void applyVertexPermutation(const Permutation& permutation) noexcept {
        permutation.mapPermutation(vertexData);
        permutation.permutate(cellOfVertex);
        permutation.permutate(incidentCellsOfVertex);
    }

    inline void applyVertexOrder(const Order& order) noexcept {
        applyVertexPermutation(Permutation(Construct::Invert, order));
    }

    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, vertexData, cellOfVertex, beginOfCell, endOfCell, incidentCellsOfVertex);
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, vertexData, cellOfVertex, beginOfCell, endOfCell, incidentCellsOfVertex);
    }

    template<typename GRAPH>
    inline void readHMetis(const GRAPH& graph, const std::string& fileName, const bool verbose = true) noexcept {
        if (verbose) std::cout << "Constructing flow Graph" << std::endl;
        DynamicFlowGraph flowGraph = Graph::generateFlowGraph(graph, true);
        flowGraph.deleteEdges([&](const Edge edge){
            return flowGraph.get(ToVertex, edge) <= flowGraph.get(FromVertex, edge);
        });
        flowGraph.packEdges();
        if (verbose) std::cout << "Reading hyper-graph partition from: " << fileName << std::endl;
        std::ifstream partIs(fileName);
        AssertMsg(partIs.is_open(), "cannot open file: " << fileName);
        std::vector<size_t> cellOfEdge;
        cellOfEdge.reserve(flowGraph.numEdges());
        size_t maxCell = 0;
        while (!partIs.eof()) {
            std::string line;
            getline(partIs, line);
            if (line == "") continue;
            const size_t cell = String::lexicalCast<size_t>(line);
            cellOfEdge.emplace_back(cell);
            if (maxCell < cell) maxCell = cell;
        }
        if (verbose) std::cout << "   number of cells: " << (maxCell + 1) << std::endl;
        if (verbose) std::cout << "Computing vertex assignments" << std::endl;
        std::vector<std::vector<size_t>> cellsOfVertex(graph.numVertices());
        for (const Vertex vertex : flowGraph.vertices()) {
            for (const Edge edge : flowGraph.edgesFrom(vertex)) {
                if (Vector::contains(cellsOfVertex[vertex], cellOfEdge[edge])) continue;
                cellsOfVertex[vertex].emplace_back(cellOfEdge[edge]);
            }
            for (const Edge edge : flowGraph.edgesTo(vertex)) {
                if (Vector::contains(cellsOfVertex[vertex], cellOfEdge[edge])) continue;
                cellsOfVertex[vertex].emplace_back(cellOfEdge[edge]);
            }
        }
        if (verbose) std::cout << "Computing nested dissection" << std::endl;
        cellOfVertex = std::vector<size_t>(graph.numVertices(), 0);
        beginOfCell = std::vector<size_t>(1, 0);
        endOfCell = std::vector<size_t>(1, graph.numVertices());
        const size_t maxLevel = ceil(log2(maxCell + 1)) - 1;
        for (const size_t cell : range(maxCell)) {
            if (verbose) std::cout << "   dissecting cell " << cell << std::endl;
            const size_t levelDiff = maxLevel - levelOfCell(cell);
            std::vector<Vertex> separator;
            std::vector<Vertex> firstSide;
            std::vector<Vertex> secondSide;
            for (const Vertex vertex : getCell(cell)) {
                bool first = false;
                bool second = false;
                for (const size_t c : cellsOfVertex[vertex]) {
                    const bool side = (((c >> levelDiff) % 2) == 0);
                    first = first | side;
                    second = second | (!side);
                }
                if (first == second) {
                    separator.emplace_back(vertex);
                } else if (first) {
                    firstSide.emplace_back(vertex);
                } else {
                    secondSide.emplace_back(vertex);
                }
            }
            divideCell(cell, separator, firstSide, secondSide);
        }
        computeIncidentCells(graph);
    }

    inline long long byteSize() const noexcept {
        long long result = Vector::byteSize(vertexData);
        result += Vector::byteSize(cellOfVertex);
        result += Vector::byteSize(beginOfCell);
        result += Vector::byteSize(endOfCell);
        result += Vector::byteSize(incidentCellsOfVertex);
        return result;
    }

private:
    inline void addLevel() noexcept {
        AssertMsg(firstCellOfLevel(numberOfLevels()) == beginOfCell.size(), "Lowest Level of binary tree is incomplete!");
        AssertMsg(firstCellOfLevel(numberOfLevels()) == endOfCell.size(), "Lowest Level of binary tree is incomplete!");
        for (const size_t cell : cellsOfLevel(numberOfLevels() - 1)) {
            beginOfCell.emplace_back(beginOfCell[cell]);
            endOfCell.emplace_back(beginOfCell[cell]);
            beginOfCell.emplace_back(endOfCell[cell]);
            endOfCell.emplace_back(endOfCell[cell]);
        }
        AssertMsg(firstCellOfLevel(numberOfLevels()) == beginOfCell.size(), "Lowest Level of binary tree is incomplete!");
        AssertMsg(firstCellOfLevel(numberOfLevels()) == endOfCell.size(), "Lowest Level of binary tree is incomplete!");
    }

private:
    std::vector<Vertex> vertexData;
    std::vector<size_t> cellOfVertex;
    std::vector<size_t> beginOfCell;
    std::vector<size_t> endOfCell;

    std::vector<std::vector<size_t>> incidentCellsOfVertex;

};
