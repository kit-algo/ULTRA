#pragma once

#include <vector>
#include <string>
#include <iostream>

#include "../Graph/Graph.h"
#include "../Container/Set.h"

#include "../../Helpers/Assert.h"
#include "../../Helpers/Types.h"
#include "../../Helpers/IO/Serialization.h"
#include "../../Helpers/Ranges/Range.h"
#include "../../Helpers/Vector/Permutation.h"

class VertexPartition {

public:
    struct CutEdge {
        CutEdge(const Vertex from, const Vertex to) :
            from(from),
            to(to) {
        }
        Vertex from;
        Vertex to;
    };

public:
    VertexPartition(const std::vector<int>& cellOfVertex) :
        cellOfVertex(cellOfVertex) {
        computeCells();
    }
    VertexPartition(const size_t numberOfVertices = 0, const bool unassigned = true) :
        cellOfVertex(numberOfVertices, unassigned ? -1 : 0) {
        if (!unassigned) {
            cells.emplace_back();
        }
        std::vector<Vertex>& cell = unassigned ? unassignedVertices : cells[0];
        for (const Vertex vertex : range(Vertex(numberOfVertices))) {
            cell.emplace_back(vertex);
        }
    }
    VertexPartition(const std::string& fileName) {
        deserialize(fileName);
    }

    inline size_t numberOfVertices() const noexcept {
        return cellOfVertex.size();
    }

    inline size_t numberOfCells() const noexcept {
        return cells.size();
    }

    inline Range<Vertex> vertices() const noexcept {
        return Range<Vertex>(Vertex(0), Vertex(numberOfVertices()));
    }

    inline std::vector<Vertex>& getCell(const int cell) noexcept {
        return cells[cell];
    }

    inline const std::vector<Vertex>& getCell(const int cell) const noexcept {
        return cells[cell];
    }

    inline const std::vector<Vertex>& getUnassignedVertices() const noexcept {
        return unassignedVertices;
    }

    inline bool hasCell(const Vertex vertex) const noexcept {
        return cellOfVertex[vertex] >= 0 && size_t(cellOfVertex[vertex]) < numberOfCells();
    }

    inline const std::vector<Vertex>& getCellOfVertex(const Vertex vertex) const noexcept {
        AssertMsg(hasCell(vertex), "Vertex " << vertex << " is not assigned to any cell!");
        return cells[cellOfVertex[vertex]];
    }

    inline int getCellIdOfVertex(const Vertex vertex) const noexcept {
        return cellOfVertex[vertex];
    }

    inline void splitCell(const int cell, const std::vector<Vertex> oldCell, const std::vector<Vertex> newCell) noexcept {
        AssertMsg(getCell(cell).size() == oldCell.size() + newCell.size(), "The union of the sub cells does not match with the original cell (Size of cell " << cell << " is " << getCell(cell).size()  << ", size of union is " << (oldCell.size() + newCell.size()) << ")!");
        for (const Vertex vertex : newCell) {
            cellOfVertex[vertex] = cells.size();
        }
        cells.emplace_back(newCell);
        cells[cell] = oldCell;
    }

    inline void applyVertexPermutation(const Permutation& permutation) noexcept {
        permutation.permutate(cellOfVertex);
        for (std::vector<Vertex>& cell : cells) {
            permutation.mapPermutation(cell);
        }
        permutation.mapPermutation(unassignedVertices);
    }

    inline void applyVertexOrder(const Order& order) noexcept {
        applyVertexPermutation(Permutation(Construct::Invert, order));
    }

    template<typename GRAPH>
    inline std::vector<CutEdge> getCutEdges(const GRAPH& graph) const noexcept {
        std::vector<CutEdge> result;
        for (const Vertex from : graph.vertices()) {
            for (const Edge edge : graph.edgesFrom(from)) {
                const Vertex to = graph.get(ToVertex, edge);
                if (cellOfVertex[from] == cellOfVertex[to]) continue;
                result.emplace_back(from, to);
            }
        }
        return result;
    }

    template<typename GRAPH>
    inline SimpleDynamicGraph getCellGraph(const GRAPH& graph) const noexcept {
        SimpleDynamicGraph result;
        result.addVertices(numberOfCells());
        for (const CutEdge& cutEdge : getCutEdges(graph)) {
            result.findOrAddEdge(Vertex(cellOfVertex[cutEdge.from]), Vertex(cellOfVertex[cutEdge.to]));
            result.findOrAddEdge(Vertex(cellOfVertex[cutEdge.to]), Vertex(cellOfVertex[cutEdge.from]));
        }
        return result;
    }

    template<typename GRAPH>
    inline std::vector<Vertex> getBorderVertices(const GRAPH& graph) const noexcept {
        IndexedSet<false, Vertex> result(numberOfVertices());
        for (const CutEdge& cutEdge : getCutEdges(graph)) {
            result.insert(cutEdge.from);
            result.insert(cutEdge.to);
        }
        return result;
    }

    template<typename GRAPH>
    inline std::vector<std::vector<Vertex>> getBorderVerticesPerCell(const GRAPH& graph) const noexcept {
        std::vector<Vertex> borderVertices = getBorderVertices(graph);
        std::vector<std::vector<Vertex>> borderVerticesPerCell(numberOfCells());
        for (const Vertex vertex : borderVertices) {
            borderVerticesPerCell[cellOfVertex[vertex]].emplace_back(vertex);
        }
        return borderVerticesPerCell;
    }

    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, cellOfVertex);
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, cellOfVertex);
        computeCells();
    }

    inline void writeCSV(const std::string& fileName) const noexcept {
        std::ofstream file(fileName);
        Assert(file);
        Assert(file.is_open());
        file << "vertexId,CellId\n";
        for (const Vertex vertex : vertices()) {
            file << vertex << ",";
            if (cellOfVertex[vertex] < 0 || size_t(cellOfVertex[vertex]) >= numberOfCells()) {
                file << "-1";
            } else {
                file << cellOfVertex[vertex];
            }
            file << "\n";
        }
        file.close();
    }

    inline long long byteSize() const noexcept {
        long long result = Vector::byteSize(cellOfVertex);
        result += Vector::byteSize(cells);
        result += Vector::byteSize(unassignedVertices);
        return result;
    }

private:
    inline void computeCells() noexcept {
        unassignedVertices.clear();
        cells.clear();
        for (const Vertex vertex : vertices()) {
            if (cellOfVertex[vertex] < 0) {
                cellOfVertex[vertex] = -1;
                unassignedVertices.emplace_back(vertex);
            } else {
                if (size_t(cellOfVertex[vertex]) >= numberOfCells()) {
                    cells.resize(cellOfVertex[vertex] + 1);
                }
                cells[cellOfVertex[vertex]].emplace_back(vertex);
            }
        }
    }

private:
    std::vector<int> cellOfVertex;
    std::vector<std::vector<Vertex>> cells;
    std::vector<Vertex> unassignedVertices;

};
