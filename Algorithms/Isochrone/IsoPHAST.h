#pragma once

#include <algorithm>
#include <vector>

#include "../../DataStructures/CH/UPGraphs.h"
#include "../CH/CH.h"
#include "../../DataStructures/Container/ExternalKHeap.h"
#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/Isochrone/DistanceBounds.h"
#include "../../DataStructures/Partition/VertexPartition.h"
#include "../../Helpers/Helpers.h"
#include "../../Helpers/Types.h"

//TODO: Parallelize
class IsoPHAST {
private:
    struct DijkstraLabel : public ExternalKHeapElement {
        DijkstraLabel(int* const distance) :
            ExternalKHeapElement(),
            distance(distance) {
        }

        inline int getDistance() const noexcept {
            return *distance;
        }

        inline bool hasSmallerKey(const DijkstraLabel* other) const noexcept {return getDistance() < other->getDistance();}

        int* const distance;
    };

public:
    IsoPHAST(const VertexPartition& partition, const CHGraph& forwardGraph, const CHGraph& backwardGraph, const std::vector<std::vector<DistanceBounds>>& distanceBounds, const size_t numSharedVertices) :
        partition(partition),
        upwardGraph(forwardGraph),
        downwardGraph(partition.numberOfCells()),
        distanceBounds(distanceBounds),
        sourceVertex(noVertex),
        range(INFTY),
        isochrone(upwardGraph.numVertices()),
        Q(upwardGraph.numVertices()),
        distance(upwardGraph.numVertices(), INFTY),
        timestamp(upwardGraph.numVertices(), 0),
        currentTimestamp(0) {
        std::vector<Vertex> sharedVertices = Vector::id<Vertex>(numSharedVertices);
        sharedDownwardGraph.build(backwardGraph, sharedVertices, false, false, 0);
        for (size_t cell = 0; cell < partition.numberOfCells(); cell++) {
            downwardGraph[cell].build(backwardGraph, partition.getCell(cell), false, false, numSharedVertices);
        }
        for (const Vertex vertex : upwardGraph.vertices()) {
            dijkstraLabel.emplace_back(&distance[vertex]);
        }
    }

    IsoPHAST(const VertexPartition& partition, const CH::CH& ch, const std::vector<std::vector<DistanceBounds>>& distanceBounds, const size_t numSharedVertices) :
        IsoPHAST(partition, ch.forward, ch.backward, distanceBounds, numSharedVertices) {
    }

    inline void run(const Vertex source, const int isochroneRange) noexcept {
        clear();
        sourceVertex = source;
        range = isochroneRange;
        upwardSearch();
        downwardSweep(sharedDownwardGraph);
        for (const size_t cell : cellSelection()) {
            downwardSweep(downwardGraph[cell]);
        }
    }

    inline std::vector<Vertex> getIsochrone() const noexcept {
        return isochrone;
    }

private:
    inline void clear() noexcept {
        sourceVertex = noVertex;
        range = INFTY;
        isochrone.clear();
        Q.clear();
        currentTimestamp++;
    }

    inline void upwardSearch() noexcept {
        check(sourceVertex);
        distance[sourceVertex] = 0;
        Q.update(&dijkstraLabel[sourceVertex]);
        isochrone.insert(sourceVertex);

        while (!Q.empty()) {
            DijkstraLabel* uLabel = Q.extractFront();
            const Vertex u(uLabel - &(dijkstraLabel[0]));
            isochrone.insert(u);
            for (const Edge edge : upwardGraph.edgesFrom(u)) {
                const Vertex v = upwardGraph.get(ToVertex, edge);
                check(v);
                const int newDistance = distance[u] + upwardGraph.get(Weight, edge);
                if (distance[v] > newDistance && newDistance <= range) {
                    distance[v] = newDistance;
                    Q.update(&dijkstraLabel[v]);
                }
            }
        }
    }

    inline std::vector<size_t> cellSelection() noexcept {
        std::vector<size_t> selectedCells;
        const size_t sourceCell = partition.getCellIdOfVertex(sourceVertex);
        for (size_t cell = 0; cell < partition.numberOfCells(); cell++) {
            if (distanceBounds[sourceCell][cell].upperBound <= range) {
                for (const Vertex vertex : partition.getCell(cell)) {
                    isochrone.insert(vertex);
                }
            }
            else if (distanceBounds[sourceCell][cell].lowerBound <= range) {
                selectedCells.emplace_back(cell);
            }
        }
        return selectedCells;
    }

    inline void downwardSweep(const CH::CompactSweepGraph& graph) {
        for (const Vertex sweepV : graph.graph.vertices()) {
            const Vertex v = graph.internalToExternal(sweepV);
            check(v);
            for (const Edge edge : graph.graph.edgesFrom(sweepV)) {
                const Vertex u = graph.toVertex[edge];
                const int weight = graph.graph.get(Weight, edge);
                const int newDistance = distance[u] + weight;
                distance[v] = branchlessConditional(newDistance < distance[v], newDistance, distance[v]);
            }
            if (distance[v] <= range) isochrone.insert(v);
        }
    }

    inline void check(const Vertex vertex) noexcept {
        if (timestamp[vertex] != currentTimestamp) {
            distance[vertex] = INFTY;
            timestamp[vertex] = currentTimestamp;
        }
    }

    const VertexPartition& partition;
    const CHGraph& upwardGraph;
    CH::CompactSweepGraph sharedDownwardGraph;
    std::vector<CH::CompactSweepGraph> downwardGraph;
    const std::vector<std::vector<DistanceBounds>>& distanceBounds;

    Vertex sourceVertex;
    int range;

    IndexedSet<false, Vertex> isochrone;

    ExternalKHeap<2, DijkstraLabel> Q;
    std::vector<DijkstraLabel> dijkstraLabel;
    std::vector<int> distance;
    std::vector<int> timestamp;
    int currentTimestamp;
};
