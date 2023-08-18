#pragma once

#include <algorithm>
#include <vector>

#include "../../DataStructures/CH/UPGraphs.h"
#include "../../DataStructures/Container/ExternalKHeap.h"
#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/Isochrone/DistanceBounds.h"
#include "../../DataStructures/Partition/VertexPartition.h"
#include "../../Helpers/Helpers.h"
#include "../../Helpers/MultiThreading.h"
#include "../../Helpers/Types.h"

struct DistanceBoundData {
    DistanceBoundData(const VertexPartition& partition, const CHGraph& forwardGraph, const CHGraph& backwardGraph, const std::vector<int>& eccentricity) :
        partition(partition),
        upwardGraph(forwardGraph),
        boundaryVertices(partition.getBorderVerticesPerCell(forwardGraph)),
        maximumEccentricity(partition.numberOfCells(), 0)  {
        downwardGraph.build(backwardGraph, IndexedSet<false, Vertex>(Construct::Complete, upwardGraph.numVertices()), false, false);
        for (Vertex vertex : upwardGraph.vertices()) {
            const size_t cell = partition.getCellIdOfVertex(vertex);
            maximumEccentricity[cell] = std::max(maximumEccentricity[cell], eccentricity[vertex]);
        }
        for (size_t cell = 0; cell < partition.numberOfCells(); cell++) {
            if (boundaryVertices[cell].empty()) {
                boundaryVertices[cell].emplace_back(partition.getCell(cell)[0]);
            }
        }
    }

    const VertexPartition& partition;
    const CHGraph& upwardGraph;
    CH::SweepGraph downwardGraph;
    std::vector<std::vector<Vertex>> boundaryVertices;
    std::vector<int> maximumEccentricity;
};

class ParallelDistanceBoundComputation {
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
    ParallelDistanceBoundComputation(const DistanceBoundData& data, std::vector<std::vector<DistanceBounds>>& distanceBounds) :
        data(data),
        distanceBounds(distanceBounds),
        Q(data.upwardGraph.numVertices()),
        distance(data.upwardGraph.numVertices(), INFTY),
        timestamp(data.upwardGraph.numVertices(), 0),
        currentTimestamp(0) {
        for (const Vertex vertex : data.upwardGraph.vertices()) {
            dijkstraLabel.emplace_back(&distance[vertex]);
        }
    }

    inline void run(const size_t fromCell) noexcept {
        clear();
        upwardSearch(fromCell);
        downwardSweep(fromCell);
        for (size_t toCell = 0; toCell < data.boundaryVertices.size(); toCell++) {
            DistanceBounds& bounds = distanceBounds[fromCell][toCell];
            if (bounds.upperBound == INFTY) {
                AssertMsg(bounds.lowerBound == INFTY, "Cell " << toCell << " is partially reachable from " << fromCell);
                continue;
            }
            bounds.upperBound += data.maximumEccentricity[fromCell];
        }
    }

private:
    inline void clear() noexcept {
        Q.clear();
        currentTimestamp++;
    }

    inline void upwardSearch(const size_t fromCell) {
        for (const Vertex source : data.boundaryVertices[fromCell]) {
            check(source);
            distance[source] = 0;
            Q.update(&dijkstraLabel[source]);
        }

        while (!Q.empty()) {
            DijkstraLabel* uLabel = Q.extractFront();
            const Vertex u(uLabel - &(dijkstraLabel[0]));
            for (const Edge edge : data.upwardGraph.edgesFrom(u)) {
                const Vertex v = data.upwardGraph.get(ToVertex, edge);
                check(v);
                const int newDistance = distance[u] + data.upwardGraph.get(Weight, edge);
                if (distance[v] > newDistance) {
                    distance[v] = newDistance;
                    Q.update(&dijkstraLabel[v]);
                }
            }
        }
    }

    inline void downwardSweep(const size_t fromCell) {
        for (const Vertex sweepV : data.downwardGraph.graph.vertices()) {
            const Vertex v = data.downwardGraph.internalToExternal(sweepV);
            check(v);
            const size_t toCell = data.partition.getCellIdOfVertex(v);
            for (const Edge edge : data.downwardGraph.graph.edgesFrom(sweepV)) {
                const Vertex u = data.downwardGraph.toVertex[edge];
                const int weight = data.downwardGraph.graph.get(Weight, edge);
                const int newDistance = distance[u] + weight;
                distance[v] = branchlessConditional(newDistance < distance[v], newDistance, distance[v]);
            }
            distanceBounds[fromCell][toCell].update(distance[v]);
        }
    }

    inline void check(const Vertex vertex) noexcept {
        if (timestamp[vertex] != currentTimestamp) {
            distance[vertex] = INFTY;
            timestamp[vertex] = currentTimestamp;
        }
    }

private:
    const DistanceBoundData& data;
    std::vector<std::vector<DistanceBounds>>& distanceBounds;

    ExternalKHeap<2, DijkstraLabel> Q;
    std::vector<DijkstraLabel> dijkstraLabel;
    std::vector<int> distance;
    std::vector<int> timestamp;
    int currentTimestamp;
};

class DistanceBoundComputation {

public:
    DistanceBoundComputation(const VertexPartition& partition, const CHGraph& forwardGraph, const CHGraph& backwardGraph, const std::vector<int>& eccentricity) :
        data(partition, forwardGraph, backwardGraph, eccentricity),
        distanceBounds(partition.numberOfCells(), std::vector<DistanceBounds>(partition.numberOfCells())) {
    }

    DistanceBoundComputation(const VertexPartition& partition, const CH::CH& ch, const std::vector<int>& eccentricity) :
        DistanceBoundComputation(partition, ch.forward, ch.backward, eccentricity) {
    }

    inline void run(const ThreadPinning& threadPinning) noexcept {
        omp_set_num_threads(threadPinning.numberOfThreads);
        #pragma omp parallel
        {
            threadPinning.pinThread();
            ParallelDistanceBoundComputation computation(data, distanceBounds);

            #pragma omp for schedule(dynamic)
            for (size_t fromCell = 0; fromCell < data.boundaryVertices.size(); fromCell++) {
                computation.run(fromCell);
            }
        }
    }

    inline std::vector<std::vector<DistanceBounds>> getDistanceBounds() const noexcept {
        return distanceBounds;
    }
private:
    DistanceBoundData data;
    std::vector<std::vector<DistanceBounds>> distanceBounds;
};
