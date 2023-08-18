#pragma once

#include <algorithm>
#include <vector>

#include "../../DataStructures/CH/UPGraphs.h"
#include "../CH/CH.h"
#include "../CH/Preprocessing/BidirectionalWitnessSearch.h"
#include "../../DataStructures/Container/ExternalKHeap.h"
#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/Partition/VertexPartition.h"
#include "../../Helpers/Helpers.h"
#include "../../Helpers/MultiThreading.h"
#include "../../Helpers/Types.h"

struct EccentricityData {
    using CHProfiler = CH::FullProfiler;
    using CHWitnessSearch = CH::BidirectionalWitnessSearch<CHCoreGraph, CHProfiler, 500>;
    using CHKeyFunction = CH::PartialKey<CHWitnessSearch>;
    using CHStopCriterion = CH::UncontractableVerticesCriterion;
    using CHBuilder = CH::Builder<CHProfiler, CHWitnessSearch, CHKeyFunction, CHStopCriterion, false, false>;

    EccentricityData(const TransferGraph& reverseGraph, const VertexPartition& partition) :
        numVertices(reverseGraph.numVertices()),
        partition(partition),
        boundaryVerticesPerCell(partition.getBorderVerticesPerCell(reverseGraph)),
        idInCell(reverseGraph.numVertices()) {
        for (size_t cellId = 0; cellId < boundaryVerticesPerCell.size(); cellId++) {
            std::vector<Vertex>& cell = boundaryVerticesPerCell[cellId];
            if (cell.empty()) {
                cell.emplace_back(partition.getCell(cellId)[0]);
            }
            for (size_t i = 0; i < cell.size(); i++) {
                const Vertex vertex = cell[i];
                idInCell[vertex] = i;
                boundaryVertices.emplace_back(vertex);
            }
        }
        contract(reverseGraph);
    }

    inline void contract(const TransferGraph& inputGraph) noexcept {
        TravelTimeGraph graph;
        Graph::copy(inputGraph, graph);
        std::vector<bool> contractable(graph.numVertices(), true);
        for (const Vertex vertex : boundaryVertices) {
            contractable[vertex] = false;
        }
        CHBuilder chBuilder(std::move(graph), graph[TravelTime], CHKeyFunction(contractable), CHStopCriterion());
        chBuilder.run();
        chBuilder.completeOrder();

        CH::Data& chData = chBuilder.getData();
        order = Order(chData.order);
        Vector::reverse(order);
        permutation = Permutation(Construct::Invert, order);
        partition.applyVertexPermutation(permutation);
        chData.applyVertexPermutation(permutation);
        permutation.mapPermutation(boundaryVertices);
        permutation.mapPermutation(boundaryVerticesPerCell);
        permutation.permutate(idInCell);

        coreGraph = chData.core;
        std::vector<CHConstructionGraph> chGraphs(partition.numberOfCells());
        for (CHConstructionGraph& chGraph : chGraphs) {
            chGraph.addVertices(inputGraph.numVertices());
        }
        for (const Edge edge : chData.backwardCH.edges()) {
            const Vertex from = chData.backwardCH.get(FromVertex, edge);
            const Vertex to = chData.backwardCH.get(ToVertex, edge);
            const size_t cell = partition.getCellIdOfVertex(from);
            chGraphs[cell].addEdge(from, to, chData.backwardCH.edgeRecord(edge));
        }
        for (size_t cell = 0; cell < chGraphs.size(); cell++) {
            sweepGraph.emplace_back();
            CHGraph graph;
            Graph::move(std::move(chGraphs[cell]), graph);
            sweepGraph.back().build(graph, partition.getCell(cell), false, false);
        }
    }

    size_t numVertices;
    VertexPartition partition;
    std::vector<Vertex> boundaryVertices;
    std::vector<std::vector<Vertex>> boundaryVerticesPerCell;
    std::vector<size_t> idInCell;
    CHCoreGraph coreGraph;
    std::vector<CH::SweepGraph> sweepGraph;
    Order order;
    Permutation permutation;
};

class ParallelEccentricityComputation {
private:
    struct DijkstraLabel : public ExternalKHeapElement {
        DijkstraLabel(int* const distance) :
            ExternalKHeapElement(),
            distance(distance) {
        }

        inline int getDistance() const noexcept {
            return *distance;
        }

        inline bool hasSmallerKey(const DijkstraLabel* other) const noexcept {
            return getDistance() < other->getDistance();
        }

        int* const distance;
    };

public:
    ParallelEccentricityComputation(const EccentricityData& data, std::vector<int>& eccentricity) :
        data(data),
        eccentricity(eccentricity),
        originalSource(noVertex),
        Q(data.numVertices),
        distance(data.numVertices, INFTY),
        timestamp(data.numVertices, 0),
        currentTimestamp(0) {
        for (size_t i = 0; i < distance.size(); i++) {
            dijkstraLabel.emplace_back(&distance[i]);
        }
    }

    inline void run(const Vertex source) noexcept {
        clear();
        originalSource = Vertex(data.order[source]);
        const size_t cell = data.partition.getCellIdOfVertex(source);
        dijkstra(cell, source);
        sweep(cell, source);
    }

private:
    inline void clear() noexcept {
        originalSource = noVertex;
        Q.clear();
        currentTimestamp++;
    }

    inline void dijkstra(const size_t cell, const Vertex source) noexcept {
        std::vector<bool> visited(data.boundaryVerticesPerCell[cell].size(), false);
        int unvisitedBoundaryVertices = visited.size();
        check(source);
        distance[source] = 0;
        Q.update(&dijkstraLabel[source]);
        while (!Q.empty()) {
            DijkstraLabel* uLabel = Q.extractFront();
            const Vertex u(uLabel - &(dijkstraLabel[0]));
            if ((size_t)data.partition.getCellIdOfVertex(u) == cell && !visited[data.idInCell[u]]) {
                visited[data.idInCell[u]] = true;
                unvisitedBoundaryVertices--;
                if (unvisitedBoundaryVertices == 0) return;
            }
            for (const Edge edge : data.coreGraph.edgesFrom(u)) {
                const Vertex v = data.coreGraph.get(ToVertex, edge);
                check(v);
                const int newDistance = distance[u] + data.coreGraph.get(Weight, edge);
                if (distance[v] > newDistance) {
                    distance[v] = newDistance;
                    Q.update(&dijkstraLabel[v]);
                }
            }
        }
    }

    inline void sweep(const size_t cell, const Vertex source) noexcept {
        for (const Vertex sweepV : data.sweepGraph[cell].graph.vertices()) {
            const Vertex v = data.sweepGraph[cell].internalToExternal(sweepV);
            check(v);
            for (const Edge edge : data.sweepGraph[cell].graph.edgesFrom(sweepV)) {
                const Vertex u = data.sweepGraph[cell].toVertex[edge];
                const int weight = data.sweepGraph[cell].graph.get(Weight, edge);
                const int newDistance = distance[u] + weight;
                distance[v] = branchlessConditional(newDistance < distance[v], newDistance, distance[v]);
            }
            AssertMsg(distance[v] < INFTY, "Did not reach " << v << " from " << source);
            eccentricity[originalSource] = std::max(eccentricity[originalSource], distance[v]);
        }
    }

    inline void check(const Vertex vertex) noexcept {
        if (timestamp[vertex] != currentTimestamp) {
            distance[vertex] = INFTY;
            timestamp[vertex] = currentTimestamp;
        }
    }

    const EccentricityData& data;
    std::vector<int>& eccentricity;

    Vertex originalSource;
    ExternalKHeap<2, DijkstraLabel> Q;
    std::vector<DijkstraLabel> dijkstraLabel;
    std::vector<int> distance;
    std::vector<int> timestamp;
    int currentTimestamp;
};


class EccentricityComputation {
public:
    EccentricityComputation(const TransferGraph& reverseGraph, const VertexPartition& partition) :
        data(reverseGraph, partition),
        eccentricity(reverseGraph.numVertices(), 0) {
    }

    inline void run(const ThreadPinning& threadPinning) noexcept {
        omp_set_num_threads(threadPinning.numberOfThreads);
        #pragma omp parallel
        {
            threadPinning.pinThread();
            ParallelEccentricityComputation computation(data, eccentricity);

            #pragma omp for schedule(dynamic)
            for (size_t i = 0; i < data.boundaryVertices.size(); i++) {
                computation.run(Vertex(i));
            }
        }
    }

    inline int getEccentricity(const Vertex vertex) const noexcept {
        return eccentricity[vertex];
    }

    inline const std::vector<int>& getEccentricities() const noexcept {
        return eccentricity;
    }

private:
    EccentricityData data;
    std::vector<int> eccentricity;
};
