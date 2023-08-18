#pragma once

#include "../MaxFlowMinCut/Dinic.h"
#include "../MaxFlowMinCut/VertexSeparator.h"
#include "../StronglyConnectedComponents.h"

#include "../../DataStructures/Geometry/Point.h"
#include "../../DataStructures/Partition/VertexPartition.h"
#include "../../DataStructures/Partition/NestedDissection.h"
#include "../../DataStructures/MaxFlowMinCut/FlowGraphs.h"
#include "../../DataStructures/Container/Set.h"

#include "../../Helpers/Timer.h"
#include "../../Helpers/MultiThreading.h"

namespace InertialFlow {

class VertexValue {

public:
    VertexValue(const std::vector<Geometry::Point>& coordinates, const Geometry::Point direction) :
        coordinates(coordinates),
        direction(direction) {
    }

public:
    inline double operator()(const Vertex v) const noexcept {
        return coordinates[v] * direction;
    }

private:
    const std::vector<Geometry::Point>& coordinates;
    const Geometry::Point direction;

};

inline std::vector<Geometry::Point> defaultDirections() noexcept {
    return std::vector<Geometry::Point>{Geometry::Point(Construct::XY, 0, 1), Geometry::Point(Construct::XY, 1, 0), Geometry::Point(Construct::XY, 1, 1), Geometry::Point(Construct::XY, 1, -1)};
}

inline std::vector<Geometry::Point> doubleDirections() noexcept {
    return std::vector<Geometry::Point>{Geometry::Point(Construct::XY, 0, 1), Geometry::Point(Construct::XY, 1, 0), Geometry::Point(Construct::XY, 1, 1), Geometry::Point(Construct::XY, 1, -1), Geometry::Point(Construct::XY, 2, 1), Geometry::Point(Construct::XY, 1, 2), Geometry::Point(Construct::XY, 2, -1), Geometry::Point(Construct::XY, 1, -2)};
}

inline void divideVerticesByQuantity(std::vector<Vertex>& vertices, std::vector<Vertex>& sources, std::vector<Vertex>& targets, const VertexValue&, const double fixedVertices) noexcept {
    for (size_t i = 0; i < vertices.size() * fixedVertices; i++) {
        sources.emplace_back(vertices[i]);
    }
    for (size_t i = vertices.size() * (1 - fixedVertices); i < vertices.size(); i++) {
        targets.emplace_back(vertices[i]);
    }
}

inline void divideVerticesByValue(std::vector<Vertex>& vertices, std::vector<Vertex>& sources, std::vector<Vertex>& targets, const VertexValue& vertexValue, const double fixedVertices) noexcept {
    const double vertexValueDifference = vertexValue(vertices.back()) - vertexValue(vertices.front());
    AssertMsg(vertexValueDifference > 0, "Vertex value difference should be positive but is " << vertexValueDifference);
    const double sourceLimit = vertexValue(vertices.front()) + (vertexValueDifference * fixedVertices);
    for (size_t i = 0; vertexValue(vertices[i]) < sourceLimit; i++) {
        sources.emplace_back(vertices[i]);
    }
    const double targetLimit = vertexValue(vertices.back()) - (vertexValueDifference * fixedVertices);
    for (size_t i = vertices.size() - 1; vertexValue(vertices[i]) > targetLimit; i--) {
        targets.emplace_back(vertices[i]);
    }
}

inline void divideVerticesByQuantityAndValue(std::vector<Vertex>& vertices, std::vector<Vertex>& sources, std::vector<Vertex>& targets, const VertexValue& vertexValue, const double fixedVertices) noexcept {
    const double vertexValueDifference = vertexValue(vertices.back()) - vertexValue(vertices.front());
    AssertMsg(vertexValueDifference > 0, "Vertex value difference should be positive but is " << vertexValueDifference);
    const size_t sizeLimit = vertices.size() * fixedVertices;
    const double sourceLimit = std::min(vertexValue(vertices.front()) + (vertexValueDifference * fixedVertices), vertexValue(vertices[vertices.size() - sizeLimit - 1]));
    for (size_t i = 0; ((vertexValue(vertices[i]) < sourceLimit) || (sources.size() <= sizeLimit)); i++) {
        sources.emplace_back(vertices[i]);
    }
    const double targetLimit = std::max(vertexValue(vertices.back()) - (vertexValueDifference * fixedVertices), vertexValue(vertices[sizeLimit + 1]));
    for (size_t i = vertices.size() - 1; ((vertexValue(vertices[i]) > targetLimit) || (targets.size() <= sizeLimit)); i--) {
        targets.emplace_back(vertices[i]);
    }
}

enum DivisionCriteria {
    QUANTITY,
    VALUE,
    QUANTITYANDVALUE
};

template<DivisionCriteria CRITERIA>
inline void divideVertices(std::vector<Vertex>& vertices, std::vector<Vertex>& sources, std::vector<Vertex>& targets, const std::vector<Geometry::Point>& coordinates, const Geometry::Point& direction, const double fixedVertices) noexcept {
    sources.clear();
    targets.clear();
    VertexValue vertexValue(coordinates, direction);
    sort(vertices, [&](const Vertex a, const Vertex b){
        return vertexValue(a) < vertexValue(b);
    });

    if constexpr (CRITERIA == QUANTITY) {
        divideVerticesByQuantity(vertices, sources, targets, vertexValue, fixedVertices);
    } else if constexpr (CRITERIA == VALUE) {
        divideVerticesByValue(vertices, sources, targets, vertexValue, fixedVertices);
    } else {
        divideVerticesByQuantityAndValue(vertices, sources, targets, vertexValue, fixedVertices);
    }
}

}

template<bool DEBUG, InertialFlow::DivisionCriteria CRITERIA = InertialFlow::QUANTITYANDVALUE>
class InertialFlowOnEdges {

public:
    static constexpr bool Debug = DEBUG;
    static constexpr InertialFlow::DivisionCriteria Criteria = CRITERIA;
    using Type = InertialFlowOnEdges<Debug, Criteria>;

private:
    struct ThreadData {
        ThreadData(DynamicFlowGraph&& graph) :
            dinic(std::move(graph)),
            bestCutSize(0) {
        }
        Dinic dinic;
        size_t bestCutSize;
        Dinic::Cut bestCut;
    };

public:
    template<typename GRAPH>
    InertialFlowOnEdges(const GRAPH& graph, const std::vector<Geometry::Point>& coordinates, const std::vector<Geometry::Point>& directions = InertialFlow::defaultDirections()) :
        coordinates(coordinates),
        directions(directions),
        threadData(1, Graph::generateFlowGraph(graph, true)),
        usedVertices(coordinates.size()) {
    }

    template<typename GRAPH>
    InertialFlowOnEdges(const GRAPH& graph, const std::vector<Geometry::Point>& coordinates, const std::vector<int>& capacities, const std::vector<Geometry::Point>& directions = InertialFlow::defaultDirections()) :
        coordinates(coordinates),
        directions(directions),
        threadData(1, Graph::generateFlowGraph(graph, capacities, true)),
        usedVertices(coordinates.size()) {
    }

public:
    inline VertexPartition runOnConnectedComponents(const size_t maxCellSize, const double fixedVertices = 0.2) noexcept {
        if constexpr (Debug) totalTimer.restart();
        cutEdges.clear();
        VertexPartition vertexPartition(coordinates.size(), false);
        splitConnectedComponents(vertexPartition);
        size_t cell = 0;
        while (cell < vertexPartition.numberOfCells()) {
            if (vertexPartition.getCell(cell).size() <= maxCellSize) {
                cell++;
            } else {
                dissect(vertexPartition, cell, fixedVertices);
            }
        }
        if constexpr (Debug) {
            std::cout << "Total cut size: " << String::prettyInt(cutEdges.size()) << std::endl;
            std::cout << "Total time: " << String::msToString(totalTimer.elapsedMilliseconds()) << std::endl;
        }
        return vertexPartition;
    }

    inline VertexPartition runOnConnectedComponents(const ThreadPinning& threadPinning, const size_t maxCellSize, const double fixedVertices = 0.2) noexcept {
        if constexpr (Debug) totalTimer.restart();
        cutEdges.clear();
        VertexPartition vertexPartition(coordinates.size(), false);
        splitConnectedComponents(vertexPartition);
        size_t cell = 0;
        while (cell < vertexPartition.numberOfCells()) {
            if (vertexPartition.getCell(cell).size() <= maxCellSize) {
                cell++;
            } else {
                dissect(threadPinning, vertexPartition, cell, fixedVertices);
            }
        }
        if constexpr (Debug) {
            std::cout << "Total cut size: " << String::prettyInt(cutEdges.size()) << std::endl;
            std::cout << "Total time: " << String::msToString(totalTimer.elapsedMilliseconds()) << std::endl;
        }
        return vertexPartition;
    }

    inline VertexPartition run(const size_t numberOfCells, const double fixedVertices = 0.2) noexcept {
        if constexpr (Debug) totalTimer.restart();
        cutEdges.clear();
        VertexPartition vertexPartition(coordinates.size(), false);
        while (vertexPartition.numberOfCells() < numberOfCells) {
            const size_t currentNumberOfCells = vertexPartition.numberOfCells();
            for (size_t cell = currentNumberOfCells - 1; cell < currentNumberOfCells && vertexPartition.numberOfCells() < numberOfCells; cell--) {
                dissect(vertexPartition, cell, fixedVertices);
            }
        }
        if constexpr (Debug) {
            std::cout << "Total cut size: " << String::prettyInt(cutEdges.size()) << std::endl;
            std::cout << "Total time: " << String::msToString(totalTimer.elapsedMilliseconds()) << std::endl;
        }
        return vertexPartition;
    }

    inline VertexPartition run(const ThreadPinning& threadPinning, const size_t numberOfCells, const double fixedVertices = 0.2) noexcept {
        if constexpr (Debug) totalTimer.restart();
        cutEdges.clear();
        VertexPartition vertexPartition(coordinates.size(), false);
        while (vertexPartition.numberOfCells() < numberOfCells) {
            const size_t currentNumberOfCells = vertexPartition.numberOfCells();
            for (size_t cell = currentNumberOfCells - 1; cell < currentNumberOfCells && vertexPartition.numberOfCells() < numberOfCells; cell--) {
                dissect(threadPinning, vertexPartition, cell, fixedVertices);
            }
        }
        if constexpr (Debug) {
            std::cout << "Total cut size: " << String::prettyInt(cutEdges.size()) << std::endl;
            std::cout << "Total time: " << String::msToString(totalTimer.elapsedMilliseconds()) << std::endl;
        }
        return vertexPartition;
    }

    inline const std::vector<Dinic::CutEdge>& getCutEdges() const noexcept {
        return cutEdges;
    }

private:
    inline void splitConnectedComponents(VertexPartition& vertexPartition) noexcept {
        StronglyConnectedComponents<StaticFlowGraph> scc(threadData[0].dinic.getFlowGraph());
        scc.run();
        for (size_t component = 1; component < scc.numComponents(); component++) {
            std::vector<Vertex> oldCell;
            std::vector<Vertex> newCell;
            for (const Vertex vertex : vertexPartition.getCell(0)) {
                if ((size_t)scc.getComponent(vertex) == component) {
                    newCell.push_back(vertex);
                } else {
                    oldCell.push_back(vertex);
                }
            }
            vertexPartition.splitCell(0, oldCell, newCell);
        }
    }

    inline void dissect(VertexPartition& vertexPartition, const size_t cell, const double fixedVertices = 0.2) noexcept {
        if constexpr (Debug) std::cout << "Dissecting cell " << cell << " (current number of cells " << vertexPartition.numberOfCells() << ")" << std::endl;
        std::vector<Vertex>& vertices = vertexPartition.getCell(cell);
        if constexpr (Debug) std::cout << "   Cell size: " << String::prettyInt(vertices.size()) << std::endl;
        ThreadData& data = threadData[0];
        data.bestCutSize = static_cast<size_t>(-1);
        for (const Geometry::Point& direction : directions) {
            if constexpr (Debug) {
                std::cout << "   Computing Separator for direction " << direction << std::endl;
                cutTimer.restart();
            }
            std::vector<Vertex> sources;
            std::vector<Vertex> targets;
            InertialFlow::divideVertices<Criteria>(vertices, sources, targets, coordinates, direction, fixedVertices);
            if constexpr (Debug) {
                std::cout << "      Sources: " << String::prettyInt(sources.size()) << " (" << String::percent(sources.size() / static_cast<double>(vertices.size())) << ")" << std::endl;
                std::cout << "      Targets: " << String::prettyInt(targets.size()) << " (" << String::percent(targets.size() / static_cast<double>(vertices.size())) << ")" << std::endl;
            }
            data.dinic.run(sources, targets);
            if constexpr (Debug) {
                std::cout << "      Cut weight: " << String::prettyInt(data.dinic.getFlow()) << std::endl;
                std::cout << "      Time: " << String::msToString(cutTimer.elapsedMilliseconds()) << std::endl;
            }
            if (data.bestCutSize <= data.dinic.getFlow()) continue;
            data.bestCutSize = data.dinic.getFlow();
            const Dinic::Cut sourceCut = data.dinic.sourceCutAndSide();
            const Dinic::Cut targetCut = data.dinic.sourceCutAndSide();
            const int64_t optimalBalancedSize = static_cast<int64_t>(vertices.size() / 2);
            if (std::abs(optimalBalancedSize - static_cast<int64_t>(sourceCut.side.size())) <= std::abs(optimalBalancedSize - static_cast<int64_t>(targetCut.side.size()))) {
                data.bestCut = sourceCut;
            } else {
                data.bestCut = targetCut;
            }
        }
        usedVertices.clear();
        for (const Vertex vertex : data.bestCut.side) {
            usedVertices.insert(vertex);
        }
        std::vector<Vertex> otherSide;
        for (const Vertex vertex : vertices) {
            if (usedVertices.contains(vertex)) continue;
            otherSide.emplace_back(vertex);
        }
        for (const Dinic::CutEdge edge : data.bestCut.edges) {
            cutEdges.emplace_back(edge);
        }
        AssertMsg(data.bestCut.side.size() >= ceil(vertices.size() * fixedVertices), "Side should have at least " << (vertices.size() * fixedVertices) << " vertices, but has only " << data.bestCut.side.size() << " vertices!");
        AssertMsg(otherSide.size() >= ceil(vertices.size() * fixedVertices), "Side should have at least " << (vertices.size() * fixedVertices) << " vertices, but has only " << otherSide.size() << " vertices!");
        vertexPartition.splitCell(cell, data.bestCut.side, otherSide);
        if constexpr (Debug) {
            std::cout << "   Cut weight: " << String::prettyInt(data.bestCutSize) << std::endl;
            std::cout << "   Cut size:   " << String::prettyInt(data.bestCut.edges.size()) << std::endl;
            std::cout << "   Removing Cut from flow graph." << std::endl;
        }
        data.dinic.removeCut(data.bestCut.edges);
        if constexpr (Debug) std::cout << "Done." << std::endl;
    }

    inline void dissect(const ThreadPinning& threadPinning, VertexPartition& vertexPartition, const size_t cell, const double fixedVertices = 0.2) noexcept {
        if constexpr (Debug) std::cout << "Dissecting cell " << cell << " (current number of cells " << vertexPartition.numberOfCells() << ")" << std::endl;
        std::vector<Vertex>& vertices = vertexPartition.getCell(cell);
        if constexpr (Debug) std::cout << "   Cell size: " << String::prettyInt(vertices.size()) << std::endl;

        if (threadData.size() < threadPinning.numberOfThreads) {
            threadData.resize(threadPinning.numberOfThreads, threadData.back());
        }
        omp_set_num_threads(threadPinning.numberOfThreads);
        #pragma omp parallel
        {
            threadPinning.pinThread();
            ThreadData& data = threadData[omp_get_thread_num()];
            std::vector<Vertex> localVertices = vertices;
            data.bestCutSize = static_cast<size_t>(-1);
            #pragma omp for
            for (size_t i = 0; i < directions.size(); i++) {
                const Geometry::Point& direction = directions[i];
                std::vector<Vertex> sources;
                std::vector<Vertex> targets;
                InertialFlow::divideVertices<Criteria>(localVertices, sources, targets, coordinates, direction, fixedVertices);
                data.dinic.run(sources, targets);
                if (data.bestCutSize <= data.dinic.getFlow()) continue;
                data.bestCutSize = data.dinic.getFlow();
                const Dinic::Cut sourceCut = data.dinic.sourceCutAndSide();
                const Dinic::Cut targetCut = data.dinic.sourceCutAndSide();
                const int64_t optimalBalancedSize = static_cast<int64_t>(localVertices.size() / 2);
                if (std::abs(optimalBalancedSize - static_cast<int64_t>(sourceCut.side.size())) <= std::abs(optimalBalancedSize - static_cast<int64_t>(targetCut.side.size()))) {
                    data.bestCut = sourceCut;
                } else {
                    data.bestCut = targetCut;
                }
            }
        }

        size_t bestCutIndex = 0;
        for (size_t i = 0; i < threadPinning.numberOfThreads; i++) {
            if (threadData[bestCutIndex].bestCutSize > threadData[i].bestCutSize) {
                bestCutIndex = i;
            }
        }
        usedVertices.clear();
        for (const Vertex vertex : threadData[bestCutIndex].bestCut.side) {
            usedVertices.insert(vertex);
        }
        std::vector<Vertex> otherSide;
        for (const Vertex vertex : vertices) {
            if (usedVertices.contains(vertex)) continue;
            otherSide.emplace_back(vertex);
        }
        for (const Dinic::CutEdge edge : threadData[bestCutIndex].bestCut.edges) {
            cutEdges.emplace_back(edge);
        }
        AssertMsg(threadData[bestCutIndex].bestCut.side.size() >= ceil(vertices.size() * fixedVertices), "Side should have at least " << (vertices.size() * fixedVertices) << " vertices, but has only " << threadData[bestCutIndex].bestCut.side.size() << " vertices!");
        AssertMsg(otherSide.size() >= ceil(vertices.size() * fixedVertices), "Side should have at least " << (vertices.size() * fixedVertices) << " vertices, but has only " << otherSide.size() << " vertices!");
        vertexPartition.splitCell(cell, threadData[bestCutIndex].bestCut.side, otherSide);
        if constexpr (Debug) {
            std::cout << "   Cut weight: " << String::prettyInt(threadData[bestCutIndex].bestCutSize) << std::endl;
            std::cout << "   Cut size:   " << String::prettyInt(threadData[bestCutIndex].bestCut.edges.size()) << std::endl;
            std::cout << "   Removing Cut from flow graph." << std::endl;
        }

        #pragma omp parallel for
        for (size_t i = 0; i < threadPinning.numberOfThreads; i++) {
            threadData[i].dinic.removeCut(threadData[bestCutIndex].bestCut.edges);
        }

        if constexpr (Debug) std::cout << "Done." << std::endl;
    }

private:
    const std::vector<Geometry::Point>& coordinates;
    const std::vector<Geometry::Point> directions;

    std::vector<ThreadData> threadData;

    IndexedSet<false, Vertex> usedVertices;
    Timer cutTimer;
    Timer totalTimer;

    std::vector<Dinic::CutEdge> cutEdges;

};

template<bool DEBUG, InertialFlow::DivisionCriteria CRITERIA = InertialFlow::QUANTITYANDVALUE>
class InertialFlowOnVertices {

public:
    static constexpr bool Debug = DEBUG;
    static constexpr InertialFlow::DivisionCriteria Criteria = CRITERIA;
    using Type = InertialFlowOnVertices<Debug, Criteria>;

private:
    struct ThreadData {
        template<typename GRAPH>
        ThreadData(const GRAPH& graph, const size_t numVertices) :
            vertexSeparator(graph),
            bestSeparatorSize(0),
            usedVertices(numVertices) {
        }
        VertexSeparatorUsingGraphExpansion vertexSeparator;
        size_t bestSeparatorSize;
        std::vector<Vertex> separator;
        std::vector<Vertex> sourceSide;
        std::vector<Vertex> targetSide;
        IndexedSet<false, Vertex> usedVertices;
    };

public:
    template<typename GRAPH>
    InertialFlowOnVertices(const GRAPH& graph, const std::vector<Geometry::Point>& coordinates, const std::vector<Geometry::Point>& directions = InertialFlow::defaultDirections()) :
        coordinates(coordinates),
        directions(directions),
        threadData(1, {graph, coordinates.size()}) {
    }

public:
    template<typename GRAPH>
    inline NestedDissection run(const GRAPH& graph, const size_t numberOfCells, const double fixedVertices = 0.2) noexcept {
        if constexpr (Debug) totalTimer.restart();
        NestedDissection nestedDissection(coordinates.size());
        for (size_t cell = 0; cell < numberOfCells - 1; cell++) {
            dissect(nestedDissection, cell, fixedVertices);
        }
        if constexpr (Debug) std::cout << "Total time: " << String::msToString(totalTimer.elapsedMilliseconds()) << std::endl;
        nestedDissection.computeIncidentCells(graph);
        return nestedDissection;
    }

    template<typename GRAPH>
    inline NestedDissection run(const ThreadPinning& threadPinning, const GRAPH& graph, const size_t numberOfCells, const double fixedVertices = 0.2) noexcept {
        if constexpr (Debug) totalTimer.restart();
        NestedDissection nestedDissection(coordinates.size());
        for (size_t cell = 0; cell < numberOfCells - 1; cell++) {
            dissect(threadPinning, nestedDissection, cell, fixedVertices);
        }
        if constexpr (Debug) std::cout << "Total time: " << String::msToString(totalTimer.elapsedMilliseconds()) << std::endl;
        nestedDissection.computeIncidentCells(graph);
        return nestedDissection;
    }

private:
    inline void dissect(NestedDissection& nestedDissection, const size_t cell, const double fixedVertices = 0.2) noexcept {
        if constexpr (Debug) std::cout << "Dissecting cell " << cell << " (level " << nestedDissection.levelOfCell(cell) << ")" << std::endl;
        std::vector<Vertex> vertices = nestedDissection.getVerticesOfCell(cell);
        if constexpr (Debug) std::cout << "   Cell size: " << String::prettyInt(vertices.size()) << std::endl;
        ThreadData& data = threadData[0];
        data.bestSeparatorSize = vertices.size();
        for (const Geometry::Point& direction : directions) {
            if constexpr (Debug) {
                std::cout << "   Computing Separator for direction " << direction << std::endl;
                cutTimer.restart();
            }
            std::vector<Vertex> sources;
            std::vector<Vertex> targets;
            InertialFlow::divideVertices<Criteria>(vertices, sources, targets, coordinates, direction, fixedVertices);
            if constexpr (Debug) {
                std::cout << "      Sources: " << String::prettyInt(sources.size()) << " (" << String::percent(sources.size() / static_cast<double>(vertices.size())) << ")" << std::endl;
                std::cout << "      Targets: " << String::prettyInt(targets.size()) << " (" << String::percent(targets.size() / static_cast<double>(vertices.size())) << ")" << std::endl;
            }
            data.vertexSeparator.run(sources, targets);
            data.separator = data.vertexSeparator.getSeparator();
            if constexpr (Debug) {
                std::cout << "      Separator size: " << String::prettyInt(data.separator.size()) << std::endl;
                std::cout << "      Time: " << String::msToString(cutTimer.elapsedMilliseconds()) << std::endl;
            }
            if (data.bestSeparatorSize <= data.separator.size()) continue;
            data.bestSeparatorSize = data.separator.size();
            data.sourceSide = data.vertexSeparator.getSide(sources);
            data.usedVertices.clear();
            for (const Vertex vertex : data.separator) {
                data.usedVertices.insert(vertex);
            }
            for (const Vertex vertex : data.sourceSide) {
                AssertMsg(!data.usedVertices.contains(vertex), "Vertex " << vertex << " is part of the separator and the source side!");
                data.usedVertices.insert(vertex);
            }
            data.targetSide.clear();
            for (const Vertex vertex : vertices) {
                if (data.usedVertices.contains(vertex)) continue;
                data.targetSide.emplace_back(vertex);
            }
            nestedDissection.divideCell(cell, data.separator, data.sourceSide, data.targetSide);
        }
        if constexpr (Debug) {
            std::cout << "   Separator size: " << String::prettyInt(data.bestSeparatorSize) << std::endl;
            std::cout << "   Removing Separator from flow graph." << std::endl;
        }
        for (const Vertex vertex : nestedDissection.getSeparatorOfCell(cell)) {
            data.vertexSeparator.isolateVertex(vertex);
        }
        if constexpr (Debug) std::cout << "Done." << std::endl;
    }

    inline void dissect(const ThreadPinning& threadPinning, NestedDissection& nestedDissection, const size_t cell, const double fixedVertices = 0.2) noexcept {
        if constexpr (Debug) std::cout << "Dissecting cell " << cell << " (level " << nestedDissection.levelOfCell(cell) << ")" << std::endl;
        std::vector<Vertex> vertices = nestedDissection.getVerticesOfCell(cell);
        if constexpr (Debug) std::cout << "   Cell size: " << String::prettyInt(vertices.size()) << std::endl;

        while (threadData.size() < threadPinning.numberOfThreads) {
            threadData.emplace_back(threadData.back());
        }
        omp_set_num_threads(threadPinning.numberOfThreads);
        #pragma omp parallel
        {
            threadPinning.pinThread();
            ThreadData& data = threadData[omp_get_thread_num()];
            std::vector<Vertex> localVertices = vertices;
            data.bestSeparatorSize = localVertices.size();
            #pragma omp for
            for (size_t i = 0; i < directions.size(); i++) {
                const Geometry::Point& direction = directions[i];
                std::vector<Vertex> sources;
                std::vector<Vertex> targets;
                InertialFlow::divideVertices<Criteria>(localVertices, sources, targets, coordinates, direction, fixedVertices);
                data.vertexSeparator.run(sources, targets);
                data.separator = data.vertexSeparator.getSeparator();
                if (data.bestSeparatorSize <= data.separator.size()) continue;
                data.bestSeparatorSize = data.separator.size();
                data.sourceSide = data.vertexSeparator.getSide(sources);
                data.usedVertices.clear();
                for (const Vertex vertex : data.separator) {
                    data.usedVertices.insert(vertex);
                }
                for (const Vertex vertex : data.sourceSide) {
                    AssertMsg(!data.usedVertices.contains(vertex), "Vertex " << vertex << " is part of the separator and the source side!");
                    data.usedVertices.insert(vertex);
                }
                data.targetSide.clear();
                for (const Vertex vertex : localVertices) {
                    if (data.usedVertices.contains(vertex)) continue;
                    data.targetSide.emplace_back(vertex);
                }
            }
        }

        size_t bestSeparatorIndex = 0;
        for (size_t i = 0; i < threadPinning.numberOfThreads; i++) {
            if (threadData[bestSeparatorIndex].bestSeparatorSize > threadData[i].bestSeparatorSize) {
                bestSeparatorIndex = i;
            }
        }
        nestedDissection.divideCell(cell, threadData[bestSeparatorIndex].separator, threadData[bestSeparatorIndex].sourceSide, threadData[bestSeparatorIndex].targetSide);

        if constexpr (Debug) {
            std::cout << "   Separator size: " << String::prettyInt(threadData[bestSeparatorIndex].bestSeparatorSize) << std::endl;
            std::cout << "   Removing Separator from flow graph." << std::endl;
        }

        #pragma omp parallel for
        for (size_t i = 0; i < threadPinning.numberOfThreads; i++) {
            for (const Vertex vertex : nestedDissection.getSeparatorOfCell(cell)) {
                threadData[i].vertexSeparator.isolateVertex(vertex);
            }
        }

        if constexpr (Debug) std::cout << "Done." << std::endl;
    }

private:
    const std::vector<Geometry::Point>& coordinates;
    const std::vector<Geometry::Point> directions;

    std::vector<ThreadData> threadData;

    Timer cutTimer;
    Timer totalTimer;

};
