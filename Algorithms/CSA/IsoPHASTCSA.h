#pragma once

#include <vector>

#include "../../DataStructures/CH/UPGraphs.h"
#include "../CH/CH.h"
#include "../../DataStructures/Container/ExternalKHeap.h"
#include "../../DataStructures/Container/Set.h"
#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/Isochrone/DistanceBounds.h"
#include "../../DataStructures/Partition/VertexPartition.h"
#include "../../Helpers/Assert.h"
#include "../../Helpers/Types.h"
#include "Profiler.h"

namespace CSA {

//TODO: Parallelization
class IsoPHASTCSATransfers {
private:
    struct DijkstraLabel : public ExternalKHeapElement {
        DijkstraLabel(int* const arrivalTime) :
            ExternalKHeapElement(),
            arrivalTime(arrivalTime) {
        }

        inline int getArrivalTime() const noexcept {
            return *arrivalTime;
        }

        inline bool hasSmallerKey(const DijkstraLabel* other) const noexcept {return getArrivalTime() < other->getArrivalTime();}

        int* const arrivalTime;
    };

public:
    IsoPHASTCSATransfers(const Order& order, const VertexPartition& partition, const CH::CH& ch, const std::vector<std::vector<DistanceBounds>>& distanceBounds, const size_t numSharedVertices, const size_t numStops) :
        partition(partition),
        initialUpwardGraph(ch.forward),
        initialDownwardGraph(partition.numberOfCells()),
        upwardGraph(partition.numberOfCells()),
        downwardGraph(partition.numberOfCells()),
        distanceBounds(distanceBounds),
        numberOfStops(numStops),
        contractionOrder(order),
        positionInOrder(Construct::Invert, order),
        internalSourceVertex(noVertex),
        sourceDepartureTime(never),
        maxArrivalTime(never),
        Q(ch.forward.numVertices()),
        distance(ch.forward.numVertices(), INFTY),
        timestamp(ch.forward.numVertices(), 0),
        currentTimestamp(0),
        isochrone(ch.forward.numVertices()),
        reachedStops(numberOfStops),
        reachedCells(partition.numberOfCells()),
        minCellDistance(partition.numberOfCells(), never),
        maxCellDistance(partition.numberOfCells(), -never) {
        std::vector<Vertex> sharedVertices = Vector::id<Vertex>(numSharedVertices);
        sharedUpwardGraph.build(ch.forward, sharedVertices, true, true, 0);
        sharedDownwardGraph.build(ch.backward, sharedVertices, false, false, 0);
        for (size_t cell = 0; cell < partition.numberOfCells(); cell++) {
            initialDownwardGraph[cell].build(ch.backward, getStopsInCell(cell), false, false, numSharedVertices);
            upwardGraph[cell].build(ch.forward, getStopsInCell(cell), true, true, numSharedVertices);
            downwardGraph[cell].build(ch.backward, partition.getCell(cell), false, false, numSharedVertices);
        }
        for (const Vertex vertex : ch.forward.vertices()) {
            dijkstraLabel.emplace_back(&distance[vertex]);
        }
    }

    inline void clear() noexcept {
        internalSourceVertex = noVertex;
        maxArrivalTime = never;
        Q.clear();
        currentTimestamp++;
        isochrone.clear();
        reachedStops.clear();
        reachedCells.clear();
        Vector::fill(minCellDistance, never);
        Vector::fill(maxCellDistance, -never);
    }

    inline void initialize(const Vertex externalSourceVertex, const int departureTime, const int maxTime) noexcept {
        internalSourceVertex = externalToInternal(externalSourceVertex);
        sourceDepartureTime = departureTime;
        maxArrivalTime = maxTime;
    }

    inline void arrivalByTrip(const Vertex externalVertex, const int time) noexcept {
        isochrone.insert(externalVertex);
        const Vertex internalVertex = externalToInternal(externalVertex);
        distance[internalVertex] = time;
        timestamp[internalVertex] = currentTimestamp;
        const size_t cell = partition.getCellIdOfVertex(internalVertex);
        reachedCells.insert(cell);
        minCellDistance[cell] = std::min(minCellDistance[cell], time);
        maxCellDistance[cell] = std::max(maxCellDistance[cell], time);
    }

    inline void runInitialTransfers() noexcept {
        initialUpwardSearch();
        sweep<true>(sharedDownwardGraph);
        for (const size_t cell : initialCellSelection()) {
            sweep<true>(initialDownwardGraph[cell]);
        }
    }

    //TODO: Select cells to do upward sweeps from?
    inline void runFinalTransfers() noexcept {
        for (const size_t cell : reachedCells) {
            sweep<false>(upwardGraph[cell]);
        }
        sweep<false>(sharedUpwardGraph);
        sweep<false>(sharedDownwardGraph);
        for (const size_t cell : finalCellSelection()) {
            sweep<false>(downwardGraph[cell]);
        }
    }

    inline const IndexedSet<false, Vertex>& getIsochrone() const noexcept {
        return isochrone;
    }

    inline const IndexedSet<false, Vertex>& getReachedStops() const noexcept {
        return reachedStops;
    }

    inline int getDistance(const Vertex externalVertex) noexcept {
        const Vertex internalVertex = externalToInternal(externalVertex);
        check(internalVertex);
        return distance[internalVertex];
    }

private:
    inline void initialUpwardSearch() noexcept {
        Q.update(&dijkstraLabel[internalSourceVertex]);
        while (!Q.empty()) {
            DijkstraLabel* uLabel = Q.extractFront();
            const Vertex u(uLabel - &(dijkstraLabel[0]));
            reach<true>(u);
            for (const Edge edge : initialUpwardGraph.edgesFrom(u)) {
                const Vertex v = initialUpwardGraph.get(ToVertex, edge);
                check(v);
                const int newDistance = distance[u] + initialUpwardGraph.get(Weight, edge);
                if (distance[v] > newDistance && newDistance <= maxArrivalTime) {
                    distance[v] = newDistance;
                    Q.update(&dijkstraLabel[v]);
                }
            }
        }
    }

    inline std::vector<size_t> initialCellSelection() noexcept {
        std::vector<size_t> selectedCells;
        const size_t sourceCell = partition.getCellIdOfVertex(internalSourceVertex);
        for (size_t cell = 0; cell < partition.numberOfCells(); cell++) {
            if (sourceDepartureTime + distanceBounds[sourceCell][cell].lowerBound <= maxArrivalTime) {
                selectedCells.emplace_back(cell);
            }
        }
        return selectedCells;
    }

    template<bool INITIAL>
    inline void sweep(const CH::CompactSweepGraph& graph) {
        for (const Vertex sweepV : graph.graph.vertices()) {
            const Vertex v = graph.internalToExternal(sweepV);
            check(v);
            for (const Edge edge : graph.graph.edgesFrom(sweepV)) {
                const Vertex u = graph.toVertex[edge];
                const int weight = graph.graph.get(Weight, edge);
                const int newDistance = distance[u] + weight;
                distance[v] = branchlessConditional(newDistance < distance[v], newDistance, distance[v]);
            }
            if (distance[v] <= maxArrivalTime) {
                reach<INITIAL>(v);
            }
        }
    }

    inline std::vector<size_t> finalCellSelection() noexcept {
        std::vector<size_t> selectedCells;
        for (size_t cell = 0; cell < partition.numberOfCells(); cell++) {
            bool fullyOutOfRange = true;
            for (const size_t reachedCell : reachedCells) {
                if (distanceBounds[reachedCell][cell].upperBound <= maxArrivalTime - maxCellDistance[reachedCell]) {
                    for (const Vertex vertex : partition.getCell(cell)) {
                        reach<false>(vertex);
                    }
                    break;
                }
                if (distanceBounds[reachedCell][cell].lowerBound <= maxArrivalTime - minCellDistance[reachedCell]) {
                    fullyOutOfRange = false;
                }
            }
            if (!fullyOutOfRange) {
                selectedCells.emplace_back(cell);
            }
        }
        return selectedCells;
    }

    inline bool isStop(const Vertex internalVertex) noexcept {
        return internalToExternal(internalVertex) < numberOfStops;
    }

    inline std::vector<Vertex> getStopsInCell(const size_t cell) noexcept {
        std::vector<Vertex> result;
        for (const Vertex vertex : partition.getCell(cell)) {
            if (!isStop(vertex)) continue;
            result.emplace_back(vertex);
        }
        return result;
    }

    template<bool INITIAL>
    inline void reach(const Vertex internalVertex) noexcept {
        const Vertex externalVertex = internalToExternal(internalVertex);
        isochrone.insert(externalVertex);
        if constexpr (INITIAL) {
            if (externalVertex < numberOfStops) reachedStops.insert(externalVertex);
        }
    }

    inline Vertex externalToInternal(const Vertex vertex) const noexcept {
        return positionInOrder.permutate(vertex);
    }

    inline Vertex internalToExternal(const Vertex vertex) const noexcept {
        return Vertex(contractionOrder[vertex]);
    }

    inline void check(const Vertex vertex) noexcept {
        if (timestamp[vertex] != currentTimestamp) {
            distance[vertex] = INFTY;
            timestamp[vertex] = currentTimestamp;
        }
    }

private:
    const VertexPartition& partition;
    const CHGraph& initialUpwardGraph;
    std::vector<CH::CompactSweepGraph> initialDownwardGraph;
    CH::CompactSweepGraph sharedUpwardGraph;
    std::vector<CH::CompactSweepGraph> upwardGraph;
    CH::CompactSweepGraph sharedDownwardGraph;
    std::vector<CH::CompactSweepGraph> downwardGraph;
    const std::vector<std::vector<DistanceBounds>>& distanceBounds;
    const size_t numberOfStops;

    const Order contractionOrder;
    const Permutation positionInOrder;

    Vertex internalSourceVertex;
    int sourceDepartureTime;
    int maxArrivalTime;

    ExternalKHeap<2, DijkstraLabel> Q;
    std::vector<DijkstraLabel> dijkstraLabel;
    std::vector<int> distance;
    std::vector<int> timestamp;
    int currentTimestamp;

    IndexedSet<false, Vertex> isochrone;
    IndexedSet<false, Vertex> reachedStops;
    IndexedSet<false, size_t> reachedCells;
    std::vector<int> minCellDistance;
    std::vector<int> maxCellDistance;
};

template<typename PROFILER = NoProfiler>
class IsoPHASTCSA {

public:
    using Profiler = PROFILER;
    using Type = IsoPHASTCSA<Profiler>;

public:
    IsoPHASTCSA(const Data& data, const Order& order, const VertexPartition& partition, const CH::CH& ch, const std::vector<std::vector<DistanceBounds>>& distanceBounds, const size_t numSharedVertices, const Profiler& profilerTemplate = Profiler()) :
        data(data),
        transferModule(order, partition, ch, distanceBounds, numSharedVertices, data.numberOfStops()),
        sourceVertex(noVertex),
        sourceDepartureTime(never),
        maxArrivalTime(never),
        tripReached(data.numberOfTrips(), false),
        arrivalTime(data.numberOfStops(), never),
        arrivalTimeByTrip(data.numberOfStops(), never),
        profiler(profilerTemplate) {
        AssertMsg(Vector::isSorted(data.connections), "Connections must be sorted in ascending order!");
        profiler.registerPhases({PHASE_CLEAR, PHASE_INITIALIZATION, PHASE_CONNECTION_SCAN, PHASE_FINAL_TRANSFERS});
        profiler.registerMetrics({METRIC_CONNECTIONS, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
    }

    inline void run(const Vertex source, const int departureTime, const int range) noexcept {
        profiler.start();

        profiler.startPhase();
        clear();
        profiler.donePhase(PHASE_CLEAR);

        profiler.startPhase();
        sourceVertex = source;
        sourceDepartureTime = departureTime;
        maxArrivalTime = departureTime + range;
        transferModule.initialize(source, departureTime, maxArrivalTime);
        if (data.isStop(source)) {
            arrivalTime[source] = departureTime;
            arrivalTimeByTrip[source] = departureTime;
        }
        runInitialTransfers();
        const ConnectionId connectionStart = firstReachableConnection(departureTime);
        const ConnectionId connectionEnd = firstUnreachableConnection(maxArrivalTime);
        profiler.donePhase(PHASE_INITIALIZATION);

        profiler.startPhase();
        scanConnections(connectionStart, connectionEnd);
        profiler.donePhase(PHASE_CONNECTION_SCAN);

        profiler.startPhase();
        transferModule.runFinalTransfers();
        profiler.donePhase(PHASE_FINAL_TRANSFERS);

        profiler.done();
    }

    inline bool reachable(const Vertex vertex) const noexcept {
        return transferModule.getIsochrone().contains(vertex);
    }

    inline const std::vector<Vertex>& getIsochrone() const noexcept {
        return transferModule.getIsochrone().getValues();
    }

    inline int getEarliestArrivalTime(const Vertex vertex) noexcept {
        return transferModule.getDistance(vertex);
    }

    inline const Profiler& getProfiler() const noexcept {
        return profiler;
    }

private:
    inline void clear() noexcept {
        transferModule.clear();

        sourceVertex = noVertex;
        sourceDepartureTime = never;
        maxArrivalTime = never;

        Vector::fill(tripReached, false);
        Vector::fill(arrivalTime, never);
        Vector::fill(arrivalTimeByTrip, never);
    }

    inline void runInitialTransfers() noexcept {
        transferModule.arrivalByTrip(sourceVertex, sourceDepartureTime);
        transferModule.runInitialTransfers();
        for (const Vertex stop : transferModule.getReachedStops()) {
            AssertMsg(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
            AssertMsg(transferModule.getDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
            arrivalTime[stop] = transferModule.getDistance(stop);
        }

    }

    inline ConnectionId firstReachableConnection(const int departureTime) const noexcept {
        return ConnectionId(Vector::lowerBound(data.connections, departureTime, [](const Connection& connection, const int time) {
            return connection.departureTime < time;
        }));
    }

    inline ConnectionId firstUnreachableConnection(const int departureTime) const noexcept {
        return ConnectionId(Vector::upperBound(data.connections, departureTime, [](const int time, const Connection& connection) {
            return time < connection.departureTime;
        }));
    }

    inline void scanConnections(const ConnectionId begin, const ConnectionId end) noexcept {
        for (ConnectionId i = begin; i < end; i++) {
            const Connection& connection = data.connections[i];
            if (connection.arrivalTime > maxArrivalTime) continue;
            if (connectionIsReachable(connection)) {
                profiler.countMetric(METRIC_CONNECTIONS);
                arrivalByTrip(connection.arrivalStopId, connection.arrivalTime);
            }
        }
    }

    inline bool connectionIsReachableFromStop(const Connection& connection) const noexcept {
        return arrivalTime[connection.departureStopId] <= connection.departureTime - data.minTransferTime(connection.departureStopId);
    }

    inline bool connectionIsReachableFromTrip(const Connection& connection) const noexcept {
        return tripReached[connection.tripId];
    }

    inline bool connectionIsReachable(const Connection& connection) noexcept {
        if (connectionIsReachableFromTrip(connection)) return true;
        if (connectionIsReachableFromStop(connection)) {
            tripReached[connection.tripId] = true;
            return true;
        }
        return false;
    }

    inline void arrivalByTrip(const StopId stop, const int time) noexcept {
        if (arrivalTimeByTrip[stop] <= time) return;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        arrivalTimeByTrip[stop] = time;

        for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
            profiler.countMetric(METRIC_EDGES);
            const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
            const int newArrivalTime = time + data.transferGraph.get(TravelTime, edge);
            if (newArrivalTime > maxArrivalTime) continue;
            if (arrivalTime[toStop] <= newArrivalTime) continue;
            profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
            arrivalTime[toStop] = newArrivalTime;
        }

        if (arrivalTime[stop] <= time) return;
        arrivalTime[stop] = time;
        transferModule.arrivalByTrip(stop, time);
    }

private:
    const Data& data;
    IsoPHASTCSATransfers transferModule;

    Vertex sourceVertex;
    int sourceDepartureTime;
    int maxArrivalTime;

    std::vector<bool> tripReached;
    std::vector<int> arrivalTime;
    std::vector<int> arrivalTimeByTrip;

    Profiler profiler;
};
}
