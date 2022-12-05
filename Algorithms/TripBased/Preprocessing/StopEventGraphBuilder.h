#pragma once

#include "../../../Helpers/Console/Progress.h"
#include "../../../DataStructures/TripBased/Data.h"

namespace TripBased {

class StopEventGraphBuilder {

private:
    struct StopLabel {

    public:
        StopLabel() :
            arrivalTime(INFTY),
            timestamp(0) {
        }

        inline void checkTimestamp(const int newTimestamp) noexcept {
            arrivalTime = (timestamp != newTimestamp) ? INFTY : arrivalTime;
            timestamp = newTimestamp;
        }

        inline void update(const int newTimestamp, const int newArrivalTime) noexcept {
            checkTimestamp(newTimestamp);
            arrivalTime = std::min(arrivalTime, newArrivalTime);
        }

        int arrivalTime;
        int timestamp;
    };

    struct RouteTransfer {
        RouteTransfer(const RouteId toRoute, const StopIndex fromIndex, const StopIndex toIndex, const int transferTime) :
            toRoute(toRoute),
            fromIndex(fromIndex),
            toIndex(toIndex),
            transferTime(transferTime) {
        }

        RouteId toRoute;
        StopIndex fromIndex;
        StopIndex toIndex;
        int transferTime;

        inline std::tuple<RouteId, StopIndex, StopIndex> getTuple() const noexcept {
            return std::make_tuple(toRoute, -fromIndex, toIndex);
        }

        inline bool operator<(const RouteTransfer& other) const noexcept {
            return getTuple() < other.getTuple();
        }
    };

public:
    StopEventGraphBuilder(const Data& data) :
        data(data),
        labels(data.numberOfStops()),
        timestamp(0) {
        generatedTransfers.addVertices(data.numberOfStopEvents());
        keptTransfers.addVertices(data.numberOfStopEvents());
    }

public:
    inline void generateRouteBasedTransfers(const RouteId fromRoute) noexcept {
        if (generatedTransfers.numEdges() > 1000000) {
            generatedTransfers.clear();
            generatedTransfers.addVertices(data.numberOfStopEvents());
        }
        const std::vector<RouteTransfer> routeTransfers = generateRouteTransfers(fromRoute);
        for (const TripId fromTrip : data.tripsOfRoute(fromRoute)) {
            RouteId toRoute = noRouteId;
            std::vector<TripId> earliestTrip;
            for (const RouteTransfer& routeTransfer : routeTransfers) {
                if (routeTransfer.toRoute != toRoute) {
                    toRoute = routeTransfer.toRoute;
                    std::vector<TripId>(data.numberOfStopsInRoute(toRoute), noTripId).swap(earliestTrip);
                }
                const StopEventId fromEvent = data.getStopEventId(fromTrip, routeTransfer.fromIndex);
                const int arrivalTime = data.raptorData.stopEvents[fromEvent].arrivalTime + routeTransfer.transferTime;
                const TripId toTrip = data.getEarliestTrip(toRoute, routeTransfer.toIndex, arrivalTime);
                if (toTrip >= earliestTrip[routeTransfer.toIndex]) continue;
                if ((toRoute == fromRoute) && (toTrip >= fromTrip) && (routeTransfer.toIndex >= routeTransfer.fromIndex)) continue;
                if (isUTurn(fromTrip, routeTransfer.fromIndex, toTrip, routeTransfer.toIndex)) continue;
                for (StopIndex i = routeTransfer.toIndex; i < data.numberOfStopsInRoute(toRoute); i++) {
                    earliestTrip[i] = std::min(earliestTrip[i], toTrip);
                }
                const StopEventId toEvent = data.getStopEventId(toTrip, routeTransfer.toIndex);
                generatedTransfers.addEdge(Vertex(fromEvent), Vertex(toEvent));
            }
        }
    }

    inline void generateFullTransfers(const TripId trip) noexcept {
        if (generatedTransfers.numEdges() > 1000000) {
            generatedTransfers.clear();
            generatedTransfers.addVertices(data.numberOfStopEvents());
        }
        const StopId* stops = data.stopArrayOfTrip(trip);
        for (StopIndex i = StopIndex(1); i < data.numberOfStopsInTrip(trip); i++) {
            const StopId stop = stops[i];
            const int arrivalTime = data.getStopEvent(trip, i).arrivalTime;
            findTransfers(trip, i, stop, arrivalTime);
            for (const Edge edge : data.raptorData.transferGraph.edgesFrom(stop)) {
                const StopId toStop = StopId(data.raptorData.transferGraph.get(ToVertex, edge));
                const int transferTime = data.raptorData.transferGraph.get(TravelTime, edge);
                findTransfers(trip, i, toStop, arrivalTime + transferTime);
            }
        }
    }

    inline void reduceTransfers(const TripId trip) noexcept {
        timestamp++;
        const StopId* stops = data.stopArrayOfTrip(trip);
        for (StopIndex i = StopIndex(data.numberOfStopsInTrip(trip) - 1); i > 0; i--) {
            const int arrivalTime = data.getStopEvent(trip, i).arrivalTime;
            labels[stops[i]].update(timestamp, arrivalTime);
            for (const Edge edge : data.raptorData.transferGraph.edgesFrom(stops[i])) {
                const StopId toStop = StopId(data.raptorData.transferGraph.get(ToVertex, edge));
                const int transferTime = data.raptorData.transferGraph.get(TravelTime, edge);
                labels[toStop].update(timestamp, arrivalTime + transferTime);
            }

            std::vector<Edge> transfers;
            const Vertex fromVertex = Vertex(data.getStopEventId(trip, i));
            for (const Edge edge : generatedTransfers.edgesFrom(fromVertex)) {
                transfers.emplace_back(edge);
            }
            std::stable_sort(transfers.begin(), transfers.end(), [&](const Edge a, const Edge b){
                return data.raptorData.stopEvents[generatedTransfers.get(ToVertex, a)].arrivalTime < data.raptorData.stopEvents[generatedTransfers.get(ToVertex, b)].arrivalTime;
            });

            std::vector<Edge> keepTransfers;
            for (const Edge transfer : transfers) {
                bool keep = false;
                const StopEventId toEvent = StopEventId(generatedTransfers.get(ToVertex, transfer));
                const StopIndex toIndex = data.indexOfStopEvent[toEvent];
                const TripId toTrip = data.tripOfStopEvent[toEvent];
                const StopId* toStops = data.stopArrayOfTrip(toTrip) + toIndex;
                for (size_t j = data.numberOfStopsInTrip(toTrip) - toIndex - 1; j > 0; j--) {
                    const StopId destinationStop = toStops[j];
                    const int destinationArrivalTime = data.raptorData.stopEvents[toEvent + j].arrivalTime;
                    labels[destinationStop].checkTimestamp(timestamp);
                    if (labels[destinationStop].arrivalTime > destinationArrivalTime) {
                        labels[destinationStop].arrivalTime = destinationArrivalTime;
                        keep = true;
                    }
                    for (const Edge edge : data.raptorData.transferGraph.edgesFrom(destinationStop)) {
                        const StopId arrivalStop = StopId(data.raptorData.transferGraph.get(ToVertex, edge));
                        const int arrivalTime = destinationArrivalTime + data.raptorData.transferGraph.get(TravelTime, edge);
                        labels[arrivalStop].checkTimestamp(timestamp);
                        if (labels[arrivalStop].arrivalTime > arrivalTime) {
                            labels[arrivalStop].arrivalTime = arrivalTime;
                            keep = true;
                        }
                    }
                }
                if (keep) keepTransfers.emplace_back(transfer);
            }

            std::stable_sort(keepTransfers.begin(), keepTransfers.end(), [](const Edge a, const Edge b) {
                return a > b;
            });
            for (const Edge transfer : keepTransfers) {
                keptTransfers.addEdge(fromVertex, generatedTransfers.get(ToVertex, transfer));
            }
        }
    }

    inline void reduceTransfers(const RouteId route) noexcept {
        for (const TripId trip : data.tripsOfRoute(route)) {
            reduceTransfers(trip);
        }
    }

    inline const SimpleDynamicGraph& getStopEventGraph() const noexcept {
        return keptTransfers;
    }

    inline SimpleDynamicGraph& getStopEventGraph() noexcept {
        return keptTransfers;
    }

private:
    inline std::vector<RouteTransfer> generateRouteTransfers(const RouteId fromRoute) const noexcept {
        std::vector<RouteTransfer> routeTransfers;
        const StopId* stops = data.raptorData.stopArrayOfRoute(fromRoute);
        for (StopIndex i(data.numberOfStopsInRoute(fromRoute) - 1); i > 0; i--) {
            const StopId fromStop = stops[i];
            for (const RAPTOR::RouteSegment& toSegment : data.raptorData.routesContainingStop(fromStop)) {
                if (toSegment.routeId == fromRoute && toSegment.stopIndex == i) continue;
                routeTransfers.emplace_back(toSegment.routeId, i, toSegment.stopIndex, 0);
            }
            for (const Edge edge : data.raptorData.transferGraph.edgesFrom(fromStop)) {
                const StopId toStop = StopId(data.raptorData.transferGraph.get(ToVertex, edge));
                const int transferTime = data.raptorData.transferGraph.get(TravelTime, edge);
                for (const RAPTOR::RouteSegment& toRouteSegment : data.raptorData.routesContainingStop(toStop)) {
                    routeTransfers.emplace_back(toRouteSegment.routeId, i, toRouteSegment.stopIndex, transferTime);
                }
            }
        }
        std::sort(routeTransfers.begin(), routeTransfers.end());
        return routeTransfers;
    }

    inline void findTransfers(const TripId fromTrip, const StopIndex fromIndex, const StopId toStop, const int toArrivalTime) noexcept  {
        const RouteId fromRoute = data.routeOfTrip[fromTrip];
        for (const RAPTOR::RouteSegment& toSegment : data.raptorData.routesContainingStop(toStop)) {
            const TripId toTrip = data.getEarliestTrip(toSegment, toArrivalTime);
            if (toTrip == noTripId) continue;
            if ((toSegment.routeId == fromRoute) && (toTrip >= fromTrip) && (toSegment.stopIndex >= fromIndex)) continue;
            if (isUTurn(fromTrip, fromIndex, toTrip, toSegment.stopIndex)) continue;
            const Vertex fromVertex = Vertex(data.getStopEventId(fromTrip, fromIndex));
            const Vertex toVertex = Vertex(data.getStopEventId(toTrip, toSegment.stopIndex));
            generatedTransfers.addEdge(fromVertex, toVertex);
        }
    }

    inline bool isUTurn(const TripId fromTrip, const StopIndex fromIndex, const TripId toTrip, const StopIndex toIndex) const noexcept {
        if (fromIndex < 2) return false;
        if (toIndex + 1 >= data.numberOfStopsInTrip(toTrip)) return false;
        if (data.getStop(fromTrip, StopIndex(fromIndex - 1)) != data.getStop(toTrip, StopIndex(toIndex + 1))) return false;
        if (data.getStopEvent(fromTrip, StopIndex(fromIndex - 1)).arrivalTime > data.getStopEvent(toTrip, StopIndex(toIndex + 1)).departureTime) return false;
        return true;
    }

private:
    const Data& data;

    SimpleDynamicGraph generatedTransfers;
    SimpleDynamicGraph keptTransfers;

    std::vector<StopLabel> labels;
    int timestamp;

};

inline void ComputeStopEventGraph(Data& data) noexcept {
    Progress progress(data.numberOfTrips());
    StopEventGraphBuilder builder(data);
    for (const TripId trip : data.trips()) {
        builder.generateFullTransfers(trip);
        builder.reduceTransfers(trip);
        progress++;
    }
    Graph::move(std::move(builder.getStopEventGraph()), data.stopEventGraph);
    data.stopEventGraph.sortEdges(ToVertex);
    progress.finished();
}

inline void ComputeStopEventGraph(Data& data, const int numberOfThreads, const int pinMultiplier = 1) noexcept {
    Progress progress(data.numberOfTrips());
    SimpleEdgeList stopEventGraph;
    stopEventGraph.addVertices(data.numberOfStopEvents());

    const int numCores = numberOfCores();

    omp_set_num_threads(numberOfThreads);
    #pragma omp parallel
    {
        int threadId = omp_get_thread_num();
        pinThreadToCoreId((threadId * pinMultiplier) % numCores);
        AssertMsg(omp_get_num_threads() == numberOfThreads, "Number of threads is " << omp_get_num_threads() << ", but should be " << numberOfThreads << "!");

        StopEventGraphBuilder builder(data);
        const size_t numberOfTrips = data.numberOfTrips();

        #pragma omp for schedule(dynamic,1)
        for (size_t i = 0; i < numberOfTrips; i++) {
            const TripId trip = TripId(i);
            builder.generateFullTransfers(trip);
            builder.reduceTransfers(trip);
            progress++;
        }

        #pragma omp critical
        {
            for (const auto [edge, from] : builder.getStopEventGraph().edgesWithFromVertex()) {
                stopEventGraph.addEdge(from, builder.getStopEventGraph().get(ToVertex, edge));
            }
        }
    }

    Graph::move(std::move(stopEventGraph), data.stopEventGraph);
    data.stopEventGraph.sortEdges(ToVertex);
    progress.finished();
}

inline void ComputeStopEventGraphRouteBased(Data& data) noexcept {
    Progress progress(data.numberOfRoutes());
    StopEventGraphBuilder builder(data);
    for (const RouteId route : data.routes()) {
        builder.generateRouteBasedTransfers(route);
        builder.reduceTransfers(route);
        progress++;
    }
    Graph::move(std::move(builder.getStopEventGraph()), data.stopEventGraph);
    data.stopEventGraph.sortEdges(ToVertex);
    progress.finished();
}

inline void ComputeStopEventGraphRouteBased(Data& data, const int numberOfThreads, const int pinMultiplier = 1) noexcept {
    Progress progress(data.numberOfRoutes());
    SimpleEdgeList stopEventGraph;
    stopEventGraph.addVertices(data.numberOfStopEvents());

    const int numCores = numberOfCores();

    omp_set_num_threads(numberOfThreads);
    #pragma omp parallel
    {
        int threadId = omp_get_thread_num();
        pinThreadToCoreId((threadId * pinMultiplier) % numCores);
        AssertMsg(omp_get_num_threads() == numberOfThreads, "Number of threads is " << omp_get_num_threads() << ", but should be " << numberOfThreads << "!");

        StopEventGraphBuilder builder(data);
        const size_t numberOfRoutes = data.numberOfRoutes();

        #pragma omp for schedule(dynamic,1)
        for (size_t i = 0; i < numberOfRoutes; i++) {
            const RouteId route = RouteId(i);
            builder.generateRouteBasedTransfers(route);
            builder.reduceTransfers(route);
            progress++;
        }

        #pragma omp critical
        {
            for (const auto [edge, from] : builder.getStopEventGraph().edgesWithFromVertex()) {
                stopEventGraph.addEdge(from, builder.getStopEventGraph().get(ToVertex, edge));
            }
        }
    }

    Graph::move(std::move(stopEventGraph), data.stopEventGraph);
    data.stopEventGraph.sortEdges(ToVertex);
    progress.finished();
}

}
