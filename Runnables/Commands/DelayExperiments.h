#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <random>

#include "../../Shell/Shell.h"

#include "../../DataStructures/Queries/Queries.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/TripBased/Data.h"
#include "../../DataStructures/TripBased/Delay.h"
#include "../../DataStructures/TripBased/DelayData.h"
#include "../../DataStructures/TripBased/DelayInfo.h"
#include "../../DataStructures/TripBased/DelayUpdateData.h"

#include "../../Algorithms/RAPTOR/DijkstraRAPTOR.h"
#include "../../Algorithms/RAPTOR/InitialTransfers.h"
#include "../../Algorithms/TripBased/Preprocessing/DelayUpdater.h"
#include "../../Algorithms/TripBased/Query/Query.h"

#include "../../Helpers/MultiThreading.h"

using namespace Shell;

struct QueryStatistics {
    QueryStatistics(const std::vector<VertexQuery>& queries, const std::vector<std::vector<RAPTOR::ArrivalLabel>>& exactResults, const std::vector<std::vector<RAPTOR::ArrivalLabel>>& algorithmResults) :
        totalQueries(0),
        totalJourneys(0),
        failedQueries(0),
        failedJourneys(0) {
        for (size_t i = 0; i < queries.size(); i++) {
            addQuery(queries[i], exactResults[i], algorithmResults[i]);
        }
        finalize();
    }

    inline void addQuery(const VertexQuery& query, const std::vector<RAPTOR::ArrivalLabel>& exactResults, const std::vector<RAPTOR::ArrivalLabel>& algorithmResults) noexcept {
        totalQueries++;
        totalJourneys += exactResults.size();
        if (Vector::equals(exactResults, algorithmResults)) return;
        failedQueries++;
        for (const RAPTOR::ArrivalLabel& exactResult : exactResults) {
            int bestArrivalTime = INFTY;
            for (const RAPTOR::ArrivalLabel& algorithmResult : algorithmResults) {
                if (algorithmResult.numberOfTrips > exactResult.numberOfTrips) break;
                bestArrivalTime = algorithmResult.arrivalTime;
            }
            if (bestArrivalTime != exactResult.arrivalTime) {
                failedJourneys++;
                if (bestArrivalTime != INFTY) {
                    const int exactTravelTime = exactResult.arrivalTime - query.departureTime;
                    detours.emplace_back((bestArrivalTime - exactResult.arrivalTime)/static_cast<double>(exactTravelTime));
                }
            }
        }
    }

    inline void finalize() noexcept {
        std::sort(detours.begin(), detours.end());
    }

    inline friend std::ostream& operator<<(std::ostream& out, const QueryStatistics& statistics) noexcept {
        out << "Queries: " << statistics.totalQueries << std::endl;
        out << "Exact journeys: " << statistics.totalJourneys << std::endl;
        out << "Failed queries: " << statistics.failedQueries << " (" << String::percent(statistics.failedQueries/static_cast<double>(statistics.totalQueries)) << ")" << std::endl;
        out << "Failed journeys: " << statistics.failedJourneys << " (" << String::percent(statistics.failedJourneys/static_cast<double>(statistics.totalJourneys)) << ")" << std::endl;
        if (!statistics.detours.empty()) {
            out << "Mean detour: " << String::percent(Vector::mean(statistics.detours)) << std::endl;
            out << "Median detour: " << String::percent(Vector::median(statistics.detours)) << std::endl;
            out << "95th perc. detour: " << String::percent(Vector::percentile(statistics.detours, 0.95)) << std::endl;
        }
        return out;
    }

    size_t totalQueries;
    size_t totalJourneys;
    size_t failedQueries;
    size_t failedJourneys;
    std::vector<double> detours;
};

struct Transfer {
    Transfer(const StopEventId from, const StopEventId to, const int travelTime) :
        from(from),
        to(to),
        travelTime(travelTime) {
    }

    StopEventId from;
    StopEventId to;
    int travelTime;

    inline friend std::ostream& operator<<(std::ostream& out, const Transfer& transfer) noexcept {
        return out << transfer.from << " -> " << transfer.to << " @ " << transfer.travelTime;
    }
};

struct JourneyData {
    JourneyData(const TripBased::Data& data, const Permutation& internalToOriginal, const RAPTOR::Journey& journey) :
        transfers(getTransfers(data, internalToOriginal, journey)),
        finalStopEvent(getFinalStopEvent(data, internalToOriginal, journey)),
        arrivalLabel(journey.back().arrivalTime, countTrips(journey)) {
    }

    JourneyData(const TripBased::DelayData& data, const TripBased::Data& delayedData, const Permutation& internalToOriginal, const RAPTOR::Journey& journey) :
        JourneyData(delayedData, internalToOriginal, journey) {
        if (finalStopEvent != noStopEvent) {
            arrivalLabel.arrivalTime -= data.arrivalDelay[finalStopEvent];
        }
    }

    JourneyData(const TripBased::Data& data, const RAPTOR::Journey& journey) :
        JourneyData(data, Permutation(Construct::Id, data.numberOfStopEvents()), journey) {
    }

    JourneyData(const TripBased::DelayData& data, const RAPTOR::Journey& journey) :
        JourneyData(data, data.data, Permutation(Construct::Id, data.data.numberOfStopEvents()), journey) {
    }

    inline bool isFeasible(const TripBased::DelayData& delayData) const noexcept {
        for (const Transfer& transfer : transfers) {
            if (!delayData.isTransferFeasible(transfer.from, transfer.to, transfer.travelTime)) return false;
        }
        return true;
    }

    inline std::vector<int> getTransferSlacks(const TripBased::Data& data) const noexcept {
        std::vector<int> slacks;
        for (const Transfer& transfer : transfers) {
            slacks.emplace_back(data.getTransferSlack(transfer.from, transfer.to, transfer.travelTime));
        }
        return slacks;
    }

    inline int getMinTransferSlack(const TripBased::Data& data) const noexcept {
        if (transfers.empty()) return 0;
        int minTransferSlack = INFTY;
        for (const Transfer& transfer : transfers) {
            minTransferSlack = std::min(minTransferSlack, data.getTransferSlack(transfer.from, transfer.to, transfer.travelTime));
        }
        return minTransferSlack;
    }

    inline int getMaxTransferSlack(const TripBased::Data& data) const noexcept {
        int maxTransferSlack = 0;
        for (const Transfer& transfer : transfers) {
            maxTransferSlack = std::max(maxTransferSlack, data.getTransferSlack(transfer.from, transfer.to, transfer.travelTime));
        }
        return maxTransferSlack;
    }

    inline int getMaxFailedTransferSlack(const TripBased::DelayData& delayData) const noexcept {
        int maxTransferSlack = 0;
        for (const Transfer& transfer : transfers) {
            if (delayData.isTransferFeasible(transfer.from, transfer.to, transfer.travelTime)) continue;
            maxTransferSlack = std::max(maxTransferSlack, delayData.getUndelayedTransferSlack(transfer.from, transfer.to, transfer.travelTime));
        }
        return maxTransferSlack;
    }

    inline RAPTOR::ArrivalLabel getDelayedArrivalLabel(const TripBased::DelayData& delayData) const noexcept {
        RAPTOR::ArrivalLabel result = arrivalLabel;
        if (finalStopEvent != noStopEvent) {
            result.arrivalTime += delayData.arrivalDelay[finalStopEvent];
        }
        return result;
    }

    inline static std::vector<Transfer> getTransfers(const TripBased::Data& data, const Permutation& internalToOriginal, const RAPTOR::Journey& journey) noexcept {
        std::vector<Transfer> transfers;
        for (size_t i = 0; i < journey.size() - 1; i++) {
            if (!journey[i].usesRoute) continue;
            size_t next = i+1;
            int travelTime = 0;
            if (i < journey.size() - 2 && !journey[i+1].usesRoute && journey[i+2].usesRoute) {
                next = i+2;
                travelTime = journey[i+1].arrivalTime - journey[i].arrivalTime;
            } else if (!journey[i+1].usesRoute) {
                continue;
            }
            const StopId fromStop = StopId(journey[i].to);
            const RouteId fromRoute = journey[i].routeId;
            const int fromTime = journey[i].arrivalTime;
            const StopId toStop = StopId(journey[next].from);
            const RouteId toRoute = journey[next].routeId;
            const int toTime = journey[next].departureTime;
            const StopEventId fromEvent = findStopEvent(data, internalToOriginal, fromStop, fromRoute, fromTime, false);
            const StopEventId toEvent = findStopEvent(data, internalToOriginal, toStop, toRoute, toTime, true);
            transfers.emplace_back(fromEvent, toEvent, travelTime);
        }
        return transfers;
    }

    inline static StopEventId getFinalStopEvent(const TripBased::Data& data, const Permutation& internalToOriginal, const RAPTOR::Journey& journey) noexcept {
        for (size_t i = journey.size() - 1; i != size_t(-1); i--) {
            if (!journey[i].usesRoute) continue;
            const StopId stop = StopId(journey[i].to);
            const RouteId route = journey[i].routeId;
            const int arrivalTime = journey[i].arrivalTime;
            return findStopEvent(data, internalToOriginal, stop, route, arrivalTime, false);
        }
        return noStopEvent;
    }

    inline static StopEventId findStopEvent(const TripBased::Data& data, const Permutation& internalToOriginal, const StopId stop, const RouteId route, const int time, const bool departure) noexcept {
        for (StopIndex stopIndex(0); stopIndex < data.numberOfStopsInRoute(route); stopIndex++) {
            if (stop != data.raptorData.stopArrayOfRoute(route)[stopIndex]) continue;
            for (const TripId trip : data.tripsOfRoute(route)) {
                const StopEventId stopEvent = data.getStopEventId(trip, stopIndex);
                if (departure) {
                    if (data.departureTime(stopEvent) == time) return StopEventId(internalToOriginal[stopEvent]);
                } else {
                    if (data.arrivalTime(stopEvent) == time) return StopEventId(internalToOriginal[stopEvent]);
                }
            }
        }
        return noStopEvent;
    }

    std::vector<Transfer> transfers;
    StopEventId finalStopEvent;
    RAPTOR::ArrivalLabel arrivalLabel;
};

inline std::vector<JourneyData> getJourneyData(const TripBased::DelayData& data, const TripBased::Data& delayedData, const Permutation& internalToOriginal, const std::vector<RAPTOR::Journey>& journeys) noexcept {
    std::vector<JourneyData> journeyData;
    for (const RAPTOR::Journey& journey : journeys) {
        journeyData.emplace_back(data, delayedData, internalToOriginal, journey);
    }
    return journeyData;
}

inline std::vector<JourneyData> getJourneyData(const TripBased::DelayData& data, const std::vector<RAPTOR::Journey>& journeys) noexcept {
    std::vector<JourneyData> journeyData;
    for (const RAPTOR::Journey& journey : journeys) {
        journeyData.emplace_back(data, journey);
    }
    return journeyData;
}

inline std::vector<JourneyData> getJourneyData(const TripBased::Data& data, const std::vector<RAPTOR::Journey>& journeys) noexcept {
    std::vector<JourneyData> journeyData;
    for (const RAPTOR::Journey& journey : journeys) {
        journeyData.emplace_back(data, journey);
    }
    return journeyData;
}

struct QueryFeasibilityStatistics {
    QueryFeasibilityStatistics() :
        infeasibleQueries(0),
        infeasibleJourneys(0),
        totalJourneys(0) {
    }

    inline void addQuery(const std::vector<JourneyData>& journeyData, const TripBased::DelayData& delayData) noexcept {
        algorithmResults.emplace_back();
        totalJourneys += journeyData.size();
        bool infeasible = false;
        for (const JourneyData& journey : journeyData) {
            if (journey.isFeasible(delayData)) {
                algorithmResults.back().emplace_back(journey.getDelayedArrivalLabel(delayData));
            } else {
                infeasibleJourneys++;
                maxTransferSlacks.emplace_back(journey.getMaxFailedTransferSlack(delayData));
                infeasible = true;
            }
        }
        if (infeasible) infeasibleQueries++;
    }

    inline void finalize() noexcept {
        std::sort(maxTransferSlacks.begin(), maxTransferSlacks.end());
    }

    inline friend std::ostream& operator<<(std::ostream& out, const QueryFeasibilityStatistics& statistics) noexcept {
        out << "Infeasible queries: " << statistics.infeasibleQueries << " (" << String::percent(statistics.infeasibleQueries/static_cast<double>(statistics.algorithmResults.size())) << ")" << std::endl;
        out << "Infeasible journeys: " << statistics.infeasibleJourneys << " (" << String::percent(statistics.infeasibleJourneys/static_cast<double>(statistics.totalJourneys)) << ")" << std::endl;
        if (statistics.infeasibleJourneys > 0) {
            out << "Mean max transfer slack: " << Vector::mean(statistics.maxTransferSlacks) << std::endl;
            out << "Median max transfer slack: " << Vector::median(statistics.maxTransferSlacks) << std::endl;
            out << "95th perc. max transfer slack: " << Vector::percentile(statistics.maxTransferSlacks, 0.95) << std::endl;
        }
        return out;
    }

    size_t infeasibleQueries;
    size_t infeasibleJourneys;
    size_t totalJourneys;
    std::vector<size_t> maxTransferSlacks;
    std::vector<std::vector<RAPTOR::ArrivalLabel>> algorithmResults;
};

class AnalyzeHeadwayDistribution : public ParameterizedCommand {

public:
    AnalyzeHeadwayDistribution(BasicShell& shell) :
        ParameterizedCommand(shell, "analyzeHeadwayDistribution", "Analyzes the headway distribution of routes in the given network.") {
        addParameter("RAPTOR data");
    }

    virtual void execute() noexcept {
        const RAPTOR::Data data(getParameter("RAPTOR data"));
        data.printInfo();

        size_t numRoutes = 0;
        std::vector<size_t> headwayLimits { 5, 10, 20, 30, 60, 120, 240, INFTY };
        std::vector<size_t> numRoutesPerHeadway(headwayLimits.size(), 0);
        for (const RouteId route : data.routes()) {
            size_t numTrips = 0;
            const int firstDepartureTime = data.firstTripOfRoute(route)->departureTime;
            int lastDepartureTime = firstDepartureTime;
            for (size_t trip = 0; trip < data.numberOfTripsInRoute(route); trip++) {
                const int tripDepartureTime = data.tripOfRoute(route, trip)->departureTime;
                if (tripDepartureTime > 24 * 60 * 60) break;
                lastDepartureTime = tripDepartureTime;
                numTrips++;
            }
            const int timespan = lastDepartureTime - firstDepartureTime;
            if (numTrips <= 1) continue;
            numRoutes++;
            const double headway =  static_cast<double>(timespan) / (numTrips - 1);
            for (size_t i = 0; i < headwayLimits.size(); i++) {
                if (headway <= headwayLimits[i] * 60) {
                    numRoutesPerHeadway[i]++;
                    break;
                }
            }
        }
        std::cout << "Routes with <= 1 trips: " << String::percent((data.numberOfRoutes() - numRoutes)/static_cast<double>(data.numberOfRoutes())) << std::endl;
        for (size_t i = 0; i < headwayLimits.size(); i++) {
            std::cout << "Headway <= " << headwayLimits[i] << " min: " << String::percent(numRoutesPerHeadway[i]/static_cast<double>(numRoutes)) << std::endl;
        }
    }
};

class AnalyzeTransferSlacks : public ParameterizedCommand {

public:
    AnalyzeTransferSlacks(BasicShell& shell) :
        ParameterizedCommand(shell, "analyzeTransferSlacks", "Analyzes the transfer slacks of optimal journeys for random queries.") {
        addParameter("Trip-Based data");
        addParameter("Bucket CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        const TripBased::Data data(getParameter("Trip-Based data"));
        data.printInfo();
        const CH::CH bucketCH(getParameter("Bucket CH data"));
        TripBased::Query<TripBased::AggregateProfiler> tripBased(data, bucketCH);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(bucketCH.numVertices(), n);

        std::vector<int> allTransferSlacks;
        std::vector<int> minTransferSlacks;
        std::vector<int> maxTransferSlacks;
        Progress progress(n);
        for (const VertexQuery& query : queries) {
            tripBased.run(query.source, query.departureTime, query.target);
            const std::vector<JourneyData> journeyData = getJourneyData(data, tripBased.getJourneys());
            for (const JourneyData& journey : journeyData) {
                allTransferSlacks += journey.getTransferSlacks(data);
                if (journey.transfers.empty()) continue;
                minTransferSlacks.emplace_back(journey.getMinTransferSlack(data));
                maxTransferSlacks.emplace_back(journey.getMaxTransferSlack(data));
            }
            progress++;
        }


        std::sort(allTransferSlacks.begin(), allTransferSlacks.end());
        std::sort(minTransferSlacks.begin(), minTransferSlacks.end());
        std::sort(maxTransferSlacks.begin(), maxTransferSlacks.end());
        std::cout << "All transfer slacks:" << std::endl;
        printSlacks(allTransferSlacks);
        std::cout << "Min transfer slacks:" << std::endl;
        printSlacks(minTransferSlacks);
        std::cout << "Max transfer slacks:" << std::endl;
        printSlacks(maxTransferSlacks);
    }

private:
    inline void printSlacks(std::vector<int>& slacks) const noexcept {
        std::sort(slacks.begin(), slacks.end());
        std::cout << "\tMin: " << Vector::min(slacks) << std::endl;
        std::cout << "\tMax: " << Vector::max(slacks) << std::endl;
        std::cout << "\tMean: " << Vector::mean(slacks) << std::endl;
        std::cout << "\tPercentiles:" << std::endl;
        for (double p = 0.05; p < 1; p += 0.05) {
            std::cout << "\t\t" << p << ": " << Vector::percentile(slacks, p) << std::endl;
        }
    }
};

class BuildFakeDelayData : public ParameterizedCommand {
public:
    BuildFakeDelayData(BasicShell& shell) :
        ParameterizedCommand(shell, "buildFakeDelayData", "Builds fake TripBased::DelayData with delay buffer 0 from TripBased::Data.") {
        addParameter("Trip-Based input file");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        TripBased::Data data(getParameter("Trip-Based input file"));
        TripBased::DelayData delayData(data);
        delayData.printInfo();
        delayData.serialize(getParameter("Output file"));
    }
};

class GenerateDelayScenario : public ParameterizedCommand {
public:
    GenerateDelayScenario(BasicShell& shell) :
        ParameterizedCommand(shell, "generateDelayScenario", "Generates a random delay scenario for the given network.") {
        addParameter("Trip-Based data");
        addParameter("Output file");
        addParameter("Start time", "00:00:00");
        addParameter("End time", "24:00:00");
        addParameter("Seed", "42");
    }

    virtual void execute() noexcept {
        const TripBased::Data data(getParameter("Trip-Based data"));
        data.printInfo();
        const int startTime = String::parseSeconds(getParameter("Start time"));
        const int endTime = String::parseSeconds(getParameter("End time"));
        const int seed = getParameter<int>("Seed");
        const TripBased::DelayScenario delayScenario(data, startTime, endTime, seed);
        delayScenario.serialize(getParameter("Output file"));
    }
};

class ValidateDelayULTRATripBased : public ParameterizedCommand {

public:
    ValidateDelayULTRATripBased(BasicShell& shell) :
        ParameterizedCommand(shell, "validateDelayULTRATripBased", "Validates journeys computed by ULTRA-TripBased with delay-tolerant shortcuts by comparing to Dijkstra-RAPTOR on random queries.") {
        addParameter("Dijkstra RAPTOR data");
        addParameter("Trip-Based data");
        addParameter("Core CH data");
        addParameter("Bucket CH data");
        addParameter("Number of queries");
        addParameter("Delay scenario file");
        addParameter("Ignore max delay?");
        addParameter("Allow departure delays?");
        addParameter("Allow overtaking?");
    }

    virtual void execute() noexcept {
        RAPTOR::Data dijkstraRaptorData(getParameter("Dijkstra RAPTOR data"));
        dijkstraRaptorData.useImplicitDepartureBufferTimes();
        dijkstraRaptorData.printInfo();
        TripBased::DelayData delayData(getParameter("Trip-Based data"));
        delayData.printInfo();
        CH::CH coreCH(getParameter("Core CH data"));
        CH::CH bucketCH(getParameter("Bucket CH data"));

        const TripBased::DelayScenario delayScenario(delayData.data, getParameter("Delay scenario file"));
        const bool ignoreMaxDelay = getParameter<bool>("Ignore max delay?");
        const bool allowDepartureDelays = getParameter<bool>("Allow departure delays?");
        const bool allowOvertaking = getParameter<bool>("Allow overtaking?");
        const TripBased::DelayQueryData queryData = delayData.applyDelayScenario(delayScenario.getAllIncidents(), allowDepartureDelays, ignoreMaxDelay, allowOvertaking);

        RAPTOR::Data delayedRaptorData = queryData.tripData.raptorData;
        Graph::move(std::move(dijkstraRaptorData.transferGraph), delayedRaptorData.transferGraph);
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler> dijkstraRaptor(delayedRaptorData, coreCH);
        TripBased::Query<TripBased::AggregateProfiler> ultraTripBased(queryData.tripData, bucketCH);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(bucketCH.numVertices(), n);

        Progress progress(n);
        for (size_t i = 0; i < n; i++) {
            dijkstraRaptor.run(queries[i].source, queries[i].departureTime, queries[i].target);
            ultraTripBased.run(queries[i].source, queries[i].departureTime, queries[i].target);
            const std::vector<RAPTOR::ArrivalLabel> dijkstraArrivals = dijkstraRaptor.getArrivals();
            const std::vector<RAPTOR::ArrivalLabel> ultraArrivals = ultraTripBased.getArrivals();
            if (!Vector::equals(dijkstraArrivals, ultraArrivals)) {
                std::cout << "Query " << i << ": " << queries[i] << std::endl;
                std::cout << "Dijkstra arrivals:" << std::endl;
                std::cout << dijkstraArrivals << std::endl;
                std::cout << "ULTRA arrivals:" << std::endl;
                std::cout << ultraArrivals << std::endl;
                std::cout << "Dijkstra journeys:" << std::endl;
                for (const RAPTOR::Journey& journey : dijkstraRaptor.getJourneys()) {
                    std::cout << journey << std::endl;
                    std::cout << "Transfers:" << std::endl;
                    const JourneyData journeyData(queryData.tripData, journey);
                    for (const Transfer& transfer : journeyData.transfers) {
                        std::cout << transfer;
                        std::cout << ". In data? " << queryData.tripData.stopEventGraph.hasEdge(Vertex(transfer.from), Vertex(transfer.to));
                        const StopEventId originalFrom = StopEventId(queryData.internalToOriginal[transfer.from]);
                        const StopEventId originalTo = StopEventId(queryData.internalToOriginal[transfer.to]);
                        std::cout << ". Original: " << Transfer(originalFrom, originalTo, transfer.travelTime);
                        const Edge originalEdge = delayData.stopEventGraph.findEdge(Vertex(originalFrom), Vertex(originalTo));
                        std::cout << ". In original data? " << (originalEdge != noEdge);
                        if (originalEdge != noEdge) {
                            std::cout << ". Min origin delay: " << delayData.stopEventGraph.get(MinOriginDelay, originalEdge);
                            std::cout << ". Max origin delay: " << delayData.stopEventGraph.get(MaxOriginDelay, originalEdge);
                        }
                        std::cout << ". From delay: " << delayData.arrivalDelay[originalFrom];
                        std::cout << ". To delay: " << delayData.departureDelay[originalTo];
                        std::cout << std::endl;
                    }
                    std::cout << std::endl;
                }
                std::cout << "ULTRA journeys:" << std::endl;
                std::cout << ultraTripBased.getJourneys() << std::endl;
                return;
            }
            progress++;
        }
        dijkstraRaptor.getProfiler().printStatistics();
        ultraTripBased.getProfiler().printStatistics();
    }
};

class RunDelayUpdatesWithReplacement : public ParameterizedCommand {

public:
    RunDelayUpdatesWithReplacement(BasicShell& shell) :
        ParameterizedCommand(shell, "runDelayUpdatesWithReplacement", "Runs Delay-ULTRA update phase with replacement search for the given delay scenario.") {
        addParameter("Trip-Based data");
        addParameter("Bucket CH data");
        addParameter("Delay scenario file");
        addParameter("Allow departure delays?");
        addParameter("Replace delayed targets?");
        addParameter("Statistics file");
        addParameter("Log file");
        addParameter("Target slack");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }
    virtual void execute() noexcept {
        TripBased::DelayData delayData(getParameter("Trip-Based data"));
        delayData.printInfo();
        const CH::CH bucketCH(getParameter("Bucket CH data"));
        const RAPTOR::BucketCHInitialTransfers initialTransfers(bucketCH.forward, bucketCH.backward, delayData.data.numberOfStops(), Weight);

        const int numberOfThreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Pin multiplier");
        const ThreadPinning threadPinning(numberOfThreads, pinMultiplier);
        const int targetSlack = getParameter<int>("Target slack");
        TripBased::DelayUpdaterWithReplacement delayUpdater(delayData, initialTransfers, threadPinning, targetSlack);

        const TripBased::DelayScenario delayScenario(delayData.data, getParameter("Delay scenario file"));
        const bool allowDepartureDelays = getParameter<bool>("Allow departure delays?");
        const bool replaceDelayedTargets = getParameter<bool>("Replace delayed targets?");
        delayUpdater.preRunUpdate(delayScenario.preStartUpdates, allowDepartureDelays, replaceDelayedTargets);
        delayUpdater.runUpdates(delayScenario.updates, allowDepartureDelays, replaceDelayedTargets);
        delayUpdater.writeStatistics(getParameter("Statistics file"));
        delayUpdater.writeLog(getParameter("Log file"));
    }

private:
    inline int getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }
};

class RunDelayUpdatesWithoutReplacement : public ParameterizedCommand {

public:
    RunDelayUpdatesWithoutReplacement(BasicShell& shell) :
        ParameterizedCommand(shell, "runDelayUpdatesWithoutReplacement", "Runs Delay-ULTRA update phase without replacement search for the given delay scenario.") {
        addParameter("Trip-Based data");
        addParameter("Delay scenario file");
        addParameter("Allow departure delays?");
        addParameter("Statistics file");
        addParameter("Log file");
    }
    virtual void execute() noexcept {
        TripBased::DelayData delayData(getParameter("Trip-Based data"));
        delayData.printInfo();
        TripBased::DelayUpdaterWithoutReplacement delayUpdater(delayData);
        const TripBased::DelayScenario delayScenario(delayData.data, getParameter("Delay scenario file"));
        const bool allowDepartureDelays = getParameter<bool>("Allow departure delays?");
        delayUpdater.preRunUpdate(delayScenario.preStartUpdates, allowDepartureDelays);
        delayUpdater.runUpdates(delayScenario.updates, allowDepartureDelays);
        delayUpdater.writeStatistics(getParameter("Statistics file"));
        delayUpdater.writeLog(getParameter("Log file"));
    }
};

struct QueryInputData {
    QueryInputData() {}

    QueryInputData(const std::string& fileName) {
        deserialize(fileName);
    }

    inline void sortQueries() noexcept {
        Order queryOrder(Construct::Id, failedQueries.size());
        std::sort(queryOrder.begin(), queryOrder.end(), [&](const size_t a, const size_t b) {
            return failedQueries[a].departureTime < failedQueries[b].departureTime;
        });
        queryOrder.order(failedQueries);
        queryOrder.order(failedQueryResults);
    }

    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, allowDepartureDelays, failedQueries, failedQueryResults, groupedDelaysByQuery);
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, allowDepartureDelays, failedQueries, failedQueryResults, groupedDelaysByQuery);
    }

    bool allowDepartureDelays;
    std::vector<VertexQuery> failedQueries;
    std::vector<std::vector<RAPTOR::ArrivalLabel>> failedQueryResults;
    std::vector<std::vector<TripBased::DelayIncident>> groupedDelaysByQuery;
};

class GenerateDelayQueries : public ParameterizedCommand {

public:
    GenerateDelayQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "generateDelayQueries", "Generates queries for which an algorithm without delay information returns incorrect results.") {
        addParameter("Dijkstra RAPTOR data");
        addParameter("Core CH data");
        addParameter("Number of queries");
        addParameter("Delay scenario file");
        addParameter("Allow departure delays?");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        RAPTOR::Data dijkstraRaptorData(getParameter("Dijkstra RAPTOR data"));
        dijkstraRaptorData.useImplicitDepartureBufferTimes();
        dijkstraRaptorData.printInfo();
        TripBased::DelayData originalDelayData(dijkstraRaptorData, INFTY, INFTY);
        const CH::CH coreCH(getParameter("Core CH data"));
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler> undelayedRaptor(dijkstraRaptorData, coreCH);

        const TripBased::DelayScenario delayScenario(originalDelayData.data, getParameter("Delay scenario file"));
        QueryInputData queryInputData;
        queryInputData.allowDepartureDelays = getParameter<bool>("Allow departure delays?");
        std::vector<VertexQuery> allQueries;
        std::vector<std::vector<RAPTOR::ArrivalLabel>> delayedResults;
        QueryFeasibilityStatistics feasibilityStatistics;

        std::mt19937 randomGenerator(42);
        std::uniform_int_distribution<> vertexDistribution(0, coreCH.numVertices() - 1);
        std::uniform_int_distribution<> timeDistribution(delayScenario.startTime, delayScenario.endTime - 1);

        const size_t n = getParameter<size_t>("Number of queries");
        Progress progress(n);
        while (queryInputData.failedQueries.size() < n) {
            const VertexQuery query(Vertex(vertexDistribution(randomGenerator)), Vertex(vertexDistribution(randomGenerator)), timeDistribution(randomGenerator));
            allQueries.emplace_back(query);
            undelayedRaptor.run(query.source, query.departureTime, query.target);
            const std::vector<JourneyData> undelayedData = getJourneyData(originalDelayData, undelayedRaptor.getJourneys());

            TripBased::DelayData delayData = originalDelayData;
            const std::vector<TripBased::DelayIncident> queryDelayScenario = delayScenario.getDelayIncidentsUntil(query);
            delayData.applyDelays(queryDelayScenario, true, queryInputData.allowDepartureDelays);
            feasibilityStatistics.addQuery(undelayedData, delayData);

            const RAPTOR::Data delayedRaptorData = getDelayedRaptorData(delayData, dijkstraRaptorData);
            RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler> delayedRaptor(delayedRaptorData, coreCH);
            delayedRaptor.run(query.source, query.departureTime, query.target);
            delayedResults.emplace_back(delayedRaptor.getArrivals());

            if (!Vector::equals(delayedResults.back(), feasibilityStatistics.algorithmResults.back())) {
                queryInputData.failedQueries.emplace_back(query);
                queryInputData.failedQueryResults.emplace_back(delayedResults.back());
                progress++;
            }
        }
        progress.finished();

        feasibilityStatistics.finalize();
        const QueryStatistics statistics(allQueries, delayedResults, feasibilityStatistics.algorithmResults);
        std::cout << statistics << feasibilityStatistics << std::endl;

        queryInputData.sortQueries();
        queryInputData.groupedDelaysByQuery = delayScenario.groupDelayIncidentsByQuery(queryInputData.failedQueries);
        queryInputData.serialize(getParameter("Output file"));
    }

private:
    inline RAPTOR::Data getDelayedRaptorData(const TripBased::DelayData& delayData, const RAPTOR::Data& raptorData) const noexcept {
        RAPTOR::Data delayedRaptorData = raptorData;
        for (StopEventId event(0); event < delayedRaptorData.numberOfStopEvents(); event++) {
            delayedRaptorData.stopEvents[event].arrivalTime += delayData.arrivalDelay[event];
            delayedRaptorData.stopEvents[event].departureTime += delayData.departureDelay[event];
        }
        delayedRaptorData.rebuildRoutes();
        return delayedRaptorData;
    }
};

class MeasureDelayULTRAQueryCoverage : public ParameterizedCommand {

public:
    MeasureDelayULTRAQueryCoverage(BasicShell& shell) :
        ParameterizedCommand(shell, "measureDelayULTRAQueryCoverage", "Evaluates query coverage of Delay-ULTRA-TB.") {
        addParameter("Trip-Based data");
        addParameter("Bucket CH data");
        addParameter("Query input data");
        addParameter("Update log file");
    }

    virtual void execute() noexcept {
        TripBased::DelayData delayData(getParameter("Trip-Based data"));
        delayData.printInfo();
        const CH::CH bucketCH(getParameter("Bucket CH data"));
        const RAPTOR::BucketCHInitialTransfers initialTransfers(bucketCH.forward, bucketCH.backward, delayData.data.numberOfStops(), Weight);

        const QueryInputData queryInputData(getParameter("Query input data"));
        const std::vector<VertexQuery>& queries = queryInputData.failedQueries;

        TripBased::DelayData updateDelayData = delayData;
        TripBased::DelayUpdateSimulator delayUpdater(updateDelayData, getParameter("Update log file"));
        QueryFeasibilityStatistics feasibilityStatistics;
        Progress progress(queries.size());
        for (size_t i = 0; i < queries.size(); i++) {
            delayUpdater.applyUpdatesUntil(queries[i].departureTime, queryInputData.allowDepartureDelays);
            delayData.applyDelays(queryInputData.groupedDelaysByQuery[i], true, queryInputData.allowDepartureDelays);
            const TripBased::DelayQueryData& queryData = delayUpdater.getQueryData();
            TripBased::Query<TripBased::AggregateProfiler> algorithm(queryData.tripData, initialTransfers);
            algorithm.run(queries[i].source, queries[i].departureTime, queries[i].target);
            const std::vector<JourneyData> journeyData = getJourneyData(delayUpdater.getDelayData(), queryData.tripData, queryData.internalToOriginal, algorithm.getJourneys());
            feasibilityStatistics.addQuery(journeyData, delayData);
            progress++;
        }

        feasibilityStatistics.finalize();
        const QueryStatistics statistics(queries, queryInputData.failedQueryResults, feasibilityStatistics.algorithmResults);
        std::cout << statistics << feasibilityStatistics << std::endl;
    }
};

class MeasureHypotheticalDelayULTRAQueryCoverage : public ParameterizedCommand {

public:
    MeasureHypotheticalDelayULTRAQueryCoverage(BasicShell& shell) :
        ParameterizedCommand(shell, "measureHypotheticalDelayULTRAQueryCoverage", "Evaluates query coverage of Delay-ULTRA-TB, assuming that replacement shortcuts are known in advance and updates are incorporated instantly.") {
        addParameter("Trip-Based data");
        addParameter("Bucket CH data");
        addParameter("Query input data");
        addParameter("Update log file");
    }

    virtual void execute() noexcept {
        TripBased::DelayData delayData(getParameter("Trip-Based data"));
        delayData.printInfo();
        const CH::CH bucketCH(getParameter("Bucket CH data"));
        const RAPTOR::BucketCHInitialTransfers initialTransfers(bucketCH.forward, bucketCH.backward, delayData.data.numberOfStops(), Weight);

        const QueryInputData queryInputData(getParameter("Query input data"));
        const std::vector<VertexQuery>& queries = queryInputData.failedQueries;

        TripBased::applyAllReplacementShortcutsFromUpdateLog(delayData, getParameter("Update log file"));
        TripBased::DelayQueryData queryData = delayData.createQueryData();

        QueryFeasibilityStatistics feasibilityStatistics;
        Progress progress(queries.size());
        for (size_t i = 0; i < queries.size(); i++) {
            delayData.applyDelays(queryInputData.groupedDelaysByQuery[i], true, queryInputData.allowDepartureDelays);
            delayData.refreshQueryData(queryData);
            TripBased::Query<TripBased::AggregateProfiler> algorithm(queryData.tripData, initialTransfers);
            algorithm.run(queries[i].source, queries[i].departureTime, queries[i].target);
            const std::vector<JourneyData> journeyData = getJourneyData(delayData, queryData.tripData, queryData.internalToOriginal, algorithm.getJourneys());
            feasibilityStatistics.addQuery(journeyData, delayData);
            progress++;
        }

        feasibilityStatistics.finalize();
        const QueryStatistics statistics(queries, queryInputData.failedQueryResults, feasibilityStatistics.algorithmResults);
        std::cout << statistics << feasibilityStatistics << std::endl;
    }
};

class MeasureDelayULTRAQueryPerformance : public ParameterizedCommand {

public:
    MeasureDelayULTRAQueryPerformance(BasicShell& shell) :
        ParameterizedCommand(shell, "measureDelayULTRAQueryPerformance", "Measures query performance of ULTRA-TB with delay shortcuts and replacement search.") {
        addParameter("Dijkstra RAPTOR data");
        addParameter("Trip-Based data");
        addParameter("Core CH data");
        addParameter("Bucket CH data");
        addParameter("Update log file");
        addParameter("Number of queries");
        addParameter("Start time", "14:00:00");
        addParameter("End time", "15:00:00");
    }

    virtual void execute() noexcept {
        RAPTOR::Data dijkstraRaptorData(getParameter("Dijkstra RAPTOR data"));
        dijkstraRaptorData.useImplicitDepartureBufferTimes();
        dijkstraRaptorData.printInfo();
        TripBased::DelayData delayData(getParameter("Trip-Based data"));
        delayData.printInfo();
        const CH::CH coreCH(getParameter("Core CH data"));
        const CH::CH bucketCH(getParameter("Bucket CH data"));

        const int startTime = String::parseSeconds(getParameter("Start time"));
        const int endTime = String::parseSeconds(getParameter("End time"));
        TripBased::DelayUpdateSimulator delayUpdater(delayData, getParameter("Update log file"));
        delayUpdater.applyUpdatesUntil(startTime, true);
        const TripBased::DelayQueryData& queryData = delayUpdater.getQueryData();

        RAPTOR::Data delayedRaptorData = queryData.tripData.raptorData;
        Graph::move(std::move(dijkstraRaptorData.transferGraph), delayedRaptorData.transferGraph);
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler> dijkstraRaptor(delayedRaptorData, coreCH);
        TripBased::Query<TripBased::AggregateProfiler> ultraTripBased(queryData.tripData, bucketCH);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(bucketCH.numVertices(), n, startTime, endTime);
        const std::vector<std::vector<RAPTOR::ArrivalLabel>> dijkstraResults = runQueries(queries, dijkstraRaptor);
        const std::vector<std::vector<RAPTOR::ArrivalLabel>> ultraResults = runQueries(queries, ultraTripBased);
        const QueryStatistics statistics(queries, dijkstraResults, ultraResults);
        std::cout << statistics << std::endl;
    }

private:
    template<typename ALGORITHM>
    inline std::vector<std::vector<RAPTOR::ArrivalLabel>> runQueries(const std::vector<VertexQuery>& queries, ALGORITHM& algorithm) noexcept {
        Progress progress(queries.size());
        std::vector<std::vector<RAPTOR::ArrivalLabel>> results;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            results.emplace_back(algorithm.getArrivals());
            progress++;
        }
        algorithm.getProfiler().printStatistics();
        return results;
    }
};
