#pragma once

#include <string>
#include <vector>
#include <iostream>

#include "../../Shell/Shell.h"
using namespace Shell;

#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/CH/Preprocessing/BidirectionalWitnessSearch.h"
#include "../../Algorithms/CH/Preprocessing/CHBuilder.h"
#include "../../Algorithms/CH/Preprocessing/KeyFunction.h"
#include "../../Algorithms/CH/Preprocessing/StopCriterion.h"
#include "../../Algorithms/CSA/OneToAllDijkstraCSA.h"
#include "../../Algorithms/CSA/UPCSA.h"
#include "../../Algorithms/RAPTOR/InitialTransfers.h"
#include "../../Algorithms/RAPTOR/OneToAllDijkstraRAPTOR.h"
#include "../../Algorithms/RAPTOR/UPRAPTOR.h"
#include "../../Algorithms/TripBased/Query/UPQuery.h"

#include "../../DataStructures/Queries/Queries.h"
#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/TripBased/Data.h"

#include "../../Helpers/String/String.h"

class RunOneToAllDijkstraCSAQueriesToVertices : public ParameterizedCommand {

public:
    RunOneToAllDijkstraCSAQueriesToVertices(BasicShell& shell) :
        ParameterizedCommand(shell, "runOneToAllDijkstraCSAQueriesToVertices", "Runs the given number of random Dijkstra-CSA queries to all vertices.") {
        addParameter("CSA data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData(getParameter("CSA data"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        CSA::OneToAllDijkstraCSA<CSA::TransferGraph, true, CSA::AggregateProfiler> algorithm(csaData);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<OneToAllQuery> queries = generateRandomOneToAllQueries(csaData.transferGraph.numVertices(), n);

        for (const OneToAllQuery& query : queries) {
            algorithm.run(query.source, query.departureTime);
        }

        algorithm.getProfiler().printStatistics();
    }
};

class RunOneToManyDijkstraCSAQueriesToStops : public ParameterizedCommand {

public:
    RunOneToManyDijkstraCSAQueriesToStops(BasicShell& shell) :
        ParameterizedCommand(shell, "runOneToManyDijkstraCSAQueriesToStops", "Runs the given number of random Dijkstra-CSA queries to all stops.") {
        addParameter("CSA data");
        addParameter("Upward graph");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData(getParameter("CSA data"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        CHGraph upwardGraph(getParameter("Upward graph"));
        CSA::OneToAllDijkstraCSA<CHGraph, true, CSA::AggregateProfiler> algorithm(csaData, upwardGraph, Weight);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<OneToAllQuery> queries = generateRandomOneToAllQueries(csaData.transferGraph.numVertices(), n);

        for (const OneToAllQuery& query : queries) {
            algorithm.run(query.source, query.departureTime);
        }

        algorithm.getProfiler().printStatistics();
    }
};

template<typename DATA>
inline IndexedSet<false, Vertex> getTargetSet(const DATA& data, const size_t numVertices, const bool vertices) noexcept {
    if (vertices) {
        return IndexedSet<false, Vertex>(Construct::Complete, numVertices);
    } else {
        IndexedSet<false, Vertex> targetSet(numVertices);
        for (const StopId stop : data.stops()) {
            targetSet.insert(stop);
        }
        return targetSet;
    }
}

class RunUPCSAQueries : public ParameterizedCommand {

public:
    RunUPCSAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runUPCSAQueries", "Runs the given number of random UP-CSA queries.") {
        addParameter("CSA data");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Initial transfers", { "Bucket", "PHAST" });
        addParameter("Final transfers", { "Bucket", "PHAST" });
        addParameter("Reorder network?");
        addParameter("Order", { "DFS", "Level" });
        addParameter("Targets", { "Vertices", "Stops" });
    }

    virtual void execute() noexcept {
        if (getParameter("Initial transfers") == "Bucket") {
            chooseFinalTransfers<true>();
        } else {
            chooseFinalTransfers<false>();
        }
    }

private:
    template<bool USE_STOP_BUCKETS>
    inline void chooseFinalTransfers() const noexcept {
        if (getParameter("Final transfers") == "Bucket") {
            run<USE_STOP_BUCKETS, true>();
        } else {
            run<USE_STOP_BUCKETS, false>();
        }
    }

    template<bool USE_STOP_BUCKETS, bool USE_TARGET_BUCKETS>
    inline void run() const noexcept {
        CSA::Data csaData(getParameter("CSA data"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        CH::CH ch(getParameter("CH data"));

        IndexedSet<false, Vertex> targetSet = getTargetSet(csaData, ch.numVertices(), getParameter("Targets") == "Vertices");

        using UPCSA = CSA::UPCSA<USE_STOP_BUCKETS, USE_TARGET_BUCKETS, true, CSA::AggregateProfiler>;
        Timer timer;
        const bool reorder = getParameter<bool>("Reorder network?");
        UPCSA algorithm(csaData, ch, targetSet, reorder, getParameter("Order") == "DFS");
        const double buildTime = timer.elapsedMicroseconds();

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<OneToAllQuery> queries = generateRandomOneToAllQueries(ch.numVertices(), n);

        for (const OneToAllQuery& query : queries) {
            algorithm.run(query.source, query.departureTime);
        }

        std::cout << "Setup:" << std::endl;
        std::cout << "\tTime: " << String::musToString(buildTime) << std::endl;
        std::cout << "\tUpward sweep graph vertices: " << String::prettyInt(algorithm.getUpwardSweepGraphVertices()) << std::endl;
        std::cout << "\tUpward sweep graph edges: " << String::prettyInt(algorithm.getUpwardSweepGraphEdges()) << std::endl;
        std::cout << "\tStop graph vertices: " << String::prettyInt(algorithm.getStopGraphVertices()) << std::endl;
        std::cout << "\tStop graph edges: " << String::prettyInt(algorithm.getStopGraphEdges()) << std::endl;
        std::cout << "\tTarget graph vertices: " << String::prettyInt(algorithm.getTargetGraphVertices()) << std::endl;
        std::cout << "\tTarget graph edges: " << String::prettyInt(algorithm.getTargetGraphEdges()) << std::endl;
        std::cout << std::endl;

        std::cout << "Query statistics:" << std::endl;
        algorithm.getProfiler().printStatistics();
    }
};

class RunOneToAllDijkstraRAPTORQueriesToVertices : public ParameterizedCommand {

public:
    RunOneToAllDijkstraRAPTORQueriesToVertices(BasicShell& shell) :
        ParameterizedCommand(shell, "runOneToAllDijkstraRAPTORQueriesToVertices", "Runs the given number of random Dijkstra-RAPTOR queries to all vertices.") {
        addParameter("RAPTOR data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData(getParameter("RAPTOR data"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        RAPTOR::Data reverseRaptorData = raptorData.reverseNetwork();
        RAPTOR::OneToAllDijkstraRAPTOR<RAPTOR::DijkstraInitialTransfers, RAPTOR::AggregateProfiler> algorithm(raptorData, raptorData.transferGraph, reverseRaptorData.transferGraph);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<OneToAllQuery> queries = generateRandomOneToAllQueries(raptorData.transferGraph.numVertices(), n);

        for (const OneToAllQuery& query : queries) {
            algorithm.run(query.source, query.departureTime);
        }
        algorithm.getProfiler().printStatistics();
    }
};

class RunOneToManyDijkstraRAPTORQueriesToStops : public ParameterizedCommand {

public:
    RunOneToManyDijkstraRAPTORQueriesToStops(BasicShell& shell) :
        ParameterizedCommand(shell, "runOneToManyDijkstraRAPTORQueriesToStops", "Runs the given number of random Dijkstra-RAPTOR queries to all stops.") {
        addParameter("RAPTOR data");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData(getParameter("RAPTOR data"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::OneToAllDijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler> algorithm(raptorData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<OneToAllQuery> queries = generateRandomOneToAllQueries(raptorData.transferGraph.numVertices(), n);

        for (const OneToAllQuery& query : queries) {
            algorithm.run(query.source, query.departureTime);
        }
        algorithm.getProfiler().printStatistics();
    }
};

class RunUPRAPTORQueries : public ParameterizedCommand {

public:
    RunUPRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runUPRAPTORQueries", "Runs the given number of random UP-RAPTOR queries.") {
        addParameter("RAPTOR data");
        addParameter("Dijkstra graph");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Reorder network?");
        addParameter("Order", { "DFS", "Level" });
        addParameter("Grouped rounds");
        addParameter("Targets", { "Vertices", "Stops" });
    }

    virtual void execute() noexcept {
        switch(getParameter<size_t>("Grouped rounds")) {
            case 0:
                run<0>();
                break;
            case 4:
                run<4>();
                break;
            case 6:
                run<6>();
                break;
            case 8:
                run<8>();
                break;
        }
    }

private:
    template<size_t GROUPED_ROUNDS>
    inline void run() const noexcept {
        RAPTOR::Data raptorData(getParameter("RAPTOR data"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        TransferGraph dijkstraGraph(getParameter("Dijkstra graph"));
        CH::CH ch(getParameter("CH data"));

        IndexedSet<false, Vertex> targetSet = getTargetSet(raptorData, ch.numVertices(), getParameter("Targets") == "Vertices");

        using UPRAPTOR = RAPTOR::UPRAPTOR<GROUPED_ROUNDS, RAPTOR::AggregateProfiler>;
        const bool reorder = getParameter<bool>("Reorder network?");
        Timer timer;
        UPRAPTOR algorithm(raptorData, dijkstraGraph, ch, targetSet, reorder, getParameter("Order") == "DFS");
        const double buildTime = timer.elapsedMicroseconds();

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<OneToAllQuery> queries = generateRandomOneToAllQueries(ch.numVertices(), n);

        for (const OneToAllQuery& query : queries) {
            algorithm.run(query.source, query.departureTime);
        }

        std::cout << "Setup:" << std::endl;
        std::cout << "\tTime: " << String::musToString(buildTime) << std::endl;
        std::cout << "\tUpward sweep graph vertices: " << String::prettyInt(algorithm.getUpwardSweepGraphVertices()) << std::endl;
        std::cout << "\tUpward sweep graph edges: " << String::prettyInt(algorithm.getUpwardSweepGraphEdges()) << std::endl;
        std::cout << "\tStop graph vertices: " << String::prettyInt(algorithm.getStopGraphVertices()) << std::endl;
        std::cout << "\tStop graph edges: " << String::prettyInt(algorithm.getStopGraphEdges()) << std::endl;
        std::cout << "\tTarget graph vertices: " << String::prettyInt(algorithm.getTargetGraphVertices()) << std::endl;
        std::cout << "\tTarget graph edges: " << String::prettyInt(algorithm.getTargetGraphEdges()) << std::endl;
        std::cout << std::endl;

        std::cout << "Query statistics:" << std::endl;
        algorithm.getProfiler().printStatistics();
    }
};

class RunUPTBQueries : public ParameterizedCommand {

public:
    RunUPTBQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runUPTBQueries", "Runs the given number of random UP-TB queries.") {
        addParameter("Trip-Based data");
        addParameter("Dijkstra graph");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Reorder network?");
        addParameter("Order", { "DFS", "Level" });
        addParameter("Grouped rounds");
        addParameter("Targets", { "Vertices", "Stops" });
    }

    virtual void execute() noexcept {
        switch(getParameter<size_t>("Grouped rounds")) {
            case 0:
                run<0>();
                break;
            case 4:
                run<4>();
                break;
            case 6:
                run<6>();
                break;
            case 8:
                run<8>();
                break;
        }
    }

private:
    template<size_t GROUPED_ROUNDS>
    inline void run() const noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based data"));
        tripBasedData.printInfo();
        TransferGraph dijkstraGraph(getParameter("Dijkstra graph"));
        CH::CH ch(getParameter("CH data"));

        IndexedSet<false, Vertex> targetSet = getTargetSet(tripBasedData.raptorData, ch.numVertices(), getParameter("Targets") == "Vertices");

        using UPTB = TripBased::UPQuery<GROUPED_ROUNDS, TripBased::AggregateProfiler>;
        const bool reorder = getParameter<bool>("Reorder network?");
        Timer timer;
        UPTB algorithm(tripBasedData, dijkstraGraph, ch, targetSet, reorder);
        const double buildTime = timer.elapsedMicroseconds();

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<OneToAllQuery> queries = generateRandomOneToAllQueries(ch.numVertices(), n);

        for (const OneToAllQuery& query : queries) {
            algorithm.run(query.source, query.departureTime);
        }

        std::cout << "Setup:" << std::endl;
        std::cout << "\tTime: " << String::musToString(buildTime) << std::endl;
        std::cout << "\tUpward sweep graph vertices: " << String::prettyInt(algorithm.getUpwardSweepGraphVertices()) << std::endl;
        std::cout << "\tUpward sweep graph edges: " << String::prettyInt(algorithm.getUpwardSweepGraphEdges()) << std::endl;
        std::cout << "\tStop graph vertices: " << String::prettyInt(algorithm.getStopGraphVertices()) << std::endl;
        std::cout << "\tStop graph edges: " << String::prettyInt(algorithm.getStopGraphEdges()) << std::endl;
        std::cout << "\tTarget graph vertices: " << String::prettyInt(algorithm.getTargetGraphVertices()) << std::endl;
        std::cout << "\tTarget graph edges: " << String::prettyInt(algorithm.getTargetGraphEdges()) << std::endl;
        std::cout << std::endl;

        std::cout << "Query statistics:" << std::endl;
        algorithm.getProfiler().printStatistics();
    }
};

inline constexpr int ShortcutWeight = 1024;
inline constexpr int LevelWeight = 256;
inline constexpr int DegreeWeight = 0;

using Profiler = CH::FullProfiler;
using WitnessSearch = CH::BidirectionalWitnessSearch<CHCoreGraph, CH::FullProfiler, 200>;
using GreedyKey = CH::GreedyKey<WitnessSearch>;
using PartialKey = CH::PartialKey<WitnessSearch, GreedyKey>;
using StaggeredKey = CH::StaggeredKey<WitnessSearch, GreedyKey>;
using StopCriterion = CH::CoreCriterion;
using NoStopCriterion = CH::NoStopCriterion;
template<typename KEY_FUNCTION, typename STOP_CRITERION>
using CHBuilder = CH::Builder<Profiler, WitnessSearch, KEY_FUNCTION, STOP_CRITERION, false, false>;

inline GreedyKey getGreedyKey() noexcept {
    return GreedyKey(ShortcutWeight, LevelWeight, DegreeWeight);
}

struct CHData {
    CHData(const std::string& fileName) {
        readBinary(fileName);
    }

    CHData(const CH::CH&& ch, const TransferGraph& originalGraph) :
        ch(std::move(ch)),
        numCoreVertices(0) {
        DynamicTransferGraph tempCore;
        tempCore.addVertices(originalGraph.numVertices());
        tempCore[Coordinates] = originalGraph[Coordinates];
        for (const Vertex vertex : tempCore.vertices()) {
            if (ch.isCoreVertex(vertex)) {
                numCoreVertices++;
                for (const Edge edge : ch.forward.edgesFrom(vertex)) {
                    tempCore.addEdge(vertex, ch.forward.get(ToVertex, edge)).set(TravelTime, ch.forward.get(Weight, edge));
                }
            }
        }
        Graph::move(std::move(tempCore), core);
    }

    inline void writeBinary(const std::string& fileName) const noexcept {
        core.writeBinary(fileName + ".core");
        ch.writeBinary(fileName + ".ch");
    }

    inline void readBinary(const std::string& fileName) noexcept {
        core.readBinary(fileName + ".core");
        ch.readBinary(fileName + ".ch");
    }

    TransferGraph core;
    CH::CH ch;
    size_t numCoreVertices;
};

template<typename KEY_FUNCTION, typename STOP_CRITERION>
inline CHData buildCH(const TransferGraph& originalGraph, const KEY_FUNCTION& keyFunction, const STOP_CRITERION& stopCriterion) noexcept {
    TravelTimeGraph graph;
    Graph::copy(originalGraph, graph);
    Graph::printInfo(graph);
    CHBuilder<KEY_FUNCTION, STOP_CRITERION> chBuilder(std::move(graph), graph[TravelTime], keyFunction, stopCriterion);
    chBuilder.run();
    chBuilder.copyCoreToCH();
    return CHData(CH::CH(std::move(chBuilder)), originalGraph);
}

inline CHData buildCoreCH(const RAPTOR::Data& data, const std::vector<Vertex>& targets, const size_t coreDegree) noexcept {
    std::vector<bool> isCoreContractable(data.transferGraph.numVertices(), true);
    for (const Vertex target : targets) {
        isCoreContractable[target] = false;
    }
    for (const StopId stop : data.stops()) {
        isCoreContractable[stop] = false;
    }
    PartialKey keyFunction(isCoreContractable, data.transferGraph.numVertices(), getGreedyKey());
    StopCriterion stopCriterion(data.numberOfStops(), coreDegree);
    return buildCH(data.transferGraph, keyFunction, stopCriterion);
}

inline CH::CH buildULTRACH(const RAPTOR::Data& data, const std::vector<Vertex>& targets, const double stopFactor, const double targetFactor) noexcept {
    IndexedSet<false, Vertex> targetsAndStops(data.transferGraph.numVertices());
    for (const StopId stop : data.stops()) {
        targetsAndStops.insert(stop);
    }
    for (const Vertex vertex : targets) {
        targetsAndStops.insert(vertex);
    }
    const size_t stopLimit = data.numberOfStops() * stopFactor;
    const size_t targetLimit = targetsAndStops.size() * targetFactor;
    const size_t targetRound = (targetLimit < stopLimit) ? 2 : 1;
    const size_t stopRound = (targetLimit < stopLimit) ? 1 : 2;
    std::vector<size_t> coreSizes(2);
    coreSizes[targetRound - 1] = targetLimit;
    coreSizes[stopRound - 1] = stopLimit;

    std::vector<size_t> firstContractableRound(data.transferGraph.numVertices(), 0);
    for (const Vertex target : targets) {
        firstContractableRound[target] = targetRound;
    }
    for (const StopId stop : data.stops()) {
        firstContractableRound[stop] = std::max(firstContractableRound[stop], stopRound);
    }

    StaggeredKey keyFunction(firstContractableRound, coreSizes);
    return buildCH(data.transferGraph, keyFunction, NoStopCriterion()).ch;
}

inline std::vector<Vertex> generateTargetSet(const TransferGraph& graph, const size_t ballSize, const size_t numTargets) noexcept {
    std::mt19937 randomGenerator(42);
    std::uniform_int_distribution<> vertexDistribution(0, graph.numVertices() - 1);
    Dijkstra<TransferGraph> dijkstra(graph);
    std::vector<Vertex> ball;
    do {
        ball.clear();
        const Vertex ballCenter(vertexDistribution(randomGenerator));
        dijkstra.run(ballCenter, noVertex, [&](const Vertex v) {
            ball.emplace_back(v);
            }, [&]() {
                return ball.size() == ballSize;
            });
    } while(ball.size() != ballSize);
    const Permutation permutation(Construct::Random, ballSize);
    std::vector<Vertex> targets;
    for (size_t t = 0; t < numTargets; t++) {
        targets.emplace_back(ball[permutation[t]]);
    }
    return targets;
}

class CreateBallTargetSets : public ParameterizedCommand {

public:
    CreateBallTargetSets(BasicShell& shell) :
        ParameterizedCommand(shell, "createBallTargetSets", "Creates random ball target sets for the given network.") {
        addParameter("RAPTOR data");
        addParameter("Number of target sets");
        addParameter("Target set size");
        addParameter("Ball size factor");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData(getParameter("RAPTOR data"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();

        const size_t numTargets = getParameter<size_t>("Target set size");
        const double ballSizeFactor = getParameter<double>("Ball size factor");
        const size_t ballSize = std::min(raptorData.transferGraph.numVertices(), size_t(numTargets * ballSizeFactor));

        std::vector<std::vector<Vertex>> targets;
        const size_t numTargetSets = getParameter<size_t>("Number of target sets");
        for (size_t i = 0; i < numTargetSets; i++) {
            targets.emplace_back(generateTargetSet(raptorData.transferGraph, ballSize, numTargets));
        }
        IO::serialize(getParameter("Output file"), targets);
    }
};

class BuildCoreCHForTargetSets : public ParameterizedCommand {

public:
    BuildCoreCHForTargetSets(BasicShell& shell) :
        ParameterizedCommand(shell, "buildCoreCHForTargetSets", "Builds Core-CHs for the given target sets.") {
        addParameter("RAPTOR data");
        addParameter("Targets file");
        addParameter("Core degree");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData(getParameter("RAPTOR data"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();

        std::vector<std::vector<Vertex>> targets;
        IO::deserialize(getParameter("Targets file"), targets);

        const size_t coreDegree = getParameter<size_t>("Core degree");
        const std::string outputFile = getParameter("Output file");

        std::vector<CHData> coreCHData;
        BenchmarkData benchmarkData;
        Timer timer;
        for (size_t i = 0; i < targets.size(); i++) {
            timer.restart();
            const CHData coreCHData = buildCoreCH(raptorData, targets[i], coreDegree);
            benchmarkData.add(timer.elapsedMicroseconds(), coreCHData);
            coreCHData.writeBinary(outputFile + "_" + std::to_string(i));
        }
        std::cout << benchmarkData << std::endl;
    }

private:
    struct BenchmarkData {
        BenchmarkData() :
            buildTime(0.0),
            coreVertices(0),
            coreEdges(0),
            chEdges(0),
            count(0) {
        }

        inline void add(const double time, const CHData& coreCHData) noexcept {
            buildTime += time;
            coreVertices += coreCHData.numCoreVertices;
            coreEdges += coreCHData.core.numEdges();
            chEdges += coreCHData.ch.numEdges();
            count++;
        }

        inline friend std::ostream& operator<<(std::ostream& out, const BenchmarkData& data) noexcept {
            out << "Time: " << String::musToString(data.buildTime/data.count) << std::endl;
            out << "Core vertices: " << String::prettyDouble(data.coreVertices/data.count) << std::endl;
            out << "Core edges: " << String::prettyDouble(data.coreEdges/data.count) << std::endl;
            out << "CH edges: " << String::prettyDouble(data.chEdges/data.count) << std::endl;
            return out;
        }

        double buildTime;
        long long coreVertices;
        long long coreEdges;
        long long chEdges;
        double count;
    };
};

class BuildUPCHForTargetSets : public ParameterizedCommand {

public:
    BuildUPCHForTargetSets(BasicShell& shell) :
        ParameterizedCommand(shell, "buildUPCHForTargetSets", "Builds ULTRA-PHAST-CHs for the given target sets.") {
        addParameter("RAPTOR data");
        addParameter("Targets file");
        addParameter("Stop factor");
        addParameter("Target factor");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData(getParameter("RAPTOR data"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();

        std::vector<std::vector<Vertex>> targets;
        IO::deserialize(getParameter("Targets file"), targets);

        const double stopFactor = getParameter<double>("Stop factor");
        const double targetFactor = getParameter<double>("Target factor");
        const std::string outputFile = getParameter("Output file");

        std::vector<CHData> coreCHData;
        BenchmarkData benchmarkData;
        Timer timer;
        for (size_t i = 0; i < targets.size(); i++) {
            timer.restart();
            const CH::CH ch = buildULTRACH(raptorData, targets[i], stopFactor, targetFactor);
            benchmarkData.add(timer.elapsedMicroseconds(), ch);
            ch.writeBinary(outputFile + "_" + std::to_string(i) + ".ultra");
        }
        std::cout << benchmarkData << std::endl;
    }

private:
    struct BenchmarkData {
        BenchmarkData() :
            buildTime(0.0),
            chEdges(0),
            count(0) {
        }

        inline void add(const double time, const CH::CH& ch) noexcept {
            buildTime += time;
            chEdges += ch.numEdges();
            count++;
        }

        inline friend std::ostream& operator<<(std::ostream& out, const BenchmarkData& data) noexcept {
            out << "Time: " << String::musToString(data.buildTime/data.count) << std::endl;
            out << "CH edges: " << String::prettyDouble(data.chEdges/data.count) << std::endl;
            return out;
        }

        double buildTime;
        long long chEdges;
        double count;
    };
};

class RunOneToManyDijkstraCSAQueriesToBall : public ParameterizedCommand {

public:
    RunOneToManyDijkstraCSAQueriesToBall(BasicShell& shell) :
        ParameterizedCommand(shell, "runOneToManyDijkstraCSAQueriesToBall", "Runs the given number of random Dijkstra-CSA queries to given ball target sets.") {
        addParameter("CSA data");
        addParameter("Targets file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData(getParameter("CSA data"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();

        std::vector<std::vector<Vertex>> targets;
        IO::deserialize(getParameter("Targets file"), targets);

        const std::string chFile = getParameter("CH data");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<OneToAllQuery> queries = generateRandomOneToAllQueries(csaData.transferGraph.numVertices(), n);

        using Algorithm = CSA::OneToAllDijkstraCSA<CHGraph, true, CSA::AggregateProfiler>;

        CSA::AggregateProfiler profiler;
        profiler.registerPhases({CSA::PHASE_CLEAR, CSA::PHASE_INITIALIZATION, CSA::PHASE_CONNECTION_SCAN, CSA::PHASE_FINAL_TRANSFERS});
        profiler.registerMetrics({CSA::METRIC_CONNECTIONS, CSA::METRIC_EDGES, CSA::METRIC_STOPS_BY_TRIP, CSA::METRIC_STOPS_BY_TRANSFER});

        for (size_t i = 0; i < targets.size(); i++) {
            const CHData coreCHData(chFile + "_" + std::to_string(i));
            CSA::Data coreCSAData = csaData;
            coreCSAData.transferGraph = coreCHData.core;
            Algorithm algorithm(coreCSAData, coreCHData.ch.forward, Weight);
            for (const OneToAllQuery& query : queries) {
                algorithm.run(query.source, query.departureTime);
            }
            profiler += algorithm.getProfiler();
        }
        profiler.printStatistics();
    }
};

struct SetupBenchmarkData {
    SetupBenchmarkData() :
        buildTime(0.0),
        upwardSweepGraphVertices(0),
        upwardSweepGraphEdges(0),
        stopGraphVertices(0),
        stopGraphEdges(0),
        targetGraphVertices(0),
        targetGraphEdges(0),
        count(0) {
    }

    template<typename ALGORITHM>
    inline void add(const double time, const ALGORITHM& algorithm) noexcept {
        buildTime += time;
        upwardSweepGraphVertices += algorithm.getUpwardSweepGraphVertices();
        upwardSweepGraphEdges += algorithm.getUpwardSweepGraphEdges();
        stopGraphVertices += algorithm.getStopGraphVertices();
        stopGraphEdges += algorithm.getStopGraphEdges();
        targetGraphVertices += algorithm.getTargetGraphVertices();
        targetGraphEdges += algorithm.getTargetGraphEdges();
        count++;
    }

    inline friend std::ostream& operator<<(std::ostream& out, const SetupBenchmarkData& data) noexcept {
        out << "Setup:" << std::endl;
        out << "\tTime: " << String::musToString(data.buildTime/data.count) << std::endl;
        out << "\tUpward sweep graph vertices: " << String::prettyDouble(data.upwardSweepGraphVertices/data.count) << std::endl;
        out << "\tUpward sweep graph edges: " << String::prettyDouble(data.upwardSweepGraphEdges/data.count) << std::endl;
        out << "\tStop graph vertices: " << String::prettyDouble(data.stopGraphVertices/data.count) << std::endl;
        out << "\tStop graph edges: " << String::prettyDouble(data.stopGraphEdges/data.count) << std::endl;
        out << "\tTarget graph vertices: " << String::prettyDouble(data.targetGraphVertices/data.count) << std::endl;
        out << "\tTarget graph edges: " << String::prettyDouble(data.targetGraphEdges/data.count) << std::endl;
        return out;
    }

    double buildTime;
    long long upwardSweepGraphVertices;
    long long upwardSweepGraphEdges;
    long long stopGraphVertices;
    long long stopGraphEdges;
    long long targetGraphVertices;
    long long targetGraphEdges;
    double count;
};

class RunUPCSAQueriesToBall : public ParameterizedCommand {

public:
    RunUPCSAQueriesToBall(BasicShell& shell) :
        ParameterizedCommand(shell, "runUPCSAQueriesToBall", "Runs the given number of random UP-CSA queries to given ball target sets.") {
        addParameter("CSA data");
        addParameter("Targets file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Initial transfers", { "Bucket", "PHAST" });
        addParameter("Final transfers", { "Bucket", "PHAST" });
        addParameter("Reorder network?");
        addParameter("Order", { "DFS", "Level" });
    }

    virtual void execute() noexcept {
        if (getParameter("Initial transfers") == "Bucket") {
            chooseFinalTransfers<true>();
        } else {
            chooseFinalTransfers<false>();
        }
    }

private:
    template<bool USE_STOP_BUCKETS>
    inline void chooseFinalTransfers() const noexcept {
        if (getParameter("Final transfers") == "Bucket") {
            run<USE_STOP_BUCKETS, true>();
        } else {
            run<USE_STOP_BUCKETS, false>();
        }
    }

    template<bool USE_STOP_BUCKETS, bool USE_TARGET_BUCKETS>
    inline void run() const noexcept {
        CSA::Data csaData(getParameter("CSA data"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();

        std::vector<std::vector<Vertex>> targets;
        IO::deserialize(getParameter("Targets file"), targets);

        const std::string chFile = getParameter("CH data");
        const CH::CH dummyCH(chFile + "_0.ultra");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<OneToAllQuery> queries = generateRandomOneToAllQueries(dummyCH.numVertices(), n);

        const bool reorder = getParameter<bool>("Reorder network?");
        const bool useDFSOrder = getParameter("Order") == "DFS";

        using Algorithm = CSA::UPCSA<USE_STOP_BUCKETS, USE_TARGET_BUCKETS, true, CSA::AggregateProfiler>;

        CSA::AggregateProfiler profiler;
        profiler.registerPhases({CSA::PHASE_CLEAR, CSA::PHASE_INITIALIZATION, CSA::PHASE_CONNECTION_SCAN, CSA::PHASE_UPWARD_SWEEP, CSA::PHASE_DOWNWARD_SEARCH});
        profiler.registerMetrics({CSA::METRIC_CONNECTIONS, CSA::METRIC_EDGES, CSA::METRIC_STOPS_BY_TRIP, CSA::METRIC_STOPS_BY_TRANSFER});

        SetupBenchmarkData benchmarkData;
        Timer timer;
        for (size_t i = 0; i < targets.size(); i++) {
            const CH::CH ch(chFile + "_" + std::to_string(i) + ".ultra");
            IndexedSet<false, Vertex> targetSet(ch.numVertices(), targets[i]);
            timer.restart();
            Algorithm algorithm(csaData, ch, targetSet, reorder, useDFSOrder);
            benchmarkData.add(timer.elapsedMicroseconds(), algorithm);
            for (const OneToAllQuery& query : queries) {
                algorithm.run(query.source, query.departureTime);
            }
            profiler += algorithm.getProfiler();
        }
        std::cout << benchmarkData << std::endl;
        std::cout << "Query statistics:" << std::endl;
        profiler.printStatistics();
    }
};

class RunOneToManyDijkstraRAPTORQueriesToBall : public ParameterizedCommand {

public:
    RunOneToManyDijkstraRAPTORQueriesToBall(BasicShell& shell) :
        ParameterizedCommand(shell, "runOneToManyDijkstraRAPTORQueriesToBall", "Runs the given number of random Dijkstra-RAPTOR queries to given ball target sets.") {
        addParameter("RAPTOR data");
        addParameter("Targets file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData(getParameter("RAPTOR data"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();

        std::vector<std::vector<Vertex>> targets;
        IO::deserialize(getParameter("Targets file"), targets);

        const std::string chFile = getParameter("CH data");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<OneToAllQuery> queries = generateRandomOneToAllQueries(raptorData.transferGraph.numVertices(), n);

        using Algorithm = RAPTOR::OneToAllDijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler>;

        RAPTOR::AggregateProfiler profiler;
        profiler.registerExtraRounds({RAPTOR::EXTRA_ROUND_CLEAR, RAPTOR::EXTRA_ROUND_INITIALIZATION});
        profiler.registerPhases({RAPTOR::PHASE_INITIALIZATION, RAPTOR::PHASE_COLLECT, RAPTOR::PHASE_SCAN, RAPTOR::PHASE_TRANSFERS});
        profiler.registerMetrics({RAPTOR::METRIC_ROUTES, RAPTOR::METRIC_ROUTE_SEGMENTS, RAPTOR::METRIC_VERTICES, RAPTOR::METRIC_EDGES, RAPTOR::METRIC_STOPS_BY_TRIP, RAPTOR::METRIC_STOPS_BY_TRANSFER});

        for (size_t i = 0; i < targets.size(); i++) {
            const CHData coreCHData(chFile + "_" + std::to_string(i));
            RAPTOR::Data coreRaptorData = raptorData;
            coreRaptorData.transferGraph = coreCHData.core;
            Algorithm algorithm(coreRaptorData, coreCHData.ch);
            for (const OneToAllQuery& query : queries) {
                algorithm.run(query.source, query.departureTime);
            }
            profiler += algorithm.getProfiler();
        }
        profiler.printStatistics();
    }
};

class RunUPRAPTORQueriesToBall : public ParameterizedCommand {

public:
    RunUPRAPTORQueriesToBall(BasicShell& shell) :
        ParameterizedCommand(shell, "runUPRAPTORQueriesToBall", "Runs the given number of random UP-RAPTOR queries to given ball target sets.") {
        addParameter("RAPTOR data");
        addParameter("Targets file");
        addParameter("Core-CH data");
        addParameter("UP-CH data");
        addParameter("Number of queries");
        addParameter("Reorder network?");
        addParameter("Order", { "DFS", "Level" });
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData(getParameter("RAPTOR data"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();

        std::vector<std::vector<Vertex>> targets;
        IO::deserialize(getParameter("Targets file"), targets);

        const std::string coreCHFile = getParameter("Core-CH data");
        const std::string upCHFile = getParameter("UP-CH data");
        const CH::CH dummyCH(upCHFile + "_0.ultra");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<OneToAllQuery> queries = generateRandomOneToAllQueries(dummyCH.numVertices(), n);

        const bool reorder = getParameter<bool>("Reorder network?");
        const bool useDFSOrder = getParameter("Order") == "DFS";

        using Algorithm = RAPTOR::UPRAPTOR<6, RAPTOR::AggregateProfiler>;

        RAPTOR::AggregateProfiler profiler;
        profiler.registerExtraRounds({RAPTOR::EXTRA_ROUND_CLEAR, RAPTOR::EXTRA_ROUND_INITIALIZATION, RAPTOR::EXTRA_ROUND_FINAL_TRANSFERS});
        profiler.registerPhases({RAPTOR::PHASE_INITIALIZATION, RAPTOR::PHASE_COLLECT, RAPTOR::PHASE_SCAN, RAPTOR::PHASE_TRANSFERS, RAPTOR::PHASE_FINAL_TRANSFERS});
        profiler.registerMetrics({RAPTOR::METRIC_ROUTES, RAPTOR::METRIC_ROUTE_SEGMENTS, RAPTOR::METRIC_EDGES, RAPTOR::METRIC_STOPS_BY_TRIP, RAPTOR::METRIC_STOPS_BY_TRANSFER});

        SetupBenchmarkData benchmarkData;
        Timer timer;
        for (size_t i = 0; i < targets.size(); i++) {
            const CHData coreCHData(coreCHFile + "_" + std::to_string(i));
            const CH::CH upCH(upCHFile + "_" + std::to_string(i) + ".ultra");
            IndexedSet<false, Vertex> targetSet(upCH.numVertices(), targets[i]);
            timer.restart();
            Algorithm algorithm(raptorData, coreCHData.core, upCH, targetSet, reorder, useDFSOrder);
            benchmarkData.add(timer.elapsedMicroseconds(), algorithm);
            for (const OneToAllQuery& query : queries) {
                algorithm.run(query.source, query.departureTime);
            }
            profiler += algorithm.getProfiler();
        }
        std::cout << benchmarkData << std::endl;
        std::cout << "Query statistics:" << std::endl;
        profiler.printStatistics();
    }
};
