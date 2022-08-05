#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <random>

#include "../../Shell/Shell.h"

#include "../../DataStructures/Queries/Queries.h"
#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/RAPTOR/MultimodalData.h"
#include "../../DataStructures/TripBased/Data.h"
#include "../../DataStructures/TripBased/MultimodalData.h"

#include "../../Algorithms/CSA/CSA.h"
#include "../../Algorithms/CSA/DijkstraCSA.h"
#include "../../Algorithms/CSA/HLCSA.h"
#include "../../Algorithms/CSA/ULTRACSA.h"
#include "../../Algorithms/RAPTOR/Bounded/BoundedMcRAPTOR.h"
#include "../../Algorithms/RAPTOR/ULTRAMcRAPTOR.h"
#include "../../Algorithms/RAPTOR/HLRAPTOR.h"
#include "../../Algorithms/RAPTOR/DijkstraRAPTOR.h"
#include "../../Algorithms/RAPTOR/InitialTransfers.h"
#include "../../Algorithms/RAPTOR/McRAPTOR.h"
#include "../../Algorithms/RAPTOR/MultimodalMCR.h"
#include "../../Algorithms/RAPTOR/MultimodalULTRAMcRAPTOR.h"
#include "../../Algorithms/RAPTOR/RAPTOR.h"
#include "../../Algorithms/RAPTOR/ULTRARAPTOR.h"
#include "../../Algorithms/RAPTOR/MCR.h"
#include "../../Algorithms/RAPTOR/ULTRABounded/MultimodalUBMHydRA.h"
#include "../../Algorithms/RAPTOR/ULTRABounded/MultimodalUBMRAPTOR.h"
#include "../../Algorithms/RAPTOR/ULTRABounded/UBMHydRA.h"
#include "../../Algorithms/RAPTOR/ULTRABounded/UBMRAPTOR.h"
#include "../../Algorithms/TripBased/BoundedMcQuery/BoundedMcQuery.h"
#include "../../Algorithms/TripBased/Query/McQuery.h"
#include "../../Algorithms/TripBased/Query/Query.h"
#include "../../Algorithms/TripBased/Query/TransitiveQuery.h"

using namespace Shell;

class RunTransitiveRAPTORQueries : public ParameterizedCommand {

public:
    RunTransitiveRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveRAPTORQueries", "Runs the given number of random transitive RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false> algorithm(raptorData);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        double numJourneys = 0;
        for (const StopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;

    }
};

class RunDijkstraRAPTORQueries : public ParameterizedCommand {

public:
    RunDijkstraRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runDijkstraRAPTORQueries", "Runs the given number of random Dijkstra RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler, true, false> algorithm(raptorData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunULTRARAPTORQueries : public ParameterizedCommand {

public:
    RunULTRARAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRARAPTORQueries", "Runs the given number of random ULTRA-RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, false> algorithm(raptorData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunHLRAPTORQueries : public ParameterizedCommand {

public:
    RunHLRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runHLRAPTORQueries", "Runs the given number of random HL-RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Out-hub file");
        addParameter("In-hub file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        const TransferGraph outHubs(getParameter("Out-hub file"));
        const TransferGraph inHubs(getParameter("In-hub file"));
        RAPTOR::HLRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, outHubs, inHubs);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(inHubs.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunTransitiveMcRAPTORQueries : public ParameterizedCommand {

public:
    RunTransitiveMcRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveMcRAPTORQueries", "Runs the given number of random transitive McRAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        RAPTOR::McRAPTOR<true, true, RAPTOR::AggregateProfiler> algorithm(raptorData);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        double numJourneys = 0;
        for (const StopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunTransitiveBoundedMcRAPTORQueries : public ParameterizedCommand {

public:
    RunTransitiveBoundedMcRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveBoundedMcRAPTORQueries", "Runs the given number of random transitive Bounded McRAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        const RAPTOR::Data reverseData = raptorData.reverseNetwork();
        RAPTOR::BoundedMcRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, reverseData);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        double numJourneys = 0;
        for (const StopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunMCRQueries : public ParameterizedCommand {

public:
    RunMCRQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runMCRQueries", "Runs the given number of random MCR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::MCR<true, RAPTOR::AggregateProfiler> algorithm(raptorData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunMultimodalMCRQueries : public ParameterizedCommand {

public:
    RunMultimodalMCRQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runMultimodalMCRQueries", "Runs the given number of random multimodal MCR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH directory");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::MultimodalData raptorData(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        switch (raptorData.modes.size()) {
            case 2:
                run<2>(raptorData);
                break;
            case 3:
                run<3>(raptorData);
                break;
            default:
                Ensure(false, "Unsupported number of modes!");
                break;
        }
    }

private:
    template<size_t NUM_MODES>
    inline void run(const RAPTOR::MultimodalData& raptorData) const noexcept {
        const std::string chDirectory(getParameter("CH directory"));
        std::vector<CH::CH> chData;
        for (const size_t mode : raptorData.modes) {
            chData.emplace_back(chDirectory + RAPTOR::TransferModeNames[mode] + "CH");
        }
        RAPTOR::MultimodalMCR<true, NUM_MODES, RAPTOR::AggregateProfiler> algorithm(raptorData, chData);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(chData[0].numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunULTRAMcRAPTORQueries : public ParameterizedCommand {

public:
    RunULTRAMcRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRAMcRAPTORQueries", "Runs the given number of random ULTRA-McRAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::ULTRAMcRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunMultimodalULTRAMcRAPTORQueries : public ParameterizedCommand {

public:
    RunMultimodalULTRAMcRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runMultimodalULTRAMcRAPTORQueries", "Runs the given number of random multimodal ULTRA-McRAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH directory");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::MultimodalData raptorData(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        switch (raptorData.modes.size()) {
            case 2:
                run<2>(raptorData);
                break;
            case 3:
                run<3>(raptorData);
                break;
            default:
                Ensure(false, "Unsupported number of modes!");
                break;
        }
    }

private:
    template<size_t NUM_MODES>
    inline void run(const RAPTOR::MultimodalData& raptorData) const noexcept {
        const std::string chDirectory(getParameter("CH directory"));
        std::vector<CH::CH> chData;
        for (const size_t mode : raptorData.modes) {
            chData.emplace_back(chDirectory + RAPTOR::TransferModeNames[mode] + "CH");
        }
        RAPTOR::MultimodalULTRAMcRAPTOR<NUM_MODES, RAPTOR::AggregateProfiler> algorithm(raptorData, chData);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(chData[0].numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunUBMRAPTORQueries : public ParameterizedCommand {

public:
    RunUBMRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runUBMRAPTORQueries", "Runs the given number of random UBM-RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        const RAPTOR::Data reverseData = raptorData.reverseNetwork();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::UBMRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, reverseData, ch);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunUBMHydRAQueries : public ParameterizedCommand {

public:
    RunUBMHydRAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runUBMHydRAQueries", "Runs the given number of random UBM-HydRA queries.") {
        addParameter("Trip-Based input file");
        addParameter("Bounded forward Trip-Based input file");
        addParameter("Bounded backward Trip-Based input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        const TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        const TripBased::Data forwardBoundedData(getParameter("Bounded forward Trip-Based input file"));
        forwardBoundedData.printInfo();
        const TripBased::Data backwardBoundedData(getParameter("Bounded backward Trip-Based input file"));
        backwardBoundedData.printInfo();
        const CH::CH ch(getParameter("CH data"));

        RAPTOR::UBMHydRA<RAPTOR::AggregateProfiler> algorithm(tripBasedData, forwardBoundedData, backwardBoundedData, ch);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunMultimodalUBMRAPTORQueries : public ParameterizedCommand {

public:
    RunMultimodalUBMRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runMultimodalUBMRAPTORQueries", "Runs the given number of random multimodal UBM-RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH directory");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        RAPTOR::MultimodalData raptorData(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        switch (raptorData.modes.size()) {
            case 2:
                run<2>(raptorData);
                break;
            case 3:
                run<3>(raptorData);
                break;
            default:
                Ensure(false, "Unsupported number of modes!");
                break;
        }
    }

private:
    template<size_t NUM_MODES>
    inline void run(const RAPTOR::MultimodalData& raptorData) const noexcept {
        const RAPTOR::Data pruningData = raptorData.getPruningData();
        const RAPTOR::Data reversePruningData = pruningData.reverseNetwork();
        const std::string chDirectory(getParameter("CH directory"));
        std::vector<CH::CH> chData;
        for (const size_t mode : raptorData.modes) {
            chData.emplace_back(chDirectory + RAPTOR::TransferModeNames[mode] + "CH");
        }
        RAPTOR::TransferGraph backwardTransitiveGraph = raptorData.raptorData.transferGraph;
        backwardTransitiveGraph.revert();
        RAPTOR::MultimodalUBMRAPTOR<NUM_MODES, RAPTOR::AggregateProfiler> algorithm(raptorData, pruningData, reversePruningData, backwardTransitiveGraph, chData);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(chData[0].numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;

    }
};

class RunMultimodalUBMHydRAQueries : public ParameterizedCommand {

public:
    RunMultimodalUBMHydRAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runMultimodalUBMHydRAQueries", "Runs the given number of random multimodal UBM-HydRA queries.") {
        addParameter("Trip-Based input file");
        addParameter("Bounded forward Trip-Based input file");
        addParameter("Bounded backward Trip-Based input file");
        addParameter("CH directory");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        const TripBased::MultimodalData tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        switch (tripBasedData.modes.size()) {
            case 2:
                run<2>(tripBasedData);
                break;
            case 3:
                run<3>(tripBasedData);
                break;
            default:
                Ensure(false, "Unsupported number of modes!");
                break;
        }
    }

private:
    template<size_t NUM_MODES>
    inline void run(const TripBased::MultimodalData& tripBasedData) const noexcept {
        const TripBased::MultimodalData forwardBoundedData(getParameter("Bounded forward Trip-Based input file"));
        forwardBoundedData.printInfo();
        Ensure(forwardBoundedData.modes == tripBasedData.modes, "Different transfer modes!");
        const TripBased::Data forwardPruningData = forwardBoundedData.getPruningData();
        const TripBased::MultimodalData backwardBoundedData(getParameter("Bounded backward Trip-Based input file"));
        backwardBoundedData.printInfo();
        Ensure(backwardBoundedData.modes == tripBasedData.modes, "Different transfer modes!");
        const TripBased::Data backwardPruningData = backwardBoundedData.getPruningData();
        const std::string chDirectory(getParameter("CH directory"));
        std::vector<CH::CH> chData;
        for (const size_t mode : tripBasedData.modes) {
            chData.emplace_back(chDirectory + RAPTOR::TransferModeNames[mode] + "CH");
        }
        RAPTOR::TransferGraph backwardTransitiveGraph = tripBasedData.tripData.raptorData.transferGraph;
        backwardTransitiveGraph.revert();
        RAPTOR::MultimodalUBMHydRA<NUM_MODES, RAPTOR::AggregateProfiler> algorithm(tripBasedData, forwardPruningData, backwardPruningData, backwardTransitiveGraph, chData);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(chData[0].numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunTransitiveCSAQueries : public ParameterizedCommand {

public:
    RunTransitiveCSAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveCSAQueries", "Runs the given number of random transitive CSA queries.") {
        addParameter("CSA input file");
        addParameter("Number of queries");
        addParameter("Target pruning?");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        CSA::CSA<true, CSA::AggregateProfiler> algorithm(csaData);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(csaData.numberOfStops(), n);

        const bool targetPruning = getParameter<bool>("Target pruning?");

        for (const StopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, targetPruning ? query.target : noStop);
        }
        algorithm.getProfiler().printStatistics();
    }
};

class RunDijkstraCSAQueries : public ParameterizedCommand {

public:
    RunDijkstraCSAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runDijkstraCSAQueries", "Runs the given number of random Dijkstra-CSA queries.") {
        addParameter("CSA input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        CH::CH ch(getParameter("CH data"));
        CSA::DijkstraCSA<RAPTOR::CoreCHInitialTransfers, true, CSA::AggregateProfiler> algorithm(csaData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
        }
        algorithm.getProfiler().printStatistics();
    }
};

class RunULTRACSAQueries : public ParameterizedCommand {

public:
    RunULTRACSAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRACSAQueries", "Runs the given number of random ULTRA-CSA queries.") {
        addParameter("CSA input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        CH::CH ch(getParameter("CH data"));
        CSA::ULTRACSA<true, CSA::AggregateProfiler> algorithm(csaData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
        }
        algorithm.getProfiler().printStatistics();
    }
};

class RunHLCSAQueries : public ParameterizedCommand {

public:
    RunHLCSAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runHLCSAQueries", "Runs the given number of random HL-CSA queries.") {
        addParameter("CSA input file");
        addParameter("Out-hub file");
        addParameter("In-hub file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        const TransferGraph outHubs(getParameter("Out-hub file"));
        const TransferGraph inHubs(getParameter("In-hub file"));
        CSA::HLCSA<CSA::AggregateProfiler> algorithm(csaData, outHubs, inHubs);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(inHubs.numVertices(), n);

        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
        }
        algorithm.getProfiler().printStatistics();
    }
};

class RunTransitiveTripBasedQueries : public ParameterizedCommand {

public:
    RunTransitiveTripBasedQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveTripBasedQueries", "Runs the given number of random transitive TripBased queries.") {
        addParameter("Trip-Based input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        TripBased::TransitiveQuery<TripBased::AggregateProfiler> algorithm(tripBasedData);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(tripBasedData.numberOfStops(), n);

        double numJourneys = 0;
        for (const StopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunULTRATripBasedQueries : public ParameterizedCommand {

public:
    RunULTRATripBasedQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRATripBasedQueries", "Runs the given number of random ULTRA-TripBased queries.") {
        addParameter("Trip-Based input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        CH::CH ch(getParameter("CH data"));
        TripBased::Query<TripBased::AggregateProfiler> algorithm(tripBasedData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunULTRAMcTripBasedQueries : public ParameterizedCommand {

public:
    RunULTRAMcTripBasedQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRAMcTripBasedQueries", "Runs the given number of random ULTRA-McTripBased queries.") {
        addParameter("Trip-Based input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        CH::CH ch(getParameter("CH data"));
        TripBased::McQuery<TripBased::AggregateProfiler> algorithm(tripBasedData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class RunBoundedULTRAMcTripBasedQueries : public ParameterizedCommand {

public:
    RunBoundedULTRAMcTripBasedQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runBoundedULTRAMcTripBasedQueries", "Runs the given number of random Bounded ULTRA-McTripBased queries.") {
        addParameter("Trip-Based input file");
        addParameter("Bounded forward Trip-Based input file");
        addParameter("Bounded backward Trip-Based input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Arrival slack");
        addParameter("Trip slack");
    }

    virtual void execute() noexcept {
        TripBased::Data tripBasedData(getParameter("Trip-Based input file"));
        tripBasedData.printInfo();
        TripBased::Data forwardBoundedData(getParameter("Bounded forward Trip-Based input file"));
        forwardBoundedData.printInfo();
        TripBased::Data backwardBoundedData(getParameter("Bounded backward Trip-Based input file"));
        backwardBoundedData.printInfo();
        CH::CH ch(getParameter("CH data"));
        TripBased::BoundedMcQuery<TripBased::AggregateProfiler> algorithm(tripBasedData, forwardBoundedData, backwardBoundedData, ch);

        const double arrivalSlack = getParameter<double>("Arrival slack");
        const double tripSlack = getParameter<double>("Trip slack");

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        double numJourneys = 0;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
            numJourneys += algorithm.getJourneys().size();
        }
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
    }
};

class ComputeTransferTimeSavings : public ParameterizedCommand {

public:
    ComputeTransferTimeSavings(BasicShell& shell) :
        ParameterizedCommand(shell, "computeTransferTimeSavings", "Computes the savings in transfer time of a 3-criteria (bounded) Pareto set compared to a 2-criteria one.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();
        const RAPTOR::Data reverseData = raptorData.reverseNetwork();
        CH::CH ch(getParameter("CH data"));
        RAPTOR::UBMRAPTOR<RAPTOR::AggregateProfiler> algorithm(raptorData, reverseData, ch);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        IO::OFStream outputFile(getParameter("Output file"));
        outputFile << "ArrivalSlack";
        for (const double tripSlack : tripSlacks) {
            const int slackAsInt = tripSlack * 100 - 100;
            for (const double threshold : thresholds) {
                const int thresholdAsInt = threshold * 100;
                outputFile << "\tTripSlack" << slackAsInt << "Savings" << thresholdAsInt;
            }
        }
        outputFile << "\n";
        outputFile.flush();

        for (const double arrivalSlack : arrivalSlacks) {
            outputFile << arrivalSlack;
            for (const double tripSlack : tripSlacks) {
                std::cout << "Arrival slack: " << arrivalSlack << ", trip slack: " << tripSlack << std::endl;
                std::vector<double> transferTimeSavings;
                for (const VertexQuery& query : queries) {
                    algorithm.run(query.source, query.departureTime, query.target, arrivalSlack, tripSlack);
                    const std::vector<RAPTOR::WalkingParetoLabel> fullLabels = algorithm.getResults();
                    const std::vector<RAPTOR::ArrivalLabel>& anchorLabels = algorithm.getAnchorLabels();
                    RAPTOR::WalkingParetoLabel bestLabel;
                    RAPTOR::WalkingParetoLabel bestAnchorLabel;
                    for (const RAPTOR::WalkingParetoLabel& label : fullLabels) {
                        if (label.walkingDistance <= bestLabel.walkingDistance) {
                            bestLabel = label;
                        }
                        if (label.walkingDistance <= bestAnchorLabel.walkingDistance && isAnchorLabel(label, anchorLabels)) {
                            bestAnchorLabel = label;
                        }
                    }
                    if (bestAnchorLabel.walkingDistance == 0) {
                        transferTimeSavings.emplace_back(0);
                    } else {
                        transferTimeSavings.emplace_back((bestAnchorLabel.walkingDistance - bestLabel.walkingDistance)/static_cast<double>(bestAnchorLabel.walkingDistance));
                    }
                }
                std::sort(transferTimeSavings.begin(), transferTimeSavings.end(), [&](const double a, const double b) {
                    return a > b;
                });
                size_t j = 0;
                std::vector<size_t> savingsCount(thresholds.size(), 0);
                for (const double s : transferTimeSavings) {
                    while (s < thresholds[j]) {
                        j++;
                        if (j == thresholds.size()) break;
                    }
                    if (j == thresholds.size()) break;
                    savingsCount[j]++;
                }
                for (const size_t c : savingsCount) {
                    const double ratio = c/static_cast<double>(transferTimeSavings.size());
                    outputFile << "\t" << ratio;
                }

            }
            outputFile << "\n";
            outputFile.flush();
        }
    }

private:
    std::vector<double> thresholds { 0.75, 0.5, 0.25 };
    std::vector<double> arrivalSlacks { 1, 1.1, 1.2, 1.3, 1.4, 1.5 };
    std::vector<double> tripSlacks { 1, 1.25, 1.5 };

    inline bool isAnchorLabel(const RAPTOR::WalkingParetoLabel& label, const std::vector<RAPTOR::ArrivalLabel>& anchorLabels) const noexcept {
        for (const RAPTOR::ArrivalLabel& anchorLabel : anchorLabels) {
            if (label.arrivalTime != anchorLabel.arrivalTime) continue;
            if (label.numberOfTrips != anchorLabel.numberOfTrips) continue;
            return true;
        }
        return false;
    }
};
