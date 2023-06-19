#pragma once

#include <string>
#include <vector>
#include <iostream>

#include "../../Shell/Shell.h"
using namespace Shell;

#include "../../Algorithms/RAPTOR/InitialTransfers.h"
#include "../../Algorithms/RAPTOR/MultimodalMCR.h"
#include "../../Algorithms/RAPTOR/MultimodalULTRAMcRAPTOR.h"
#include "../../Algorithms/RAPTOR/ULTRABounded/MultimodalUBMHydRA.h"
#include "../../Algorithms/RAPTOR/ULTRABounded/MultimodalUBMRAPTOR.h"

#include "../../DataStructures/Queries/Queries.h"
#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/RAPTOR/MultimodalData.h"
#include "../../DataStructures/TripBased/Data.h"
#include "../../DataStructures/TripBased/MultimodalData.h"

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
