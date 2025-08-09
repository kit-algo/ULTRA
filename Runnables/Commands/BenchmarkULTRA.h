#pragma once

#include <string>
#include <vector>
#include <iostream>

#include "../../Shell/Shell.h"
using namespace Shell;

#include "../../Algorithms/CSA/CSA.h"
#include "../../Algorithms/CSA/DijkstraCSA.h"
#include "../../Algorithms/CSA/HLCSA.h"
#include "../../Algorithms/CSA/ULTRACSA.h"
#include "../../Algorithms/RAPTOR/HLRAPTOR.h"
#include "../../Algorithms/RAPTOR/DijkstraRAPTOR.h"
#include "../../Algorithms/RAPTOR/InitialTransfers.h"
#include "../../Algorithms/RAPTOR/RAPTOR.h"
#include "../../Algorithms/RAPTOR/ULTRARAPTOR.h"
#include "../../Algorithms/TripBased/Query/Query.h"
#include "../../Algorithms/TripBased/Query/TransitiveQuery.h"
#include "../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../DataStructures/CSA/Entities/Journey.h"

#include "../../DataStructures/Queries/Queries.h"
#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/TripBased/Data.h"

class RunTransitiveCSAQueries : public ParameterizedCommand {
public:
    RunTransitiveCSAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveCSAQueries", "Runs the given number of random transitive CSA queries.") {
        addParameter("CSA input file");
        addParameter("Number of queries");
        addParameter("Pruning rule (0 or 1)");
        addParameter("Target pruning?");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.sortTransferGraphEdgesByTravelTime();
        csaData.printInfo();

        const size_t n = getParameter<size_t>("Number of queries");
        const int pruningRule = getParameter<int>("Pruning rule (0 or 1)");
        const bool targetPruning = getParameter<bool>("Target pruning?");
        const std::vector<StopQuery> queries = generateRandomStopQueries(csaData.numberOfStops(), n);

        if (pruningRule == 1) {
            CSA::CSA<true, CSA::AggregateProfiler> algorithm(csaData);

            for (const StopQuery& query : queries) {
                algorithm.run(query.source, query.departureTime, targetPruning ? query.target : noStop);
            }
            algorithm.getProfiler().printStatistics();
        } else {
            CSA::CSA_prune<true, CSA::AggregateProfiler> algorithm(csaData);

            for (const StopQuery& query : queries) {
                algorithm.run(query.source, query.departureTime, targetPruning ? query.target : noStop);
            }
            algorithm.getProfiler().printStatistics();
        }
    }
};

class CheckCSAPruning : public ParameterizedCommand {

public:
    CheckCSAPruning(BasicShell& shell) :
        ParameterizedCommand(shell, "checkCSAPruning", "Checks if pruning rules yield the same results as no pruning for CSA.") {
        addParameter("CSA input file");
        addParameter("Number of queries");
        addParameter("Precompute directed transitive graph?");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(csaData.numberOfStops(), n);

        // RETRIEVE THE NEW PARAMETER'S VALUE
        const bool precomputeTransitive = getParameter<bool>("Precompute directed transitive graph?");

        // CONDITIONAL PRECOMPUTATION
        if (precomputeTransitive) {
            std::cout << "--- Precomputing directed transitive stop graph... ---" << std::endl;
            csaData.makeDirectedTransitiveStopGraph(true); // 'true' enables verbose output
            std::cout << "--- Done. ---" << std::endl;
        }

        std::vector<int> results_no_pruning;
        std::vector<int> results_pruning_1;

        // Run with pruning rule 0 (no pruning)
        std::cout << "--- Running queries with No Pruning (Rule 0) ---" << std::endl;
        CSA::CSA<false, CSA::AggregateProfiler> algo_no_pruning(csaData);
        for (const StopQuery& query : queries) {
            algo_no_pruning.run(query.source, query.departureTime, query.target);
            results_no_pruning.push_back(algo_no_pruning.getEarliestArrivalTime(query.target));
        }
        std::cout << "--- Statistics for No Pruning (Rule 0) ---" << std::endl;
        algo_no_pruning.getProfiler().printStatistics();

        // Run with pruning rule 1
        std::cout << "\n--- Running queries with Pruning Rule 1 ---" << std::endl;
        csaData.sortTransferGraphEdgesByTravelTime();
        CSA::CSA_prune<false, CSA::AggregateProfiler> algo_pruning_1(csaData);
        for (const StopQuery& query : queries) {
            algo_pruning_1.run(query.source, query.departureTime, query.target);
            results_pruning_1.push_back(algo_pruning_1.getEarliestArrivalTime(query.target));
        }
        std::cout << "--- Statistics for Pruning Rule 1 ---" << std::endl;
        algo_pruning_1.getProfiler().printStatistics();

        // Compare the results
        bool pruning_is_correct = (results_no_pruning == results_pruning_1);

        if (pruning_is_correct) {
            std::cout << "\nPruning rule 1 yields the same results as no pruning." << std::endl;
        } else {
            std::cout << "\nPruning rule 1 failed comparison." << std::endl;
        }
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

class RunULTRACSAQueries : public ParameterizedCommand {

public:
    RunULTRACSAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRACSAQueries", "Runs the given number of random ULTRA-CSA queries.") {
        addParameter("CSA input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Pruning rule (0 or 1)");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.sortTransferGraphEdgesByTravelTime(); // Call to sort the transfer graph edges
        csaData.printInfo();
        CH::CH ch(getParameter("CH data"));

        const size_t n = getParameter<size_t>("Number of queries");
        const int pruningRule = getParameter<int>("Pruning rule (0 or 1)");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        if (pruningRule == 1) {
            executeWithPruning<1>(csaData, ch, queries);
        } else {
            executeWithPruning<0>(csaData, ch, queries);
        }
    }

private:
    template<int ENABLE_PRUNING>
    void executeWithPruning(const CSA::Data& csaData, const CH::CH& ch, const std::vector<VertexQuery>& queries) {
        CSA::ULTRACSA<true, ENABLE_PRUNING, CSA::AggregateProfiler> algorithm(csaData, ch);

        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
        }
        algorithm.getProfiler().printStatistics();
    }
};


class CheckULTRACSAPruning : public ParameterizedCommand {

public:
    CheckULTRACSAPruning(BasicShell& shell) :
        ParameterizedCommand(shell, "checkULTRACSAPruning", "Checks if pruning rules yield the same results as no pruning for ULTRACSA.") {
        addParameter("CSA input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();
        CH::CH ch(getParameter("CH data"));

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        std::vector<int> results_no_pruning;
        std::vector<int> results_pruning;

        // Run with pruning rule 0 (no pruning)
        CSA::ULTRACSA<false, 0, CSA::AggregateProfiler> algo_no_pruning(csaData, ch);
        for (const VertexQuery& query : queries) {
            algo_no_pruning.run(query.source, query.departureTime, query.target);
            results_no_pruning.push_back(algo_no_pruning.getEarliestArrivalTime(query.target));
        }
        std::cout << "--- Statistics for No Pruning (Rule 0) ---" << std::endl;
        algo_no_pruning.getProfiler().printStatistics();

        // Run with pruning rule 1
        csaData.sortTransferGraphEdgesByTravelTime(); // Call to sort the transfer graph edges
        CSA::ULTRACSA<false, 1, CSA::AggregateProfiler> algo_pruning(csaData, ch);
        for (const VertexQuery& query : queries) {
            algo_pruning.run(query.source, query.departureTime, query.target);
            results_pruning.push_back(algo_pruning.getEarliestArrivalTime(query.target));
        }
        std::cout << "--- Statistics for Pruning Rule 1 ---" << std::endl;
        algo_pruning.getProfiler().printStatistics();

        // Compare the results
        bool pruning_is_correct = (results_no_pruning == results_pruning);

        if (pruning_is_correct) {
            std::cout << "Pruning rule 1 yields the same results as no pruning." << std::endl;
        } else {
            std::cout << "Pruning rule 1 failed comparison." << std::endl;
        }
    }
};

class RunTransitiveRAPTORQueries : public ParameterizedCommand {

public:
    RunTransitiveRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveRAPTORQueries", "Runs the given number of random transitive RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
        addParameter("Pruning rule (0 or 1)");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.writeCSV("");
        raptorData.printInfo();

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        // READ THE NEW INTEGER PARAMETER
        const int pruningRule = getParameter<int>("Pruning rule (0 or 1)");

        // We use an if-else block to instantiate the correct version of the algorithm,
        // as the `ENABLE_PRUNING` template parameter must be a compile-time constant.
        if (pruningRule == 1) {
            // Instantiate with TARGET_PRUNING=true and ENABLE_PRUNING=1
            RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false, false> algorithm(raptorData);

            double numJourneys = 0;
            for (const StopQuery& query : queries) {
                algorithm.run(query.source, query.departureTime, query.target);
                numJourneys += algorithm.getJourneys().size();
            }
            algorithm.getProfiler().printStatistics();
            std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n) << std::endl;
        } else {
            // Instantiate with TARGET_PRUNING=true and ENABLE_PRUNING=0 (default)
            RAPTOR::RAPTOR_prune<true, RAPTOR::AggregateProfiler, true, false, false> algorithm(raptorData);

            double numJourneys = 0;
            for (const StopQuery& query : queries) {
                algorithm.run(query.source, query.departureTime, query.target);
                numJourneys += algorithm.getJourneys().size();
            }
            algorithm.getProfiler().printStatistics();
            std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys / n) << std::endl;
        }
    }
};

class TestTransitiveRAPTORQueries : public ParameterizedCommand {

public:
    TestTransitiveRAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "testTransitiveRAPTORQueries", "Tests a specific transitive RAPTOR query.") {
        addParameter("RAPTOR input file");
        addParameter("sourceStop");
        addParameter("targetStop");
        addParameter("startTime");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();

        const StopId sourceStop = StopId(getParameter<int>("sourceStop"));
        const StopId targetStop = StopId(getParameter<int>("targetStop"));

        const int startTime = getParameter<int>("startTime");

        std::cout << "Running query from stop " << sourceStop << " to stop " << targetStop << " at time " << startTime << std::endl;

        RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false, false> algorithm(raptorData);
        algorithm.run(sourceStop, startTime, targetStop);

        // Corrected line: calling the existing getEarliestJourney function
        const RAPTOR::Journey journey = algorithm.getEarliestJourney(targetStop);
        std::cout << "Journey: " << journey << std::endl;

    }
};

class TestTransitiveCSAQueries : public ParameterizedCommand {

public:
    TestTransitiveCSAQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "testTransitiveCSAQueries", "Tests a specific transitive CSA query.") {
        addParameter("CSA input file");
        addParameter("sourceStop");
        addParameter("targetStop");
        addParameter("startTime");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();

        const StopId sourceStop = StopId(getParameter<int>("sourceStop"));
        const StopId targetStop = StopId(getParameter<int>("targetStop"));
        const int startTime = getParameter<int>("startTime");

        std::cout << "Running query from stop " << sourceStop << " to stop " << targetStop << " at time " << startTime << std::endl;

        CSA::CSA<true, CSA::AggregateProfiler> algorithm(csaData);
        algorithm.run(sourceStop, startTime, targetStop);

        const int arrivalTime = algorithm.getEarliestArrivalTime(targetStop);
        std::cout << "Earliest Arrival Time: " << arrivalTime << std::endl;

        const CSA::Journey journey = algorithm.getJourney(targetStop);
        std::cout << "Journey: " << journey << std::endl;
    }
};

class CompareCSAandRAPTOR : public ParameterizedCommand {

public:
    CompareCSAandRAPTOR(BasicShell& shell) :
        ParameterizedCommand(shell, "compareCSAandRAPTOR", "Compares journeys from CSA and RAPTOR for random queries.") {
        addParameter("RAPTOR input file");
        addParameter("CSA input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        // Load data for both algorithms
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.printInfo();

        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA input file"));
        csaData.sortConnectionsAscending();
        csaData.printInfo();

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        runComparison(raptorData, csaData, queries);
    }

private:

    // Function to run the queries and compare the results
    void runComparison(const RAPTOR::Data& raptorData, const CSA::Data& csaData, const std::vector<StopQuery>& queries) const noexcept {
        const bool targetPruning = true;

        RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false, false> raptorAlgorithm(raptorData);
        CSA::CSA<true, CSA::AggregateProfiler> csaAlgorithm(csaData);

        size_t mismatches = 0;

        for (size_t i = 0; i < queries.size(); i++) {
            const auto& query = queries[i];

            // Run RAPTOR and get the arrival time
            raptorAlgorithm.run(query.source, query.departureTime, query.target);
            const int raptorArrivalTime = raptorAlgorithm.getEarliestArrivalTime(query.target);

            // Run CSA and get the arrival time
            csaAlgorithm.run(query.source, query.departureTime, targetPruning ? query.target : noStop);
            const int csaArrivalTime = csaAlgorithm.getEarliestArrivalTime(query.target);

            // Compare the arrival times
            if (raptorArrivalTime != csaArrivalTime) {
                mismatches++;
                std::cout << "Mismatch found for query #" << i + 1 << ":" << std::endl;
                std::cout << "  Source: " << query.source << ", Target: " << query.target << ", Time: " << query.departureTime << std::endl;
                std::cout << "--- RAPTOR Arrival Time ---" << std::endl;
                std::cout << raptorArrivalTime << std::endl;
                std::cout << "--- CSA Arrival Time ---" << std::endl;
                std::cout << csaArrivalTime << std::endl;
                std::cout << "-----------------------------------" << std::endl;
            }
        }
        std::cout << "\nTotal queries: " << queries.size() << ", Total mismatches: " << mismatches << std::endl;
    }
};

class CheckRAPTORPruning : public ParameterizedCommand {

public:
    CheckRAPTORPruning(BasicShell& shell) :
        ParameterizedCommand(shell, "checkRAPTORPruning", "Checks if RAPTOR pruning rules yield the same results as no pruning.") {
        addParameter("RAPTOR input file");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();

        const size_t n = getParameter<size_t>("Number of queries");
        // Generate StopQueries, not VertexQueries
        const std::vector<StopQuery> queries = generateRandomStopQueries(raptorData.numberOfStops(), n);

        std::vector<int> results_no_pruning;
        std::vector<int> results_pruning_1;

        // Run with pruning rule 0 (no pruning)
        std::cout << "--- Running with No Pruning (Rule 0) ---" << std::endl;
        RAPTOR::RAPTOR<true, RAPTOR::AggregateProfiler, true, false, false> algo_no_pruning(raptorData);
        for (const StopQuery& query : queries) {
            algo_no_pruning.run(query.source, query.departureTime, query.target);
            results_no_pruning.push_back(algo_no_pruning.getEarliestArrivalTime(query.target));
        }
        std::cout << "--- Statistics for No Pruning (Rule 0) ---" << std::endl;
        algo_no_pruning.getProfiler().printStatistics();

        // Run with pruning rule 1
        std::cout << "\n--- Running with Pruning Rule 1 ---" << std::endl;
        // The transfer graph must be sorted for pruning rule 1 to be effective
        raptorData.sortTransferGraphEdgesByTravelTime();
        RAPTOR::RAPTOR_prune<true, RAPTOR::AggregateProfiler, true, false, false> algo_pruning_1(raptorData);
        for (const StopQuery& query : queries) {
            algo_pruning_1.run(query.source, query.departureTime, query.target);
            results_pruning_1.push_back(algo_pruning_1.getEarliestArrivalTime(query.target));
        }
        std::cout << "--- Statistics for Pruning Rule 1 ---" << std::endl;
        algo_pruning_1.getProfiler().printStatistics();

        // Compare the results
        bool pruning_1_correct = (results_no_pruning == results_pruning_1);
        std::cout << "\n--- Comparison Results ---" << std::endl;
        if (pruning_1_correct) {
            std::cout << "Pruning rule 1 results match no-pruning results. The pruning is correct." << std::endl;
        } else {
            std::cout << "ERROR: Pruning rule 1 failed comparison. Results are not identical." << std::endl;
        }
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

class RunULTRARAPTORQueries : public ParameterizedCommand {

public:
    RunULTRARAPTORQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRARAPTORQueries", "Runs the given number of random ULTRA-RAPTOR queries.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
        addParameter("Pruning rule (0 or 1)");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        raptorData.sortTransferGraphEdgesByTravelTime(); // Call to sort the transfer graph edges
        raptorData.printInfo();
        CH::CH ch(getParameter("CH data"));
        // RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, false> algorithm(raptorData, ch);
        const size_t n = getParameter<size_t>("Number of queries");
        // Read the pruning rule from the user
        const int pruningRule = getParameter<int>("Pruning rule (0 or 1)");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);
        // double numJourneys = 0;
        //for (const VertexQuery& query : queries) {
        //    algorithm.run(query.source, query.departureTime, query.target);
        //    numJourneys += algorithm.getJourneys().size();
        //}
        //algorithm.getProfiler().printStatistics();
        auto runAndProfile = [&](auto& algorithm) {
            double numJourneys = 0;
            for (const VertexQuery& query : queries) {
                algorithm.run(query.source, query.departureTime, query.target);
                numJourneys += algorithm.getJourneys().size();
            }
            algorithm.getProfiler().printStatistics();
            std::cout << "Avg. journeys: " << String::prettyDouble(numJourneys/n) << std::endl;
        };

        switch (pruningRule) {
            case 0: {
                RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, false> algorithm(raptorData, ch);
                runAndProfile(algorithm);
                break;
            }
            case 1: {
                RAPTOR::ULTRARAPTOR_prune<RAPTOR::AggregateProfiler, false> algorithm(raptorData, ch);
                runAndProfile(algorithm);
                break;
            }
            default: {
                std::cout << "Invalid pruning rule. Please choose 0 or 1" << std::endl;
                break;
            }
        }
    }
};

class CheckULTRARAPTORPruning : public ParameterizedCommand {

public:
    CheckULTRARAPTORPruning(BasicShell& shell) :
        ParameterizedCommand(shell, "checkULTRARAPTORPruning", "Checks if pruning rules yield the same results as no pruning.") {
        addParameter("RAPTOR input file");
        addParameter("CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR input file"));
        raptorData.useImplicitDepartureBufferTimes();
        CH::CH ch(getParameter("CH data"));

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(ch.numVertices(), n);

        std::vector<int> results_no_pruning;
        std::vector<int> results_pruning_1;
        std::vector<int> results_pruning_2;

        // Run with pruning rule 0 (no pruning)
        RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler, false> algo_no_pruning(raptorData, ch);
        for (const VertexQuery& query : queries) {
            algo_no_pruning.run(query.source, query.departureTime, query.target);
            results_no_pruning.push_back(algo_no_pruning.getEarliestArrivalTime());
        }
        std::cout << "--- Statistics for No Pruning (Rule 0) ---" << std::endl;
        algo_no_pruning.getProfiler().printStatistics();


        // Run with pruning rule 1
        raptorData.sortTransferGraphEdgesByTravelTime();
        RAPTOR::ULTRARAPTOR_prune<RAPTOR::AggregateProfiler, false> algo_pruning_1(raptorData, ch);
        for (const VertexQuery& query : queries) {
            algo_pruning_1.run(query.source, query.departureTime, query.target);
            results_pruning_1.push_back(algo_pruning_1.getEarliestArrivalTime());
        }
        std::cout << "--- Statistics for Pruning Rule 1 ---" << std::endl;
        algo_pruning_1.getProfiler().printStatistics();

        // Compare the results
        bool pruning_1_correct = (results_no_pruning == results_pruning_1);

        if (!pruning_1_correct) {
            std::cout << "Pruning rule 1 failed comparison." << std::endl;
        }
    }
};

class RunULTRARAPTORQueries_updated : public ParameterizedCommand {

public:
    RunULTRARAPTORQueries_updated(BasicShell& shell) :
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

class RunTransitiveTBQueries : public ParameterizedCommand {

public:
    RunTransitiveTBQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runTransitiveTBQueries", "Runs the given number of random transitive TB queries.") {
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

class RunULTRATBQueries : public ParameterizedCommand {
public:
    RunULTRATBQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runULTRATBQueries", "Runs the given number of random ULTRA-TB queries.") {
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
