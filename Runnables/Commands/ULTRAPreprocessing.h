#pragma once

#include <iostream>
#include <string>

#include "../../Algorithms/RAPTOR/ULTRA/Builder.h"
#include "../../Algorithms/RAPTOR/ULTRA/McBuilder.h"
#include "../../Algorithms/RAPTOR/ULTRA/MultimodalMcBuilder.h"
#include "../../Algorithms/TripBased/Preprocessing/McULTRABuilder.h"
#include "../../Algorithms/TripBased/Preprocessing/MultimodalMcULTRABuilder.h"
#include "../../Algorithms/TripBased/Preprocessing/ShortcutAugmenter.h"
#include "../../Algorithms/TripBased/Preprocessing/StopEventGraphBuilder.h"
#include "../../Algorithms/TripBased/Preprocessing/ULTRABuilder.h"

#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/RAPTOR/TransferModes.h"
#include "../../DataStructures/TripBased/Data.h"

#include "../../Helpers/MultiThreading.h"
#include "../../Helpers/String/String.h"

#include "../../Shell/Shell.h"

using namespace Shell;

inline TransferGraph getOverheadGraph(const RAPTOR::Data& raptorData, const size_t mode) noexcept {
    DynamicTransferGraph temp;
    Graph::copy(raptorData.transferGraph, temp);
    for (const StopId stop : raptorData.stops()) {
        temp.addVertex(temp.vertexRecord(stop));
    }
    Permutation permutation(Construct::Id, temp.numVertices());
    for (const StopId stop : raptorData.stops()) {
        const size_t newStopId = stop + raptorData.transferGraph.numVertices();
        permutation[stop] = newStopId;
        permutation[newStopId] = stop;
    }
    temp.applyVertexPermutation(permutation);
    for (const StopId stop : raptorData.stops()) {
        const Vertex stopVertex(stop + raptorData.transferGraph.numVertices());
        temp.addEdge(stop, stopVertex).set(TravelTime, RAPTOR::TransferModeOverhead[mode]);
        temp.addEdge(stopVertex, stop).set(TravelTime, 0);
    }
    TransferGraph result;
    Graph::move(std::move(temp), result);
    return result;
}

class ComputeStopToStopShortcuts : public ParameterizedCommand {

public:
    ComputeStopToStopShortcuts(BasicShell& shell) :
        ParameterizedCommand(shell, "computeStopToStopShortcuts", "Computes stop-to-stop transfer shortcuts using ULTRA.") {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Witness limit");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
        addParameter("Count optimal candidates?", "false");
        addParameter("Ignore isolated candidates?", "false");
    }

    virtual void execute() noexcept {
        if (getParameter<bool>("Count optimal candidates?")) {
            chooseIgnoreIsolated<true>();
        } else {
            chooseIgnoreIsolated<false>();
        }
    }

private:
    inline size_t getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }

    template<bool COUNT_OPTIMAL_CANDIDATES>
    inline void chooseIgnoreIsolated() const noexcept {
        if (getParameter<bool>("Ignore isolated candidates?")) {
            run<COUNT_OPTIMAL_CANDIDATES, true>();
        } else {
            run<COUNT_OPTIMAL_CANDIDATES, false>();
        }
    }

    template<bool COUNT_OPTIMAL_CANDIDATES, bool IGNORE_ISOLATED_CANDIDATES>
    inline void run() const noexcept {
        const std::string inputFile = getParameter("Input file");
        const size_t witnessLimit = getParameter<size_t>("Witness limit");
        const std::string outputFile = getParameter("Output file");
        const size_t numberOfThreads = getNumberOfThreads();
        const size_t pinMultiplier = getParameter<size_t>("Pin multiplier");

        RAPTOR::Data data(inputFile);
        data.useImplicitDepartureBufferTimes();
        data.printInfo();

        RAPTOR::ULTRA::Builder<false, COUNT_OPTIMAL_CANDIDATES, IGNORE_ISOLATED_CANDIDATES> shortcutGraphBuilder(data);
        std::cout << "Computing stop-to-stop ULTRA shortcuts (parallel with " << numberOfThreads << " threads)." << std::endl;
        shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfThreads, pinMultiplier), witnessLimit);
        Graph::move(std::move(shortcutGraphBuilder.getShortcutGraph()), data.transferGraph);

        data.dontUseImplicitDepartureBufferTimes();
        Graph::printInfo(data.transferGraph);
        data.transferGraph.printAnalysis();
        data.serialize(outputFile);
    }
};

class ComputeMcStopToStopShortcuts : public ParameterizedCommand {

public:
    ComputeMcStopToStopShortcuts(BasicShell& shell) :
        ParameterizedCommand(shell, "computeMcStopToStopShortcuts", "Computes stop-to-stop transfer shortcuts using ULTRA.") {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Intermediate witness limit");
        addParameter("Final witness limit");
        addParameter("Use arrival key?");
        addParameter("Perform full initial route scans?");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept {
        if (getParameter<bool>("Use arrival key?")) {
            chooseRouteScans<true>();
        } else {
            chooseRouteScans<false>();
        }
    }

private:
    inline size_t getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }

    template<bool USE_ARRIVAL_KEY>
    inline void chooseRouteScans() const noexcept {
        if (getParameter<bool>("Perform full initial route scans?")) {
            run<USE_ARRIVAL_KEY, true>();
        } else {
            run<USE_ARRIVAL_KEY, false>();
        }
    }

    template<bool USE_ARRIVAL_KEY, bool FULL_ROUTE_SCANS>
    inline void run() const noexcept {
        const std::string inputFile = getParameter("Input file");
        const size_t intermediateWitnessLimit = getParameter<size_t>("Intermediate witness limit");
        const size_t finalWitnessLimit = getParameter<size_t>("Final witness limit");
        const std::string outputFile = getParameter("Output file");
        const size_t numberOfThreads = getNumberOfThreads();
        const size_t pinMultiplier = getParameter<size_t>("Pin multiplier");

        RAPTOR::Data data(inputFile);
        data.useImplicitDepartureBufferTimes();
        data.printInfo();

        RAPTOR::ULTRA::McBuilder<false, USE_ARRIVAL_KEY, FULL_ROUTE_SCANS> shortcutGraphBuilder(data);
        std::cout << "Computing multicriteria stop-to-stop ULTRA shortcuts (parallel with " << numberOfThreads << " threads)." << std::endl;
        shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfThreads, pinMultiplier), intermediateWitnessLimit, finalWitnessLimit);
        Graph::move(std::move(shortcutGraphBuilder.getShortcutGraph()), data.transferGraph);

        data.dontUseImplicitDepartureBufferTimes();
        Graph::printInfo(data.transferGraph);
        data.transferGraph.printAnalysis();
        data.serialize(outputFile);
    }
};

class ComputeMultimodalMcStopToStopShortcuts : public ParameterizedCommand {

public:
    ComputeMultimodalMcStopToStopShortcuts(BasicShell& shell) :
        ParameterizedCommand(shell, "computeMultimodalMcStopToStopShortcuts", "Computes multimodal multicriteria stop-to-stop transfer shortcuts using ULTRA.") {
        addParameter("RAPTOR data");
        addParameter("Transitive transfer graph");
        addParameter("Mode");
        addParameter("Output file");
        addParameter("Intermediate witness limit");
        addParameter("Final witness limit");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
        addParameter("Discretization factor", "1", {"1", "300", "600", "900"});
    }

    virtual void execute() noexcept {
        switch (getParameter<int>("Discretization factor")) {
            case 1:
                run<1>();
                break;
            case 300:
                run<300>();
                break;
            case 600:
                run<600>();
                break;
            case 900:
                run<900>();
                break;
        }
    }

private:
    inline int getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }

    template<int TIME_FACTOR>
    inline void run() const noexcept {
        const std::string inputFile = getParameter("RAPTOR data");
        const size_t mode = RAPTOR::getTransferModeFromName(getParameter("Mode"));
        const std::string outputFile = getParameter("Output file");
        const int numberOfThreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Pin multiplier");
        const int intermediateWitnessLimit = getParameter<int>("Intermediate witness limit");
        const int finalWitnessLimit = getParameter<int>("Final witness limit");

        RAPTOR::Data data(inputFile);
        data.useImplicitDepartureBufferTimes();
        data.printInfo();
        data.transferGraph = getOverheadGraph(data, mode);
        TransferGraph transitiveTransferGraph;
        transitiveTransferGraph.readBinary(getParameter("Transitive transfer graph"));

        RAPTOR::ULTRA::MultimodalMcBuilder<false, TIME_FACTOR> shortcutGraphBuilder(data, transitiveTransferGraph);
        std::cout << "Computing multimodal multicriteria stop-to-stop ULTRA shortcuts (parallel with " << numberOfThreads << " threads)." << std::endl;
        shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfThreads, pinMultiplier), intermediateWitnessLimit, finalWitnessLimit);
        Graph::move(std::move(shortcutGraphBuilder.getShortcutGraph()), data.transferGraph);

        data.dontUseImplicitDepartureBufferTimes();
        Graph::printInfo(data.transferGraph);
        data.transferGraph.printAnalysis();
        data.serialize(outputFile);
    }
};

class RAPTORToTripBased : public ParameterizedCommand {

public:
    RAPTORToTripBased(BasicShell& shell) :
        ParameterizedCommand(shell, "raptorToTripBased", "Converts stop-to-stop transfers to event-to-event transfers and saves the resulting network in Trip-Based format.") {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Route-based pruning?");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");
        const bool routeBasedPruning = getParameter<bool>("Route-based pruning?");
        const int numberOfThreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Pin multiplier");

        RAPTOR::Data raptor(inputFile);
        raptor.printInfo();
        TripBased::Data data(raptor);

        if (numberOfThreads == 0) {
            if (routeBasedPruning) {
                TripBased::ComputeStopEventGraphRouteBased(data);
            } else {
                TripBased::ComputeStopEventGraph(data);
            }
        } else {
            if (routeBasedPruning) {
                TripBased::ComputeStopEventGraphRouteBased(data, numberOfThreads, pinMultiplier);
            } else {
                TripBased::ComputeStopEventGraph(data, numberOfThreads, pinMultiplier);
            }
        }

        data.printInfo();
        data.serialize(outputFile);
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

class ComputeEventToEventShortcuts : public ParameterizedCommand {

public:
    ComputeEventToEventShortcuts(BasicShell& shell) :
        ParameterizedCommand(shell, "computeEventToEventShortcuts", "Computes event-to-event transfer shortcuts using ULTRA and saves the resulting network in Trip-Based format.") {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Witness limit");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
        addParameter("Ignore isolated candidates?", "false");
    }

    virtual void execute() noexcept {
        if (getParameter<bool>("Ignore isolated candidates?")) {
            run<true>();
        } else {
            run<false>();
        }
    }

private:
    inline int getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }

    template<bool IGNORE_ISOLATED_CANDIDATES>
    inline void run() noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");
        const int witnessLimit = getParameter<int>("Witness limit");
        const int numberOfThreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Pin multiplier");

        RAPTOR::Data raptor(inputFile);
        raptor.printInfo();
        TripBased::Data data(raptor);

        TripBased::ULTRABuilder<false, IGNORE_ISOLATED_CANDIDATES> shortcutGraphBuilder(data);
        std::cout << "Computing event-to-event ULTRA shortcuts (parallel with " << numberOfThreads << " threads)." << std::endl;
        shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfThreads, pinMultiplier), witnessLimit);
        Graph::move(std::move(shortcutGraphBuilder.getStopEventGraph()), data.stopEventGraph);

        data.printInfo();
        data.serialize(outputFile);
    }

};

class ComputeMcEventToEventShortcuts : public ParameterizedCommand {

public:
    ComputeMcEventToEventShortcuts(BasicShell& shell) :
        ParameterizedCommand(shell, "computeMcEventToEventShortcuts", "Computes multicriteria event-to-event transfer shortcuts using ULTRA and saves the resulting network in Trip-Based format.") {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Intermediate witness limit");
        addParameter("Final witness limit");
        addParameter("Use arrival key?");
        addParameter("Perform full initial route scans?");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept {
        if (getParameter<bool>("Use arrival key?")) {
            chooseRouteScans<true>();
        } else {
            chooseRouteScans<false>();
        }
    }

private:
    inline int getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }

    template<bool USE_ARRIVAL_KEY>
    inline void chooseRouteScans() const noexcept {
        if (getParameter<bool>("Perform full initial route scans?")) {
            run<USE_ARRIVAL_KEY, true>();
        } else {
            run<USE_ARRIVAL_KEY, false>();
        }
    }

    template<bool USE_ARRIVAL_KEY, bool FULL_ROUTE_SCANS>
    inline void run() const noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");
        const int numberOfThreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Pin multiplier");
        const int intermediateWitnessLimit = getParameter<int>("Intermediate witness limit");
        const int finalWitnessLimit = getParameter<int>("Final witness limit");

        RAPTOR::Data raptor(inputFile);
        raptor.printInfo();
        TripBased::Data data(raptor);

        TripBased::McULTRABuilder<false, USE_ARRIVAL_KEY, FULL_ROUTE_SCANS> shortcutGraphBuilder(data);
        std::cout << "Computing multicriteria event-to-event ULTRA shortcuts (parallel with " << numberOfThreads << " threads)." << std::endl;
        shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfThreads, pinMultiplier), intermediateWitnessLimit, finalWitnessLimit);
        Graph::move(std::move(shortcutGraphBuilder.getStopEventGraph()), data.stopEventGraph);

        data.printInfo();
        data.serialize(outputFile);
    }

};

class ComputeMultimodalMcEventToEventShortcuts : public ParameterizedCommand {

public:
    ComputeMultimodalMcEventToEventShortcuts(BasicShell& shell) :
        ParameterizedCommand(shell, "computeMultimodalMcEventToEventShortcuts", "Computes multimodal multicriteria event-to-event transfer shortcuts using ULTRA and saves the resulting network in Trip-Based format.") {
        addParameter("RAPTOR data");
        addParameter("Transitive transfer graph");
        addParameter("Mode");
        addParameter("Output file");
        addParameter("Intermediate witness limit");
        addParameter("Final witness limit");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
        addParameter("Discretization factor", "1", {"1", "300", "600", "900"});
    }

    virtual void execute() noexcept {
        switch (getParameter<int>("Discretization factor")) {
            case 1:
                run<1>();
                break;
            case 300:
                run<300>();
                break;
            case 600:
                run<600>();
                break;
            case 900:
                run<900>();
                break;
        }
    }

private:
    inline int getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }

    template<int TIME_FACTOR>
    inline void run() const noexcept {
        const std::string inputFile = getParameter("RAPTOR data");
        const size_t mode = RAPTOR::getTransferModeFromName(getParameter("Mode"));
        const std::string outputFile = getParameter("Output file");
        const int numberOfThreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Pin multiplier");
        const int intermediateWitnessLimit = getParameter<int>("Intermediate witness limit");
        const int finalWitnessLimit = getParameter<int>("Final witness limit");

        RAPTOR::Data raptor(inputFile);
        raptor.printInfo();
        raptor.transferGraph = getOverheadGraph(raptor, mode);
        TripBased::Data data(raptor);
        TransferGraph transitiveTransferGraph;
        transitiveTransferGraph.readBinary(getParameter("Transitive transfer graph"));

        TripBased::MultimodalMcULTRABuilder<false, TIME_FACTOR> shortcutGraphBuilder(data, transitiveTransferGraph);
        std::cout << "Computing multimodal multicriteria event-to-event ULTRA shortcuts (parallel with " << numberOfThreads << " threads)." << std::endl;
        shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfThreads, pinMultiplier), intermediateWitnessLimit, finalWitnessLimit);
        Graph::move(std::move(shortcutGraphBuilder.getStopEventGraph()), data.stopEventGraph);

        data.printInfo();
        data.serialize(outputFile);
    }
};

class AugmentTripBasedShortcuts : public ParameterizedCommand {

public:
    AugmentTripBasedShortcuts(BasicShell& shell) :
        ParameterizedCommand(shell, "augmentTripBasedData", "Augments Trip-Based shortcuts for bounded multicriteria search.") {
        addParameter("Input file");
        addParameter("Forward output file");
        addParameter("Backward output file");
        addParameter("Trip limit", "1073741823");
    }

    virtual void execute() noexcept {
        TripBased::Data data(getParameter("Input file"));
        data.printInfo();
        TripBased::Data reverseData = data.reverseNetwork();
        TripBased::ShortcutAugmenter augmenter;
        const size_t tripLimit = getParameter<size_t>("Trip limit");
        augmenter.augmentShortcuts(data, tripLimit);
        augmenter.augmentShortcuts(reverseData, tripLimit);
        data.serialize(getParameter("Forward output file"));
        reverseData.serialize(getParameter("Backward output file"));
    }
};

inline void validateShortcutGraph(const TransferGraph& original, const TransferGraph& shortcuts, const Vertex maxVertex) noexcept {
    std::vector<int> shortcutDistance(std::max(original.numVertices(), shortcuts.numVertices()), 0);
    IndexedSet<false, Vertex> targets(shortcutDistance.size());
    Dijkstra<RAPTOR::TransferGraph, false> originalDijkstra(original);
    Dijkstra<RAPTOR::TransferGraph, false> shortcutDijkstra(shortcuts);
    size_t longCount = 0;
    size_t longPathCount = 0;
    size_t shortCount = 0;
    size_t missingCount = 0;
    Progress progress(shortcuts.numVertices());
    for (const Vertex vertex : shortcuts.vertices()) {
        if (vertex == maxVertex) break;
        for (const Edge edge : shortcuts.edgesFrom(vertex)) {
            const Vertex target = shortcuts.get(ToVertex, edge);
            if (target >= maxVertex) continue;
            shortcutDistance[target] = shortcuts.get(TravelTime, edge);
            targets.insert(target);
        }
        originalDijkstra.run(vertex, targets, [&](const Vertex other){
            if (targets.contains(other)) {
                if (originalDijkstra.getDistance(other) < shortcutDistance[other]) {
                    longCount++;
                    std::cout << "\nShortcut from " << vertex << " to " << other << " is too long (Distance should be " << originalDijkstra.getDistance(other) << ", but is " << shortcutDistance[other] << ")!" << std::endl;
                    shortcutDijkstra.run(vertex, other);
                    if (shortcutDijkstra.getDistance(other) == originalDijkstra.getDistance(other)) {
                        std::cout << "   But a path of the same length still exists." << std::endl;
                    } else {
                        longPathCount++;
                        std::cout << "   No path of the same length exists." << std::endl;
                    }
                } else if (originalDijkstra.getDistance(other) > shortcutDistance[other]) {
                    shortCount++;
                    std::cout << "\nShortcut from " << vertex << " to " << other << " is too short (Distance should be " << originalDijkstra.getDistance(other) << ", but is " << shortcutDistance[other] << ")!" << std::endl;
                }
            }
        });
        for (const Vertex other : targets) {
            missingCount++;
            std::cout << "\nOriginal graph does not contain a path from " << vertex << " to " << other << "!" << std::endl;
        }
        progress++;
    }
    std::cout << std::endl;
    if (longCount == 0) {
        std::cout << green("No shortcut is too long!") << std::endl;
    } else {
        std::cout << yellow("Number of shortcuts that are too long: ", longCount) << std::endl;
        if (longPathCount == 0) {
            std::cout << green("No shortest path is too long!") << std::endl;
        } else {
            std::cout << red("Number of shortest paths that are too long: ", longPathCount) << std::endl;
        }
    }
    if (shortCount == 0) {
        std::cout << green("No shortcut is too short!") << std::endl;
    } else {
        std::cout << red("Number of shortcuts that are too short: ", shortCount) << std::endl;
    }
    if (missingCount == 0) {
        std::cout << green("All shortcuts represent existing paths!") << std::endl;
    } else {
        std::cout << red("Number of shortcuts that do not exist in the original graph: ", missingCount) << std::endl;
    }
}

class ValidateStopToStopShortcuts : public ParameterizedCommand {

public:
    ValidateStopToStopShortcuts(BasicShell& shell) :
        ParameterizedCommand(shell, "validateStopToStopShortcuts", "Compares stop-to-stop ULTRA shortcuts to paths in the original transfer graph.") {
        addParameter("Original graph");
        addParameter("Shortcut graph");
        addParameter("Max vertex", "-1");
    }

    virtual void execute() noexcept {
        const TransferGraph original(getParameter("Original graph"));
        Graph::printInfo(original);
        original.printAnalysis();
        const TransferGraph shortcuts(getParameter("Shortcut graph"));
        Graph::printInfo(shortcuts);
        shortcuts.printAnalysis();
        const int maxVertexNum = getParameter<int>("Max vertex");
        const Vertex maxVertex(maxVertexNum == -1 ? shortcuts.numVertices() : maxVertexNum);
        validateShortcutGraph(original, shortcuts, maxVertex);
    }
};

class ValidateEventToEventShortcuts : public ParameterizedCommand {

public:
    ValidateEventToEventShortcuts(BasicShell& shell) :
        ParameterizedCommand(shell, "validateEventToEventShortcuts", "Compares event-to-event ULTRA shortcuts to paths in the original transfer graph.") {
        addParameter("Original graph");
        addParameter("ULTRA-TB data");
    }

    virtual void execute() noexcept {
        const TransferGraph original(getParameter("Original graph"));
        Graph::printInfo(original);
        original.printAnalysis();
        const TripBased::Data ultraTBData(getParameter("ULTRA-TB data"));
        ultraTBData.printInfo();


        DynamicTransferGraph stopShortcuts;
        stopShortcuts.addVertices(ultraTBData.numberOfStops());
        for (const Vertex fromEventVertex : ultraTBData.stopEventGraph.vertices()) {
            const StopId fromStop = ultraTBData.getStopOfStopEvent(StopEventId(fromEventVertex));
            const int arrivalTime = ultraTBData.raptorData.stopEvents[fromEventVertex].arrivalTime;
            for (const Edge edge : ultraTBData.stopEventGraph.edgesFrom(fromEventVertex)) {
                const Vertex toEventVertex = ultraTBData.stopEventGraph.get(ToVertex, edge);
                const int departureTime = ultraTBData.raptorData.stopEvents[toEventVertex].departureTime;
                const int travelTime = ultraTBData.stopEventGraph.get(TravelTime, edge);
                if (travelTime > departureTime - arrivalTime) {
                    std::cout << "\nShortcut from " << fromEventVertex << " to " << toEventVertex << " is too long to reach destination stop event! (Should be at most " << departureTime - arrivalTime << ", but is " << travelTime << ")!" << std::endl;
                }

                const StopId toStop = ultraTBData.getStopOfStopEvent(StopEventId(toEventVertex));
                const Edge shortcutEdge = stopShortcuts.findEdge(fromStop, toStop);
                if (shortcutEdge == noEdge) {
                    stopShortcuts.addEdge(fromStop, toStop).set(TravelTime, travelTime);
                } else {
                    const int realTravelTime = stopShortcuts.get(TravelTime, shortcutEdge);
                    if (realTravelTime != travelTime) {
                        std::cout << "\nShortcut from " << fromEventVertex << " to " << toEventVertex << " has inconsistent distance (Should be " << realTravelTime << ", but is " << travelTime << ")!" << std::endl;
                    }
                }
            }
        }

        TransferGraph shortcuts;
        Graph::move(std::move(stopShortcuts), shortcuts);
        validateShortcutGraph(original, shortcuts, Vertex(original.numVertices()));
    }
};
