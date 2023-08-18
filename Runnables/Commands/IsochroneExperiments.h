#pragma once

#include <iostream>
#include <vector>

#include "../../Shell/Shell.h"

#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/CSA/IsoDijkstraCSA.h"
#include "../../Algorithms/CSA/IsoPHASTCSA.h"
#include "../../Algorithms/CSA/OneToAllDijkstraCSA.h"
#include "../../Algorithms/Dijkstra/IsoDijkstra.h"
#include "../../Algorithms/Isochrone/DistanceBoundComputation.h"
#include "../../Algorithms/Isochrone/EccentricityComputation.h"
#include "../../Algorithms/Isochrone/IsoPHAST.h"
#include "../../Algorithms/Partition/InertialFlow.h"
#include "../../Algorithms/GreedyVertexColoring.h"
#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/Partition/VertexPartition.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../Helpers/Console/Progress.h"
#include "../../Helpers/IO/Serialization.h"
#include "../../Helpers/Vector/Vector.h"

#include "../../Visualization/MapVisualization.h"
#include "../../Visualization/TimeTableVisualization.h"
#include "../../Visualization/PDF.h"
#include "../../Visualization/PNG.h"
#include "../../Visualization/SVG.h"

using namespace Shell;

class ComputePartition : public ParameterizedCommand {

public:
    ComputePartition(BasicShell& shell) :
        ParameterizedCommand(shell, "computePartition", "Computes a vertex partition for the given graph and visualizes the result.") {
        addParameter("Input graph");
        addParameter("Output file");
        addParameter("Format", { "pdf", "png", "svg" });
        addParameter("Maximum cell size");
        addParameter("Fixed vertices", "0.2");
        addParameter("Number of threads", "-1");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept {
        const TransferGraph graph(getParameter("Input graph"));
        Graph::printInfo(graph);
        graph.printAnalysis();
        const VertexPartition vp = computePartition(graph);
        const std::string format = getParameter("Format");
        const std::string outputFile = getParameter("Output file");
        if (format == "pdf") {
            drawPartition<PDF>(graph, vp, outputFile);
        } else if (format == "png") {
            drawPartition<PNG>(graph, vp, outputFile);
        } else {
            drawPartition<SVG>(graph, vp, outputFile);
        }
        std::cout << "Saving partition." << std::endl;
        vp.serialize(outputFile + ".vp");
    }

private:
    inline VertexPartition computePartition(const TransferGraph& graph) const noexcept {
        const int numberOfThreads = getNumberOfThreads();
        const size_t pinMultiplier = getParameter<size_t>("Pin multiplier");
        const size_t maxCellSize = getParameter<size_t>("Maximum cell size");
        const double fixedVertices = getParameter<double>("Fixed vertices");
        std::cout << "Initializing InertialFlow." << std::endl;
        InertialFlowOnEdges<true, InertialFlow::QUANTITY> inertialFlow(graph, graph[Coordinates]);
        if (numberOfThreads <= 0) {
            std::cout << "Computing partition (sequential)." << std::endl;
            return inertialFlow.runOnConnectedComponents(maxCellSize, fixedVertices);
        } else {
            const ThreadPinning threadPinning(numberOfThreads, pinMultiplier);
            std::cout << "Computing partition (parallel with " << numberOfThreads << " threads)." << std::endl;
            return inertialFlow.runOnConnectedComponents(threadPinning, maxCellSize, fixedVertices);
        }
    }

    template<typename FORMAT>
    inline void drawPartition(const TransferGraph& graph, const VertexPartition& vp, const std::string& outputFile) const noexcept {
        std::cout << "Drawing partition." << std::endl;
        MapVisualization<FORMAT> doc(outputFile, graph[Coordinates], 0.3);
        const std::vector<int> cellColors = greedyVertexColors(vp.getCellGraph(graph));
        for (const Vertex vertex : vp.getBorderVertices(graph)) {
            const size_t cell = vp.getCellIdOfVertex(vertex);
            doc.drawPoint(graph.get(Coordinates, vertex), cyclicColor(cellColors[cell]), Icon::Dot, 10);
        }
        for (const VertexPartition::CutEdge edge : vp.getCutEdges(graph)) {
            doc.drawLine(graph.get(Coordinates, edge.from), graph.get(Coordinates, edge.to));
        }
        doc.close();
    }

    inline int getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        }
        else {
            return getParameter<int>("Number of threads");
        }
    }
};

class RunIsoPHASTPreprocessing : public ParameterizedCommand {

public:
    RunIsoPHASTPreprocessing(BasicShell& shell) :
        ParameterizedCommand(shell, "runIsoPHASTPreprocessing", "Runs the IsoPHAST preprocessing for the given graph and partition.") {
        addParameter("Graph binary");
        addParameter("Partition binary");
        addParameter("Output directory");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept {
        const size_t numberOfThreads = getNumberOfThreads();
        IsoPHASTData data(getParameter("Graph binary"), getParameter("Partition binary"), numberOfThreads, getParameter<size_t>("Pin multiplier"));
        data.computeEccentricities();
        data.computeCH();
        data.computeDistanceBounds();
        data.serialize(getParameter("Output directory"));
    }

private:
    struct IsoPHASTData {
        IsoPHASTData(const std::string& graphFile, const std::string& partitionFile, const size_t numberOfThreads, const size_t pinMultiplier) :
            graph(graphFile),
            partition(partitionFile),
            threadPinning(numberOfThreads, pinMultiplier) {
            Graph::printInfo(graph);
            graph.printAnalysis();
        }

        inline void computeEccentricities() noexcept {
            TransferGraph reverseGraph = graph;
            reverseGraph.revert();
            EccentricityComputation ec(reverseGraph, partition);
            std::cout << "Computing eccentricities with " << threadPinning.numberOfThreads << " threads" << std::endl;
            ec.run(threadPinning);
            eccentricities = ec.getEccentricities();
        }

        inline void computeCH() noexcept {
            using CHProfiler = CH::FullProfiler;
            using CHWitnessSearch = CH::BidirectionalWitnessSearch<CHCoreGraph, CHProfiler, 500>;
            using CHKeyFunction = CH::GreedyKey<CHWitnessSearch>;
            using CHStopCriterion = CH::NoStopCriterion;
            using CHBuilder = CH::Builder<CHProfiler, CHWitnessSearch, CHKeyFunction, CHStopCriterion, false, false>;

            TransferGraph chGraph;
            Graph::copy(graph, chGraph);
            CHBuilder chBuilder(std::move(chGraph), chGraph[TravelTime], CHKeyFunction(), CHStopCriterion());
            chBuilder.run();

            order = Order(chBuilder.getData().order);
            Vector::reverse(order);
            ch = std::move(chBuilder);

            graph.applyVertexOrder(order);
            partition.applyVertexOrder(order);
            ch.applyVertexOrder(order);
            order.order(eccentricities);
        }

        inline void computeDistanceBounds() noexcept {
            DistanceBoundComputation dbc(partition, ch, eccentricities);
            std::cout << "Computing distance bounds with " << threadPinning.numberOfThreads << " threads" << std::endl;
            dbc.run(threadPinning);
            distanceBounds = dbc.getDistanceBounds();
        }

        inline void serialize(const std::string& outputDirectory) const noexcept {
            graph.writeBinary(outputDirectory + "graph");
            partition.serialize(outputDirectory + "partition.vp");
            ch.writeBinary(outputDirectory + "ch");
            IO::serialize(outputDirectory + "eccentricities", eccentricities);
            IO::serialize(outputDirectory + "order", order);
            IO::serialize(outputDirectory + "bounds", distanceBounds);
        }

        TransferGraph graph;
        VertexPartition partition;
        Order order;
        CH::CH ch;
        std::vector<int> eccentricities;
        std::vector<std::vector<DistanceBounds>> distanceBounds;
        ThreadPinning threadPinning;
    };

    inline size_t getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        }
        else {
            return getParameter<size_t>("Number of threads");
        }
    }
};

class ValidateIsoPHASTQuery : public ParameterizedCommand {

public:
    ValidateIsoPHASTQuery(BasicShell& shell) :
        ParameterizedCommand(shell, "validateIsoPHASTQuery", "Compares IsoPHAST to IsoDijkstra for the given query.") {
        addParameter("Graph binary");
        addParameter("CH binary");
        addParameter("Partition binary");
        addParameter("Distance bounds binary");
        addParameter("Shared downward graph size");
        addParameter("Source");
        addParameter("Range");
        //addParameter("Number of threads", "max");
        //addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept {
        TransferGraph graph(getParameter("Graph binary"));
        CH::CH ch(getParameter("CH binary"));
        VertexPartition partition(getParameter("Partition binary"));
        std::vector<std::vector<DistanceBounds>> distanceBounds;
        IO::deserialize(getParameter("Distance bounds binary"), distanceBounds);
        IsoDijkstra<TransferGraph> isoDijkstra(graph);
        Dijkstra<TransferGraph> dijkstra(graph);
        IsoPHAST isoPHAST(partition, ch, distanceBounds, getParameter<size_t>("Shared downward graph size"));

        const Vertex source = getParameter<Vertex>("Source");
        const int range = getParameter<int>("Range");
        isoDijkstra.run(source, range, TravelTime);
        dijkstra.run(source);
        //const size_t numberOfThreads = getNumberOfThreads();
        //const size_t pinMultiplier = getParameter<size_t>("Pin multiplier");
        //const ThreadPinning threadPinning(numberOfThreads, pinMultiplier);
        isoPHAST.run(source, range);

        const IndexedSet<false, Vertex> dijkstraIsochrone(graph.numVertices(), isoDijkstra.getIsochrone());
        const IndexedSet<false, Vertex> phastIsochrone(graph.numVertices(), isoPHAST.getIsochrone());
        for (const Vertex vertex : graph.vertices()) {
            if (dijkstraIsochrone.contains(vertex)) {
                Ensure(dijkstra.getDistance(vertex) == isoDijkstra.getDistance(vertex), "IsoDijkstra found distance " << dijkstra.getDistance(vertex) << " at " << vertex << ", should be " << isoDijkstra.getDistance(vertex));
            } else {
                Ensure(isoDijkstra.getDistance(vertex) == INFTY, "Distance at out-of-range vertex " << vertex << " is not INFTY");
            }
            if (dijkstraIsochrone.contains(vertex) != phastIsochrone.contains(vertex)) {
                std::cout << "Vertex " << vertex << " only in " << (dijkstraIsochrone.contains(vertex) ? "Dijkstra" : "PHAST") << " isochrone" << std::endl;
                std::cout << "Dijkstra distance: " << dijkstra.getDistance(vertex) << std::endl;
                std::cout << "Cell " << partition.getCellIdOfVertex(vertex) << std::endl;
            }
        }
    }

/*private:
    inline size_t getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        }
        else {
            return getParameter<size_t>("Number of threads");
        }
    }*/
};

class ValidateIsoPHAST : public ParameterizedCommand {

public:
    ValidateIsoPHAST(BasicShell& shell) :
        ParameterizedCommand(shell, "validateIsoPHAST", "Compares IsoPHAST to IsoDijkstra on random queries.") {
        addParameter("Graph binary");
        addParameter("CH binary");
        addParameter("Partition binary");
        addParameter("Distance bounds binary");
        addParameter("Shared downward graph size");
        addParameter("Number of queries");
        //addParameter("Number of threads", "max");
        //addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept {
        TransferGraph graph(getParameter("Graph binary"));
        Dijkstra<TransferGraph> dijkstra(graph);
        srand(42);
        dijkstra.run(Vertex(rand() % graph.numVertices()));
        int maxDistance = 0;
        for (const Vertex vertex : graph.vertices()) {
            if (dijkstra.reachable(vertex)) {
                maxDistance = std::max(maxDistance, dijkstra.getDistance(vertex));
            }
        }

        CH::CH ch(getParameter("CH binary"));
        VertexPartition partition(getParameter("Partition binary"));
        std::vector<std::vector<DistanceBounds>> distanceBounds;
        IO::deserialize(getParameter("Distance bounds binary"), distanceBounds);
        IsoDijkstra<TransferGraph> isoDijkstra(graph);
        IsoPHAST isoPHAST(partition, ch, distanceBounds, getParameter<size_t>("Shared downward graph size"));

        const size_t numQueries = getParameter<size_t>("Number of queries");
        //const size_t numberOfThreads = getNumberOfThreads();
        //const size_t pinMultiplier = getParameter<size_t>("Pin multiplier");
        //const ThreadPinning threadPinning(numberOfThreads, pinMultiplier);

        Progress progress(numQueries);
        for (size_t i = 0; i < numQueries; i++) {
            const Vertex source(rand() % graph.numVertices());
            for (int range = 1; range <= maxDistance; range *= 2) {
                isoDijkstra.run(source, range, TravelTime);
                isoPHAST.run(source, range);

                const IndexedSet<false, Vertex> dijkstraIsochrone(graph.numVertices(), isoDijkstra.getIsochrone());
                const IndexedSet<false, Vertex> phastIsochrone(graph.numVertices(), isoPHAST.getIsochrone());
                for (const Vertex vertex : graph.vertices()) {
                    if (dijkstraIsochrone.contains(vertex) != phastIsochrone.contains(vertex)) {
                        std::cout << "Source " << source << ", range " << range << std::endl;
                        std::cout << "Vertex " << vertex << " only in " << (dijkstraIsochrone.contains(vertex) ? "Dijkstra" : "PHAST") << " isochrone" << std::endl;
                        std::cout << "IsoDijkstra distance: " << isoDijkstra.getDistance(vertex) << std::endl;
                        std::cout << "Cell " << partition.getCellIdOfVertex(vertex) << std::endl;
                    }
                }
            }
            progress++;
        }
    }

/*private:
    inline size_t getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        }
        else {
            return getParameter<size_t>("Number of threads");
        }
    }*/
};

class ValidateIsoDijkstraCSA : public ParameterizedCommand {

public:
    ValidateIsoDijkstraCSA(BasicShell& shell) :
        ParameterizedCommand(shell, "validateIsoDijkstraCSA", "Compares IsoDijkstraCSA to DijkstraCSA on random queries.") {
        addParameter("CSA binary");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA binary"));
        csaData.sortConnectionsAscending();
        Dijkstra<CSA::TransferGraph> dijkstra(csaData.transferGraph);
        srand(42);
        dijkstra.run(Vertex(rand() % csaData.transferGraph.numVertices()));
        int maxDistance = 0;
        for (const Vertex vertex : csaData.transferGraph.vertices()) {
            if (dijkstra.reachable(vertex)) {
                maxDistance = std::max(maxDistance, dijkstra.getDistance(vertex));
            }
        }

        CSA::IsoDijkstraCSA isoDijkstraCSA(csaData);
        CSA::OneToAllDijkstraCSA<CSA::TransferGraph, true> dijkstraCSA(csaData);

        const size_t numQueries = getParameter<size_t>("Number of queries");
        Progress progress(numQueries);
        for (size_t i = 0; i < numQueries; i++) {
            const Vertex source(rand() % csaData.transferGraph.numVertices());
            const int departureTime(rand() % 24 * 60 * 60);
            for (int range = 1; range <= maxDistance; range *= 2) {
                dijkstraCSA.run(source, departureTime);
                isoDijkstraCSA.run(source, departureTime, range);
                const IndexedSet<false, Vertex> isochrone(csaData.transferGraph.numVertices(), isoDijkstraCSA.getIsochrone());

                for (const Vertex vertex : csaData.transferGraph.vertices()) {
                    const int arrivalTime = dijkstraCSA.getEarliestArrivalTime(vertex);
                    const int isoArrivalTime = isoDijkstraCSA.getEarliestArrivalTime(vertex);
                    if (isochrone.contains(vertex)) {
                        if (isoArrivalTime != arrivalTime) {
                            std::cout << "Source " << source << ", range " << range << ", departure time " << departureTime << std::endl;
                            std::cout << "IsoDijkstraCSA found arrival time " << isoArrivalTime << " at vertex " << vertex << std::endl;
                            std::cout << "Real arrival time: " << arrivalTime << std::endl;
                            std::cout << "Journey:" << std::endl;
                            std::cout << dijkstraCSA.getJourney(vertex);
                            Ensure(false, "");
                        } else if (isoArrivalTime > departureTime + range) {
                            std::cout << "Source " << source << ", range " << range << ", departure time " << departureTime << std::endl;
                            std::cout << "Out-of-range vertex " << vertex << " with arrival time " << isoArrivalTime << " added to isochrone" << std::endl;
                            std::cout << "Journey:" << std::endl;
                            std::cout << dijkstraCSA.getJourney(vertex);
                            Ensure(false, "");
                        }
                    } else {
                        if (arrivalTime <= departureTime + range) {
                            std::cout << "Source " << source << ", range " << range << ", departure time " << departureTime << std::endl;
                            std::cout << "In-range vertex " << vertex << " with arrival time " << arrivalTime << " not added to isochrone" << std::endl;
                            std::cout << "Iso arrival time: " << isoArrivalTime << std::endl;
                            std::cout << "Journey:" << std::endl;
                            std::cout << dijkstraCSA.getJourney(vertex);
                            Ensure(false, "");
                        } else if (isoArrivalTime <= departureTime + range) {
                            std::cout << "Source " << source << ", range " << range << ", departure time " << departureTime << std::endl;
                            std::cout << "IsoDijkstraCSA found arrival time " << isoArrivalTime << " at out-of-range vertex " << vertex << std::endl;
                            std::cout << "Real arrival time: " << arrivalTime << std::endl;
                            std::cout << "Journey:" << std::endl;
                            std::cout << dijkstraCSA.getJourney(vertex);
                            Ensure(false, "");
                        }
                    }
                }
            }
            progress++;
        }
    }
};

class ValidateIsoPHASTCSA : public ParameterizedCommand {

public:
    ValidateIsoPHASTCSA(BasicShell& shell) :
        ParameterizedCommand(shell, "validateIsoPHASTCSA", "Compares IsoPHASTCSA to IsoDijkstraCSA on random queries.") {
        addParameter("Dijkstra CSA binary");
        addParameter("ULTRA CSA binary");
        addParameter("Order binary");
        addParameter("CH binary");
        addParameter("Partition binary");
        addParameter("Distance bounds binary");
        addParameter("Shared downward graph size");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        CSA::Data dijkstraCSAData = CSA::Data::FromBinary(getParameter("Dijkstra CSA binary"));
        dijkstraCSAData.sortConnectionsAscending();
        Dijkstra<CSA::TransferGraph> dijkstra(dijkstraCSAData.transferGraph);
        srand(42);
        dijkstra.run(Vertex(rand() % dijkstraCSAData.transferGraph.numVertices()));
        int maxDistance = 0;
        for (const Vertex vertex : dijkstraCSAData.transferGraph.vertices()) {
            if (dijkstra.reachable(vertex)) {
                maxDistance = std::max(maxDistance, dijkstra.getDistance(vertex));
            }
        }
        CSA::Data ultraCSAData = CSA::Data::FromBinary(getParameter("ULTRA CSA binary"));
        ultraCSAData.sortConnectionsAscending();

        Order order;
        IO::deserialize(getParameter("Order binary"), order);
        CH::CH ch(getParameter("CH binary"));
        VertexPartition partition(getParameter("Partition binary"));
        std::vector<std::vector<DistanceBounds>> distanceBounds;
        IO::deserialize(getParameter("Distance bounds binary"), distanceBounds);
        CSA::OneToAllDijkstraCSA<CSA::TransferGraph, true> dijkstraCSA(dijkstraCSAData);
        CSA::IsoDijkstraCSA isoDijkstraCSA(dijkstraCSAData);
        CSA::IsoPHASTCSA isoPHASTCSA(ultraCSAData, order, partition, ch, distanceBounds, getParameter<size_t>("Shared downward graph size"));

        const size_t numQueries = getParameter<size_t>("Number of queries");
        Progress progress(numQueries);
        for (size_t i = 0; i < numQueries; i++) {
            const Vertex source(rand() % dijkstraCSAData.transferGraph.numVertices());
            const int departureTime(rand() % 24 * 60 * 60);
            for (int range = 1; range <= maxDistance; range *= 2) {
                dijkstraCSA.run(source, departureTime);
                isoDijkstraCSA.run(source, departureTime, range);
                isoPHASTCSA.run(source, departureTime, range);
                const IndexedSet<false, Vertex> dijkstraIsochrone(dijkstraCSAData.transferGraph.numVertices(), isoDijkstraCSA.getIsochrone());
                const IndexedSet<false, Vertex> phastIsochrone(dijkstraCSAData.transferGraph.numVertices(), isoPHASTCSA.getIsochrone());

                for (const Vertex vertex : ch.forward.vertices()) {
                    if (dijkstraIsochrone.contains(vertex) != phastIsochrone.contains(vertex)) {
                        std::cout << "Query " << i << ", source " << source << ", departure time " << departureTime << ", range " << range << ", max arrival time " << departureTime + range << std::endl;
                        std::cout << "Vertex " << vertex << " only in " << (dijkstraIsochrone.contains(vertex) ? "Dijkstra" : "PHAST") << " isochrone" << std::endl;
                        std::cout << "IsoDijkstra arrival time: " << isoDijkstraCSA.getEarliestArrivalTime(vertex) << std::endl;
                        std::cout << "IsoPHAST arrival time: " << isoPHASTCSA.getEarliestArrivalTime(vertex) << std::endl;
                        std::cout << "Cell " << partition.getCellIdOfVertex(vertex) << std::endl;
                        std::cout << "Journey:" << std::endl;
                        std::cout << dijkstraCSA.getJourney(vertex);
                        return;
                    }
                }
            }
            progress++;
        }
    }
};

class VisualizeWalkingIsochrone : public ParameterizedCommand {

public:
    VisualizeWalkingIsochrone(BasicShell& shell) :
        ParameterizedCommand(shell, "visualizeWalkingIsochrone", "Visualizes walking isochrone for the given query.") {
        addParameter("RAPTOR binary");
        addParameter("Partition binary");
        addParameter("Source");
        addParameter("Range");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR binary"));
        VertexPartition partition(getParameter("Partition binary"));
        IsoDijkstra<RAPTOR::TransferGraph> isoDijkstra(raptorData.transferGraph);
        const Vertex source = getParameter<Vertex>("Source");
        const int range = getParameter<int>("Range");
        isoDijkstra.run(source, range, TravelTime);
        const IndexedSet<false, Vertex> isochrone(raptorData.transferGraph.numVertices(), isoDijkstra.getIsochrone());
        IndexedSet<false, int> reachedCells(partition.numberOfCells());
        for (const Vertex vertex : isochrone) {
            reachedCells.insert(partition.getCellIdOfVertex(vertex));
        }
        size_t visitedVertices = 0;
        size_t partialCells = 0;
        for (const size_t cell : reachedCells) {
            visitedVertices += partition.getCell(cell).size();
            for (const Vertex vertex : partition.getCell(cell)) {
                if (!isochrone.contains(vertex)) {
                    partialCells++;
                    break;
                }
            }
        }
        std::cout << "Vertices in isochrone: " << isochrone.size() << "/" << raptorData.transferGraph.numVertices() << std::endl;
        std::cout << "Vertices in isochrone cells: " << visitedVertices << "/" << raptorData.transferGraph.numVertices() << std::endl;
        std::cout << "Fully visited cells: " << reachedCells.size() - partialCells << "/" << partition.numberOfCells() << std::endl;
        std::cout << "Partially visited cells: " << partialCells << "/" << partition.numberOfCells() << std::endl;
        std::cout << "Unvisited cells: " << partition.numberOfCells() - reachedCells.size() << "/" << partition.numberOfCells() << std::endl;

        TimeTableVisualization<PNG> doc = TimeTableVisualization<PNG>::FromRAPTOR(getParameter("Output file"), raptorData, 0.3);
        std::vector<int> cellColors = greedyVertexColors(partition.getCellGraph(raptorData.transferGraph));
        doc.drawRoutes(Color::KITseablue, 10.0);
        for (const Vertex vertex : raptorData.transferGraph.vertices()) {
            if (isochrone.contains(vertex)) continue;
            const int cell = partition.getCellIdOfVertex(vertex);
            const Color color = reachedCells.contains(cell) ? cyclicColor(cellColors[cell]) : Color::Grey;
            doc.drawPoint(raptorData.transferGraph.get(Coordinates, vertex), color, 5.0);
        }
        for (const Vertex vertex : isochrone) {
            doc.drawPoint(raptorData.transferGraph.get(Coordinates, vertex), Color::Green, 5.0);
        }
    }
};

class VisualizeMultimodalIsochrone : public ParameterizedCommand {

public:
    VisualizeMultimodalIsochrone(BasicShell& shell) :
        ParameterizedCommand(shell, "visualizeMultimodalIsochrone", "Visualizes multimodal isochrone for the given query.") {
        addParameter("RAPTOR binary");
        addParameter("CSA binary");
        addParameter("Partition binary");
        addParameter("Source");
        addParameter("Departure time");
        addParameter("Range");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        RAPTOR::Data raptorData = RAPTOR::Data::FromBinary(getParameter("RAPTOR binary"));
        CSA::Data csaData = CSA::Data::FromBinary(getParameter("CSA binary"));
        csaData.sortConnectionsAscending();
        VertexPartition partition(getParameter("Partition binary"));
        CSA::IsoDijkstraCSA isoDijkstraCSA(csaData);

        const Vertex source = getParameter<Vertex>("Source");
        const int departureTime = getParameter<int>("Departure time");
        const int range = getParameter<int>("Range");
        isoDijkstraCSA.run(source, departureTime, range);
        const IndexedSet<false, Vertex> isochrone(raptorData.transferGraph.numVertices(), isoDijkstraCSA.getIsochrone());
        IndexedSet<false, int> reachedCells(partition.numberOfCells());
        for (const Vertex vertex : isochrone) {
            reachedCells.insert(partition.getCellIdOfVertex(vertex));
        }
        size_t visitedVertices = 0;
        size_t partialCells = 0;
        for (const size_t cell : reachedCells) {
            visitedVertices += partition.getCell(cell).size();
            for (const Vertex vertex : partition.getCell(cell)) {
                if (!isochrone.contains(vertex)) {
                    partialCells++;
                    break;
                }
            }
        }
        std::cout << "Vertices in isochrone: " << isochrone.size() << "/" << raptorData.transferGraph.numVertices() << std::endl;
        std::cout << "Vertices in isochrone cells: " << visitedVertices << "/" << raptorData.transferGraph.numVertices() << std::endl;
        std::cout << "Fully visited cells: " << reachedCells.size() - partialCells << "/" << partition.numberOfCells() << std::endl;
        std::cout << "Partially visited cells: " << partialCells << "/" << partition.numberOfCells() << std::endl;
        std::cout << "Unvisited cells: " << partition.numberOfCells() - reachedCells.size() << "/" << partition.numberOfCells() << std::endl;

        TimeTableVisualization<PNG> doc = TimeTableVisualization<PNG>::FromRAPTOR(getParameter("Output file"), raptorData, 0.3);
        std::vector<int> cellColors = greedyVertexColors(partition.getCellGraph(raptorData.transferGraph));
        doc.drawRoutes(Color::KITseablue, 10.0);
        for (const Vertex vertex : raptorData.transferGraph.vertices()) {
            if (isochrone.contains(vertex)) continue;
            const int cell = partition.getCellIdOfVertex(vertex);
            const Color color = reachedCells.contains(cell) ? cyclicColor(cellColors[cell]) : Color::Grey;
            doc.drawPoint(raptorData.transferGraph.get(Coordinates, vertex), color, 5.0);
        }
        for (const Vertex vertex : isochrone) {
            doc.drawPoint(raptorData.transferGraph.get(Coordinates, vertex), Color::Green, 5.0);
        }
    }
};
