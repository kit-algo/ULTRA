#pragma once

#include <string>

#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/GTFS/Data.h"
#include "../../DataStructures/Intermediate/Data.h"
#include "../../DataStructures/RAPTOR/Data.h"

#include "../../Shell/Shell.h"

using namespace Shell;

class ParseGTFS : public ParameterizedCommand {

public:
    ParseGTFS(BasicShell& shell) :
        ParameterizedCommand(shell, "parseGTFS", "Parses raw GTFS data from the given directory and converts it to a binary representation.") {
        addParameter("Input directory");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string gtfsDirectory = getParameter("Input directory");
        const std::string outputFile = getParameter("Output file");

        GTFS::Data data = GTFS::Data::FromGTFS(gtfsDirectory);
        data.printInfo();
        data.serialize(outputFile);
    }

};

class GTFSToIntermediate : public ParameterizedCommand {

public:
    GTFSToIntermediate(BasicShell& shell) :
        ParameterizedCommand(shell, "gtfsToIntermediate", "Converts binary GTFS data to the intermediate network format.") {
        addParameter("Input directory");
        addParameter("First day");
        addParameter("Last day");
        addParameter("Use days of operation?");
        addParameter("Use frequencies?");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string gtfsDirectory = getParameter("Input directory");
        const std::string outputFile = getParameter("Output file");
        const int firstDay = stringToDay(getParameter("First day"));
        const int lastDay = stringToDay(getParameter("Last day"));
        const bool useDaysOfOperation = getParameter<bool>("Use days of operation?");
        const bool useFrequencies = getParameter<bool>("Use frequencies?");

        GTFS::Data gtfs = GTFS::Data::FromBinary(gtfsDirectory);
        gtfs.printInfo();
        Intermediate::Data inter = Intermediate::Data::FromGTFS(gtfs, firstDay, lastDay, !useDaysOfOperation, !useFrequencies);
        inter.printInfo();
        inter.serialize(outputFile);
    }

};

class IntermediateToCSA : public ParameterizedCommand {

public:
    IntermediateToCSA(BasicShell& shell) :
        ParameterizedCommand(shell, "intermediateToCSA", "Converts binary intermediate data to CSA network format.") {
        addParameter("Input file");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");

        Intermediate::Data inter = Intermediate::Data::FromBinary(inputFile);
        inter.printInfo();
        CSA::Data data = CSA::Data::FromIntermediate(inter);
        data.printInfo();
        data.serialize(outputFile);
    }

};

class IntermediateToRAPTOR : public ParameterizedCommand {

public:
    IntermediateToRAPTOR(BasicShell& shell) :
        ParameterizedCommand(shell, "intermediateToRAPTOR", "Converts binary intermediate data to RAPTOR network format.") {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Route type", "FIFO", {"Geographic", "FIFO", "Offset", "Frequency"});
    }

    virtual void execute() noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");
        const std::string routeTypeString = getParameter("Route type");
        int routeType;
        if (routeTypeString == "Geographic") {
            routeType = 0;
        } else if (routeTypeString == "FIFO") {
            routeType = 1;
        } else if (routeTypeString == "Offset") {
            routeType = 2;
        } else {
            routeType = 3;
        }

        Intermediate::Data inter = Intermediate::Data::FromBinary(inputFile);
        inter.printInfo();
        RAPTOR::Data data = RAPTOR::Data::FromIntermediate(inter, routeType);
        data.printInfo();
        Graph::printInfo(data.transferGraph);
        data.transferGraph.printAnalysis();
        data.serialize(outputFile);
    }

};

class LoadDimacsGraph : public ParameterizedCommand {

public:
    LoadDimacsGraph(BasicShell& shell) :
        ParameterizedCommand(shell, "loadDimacsGraph", "Converts DIMACS graph data to our transfer graph format.") {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Graph type", "dynamic", { "static", "dynamic" });
        addParameter("Coordinate factor", "0.000001");
    }

    virtual void execute() noexcept {
        std::string graphType = getParameter("Graph type");
        if (graphType == "static") {
            load<TransferGraph>();
        } else {
            load<DynamicTransferGraph>();
        }
    }

private:
    template<typename GRAPH_TYPE>
    inline void load() const noexcept {
        DimacsGraphWithCoordinates dimacs;
        dimacs.fromDimacs<true>(getParameter("Input file"), getParameter<double>("Coordinate factor"));
        Graph::printInfo(dimacs);
        dimacs.printAnalysis();
        GRAPH_TYPE graph;
        Graph::move(std::move(dimacs), graph);
        Graph::printInfo(graph);
        graph.printAnalysis();
        graph.writeBinary(getParameter("Output file"));
    }
};
