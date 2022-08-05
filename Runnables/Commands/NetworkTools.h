#pragma once

#include <iostream>
#include <string>

#include "../../Algorithms/StronglyConnectedComponents.h"

#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/Intermediate/Data.h"
#include "../../DataStructures/RAPTOR/Data.h"

#include "../../Helpers/HighlightText.h"

#include "../../Shell/Shell.h"

using namespace Shell;

class DuplicateTrips : public ParameterizedCommand {

public:
    DuplicateTrips(BasicShell& shell) :
        ParameterizedCommand(shell, "duplicateTrips", "Duplicates all trips and shifts them by the given time offset.") {
        addParameter("Intermediate binary");
        addParameter("Time offset");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string intermediateFile = getParameter("Intermediate binary");
        const int maxSpeed = getParameter<int>("Time offset");
        const std::string outputFile = getParameter("Output file");

        Intermediate::Data inter = Intermediate::Data::FromBinary(intermediateFile);
        inter.printInfo();
        inter.duplicateTrips(String::lexicalCast<double>(maxSpeed));
        inter.printInfo();
        inter.serialize(outputFile);
    }

};

class ReverseRAPTORNetwork : public ParameterizedCommand {

public:
    ReverseRAPTORNetwork(BasicShell& shell) :
        ParameterizedCommand(shell, "reverseRAPTORNetwork", "Generates a reversed copy of the given RAPTOR network.") {
        addParameter("RAPTOR binary");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string raptorFile = getParameter("RAPTOR binary");
        const std::string outputFile = getParameter("Output file");

        RAPTOR::Data data = RAPTOR::Data::FromBinary(raptorFile);
        data.printInfo();
        RAPTOR::Data reverse = data.reverseNetwork();
        reverse.printInfo();
        reverse.serialize(outputFile);
    }

};

class AddGraph : public ParameterizedCommand {

public:
    AddGraph(BasicShell& shell) :
        ParameterizedCommand(shell, "addGraph", "Adds a transfer graph to the intermediate network data.") {
        addParameter("Intermediate file");
        addParameter("Graph file");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string intermediateFile = getParameter("Intermediate file");
        const std::string graphFile = getParameter("Graph file");
        const std::string outputFile = getParameter("Output file");

        Intermediate::Data inter = Intermediate::Data::FromBinary(intermediateFile);
        inter.printInfo();
        Intermediate::TransferGraph graph(graphFile);
        Graph::printInfo(graph);
        graph.printAnalysis();
        Graph::applyBoundingBox(graph, inter.boundingBox());
        inter.addTransferGraph(graph);
        inter.printInfo();
        inter.serialize(outputFile);
    }

};

class ReplaceGraph : public ParameterizedCommand {

public:
    ReplaceGraph(BasicShell& shell) :
        ParameterizedCommand(shell, "replaceGraph", "Replaces the transfer graph of a network.") {
        addParameter("Input file");
        addParameter("Graph file");
        addParameter("Output file");
        addParameter("Network type", {"csa", "raptor", "intermediate"});
        addParameter("Graph type", {"static", "dynamic"});
    }

    virtual void execute() noexcept {
        const std::string graphFile = getParameter("Graph file");
        const std::string graphType = getParameter("Graph type");

        if (graphType == "static") {
            TransferGraph graph(graphFile);
            chooseNetwork(std::move(graph));
        } else {
            DynamicTransferGraph graph(graphFile);
            chooseNetwork(std::move(graph));
        }
    }

private:
    template<typename GRAPH, typename = std::enable_if_t<std::is_rvalue_reference<GRAPH&&>::value>>
    inline void chooseNetwork(GRAPH&& graph) noexcept {
        const std::string inputFile = getParameter("Input file");
        const std::string networkType = getParameter("Network type");

        if (networkType == "csa") {
            CSA::Data data = CSA::Data::FromBinary(inputFile);
            replaceGraph(std::move(graph), data);
        } else if (networkType == "raptor") {
            RAPTOR::Data data = RAPTOR::Data::FromBinary(inputFile);
            replaceGraph(std::move(graph), data);
        } else {
            Intermediate::Data data = Intermediate::Data::FromBinary(inputFile);
            replaceGraph(std::move(graph), data);
        }
    }

    template<typename GRAPH, typename NETWORK, typename = std::enable_if_t<std::is_rvalue_reference<GRAPH&&>::value>>
    inline void replaceGraph(GRAPH&& graph, NETWORK& network) noexcept {
        const std::string outputFile = getParameter("Output file");

        network.printInfo();
        Graph::printInfo(graph);
        graph.printAnalysis();
        Graph::applyBoundingBox(graph, network.boundingBox());
        Graph::move(std::move(graph), network.transferGraph);
        network.printInfo();
        network.serialize(outputFile);
    }

};

class ReduceGraph : public ParameterizedCommand {

public:
    ReduceGraph(BasicShell& shell) :
        ParameterizedCommand(shell, "reduceGraph", "Contracts vertices with degree <= 2.") {
        addParameter("Intermediate file");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string intermediateFile = getParameter("Intermediate file");
        const std::string outputFile = getParameter("Output file");

        Intermediate::Data inter = Intermediate::Data::FromBinary(intermediateFile);
        inter.printInfo();
        inter.contractDegreeTwoVertices();
        inter.printInfo();
        inter.serialize(outputFile);
    }

};

class ReduceToMaximumConnectedComponent : public ParameterizedCommand {

public:
    ReduceToMaximumConnectedComponent(BasicShell& shell) :
        ParameterizedCommand(shell, "reduceToMaximumConnectedComponent", "Removes everything that is not part of the largest connected component.") {
        addParameter("Intermediate file");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        Intermediate::Data inter(getParameter("Intermediate file"));
        inter.printInfo();
        Intermediate::TransferGraph graph = inter.minTravelTimeGraph();
        Graph::printInfo(graph);
        graph.printAnalysis();
        StronglyConnectedComponents<Intermediate::TransferGraph, true> scc(graph);
        scc.run();
        const int maxComponent = scc.maxComponent();
        std::cout << "Max component size: " << String::prettyInt(scc.getComponentSize(maxComponent)) << std::endl;
        inter.deleteVertices([&](const Vertex vertex) {
            return scc.getComponent(vertex) != maxComponent;
        });
        inter.printInfo();
        inter.serialize(getParameter("Output file"));
        for (const Intermediate::Stop& stop : inter.stops) {
            if (stop.coordinates.x != 0) continue;
            if (stop.coordinates.y != 0) continue;
            warning(stop);
        }
    }

};

class ReduceToMaximumConnectedComponentWithTransitive : public ParameterizedCommand {

public:
    ReduceToMaximumConnectedComponentWithTransitive(BasicShell& shell) :
        ParameterizedCommand(shell, "reduceToMaximumConnectedComponentWithTransitive", "Removes everything that is not part of the largest connected component in the full network and reduces the transitive network accordingly.") {
        addParameter("Full intermediate file");
        addParameter("Transitive intermediate file");
        addParameter("Full output file");
        addParameter("Transitive output file");
    }

    virtual void execute() noexcept {
        Intermediate::Data fullData(getParameter("Full intermediate file"));
        fullData.printInfo();
        Intermediate::TransferGraph graph = fullData.minTravelTimeGraph();
        Graph::printInfo(graph);
        graph.printAnalysis();
        StronglyConnectedComponents<Intermediate::TransferGraph, true> scc(graph);
        scc.run();
        const int maxComponent = scc.maxComponent();
        std::cout << "Max component size: " << String::prettyInt(scc.getComponentSize(maxComponent)) << std::endl;
        fullData.deleteVertices([&](const Vertex vertex) {
            return scc.getComponent(vertex) != maxComponent;
        });
        fullData.printInfo();
        fullData.serialize(getParameter("Full output file"));
        for (const Intermediate::Stop& stop : fullData.stops) {
            if (stop.coordinates.x != 0) continue;
            if (stop.coordinates.y != 0) continue;
            warning(stop);
        }

        Intermediate::Data transitiveData(getParameter("Transitive intermediate file"));
        transitiveData.printInfo();
        transitiveData.deleteVertices([&](const Vertex vertex) {
            return scc.getComponent(vertex) != maxComponent;
        });
        transitiveData.printInfo();
        transitiveData.serialize(getParameter("Transitive output file"));
    }

};

class ApplyBoundingBox : public ParameterizedCommand {

public:
    ApplyBoundingBox(BasicShell& shell) :
        ParameterizedCommand(shell, "applyBoundingBox", "Applies a bounding box to the intermediate network data.") {
        addParameter("Intermediate binary");
        addParameter("Bounding box", {"germany", "deutschland", "switzerland", "bern", "london"});
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string intermediateFile = getParameter("Intermediate binary");
        const std::string boundingBox = getParameter("Bounding box");
        const std::string outputFile = getParameter("Output file");

        Intermediate::Data inter = Intermediate::Data::FromBinary(intermediateFile);
        inter.printInfo();
        if (boundingBox == "germany" || boundingBox == "deutschland") {
            inter.applyBoundingBox(Germany);
        } else if (boundingBox == "switzerland") {
            inter.applyBoundingBox(Switzerland);
        } else if (boundingBox == "london") {
            inter.applyBoundingBox(London);
        } else if (boundingBox == "bern") {
            inter.applyBoundingBox(Bern);
        }
        inter.printInfo();
        inter.serialize(outputFile);
    }

private:
    const Geometry::Rectangle Switzerland = Geometry::Rectangle::BoundingBox(Geometry::Point(Construct::XY, 5.826, 45.487), Geometry::Point(Construct::XY, 10.819, 48.142));
    const Geometry::Rectangle Bern = Geometry::Rectangle::BoundingBox(Geometry::Point(Construct::XY, 7.307, 46.868), Geometry::Point(Construct::XY, 7.563, 47.085));
    const Geometry::Rectangle Germany = Geometry::Rectangle::BoundingBox(Geometry::Point(Construct::XY, 5.730, 47.160), Geometry::Point(Construct::XY, 15.130, 55.070));
    const Geometry::Rectangle London = Geometry::Rectangle::BoundingBox(Geometry::Point(Construct::XY, -0.612, 51.233), Geometry::Point(Construct::XY, 0.715, 51.707));

};

class ApplyCustomBoundingBox : public ParameterizedCommand {

public:
    ApplyCustomBoundingBox(BasicShell& shell) :
        ParameterizedCommand(shell, "applyCustomBoundingBox", "Applies the specified bounding box to the intermediate network data.") {
        addParameter("Intermediate binary");
        addParameter("lon-min");
        addParameter("lon-max");
        addParameter("lat-min");
        addParameter("lat-max");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string intermediateFile = getParameter("Intermediate binary");
        const double lonMin = getParameter<double>("lon-min");
        const double lonMax = getParameter<double>("lon-max");
        const double latMin = getParameter<double>("lat-min");
        const double latMax = getParameter<double>("lat-max");
        const std::string outputFile = getParameter("Output file");

        Intermediate::Data inter = Intermediate::Data::FromBinary(intermediateFile);
        inter.printInfo();
        const Geometry::Point min(Construct::XY, lonMin, latMin);
        const Geometry::Point max(Construct::XY, lonMax, latMax);
        const Geometry::Rectangle boundingBox = Geometry::Rectangle::BoundingBox(min, max);
        inter.applyBoundingBox(boundingBox);
        inter.printInfo();
        inter.serialize(outputFile);
    }

};

class MakeOneHopTransfers : public ParameterizedCommand {

public:
    MakeOneHopTransfers(BasicShell& shell) :
        ParameterizedCommand(shell, "makeOneHopTransfers", "Constructs one-hop transfers between all stops within the given travel time limit.") {
        addParameter("Intermediate file");
        addParameter("Max travel time");
        addParameter("Output file");
        addParameter("Build transitive closure?", "false");
    }

    virtual void execute() noexcept {
        const std::string intermediateFile = getParameter("Intermediate file");
        const int maxTravelTime = getParameter<int>("Max travel time");
        const std::string outputFile = getParameter("Output file");
        const bool buildTransitiveClosure = getParameter<bool>("Build transitive closure?");

        Intermediate::Data inter = Intermediate::Data::FromBinary(intermediateFile);
        inter.printInfo();
        inter.makeDirectTransfers(maxTravelTime, true);
        inter.printInfo();
        if (buildTransitiveClosure) {
            inter.makeDirectTransfers(8640000, true);
            inter.printInfo();
        }
        inter.serialize(outputFile);
    }

};

class MakeOneHopTransfersByGeoDistance : public ParameterizedCommand {

public:
    MakeOneHopTransfersByGeoDistance(BasicShell& shell) :
        ParameterizedCommand(shell, "makeOneHopTransfersByGeoDistance", "Constructs one-hop transfers between all stops within the given geo distance limit.") {
        addParameter("Intermediate file");
        addParameter("Max distance");
        addParameter("Speed in km/h");
        addParameter("Output file");
        addParameter("Build transitive closure?", "false");
    }

    virtual void execute() noexcept {
        const std::string intermediateFile = getParameter("Intermediate file");
        const int maxDistance = getParameter<int>("Max distance");
        const double speed = getParameter<double>("Speed in km/h");
        const std::string outputFile = getParameter("Output file");
        const bool buildTransitiveClosure = getParameter<bool>("Build transitive closure?");

        Intermediate::Data inter = Intermediate::Data::FromBinary(intermediateFile);
        inter.printInfo();
        inter.makeDirectTransfersByGeoDistance(maxDistance, speed, true);
        inter.printInfo();
        if (buildTransitiveClosure) {
            inter.makeDirectTransfers(8640000, true);
            inter.printInfo();
        }
        inter.serialize(outputFile);
    }

};

class ApplyMaxTransferSpeed : public ParameterizedCommand {

public:
    ApplyMaxTransferSpeed(BasicShell& shell) :
        ParameterizedCommand(shell, "applyMaxTransferSpeed", "Applies a speed limit to all transfers.") {
        addParameter("Intermediate binary");
        addParameter("Max speed in km/h");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string intermediateFile = getParameter("Intermediate binary");
        const double maxSpeed = getParameter<double>("Max speed in km/h");
        const std::string outputFile = getParameter("Output file");

        Intermediate::Data inter = Intermediate::Data::FromBinary(intermediateFile);
        inter.printInfo();
        inter.applyMaxSpeed(maxSpeed);
        inter.printInfo();
        inter.serialize(outputFile);
    }
};

class ApplyConstantTransferSpeed : public ParameterizedCommand {

public:
    ApplyConstantTransferSpeed(BasicShell& shell) :
        ParameterizedCommand(shell, "applyConstantTransferSpeed", "Applies a constant speed to all transfers.") {
        addParameter("Graph binary");
        addParameter("Speed in km/h");
        addParameter("Obey speed limits?");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string graphFile = getParameter("Graph binary");
        const size_t speed = getParameter<size_t>("Speed in km/h");
        const bool obeySpeedLimits = getParameter<bool>("Obey speed limits?");
        const std::string outputFile = getParameter("Output file");

        Intermediate::TransferGraph graph;
        graph.readBinary(graphFile);
        graph.printAnalysis();
        Graph::computeTravelTimes(graph, speed, obeySpeedLimits);
        graph.printAnalysis();
        graph.writeBinary(outputFile);
    }
};

class ApplyMinTransferTravelTime : public ParameterizedCommand {

public:
    ApplyMinTransferTravelTime(BasicShell& shell) :
        ParameterizedCommand(shell, "applyMinTransferTravelTime", "Applies a minimum travel time to all transfers.") {
        addParameter("Network binary");
        addParameter("Network type", { "intermediate", "raptor", "csa" });
        addParameter("Min travel time");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string networkFile = getParameter("Network binary");
        const std::string networkType = getParameter("Network type");
        const double minTravelTime = getParameter<double>("Min travel time");
        const std::string outputFile = getParameter("Output file");

        if (networkType == "intermediate") {
            Intermediate::Data network = Intermediate::Data::FromBinary(networkFile);
            applyMinTravelTime(network, minTravelTime, outputFile);
        } else if (networkType == "raptor") {
            RAPTOR::Data network = RAPTOR::Data::FromBinary(networkFile);
            applyMinTravelTime(network, minTravelTime, outputFile);
        } else {
            CSA::Data network = CSA::Data::FromBinary(networkFile);
            applyMinTravelTime(network, minTravelTime, outputFile);
        }
    }

private:
    template<typename NETWORK_TYPE>
    inline static void applyMinTravelTime(NETWORK_TYPE& network, const double minTravelTime, const std::string& outputFile) noexcept {
        network.printInfo();
        network.applyMinTravelTime(minTravelTime);
        network.printInfo();
        network.serialize(outputFile);
    }
};
