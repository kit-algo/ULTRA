#pragma once

#include "Data.h"
#include "../RAPTOR/TransferModes.h"

namespace TripBased {

using TransferGraph = ::TransferGraph;

class MultimodalData {

public:
    MultimodalData(const std::string& fileName) {
        deserialize(fileName);
    }

    MultimodalData(const Data& data) : tripData(data) {}

public:
    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, modes);
        tripData.serialize(fileName + ".trip");
        for (const size_t mode : modes) {
            stopEventGraphs[mode].writeBinary(fileName + "." + RAPTOR::TransferModeNames[mode] + ".graph");
        }
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, modes);
        tripData.deserialize(fileName + ".trip");
        for (const size_t mode : modes) {
            stopEventGraphs[mode].readBinary(fileName + "." + RAPTOR::TransferModeNames[mode] + ".graph");
        }
    }

    inline void printInfo() const noexcept {
        std::cout << "Trip-Based data:" << std::endl;
        tripData.printInfo();
        for (const size_t mode : modes) {
            std::cout << "Graph for " << RAPTOR::TransferModeNames[mode] << ":" << std::endl;
            Graph::printInfo(stopEventGraphs[mode]);
        }
    }

    inline void addTransferGraph(const size_t mode, const TransferGraph& graph) noexcept {
        AssertMsg(mode < RAPTOR::NUM_TRANSFER_MODES, "Mode is not supported!");
        if (!Vector::contains(modes, mode)) {
            modes.emplace_back(mode);
        }
        stopEventGraphs[mode] = graph;
    }

    inline const TransferGraph& getTransferGraph(const size_t mode) const noexcept {
        AssertMsg(Vector::contains(modes, mode), "Mode is not supported!");
        return stopEventGraphs[mode];
    }

    inline Data getBimodalData(const size_t mode) const noexcept {
        Data resultData(tripData);
        resultData.stopEventGraph = getTransferGraph(mode);
        return resultData;
    }

    inline Data getPruningData() const noexcept {
        return getPruningData(modes);
    }

    inline Data getPruningData(const std::vector<size_t>& pruningModes) const noexcept {
        AssertMsg(!pruningModes.empty(), "Cannot build pruning data without transfer modes!");
        Data resultData(tripData);
        DynamicTransferGraph temp;
        Graph::copy(tripData.stopEventGraph, temp);
        for (const size_t mode : pruningModes) {
            const TransferGraph& graph = getTransferGraph(mode);
            for (const Vertex from : graph.vertices()) {
                for (const Edge edge : graph.edgesFrom(from)) {
                    const Vertex to = graph.get(ToVertex, edge);
                    const Edge originalEdge = temp.findEdge(from, to);
                    if (originalEdge == noEdge) {
                        temp.addEdge(from, to, graph.edgeRecord(edge));
                    } else {
                        const int travelTime = graph.get(TravelTime, edge);
                        const int originalTravelTime = temp.get(TravelTime, originalEdge);
                        temp.set(TravelTime, originalEdge, std::min(travelTime, originalTravelTime));
                    }
                }
            }
        }
        Graph::move(std::move(temp), resultData.stopEventGraph);
        return resultData;
    }

public:
    Data tripData;
    std::vector<size_t> modes;
    TransferGraph stopEventGraphs[RAPTOR::NUM_TRANSFER_MODES];
};

}
