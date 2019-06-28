/**********************************************************************************

 Copyright (c) 2019 Jonas Sauer

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
 modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************************/

#include <iostream>
#include <string>

#include "../Algorithms/CH/Preprocessing/BidirectionalWitnessSearch.h"
#include "../Algorithms/CH/Preprocessing/CHBuilder.h"
#include "../Algorithms/CH/Preprocessing/KeyFunction.h"
#include "../Algorithms/CH/Preprocessing/StopCriterion.h"
#include "../Algorithms/CH/CH.h"
#include "../DataStructures/RAPTOR/Data.h"

using WitnessSearch = CH::BidirectionalWitnessSearch<CHCoreGraph, 500>;
inline static constexpr int ShortcutWeight = 1024;
inline static constexpr int LevelWeight = 256;
using KeyFunction = CH::PartialKey<WitnessSearch, CH::GreedyKey<WitnessSearch, ShortcutWeight, LevelWeight, 0>>;
using StopCriterion = CH::CoreCriterion;
using CHBuilder = CH::Builder<WitnessSearch, KeyFunction, StopCriterion, false, false>;


struct CoreCHData {
    CH::CH ch;
    RAPTOR::Data coreData;
};

inline CoreCHData buildCoreCH(const RAPTOR::Data& raptorData, const size_t coreDegree) noexcept {
    CHCoreGraph graph;
    Graph::copy(raptorData.transferGraph, graph, Weight << TravelTime);
    const size_t numberOfStops = raptorData.numberOfStops();
    std::vector<bool> isNormalVertex(numberOfStops, false);
    isNormalVertex.resize(graph.numVertices(), true);

    CHBuilder chBuilder(std::move(graph), KeyFunction(isNormalVertex, graph.numVertices()), StopCriterion(numberOfStops, coreDegree));
    chBuilder.run();
    chBuilder.copyCoreToCH();
    CH::CH ch(std::move(chBuilder));

    DynamicTransferGraph coreGraph;
    coreGraph.addVertices(raptorData.transferGraph.numVertices());
    coreGraph[Coordinates] = raptorData.transferGraph[Coordinates];
    for (const Vertex vertex : coreGraph.vertices()) {
        if (ch.isCoreVertex(vertex)) {
            for (const Edge edge : ch.forward.edgesFrom(vertex)) {
                coreGraph.addEdge(vertex, ch.forward.get(ToVertex, edge)).set(TravelTime, ch.forward.get(Weight, edge));
            }
        }
    }

    CoreCHData result{ch, raptorData};
    Graph::move(std::move(coreGraph), result.coreData.transferGraph);
    return result;
}

inline void usage() noexcept {
    std::cout << "Usage: BuildCoreCH <RAPTOR binary> <core degree> <output directory>" << std::endl;
    exit(0);
}

int main(int argc, char** argv) {
    if (argc < 4) usage();
    const std::string raptorFile = argv[1];
    RAPTOR::Data data = RAPTOR::Data::FromBinary(raptorFile);
    data.useImplicitDepartureBufferTimes();
    data.printInfo();
    const size_t coreDegree = String::lexicalCast<size_t>(argv[2]);
    CoreCHData contractionData = buildCoreCH(data, coreDegree);
    const std::string outputDirectory = argv[3];
    contractionData.ch.writeBinary(outputDirectory + "ch");
    contractionData.coreData.serialize(outputDirectory + "raptor.binary");
    return 0;
}
