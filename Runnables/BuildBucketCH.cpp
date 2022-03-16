#include <iostream>
#include <string>

#include "../Algorithms/CH/Preprocessing/BidirectionalWitnessSearch.h"
#include "../Algorithms/CH/Preprocessing/CHBuilder.h"
#include "../Algorithms/CH/Preprocessing/KeyFunction.h"
#include "../Algorithms/CH/Preprocessing/StopCriterion.h"
#include "../Algorithms/CH/CH.h"

using WitnessSearch = CH::BidirectionalWitnessSearch<CHCoreGraph, 500>;
inline static constexpr int ShortcutWeight = 1024;
inline static constexpr int LevelWeight = 256;
using KeyFunction = CH::GreedyKey<WitnessSearch, ShortcutWeight, LevelWeight, 0>;
using StopCriterion = CH::NoStopCriterion;
using CHBuilder = CH::Builder<WitnessSearch, KeyFunction, StopCriterion, false, false>;

inline CH::CH buildCH(TravelTimeGraph&& graph) noexcept {
    CHBuilder chBuilder(std::move(graph), graph[TravelTime]);
    chBuilder.run();
    chBuilder.copyCoreToCH();
    return CH::CH(std::move(chBuilder));
}

inline void usage() noexcept {
    std::cout << "Usage: BuildBucketCH <graph binary> <output file>" << std::endl;
    exit(0);
}

int main(int argc, char** argv) {
    if (argc < 3) usage();
    const std::string transferGraphFile = argv[1];
    TravelTimeGraph transferGraph;
    transferGraph.readBinary(transferGraphFile);
    Graph::printInfo(transferGraph);
    CH::CH ch = buildCH(std::move(transferGraph));
    const std::string outputFile = argv[2];
    ch.writeBinary(outputFile);
    return 0;
}
