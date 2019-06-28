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
