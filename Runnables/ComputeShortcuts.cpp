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

#include "../DataStructures/RAPTOR/Data.h"
#include "../Helpers/MultiThreading.h"
#include "../Helpers/String/String.h"
#include "../Algorithms/ULTRA/Builder.h"

template<bool REQUIRE_DIRECT_TRANSFER>
inline void run(RAPTOR::Data& data, const size_t numberOfThreads, const size_t pinMultiplier, const size_t transferLimit) noexcept {
    ULTRA::Builder<false, REQUIRE_DIRECT_TRANSFER> shortcutGraphBuilder(data);
    std::cout << "Computing transfer shortcuts (parallel with " << numberOfThreads << " threads)." << std::endl;
    Timer timer;
    shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfThreads, pinMultiplier), transferLimit);
    std::cout << "Took " << String::msToString(timer.elapsedMilliseconds()) << std::endl;
    Graph::move(std::move(shortcutGraphBuilder.getShortcutGraph()), data.transferGraph);
    std::cout << "Number of shortcuts: " << String::prettyInt(data.transferGraph.numEdges()) << std::endl;
}

inline void chooseRequireDirectTransfer(RAPTOR::Data& data, const size_t numberOfThreads, const size_t pinMultiplier, const size_t transferLimit, const bool requireDirectTransfer) noexcept {
    if (requireDirectTransfer) {
        run<true>(data, numberOfThreads, pinMultiplier, transferLimit);
    } else {
        run<false>(data, numberOfThreads, pinMultiplier, transferLimit);
    }
}

inline void usage() noexcept {
    std::cout << "Usage: ComputeShortcuts <RAPTOR binary> <transfer limit> <output file> <number of threads> <pin multiplier> <require direct transfer?>" << std::endl;
    exit(0);
}

int main(int argc, char** argv) {
    if (argc < 7) usage();
    const std::string raptorFile = argv[1];
    RAPTOR::Data data = RAPTOR::Data::FromBinary(raptorFile);
    data.useImplicitDepartureBufferTimes();
    data.printInfo();
    const size_t transferLimit = String::lexicalCast<size_t>(argv[2]);
    std::string outputFile = argv[3];
    const size_t numberOfThreads = String::lexicalCast<size_t>(argv[4]);
    const size_t pinMultiplier = String::lexicalCast<size_t>(argv[5]);
    const bool requireDirectTransfer = String::lexicalCast<bool>(argv[6]);
    chooseRequireDirectTransfer(data, numberOfThreads, pinMultiplier, transferLimit, requireDirectTransfer);
    data.dontUseImplicitDepartureBufferTimes();
    Graph::printInfo(data.transferGraph);
    data.transferGraph.printAnalysis();
    data.serialize(outputFile);
    return 0;
}
