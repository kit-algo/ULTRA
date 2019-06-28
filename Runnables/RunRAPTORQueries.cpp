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
#include <random>

#include "../Algorithms/CH/CH.h"
#include "../Algorithms/RAPTOR/Debugger.h"
#include "../Algorithms/RAPTOR/DijkstraRAPTOR.h"
#include "../Algorithms/RAPTOR/RAPTOR.h"
#include "../Algorithms/RAPTOR/ULTRARAPTOR.h"
#include "../DataStructures/RAPTOR/Data.h"
#include "../Helpers/IO/File.h"
#include "../Helpers/String/String.h"

using FullRAPTOR = RAPTOR::DijkstraRAPTOR<RAPTOR::TimeDebugger>;
using ShortcutRAPTOR = RAPTOR::ULTRARAPTOR<RAPTOR::TimeDebugger>;
using TransitiveRAPTOR = RAPTOR::RAPTOR<RAPTOR::TimeDebugger>;

struct Statistics {
    Statistics() :
        initialTransfersTime(0),
        collectRoutesTime(0),
        scanRoutesTime(0),
        intermediateTransfersTime(0),
        totalTime(0),
        numberOfScannedRoutes(0),
        numberOfScannedRouteSegments(0),
        numberOfUpdatedStopsByTransfer(0),
        numberOfRelaxedEdges(0) {
    }

    double initialTransfersTime;
    double collectRoutesTime;
    double scanRoutesTime;
    double intermediateTransfersTime;
    double totalTime;
    double numberOfScannedRoutes;
    double numberOfScannedRouteSegments;
    double numberOfUpdatedStopsByTransfer;
    double numberOfRelaxedEdges;

    inline void initialize(const RAPTOR::TimeDebugger& debugger) noexcept {
        initialTransfersTime = debugger.getInitialTransfersTime();
        collectRoutesTime = debugger.getCollectRoutesTime();
        scanRoutesTime = debugger.getScanRoutesTime();
        intermediateTransfersTime = debugger.getIntermediateTransfersTime();
        totalTime = debugger.getTotalTime();
        numberOfScannedRoutes = debugger.getNumberOfScannedRoutes();
        numberOfScannedRouteSegments = debugger.getNumberOfScannedRouteSegments();
        numberOfUpdatedStopsByTransfer = debugger.getNumberOfUpdatedStopsByTransfer();
        numberOfRelaxedEdges = debugger.getNumberOfRelaxedEdges();
    }

    inline Statistics& operator+=(const Statistics& other) noexcept {
        initialTransfersTime += other.initialTransfersTime;
        collectRoutesTime += other.collectRoutesTime;
        scanRoutesTime += other.scanRoutesTime;
        intermediateTransfersTime += other.intermediateTransfersTime;
        totalTime += other.totalTime;
        numberOfScannedRoutes += other.numberOfScannedRoutes;
        numberOfScannedRouteSegments += other.numberOfScannedRouteSegments;
        numberOfUpdatedStopsByTransfer += other.numberOfUpdatedStopsByTransfer;
        numberOfRelaxedEdges += other.numberOfRelaxedEdges;
        return *this;
    }

    inline void divide(const size_t factor) noexcept {
        initialTransfersTime /= factor;
        collectRoutesTime /= factor;
        scanRoutesTime /= factor;
        intermediateTransfersTime /= factor;
        totalTime /= factor;
        numberOfScannedRoutes /= factor;
        numberOfScannedRouteSegments /= factor;
        numberOfUpdatedStopsByTransfer /= factor;
        numberOfRelaxedEdges /= factor;
    }

    inline static void printHeader(IO::OFStream& out) noexcept {
        out << "Query";
        out << "," << "InitialTransfersTime";
        out << "," << "CollectRoutesTime";
        out << "," << "ScanRoutesTime";
        out << "," << "IntermediateTransfersTime";
        out << "," << "TotalTime";
        out << "," << "ScannedRoutes";
        out << "," << "ScannedRouteSegments";
        out << "," << "UpdatedStopsByTransfer";
        out << "," << "RelaxedEdges";
        out << "\n";
        out.flush();
    }

    inline void print(IO::OFStream& out, const int queryNumber) const noexcept {
        out << queryNumber;
        out << "," << initialTransfersTime;
        out << "," << collectRoutesTime;
        out << "," << scanRoutesTime;
        out << "," << intermediateTransfersTime;
        out << "," << totalTime;
        out << "," << numberOfScannedRoutes;
        out << "," << numberOfScannedRouteSegments;
        out << "," << numberOfUpdatedStopsByTransfer;
        out << "," << numberOfRelaxedEdges;
        out << "\n";
        out.flush();
    }
};

template<typename VERTEX_TYPE, typename ALGORITHM>
inline void runQueries(ALGORITHM& algorithm, const size_t numberOfVertices, const size_t numberOfQueries, const std::string& outputFile) noexcept {
    IO::OFStream out(outputFile);
    Statistics::printHeader(out);
    std::vector<Statistics> statistics(numberOfQueries);
    Statistics total;
    for (size_t i = 0; i < numberOfQueries; i++) {
        const VERTEX_TYPE source = VERTEX_TYPE(rand() % numberOfVertices);
        const VERTEX_TYPE target = VERTEX_TYPE(rand() % numberOfVertices);
        const int departureTime = (rand() % (16 * 60 * 60)) + (5 * 60 * 60);
        algorithm.run(source, departureTime, target);
        const RAPTOR::TimeDebugger& debugger = algorithm.getDebugger();
        statistics[i].initialize(debugger);
        statistics[i].print(out, i);
        total += statistics[i];
    }
    total.divide(numberOfQueries);
    total.print(out, -1);
}

inline void usage() noexcept {
    std::cout << "Usage: RunRAPTORQueries <transfers: transitive/full/shortcuts> <RAPTOR binary> <number of queries> <seed> <output file> <CH data (unless transfers = transitive)>" << std::endl;
    exit(0);
}

int main(int argc, char** argv) {
    if (argc < 2) usage();
    const std::string type = argv[1];
    if (type == "transitive") {
        if (argc < 6) usage();
    } else if (type == "full" || type == "shortcuts") {
        if (argc < 7) usage();
    } else {
        usage();
    }

    const std::string raptorFile = argv[2];
    RAPTOR::Data data = RAPTOR::Data::FromBinary(raptorFile);
    data.useImplicitDepartureBufferTimes();
    data.printInfo();
    const size_t numberOfQueries = String::lexicalCast<size_t>(argv[3]);
    const size_t seed = String::lexicalCast<size_t>(argv[4]);
    srand(seed);
    const std::string outputFile = argv[5];

    if (type == "transitive") {
        TransitiveRAPTOR algorithm(data);
        runQueries<StopId>(algorithm, data.numberOfStops(), numberOfQueries, outputFile);
    } else {
        const std::string& chFile = argv[6];
        CH::CH ch(chFile);
        if (type == "full") {
            FullRAPTOR algorithm(data, ch);
            runQueries<Vertex>(algorithm, data.transferGraph.numVertices(), numberOfQueries, outputFile);
        } else {
            ShortcutRAPTOR algorithm(data, ch);
            runQueries<Vertex>(algorithm, data.transferGraph.numVertices(), numberOfQueries, outputFile);
        }
    }

    return 0;
}
