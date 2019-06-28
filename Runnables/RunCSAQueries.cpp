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
#include "../Algorithms/CSA/Debugger.h"
#include "../Algorithms/CSA/DijkstraCSA.h"
#include "../Algorithms/CSA/CSA.h"
#include "../Algorithms/CSA/ULTRACSA.h"
#include "../DataStructures/CSA/Data.h"
#include "../Helpers/IO/File.h"
#include "../Helpers/String/String.h"

using FullCSA = CSA::DijkstraCSA<CSA::TimeDebugger>;
using ShortcutCSA = CSA::ULTRACSA<CSA::TimeDebugger>;
using TransitiveCSA = CSA::CSA<CSA::TimeDebugger>;


struct Statistics {
    Statistics() :
        clearTime(0),
        initializationTime(0),
        connectionScanTime(0),
        totalTime(0),
        numberOfScannedConnections(0),
        numberOfUpdatedStopsByTransfer(0),
        numberOfRelaxedEdges(0) {
    }

    double clearTime;
    double initializationTime;
    double connectionScanTime;
    double totalTime;
    double numberOfScannedConnections;
    double numberOfUpdatedStopsByTransfer;
    double numberOfRelaxedEdges;

    inline void initialize(const CSA::TimeDebugger& debugger) noexcept {
        clearTime = debugger.getClearTime();
        initializationTime = debugger.getInitializationTime();
        connectionScanTime = debugger.getConnectionScanTime();
        totalTime = debugger.getTotalTime();
        numberOfScannedConnections = debugger.getNumberOfScannedConnections();
        numberOfUpdatedStopsByTransfer = debugger.getNumberOfUpdatedStopsByTransfer();
        numberOfRelaxedEdges = debugger.getNumberOfRelaxedTransfers();
    }

    inline Statistics& operator+=(const Statistics& other) noexcept {
        clearTime += other.clearTime;
        initializationTime += other.initializationTime;
        connectionScanTime += other.connectionScanTime;
        totalTime += other.totalTime;
        numberOfScannedConnections += other.numberOfScannedConnections;
        numberOfUpdatedStopsByTransfer += other.numberOfUpdatedStopsByTransfer;
        numberOfRelaxedEdges += other.numberOfRelaxedEdges;
        return *this;
    }

    inline void divide(const size_t factor) noexcept {
        clearTime /= factor;
        initializationTime /= factor;
        connectionScanTime /= factor;
        totalTime /= factor;
        numberOfScannedConnections /= factor;
        numberOfUpdatedStopsByTransfer /= factor;
        numberOfRelaxedEdges /= factor;
    }

    inline static void printHeader(IO::OFStream& out) noexcept {
        out << "Query";
        out << "," << "ClearTime";
        out << "," << "InitializationTime";
        out << "," << "ConnectionScanTime";
        out << "," << "TotalTime";
        out << "," << "ScannedConnections";
        out << "," << "UpdatedStopsByTransfer";
        out << "," << "RelaxedEdges";
        out << "\n";
        out.flush();
    }

    inline void print(IO::OFStream& out, const int queryNumber) const noexcept {
        out << queryNumber;
        out << "," << clearTime;
        out << "," << initializationTime;
        out << "," << connectionScanTime;
        out << "," << totalTime;
        out << "," << numberOfScannedConnections;
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
        const CSA::TimeDebugger& debugger = algorithm.getDebugger();
        statistics[i].initialize(debugger);
        statistics[i].print(out, i);
        total += statistics[i];
    }
    total.divide(numberOfQueries);
    total.print(out, -1);
}

inline void usage() noexcept {
    std::cout << "Usage: RunCSAQueries <transfers: transitive/full/shortcuts> <CSA binary> <number of queries> <seed> <output file> <CH data (unless transfers = transitive)>" << std::endl;
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

    const std::string csaFile = argv[2];
    CSA::Data data = CSA::Data::FromBinary(csaFile);
    data.useImplicitDepartureBufferTimes();
    data.sortConnectionsAscending();
    data.printInfo();
    const size_t numberOfQueries = String::lexicalCast<size_t>(argv[3]);
    const size_t seed = String::lexicalCast<size_t>(argv[4]);
    srand(seed);
    const std::string outputFile = argv[5];

    if (type == "transitive") {
        TransitiveCSA algorithm(data);
        runQueries<StopId>(algorithm, data.numberOfStops(), numberOfQueries, outputFile);
    } else {
        const std::string& chFile = argv[6];
        CH::CH ch(chFile);
        if (type == "full") {
            FullCSA algorithm(data, ch);
            runQueries<Vertex>(algorithm, data.transferGraph.numVertices(), numberOfQueries, outputFile);
        } else {
            ShortcutCSA algorithm(data, ch);
            runQueries<Vertex>(algorithm, data.transferGraph.numVertices(), numberOfQueries, outputFile);
        }
    }

    return 0;
}
