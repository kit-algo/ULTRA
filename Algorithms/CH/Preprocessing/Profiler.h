#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "CHData.h"

#include "../../../Helpers/Types.h"
#include "../../../Helpers/Timer.h"
#include "../../../Helpers/Console/ProgressBar.h"

namespace CH {

class NoProfiler {

public:
    inline void initialize(Data*) noexcept {}

    inline void start() noexcept {}
    inline void done() noexcept {}

    inline void startBuildingQ() noexcept {}
    inline void doneBuildingQ() noexcept {}
    inline void enQ(const Vertex, const int) noexcept {}

    inline void startContracting() noexcept {}
    inline void doneContracting() noexcept {}

    inline void startContraction(const Vertex) noexcept {}
    inline void doneContraction(const Vertex) noexcept {}
    inline void testShortcut() noexcept {}
    inline void addShortcut() noexcept {}
    inline void updateIncomingNeighbor(const Vertex, const int) noexcept {}
    inline void updateOutgoingNeighbor(const Vertex, const int) noexcept {}

    inline void startWitnessSearch() noexcept {}
    inline void doneWitnessSearch() noexcept {}
    inline void settledVertex() noexcept {}

};

class TimeProfiler : public NoProfiler {

public:
    inline void start() noexcept {
        std::cout << "Start building CH." << std::flush;
        timer.restart();
    }

    inline void done() noexcept {
        std::cout << " Done (" << String::msToString(timer.elapsedMilliseconds()) << ")." << std::endl << std::flush;
    }

private:
    Timer timer;

};

class ProgressProfiler : public NoProfiler {

public:
    inline void initialize(Data* data) noexcept {
        this->data = data;
    }

    inline void start() noexcept {
        std::cout << "Start building CH." << std::flush;
        timer.restart();
    }

    inline void doneContraction(const Vertex) noexcept {
        if (data->order.size() % 100000 == 0) {
            std::cout << data->order.size() << " " << String::msToString(timer.elapsedMilliseconds()) << std::endl;
        }
    }

    inline void done() noexcept {
        std::cout << " Done (" << String::msToString(timer.elapsedMilliseconds()) << ")." << std::endl << std::flush;
    }

private:
    Timer timer;
    Data* data;

};

class FullProfiler : public NoProfiler {

public:
    FullProfiler() :
        data(0),
        QBar(0),
        printTime(-1),
        expectedTime(-1),
        QTime(-1),
        printCounter(0),
        maxLevel(0),
        contractions(0),
        shortcutsAdded(0),
        shortcutsTested(0),
        inDegSum(0),
        outDegSum(0),
        witnessSearchTime(0),
        witnessSearchCount(0),
        lastWitnessSearchCount(0),
        settledVertices(0),
        lastSettledVertices(0) {
    }

    inline void initialize(Data* data) noexcept {
        this->data = data;
        this->printTime = -1;
        this->expectedTime = -1;
        this->QTime = 0;
        this->printCounter = 0;
        this->maxLevel = 0;
        this->contractions = 0;
        this->shortcutsAdded = 0;
        this->shortcutsTested = 0;
        this->inDegSum = 0;
        this->outDegSum = 0;
        this->witnessSearchTime = 0;
        this->witnessSearchCount = 0;
        this->lastWitnessSearchCount = 0;
        this->settledVertices = 0;
        this->lastSettledVertices = 0;
        std::vector<bool>(data->numVertices, false).swap(this->contracted);
        for (const Vertex vertex : data->order) {
            contracted[vertex] = true;
        }
    }

    inline void start() noexcept {
        std::cout << "Start building CH." << std::endl;
        timer.restart();
    }

    inline void done() noexcept {
        double contractionTime = timer.elapsedMilliseconds();
        std::cout << "Done building CH (" << String::msToString(contractionTime + QTime) << ")." << std::endl;
        std::cout << "  Building Q: " << String::msToString(QTime) << std::endl;
        std::cout << "  Contraction: " << String::msToString(contractionTime) << std::endl;
        std::cout << "  Core vertices: " << String::prettyInt(data->numVertices - data->order.size()) << std::endl;
        std::cout << "  Core edges: " << String::prettyInt(data->core.numEdges()) << std::endl;
        std::cout << "  Max CH Level: " << maxLevel << std::endl;
        std::cout << "  Forward edges: " << String::prettyInt(data->forwardCH.numEdges()) << " (" << (data->forwardCH.numEdges() / (double) data->numVertices) << " per vertex)" << std::endl;
        std::cout << "  Backward edges: " << String::prettyInt(data->backwardCH.numEdges()) << " (" << (data->backwardCH.numEdges() / (double) data->numVertices) << " per vertex)" << std::endl;
        std::cout << "  Number of witness searches: " << String::prettyInt(witnessSearchCount) << std::endl;
        std::cout << "  Number settled vertices: " << String::prettyInt(settledVertices) << std::endl;
        std::cout << "  Witness search time: " << String::msToString(witnessSearchTime) << std::endl;
    }

    inline void startBuildingQ() noexcept {
        QTime = 0;
        std::cout << "Start building Q." << std::endl;
        QBar = new ProgressBar(data->numVertices - data->order.size());
    }

    inline void doneBuildingQ() noexcept {
        QTime = timer.elapsedMilliseconds();
        std::cout << std::endl << "Done building Q (" << String::msToString(QTime) << ")." << std::endl;
        delete QBar;
        QBar = 0;
        timer.restart();
    }

    inline void enQ(const Vertex, const int) noexcept {
        (*QBar)++;
    }

    inline void startContracting() noexcept {
        std::cout << "Start contracting vertices." << std::endl;
        printHeader();
        printStatistics();
    }

    inline void doneContracting() noexcept {
        std::cout << "Done contracting vertices." << std::endl;
    }

    inline void startContraction(const Vertex vertex) noexcept {
        if (contracted[vertex]) std::cout << "ERROR! Vertex " << vertex << " got contracted more than once!" << std::endl;
        contractions++;
    }

    inline void doneContraction(const Vertex vertex) noexcept {
        contracted[vertex] = true;
        if ((timer.elapsedMilliseconds() > printTime + 2000) || (data->order.size() % 200000 == 0)) {
            if (printCounter % 50 == 0) printHeader();
            printStatistics();
        }
    }

    inline void testShortcut() noexcept {
        shortcutsTested++;
    }

    inline void addShortcut() noexcept {
        shortcutsAdded++;
    }

    inline void updateIncomingNeighbor(const Vertex vertex, const int) noexcept {
        if (maxLevel < data->level[vertex]) maxLevel = data->level[vertex];
        inDegSum++;
    }

    inline void updateOutgoingNeighbor(const Vertex vertex, const int) noexcept {
        if (maxLevel < data->level[vertex]) maxLevel = data->level[vertex];
        outDegSum++;
    }

    inline void startWitnessSearch() noexcept {
        witnessSearchTimer.restart();
        witnessSearchCount++;
    }

    inline void doneWitnessSearch() noexcept {
        witnessSearchTime += witnessSearchTimer.elapsedMilliseconds();
    }

    inline void settledVertex() noexcept {
        settledVertices++;
    }

private:
    inline void printHeader() noexcept {
        std::cout << std::endl
                  << std::setw(11) << "Contracted" << std::setw(11) << "Missing" << std::setw(10) << "percent"
                  << std::setw(11) << "Max Level"
                  << std::setw(12) << "CH degree" << std::setw(13) << "Core degree"
                  << std::setw(14) << "Contractions" << std::setw(12) << "(per ms)"
                  << std::setw(21) << "Shortcuts(add/test)" << std::setw(10) << "percent"
                  << std::setw(10) << "indeg" << std::setw(10) << "outdeg"
                  << std::setw(18) << "Expected Time" << std::setw(16) << "Total Time"
                  << std::setw(15) << "WS/contract"
                  << std::setw(12) << "Qpop/WS"
                  << std::setw(13) << "addedSC/WS" << std::endl;
        printCounter++;
    }

    inline void printStatistics() noexcept {
        if (contractions == 0) return;
        const double totalTime = timer.elapsedMilliseconds();
        const double roundTime = totalTime - printTime;
        const int contracted = data->order.size();
        const int missing = data->numVertices - contracted;
        const double missingPercent = missing / static_cast<double>(missing + contracted);
        const double coreDegree = data->core.numEdges() / static_cast<double>(missing);
        const double smooth = (expectedTime <= roundTime) ? 1 : 0.9;
        if (contracted > 0) {
            const double chDegree = (data->forwardCH.numEdges() + data->backwardCH.numEdges()) / static_cast<double>(contracted);
            const double contractionsPerMS = contractions / roundTime;
            const double shortcutsAddedPercent = shortcutsAdded / static_cast<double>(shortcutsTested);
            const double indeg = inDegSum / static_cast<double>(contractions);
            const double outdeg = outDegSum / static_cast<double>(contractions);
            const double witnessSearches = static_cast<double>(witnessSearchCount - lastWitnessSearchCount);
            const double witnessSearchPerContraction = witnessSearches / contractions;
            const double settledVerticesPerWitnessSearch = (settledVertices - lastSettledVertices) / witnessSearches;
            const double addedShortcutsPerWitnessSearch = shortcutsAdded / witnessSearches;
            expectedTime = (smooth * missing / contractionsPerMS) + ((1 - smooth) * (expectedTime - roundTime));
            std::cout << std::setw(11) << String::prettyInt(contracted) << std::setw(11) << String::prettyInt(missing) << std::setw(10) << String::percent(missingPercent)
                      << std::setw(11) << String::prettyInt(maxLevel)
                      << std::setw(12) << String::prettyDouble(chDegree, 2) << std::setw(13) << String::prettyDouble(coreDegree, 2)
                      << std::setw(14) << String::prettyInt(contractions) << std::setw(12) << String::prettyDouble(contractionsPerMS ,2)
                      << std::setw(21) << (String::prettyInt(shortcutsAdded) + "/" + String::prettyInt(shortcutsTested)) << std::setw(10) << String::percent(shortcutsAddedPercent)
                      << std::setw(10) << String::prettyDouble(indeg, 2) << std::setw(10) << String::prettyDouble(outdeg, 2)
                      << std::setw(18) << String::secToString(expectedTime / 1000.0) << std::setw(16) << String::secToString(totalTime / 1000.0)
                      << std::setw(15) << String::prettyDouble(witnessSearchPerContraction, 2)
                      << std::setw(12) << String::prettyDouble(settledVerticesPerWitnessSearch, 2)
                      << std::setw(13) << String::percent(addedShortcutsPerWitnessSearch) << std::endl;
        } else {
            std::cout << std::setw(11) << String::prettyInt(contracted) << std::setw(11) << String::prettyInt(missing) << std::setw(10) << String::percent(missingPercent)
                      << std::setw(11) << String::prettyInt(maxLevel)
                      << std::setw(12) << "-" << std::setw(13) << String::prettyDouble(coreDegree, 2)
                      << std::setw(14) << String::prettyInt(contractions) << std::setw(12) << "-"
                      << std::setw(21) << (String::prettyInt(shortcutsAdded) + "/" + String::prettyInt(shortcutsTested)) << std::setw(10) << "0.00%"
                      << std::setw(10) << "-" << std::setw(10) << "-"
                      << std::setw(18) << "-" << std::setw(16) << String::secToString(totalTime / 1000.0)
                      << std::setw(15) << "-"
                      << std::setw(12) << "-"
                      << std::setw(13) << "-" << std::endl;
        }
        printTime = totalTime;
        printCounter++;
        contractions = 0;
        shortcutsAdded = 0;
        shortcutsTested = 0;
        inDegSum = 0;
        outDegSum = 0;
        lastSettledVertices = settledVertices;
        lastWitnessSearchCount = witnessSearchCount;
    }

private:
    Data* data;
    ProgressBar* QBar;

    Timer timer;
    double printTime;
    double expectedTime;
    double QTime;

    std::vector<bool> contracted;
    int printCounter;
    size_t maxLevel;
    int contractions;
    int shortcutsAdded;
    int shortcutsTested;
    int inDegSum;
    int outDegSum;

    Timer witnessSearchTimer;
    double witnessSearchTime;
    int witnessSearchCount;
    int lastWitnessSearchCount;
    long long settledVertices;
    long long lastSettledVertices;

};

}
