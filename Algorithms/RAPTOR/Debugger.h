#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

#include "../../DataStructures/RAPTOR/Data.h"

#include "../../Helpers/Timer.h"

namespace RAPTOR {

class NoDebugger {

public:
    inline void initialize(const Data&, const TransferGraph* = NULL) const noexcept {}

    inline void start() const noexcept {}
    inline void done() const noexcept {}

    inline void startInitialization() const noexcept {}
    inline void doneInitialization() const noexcept {}
    inline void directWalking(const int) const noexcept {}

    inline void newRound() const noexcept {}

    inline void startCollectRoutes() const noexcept {}
    inline void stopCollectRoutes() const noexcept {}
    inline void startScanRoutes() const noexcept {}
    inline void stopScanRoutes() const noexcept {}
    inline void startRelaxTransfers() const noexcept {}
    inline void stopRelaxTransfers() const noexcept {}

    inline void scanRoute(const RouteId) const noexcept {}
    inline void scanRouteSegment(const size_t) const noexcept {}

    inline void settleVertex(const Vertex) const noexcept {}
    inline void relaxEdge(const Edge) const noexcept {}
    inline void relaxShortcut(const Vertex, const Vertex) const noexcept {}

    inline void updateStopByRoute(const StopId, const int) const noexcept {}
    inline void updateStopByTransfer(const StopId, const int) const noexcept {}
};

class TimeDebugger : public NoDebugger {

private:
    struct RoundData {
        inline RoundData& operator+=(const RoundData& other) noexcept {
            numberOfScannedRoutes += other.numberOfScannedRoutes;
            numberOfScannedRouteSegments += other.numberOfScannedRouteSegments;
            numberOfUpdatedStopsByRoute += other.numberOfUpdatedStopsByRoute;
            numberOfSettledVertices += other.numberOfSettledVertices;
            numberOfRelaxedEdges += other.numberOfRelaxedEdges;
            numberOfUpdatedStopsByTransfer += other.numberOfUpdatedStopsByTransfer;
            collectRoutesTime += other.collectRoutesTime;
            scanRoutesTime += other.scanRoutesTime;
            relaxTransfersTime += other.relaxTransfersTime;
            totalTime += other.totalTime;
            return *this;
        }
        int numberOfScannedRoutes{0};
        int numberOfScannedRouteSegments{0};
        int numberOfUpdatedStopsByRoute{0};
        int numberOfSettledVertices{0};
        int numberOfRelaxedEdges{0};
        int numberOfUpdatedStopsByTransfer{0};
        double collectRoutesTime{0.0};
        double scanRoutesTime{0.0};
        double relaxTransfersTime{0.0};
        double totalTime{0.0};
    };

public:
    inline void start() noexcept {
        roundTimer.restart();
        phaseTimer.restart();
        roundTime = 0.0;
        phaseTime = 0.0;
        statistics.clear();
        statistics.emplace_back();
    }
    inline void done() noexcept {
        stopRoundTime();
        printHeader();
        printStatistics();
    }

    inline void directWalking(const int time) const noexcept {
        std::cout << "Time required for direct walking: " << String::secToTime(time) << std::endl;
    }

    inline void newRound() noexcept {
        stopRoundTime();
        statistics.emplace_back();
    }

    inline void startCollectRoutes() noexcept {
        phaseTime = phaseTimer.elapsedMicroseconds();
    }

    inline void stopCollectRoutes() noexcept {
        statistics.back().collectRoutesTime = phaseTimer.elapsedMicroseconds() - phaseTime;
    }

    inline void startScanRoutes() noexcept {
        phaseTime = phaseTimer.elapsedMicroseconds();
    }

    inline void stopScanRoutes() noexcept {
        statistics.back().scanRoutesTime = phaseTimer.elapsedMicroseconds() - phaseTime;
    }

    inline void startRelaxTransfers() noexcept {
        phaseTime = phaseTimer.elapsedMicroseconds();
    }

    inline void stopRelaxTransfers() noexcept {
        statistics.back().relaxTransfersTime = phaseTimer.elapsedMicroseconds() - phaseTime;
    }

    inline void scanRoute(const RouteId) noexcept {
        statistics.back().numberOfScannedRoutes++;
    }

    inline void scanRouteSegment(const size_t) noexcept {
        statistics.back().numberOfScannedRouteSegments++;
    }

    inline void settleVertex(const Vertex) noexcept {
        statistics.back().numberOfSettledVertices++;
    }

    inline void relaxEdge(const Edge) noexcept {
        statistics.back().numberOfRelaxedEdges++;
    }

    inline void relaxShortcut(const Vertex, const Vertex) noexcept {
        statistics.back().numberOfRelaxedEdges++;
    }

    inline void updateStopByRoute(const StopId, const int) noexcept {
        statistics.back().numberOfUpdatedStopsByRoute++;
    }

    inline void updateStopByTransfer(const StopId, const int) noexcept {
        statistics.back().numberOfUpdatedStopsByTransfer++;
    }

    inline int getNumberOfScannedRoutes() const noexcept {
        return Vector::sum<int>(statistics, [&](const RoundData& data) {
            return data.numberOfScannedRoutes;
        });
    }

    inline int getNumberOfScannedRouteSegments() const noexcept {
        return Vector::sum<int>(statistics, [&](const RoundData& data) {
            return data.numberOfScannedRouteSegments;
        });
    }

    inline int getNumberOfUpdatedStopsByRoute() const noexcept {
        return Vector::sum<int>(statistics, [&](const RoundData& data) {
            return data.numberOfUpdatedStopsByRoute;
        });
    }

    inline int getNumberOfSettledVertices() const noexcept {
        return Vector::sum<int>(statistics, [&](const RoundData& data) {
            return data.numberOfSettledVertices;
        });
    }

    inline int getNumberOfRelaxedEdges() const noexcept {
        return Vector::sum<int>(statistics, [&](const RoundData& data) {
            return data.numberOfRelaxedEdges;
        });
    }

    inline int getNumberOfUpdatedStopsByTransfer() const noexcept {
        return Vector::sum<int>(statistics, [&](const RoundData& data) {
            return data.numberOfUpdatedStopsByTransfer;
        });
    }

    inline double getCollectRoutesTime() const noexcept {
        return Vector::sum<double>(statistics, [&](const RoundData& data) {
            return data.collectRoutesTime;
        });
    }

    inline double getScanRoutesTime() const noexcept {
        return Vector::sum<double>(statistics, [&](const RoundData& data) {
            return data.scanRoutesTime;
        });
    }

    inline double getInitialTransfersTime() const noexcept {
        return statistics.front().relaxTransfersTime;
    }

    inline double getTransfersTime() const noexcept {
        return Vector::sum<double>(statistics, [&](const RoundData& data) {
            return data.relaxTransfersTime;
        });
    }

    inline double getIntermediateTransfersTime() const noexcept {
        return getTransfersTime() - getInitialTransfersTime();
    }

    inline double getTotalTime() const noexcept {
        return Vector::sum<double>(statistics, [&](const RoundData& data) {
            return data.totalTime;
        });
    }

private:
    inline void stopRoundTime() noexcept {
        double elapsedMicroseconds = roundTimer.elapsedMicroseconds();
        statistics.back().totalTime = elapsedMicroseconds - roundTime;
        roundTime = elapsedMicroseconds;
    }

    inline void printHeader() const noexcept {
        std::cout << "\nStatistics:\n"
                  << std::setw(8) << "Round"
                  << std::setw(54) << "Scan Routes"
                  << std::setw(54) << "Relax Transfers"
                  << std::setw(53) << "Time\n"
                  << std::setw(26) << "Scanned Routes"
                  << std::setw(18) << "Scanned Segments"
                  << std::setw(18) << "Updated Stops"
                  << std::setw(18) << "Settled Vertices"
                  << std::setw(18) << "Relaxed Edges"
                  << std::setw(18) << "Updated Stops"
                  << std::setw(13) << "Collect"
                  << std::setw(13) << "Routes"
                  << std::setw(13) << "Transfers"
                  << std::setw(13) << "Total" << std::endl;
    }

    inline void printRow(const std::string& name, const RoundData& roundData) const noexcept {
        std::cout << std::setw(8) << name
                  << std::setw(18) << String::prettyInt(roundData.numberOfScannedRoutes)
                  << std::setw(18) << String::prettyInt(roundData.numberOfScannedRouteSegments)
                  << std::setw(18) << String::prettyInt(roundData.numberOfUpdatedStopsByRoute)
                  << std::setw(18) << String::prettyInt(roundData.numberOfSettledVertices)
                  << std::setw(18) << String::prettyInt(roundData.numberOfRelaxedEdges)
                  << std::setw(18) << String::prettyInt(roundData.numberOfUpdatedStopsByTransfer)
                  << std::setw(14) << String::musToString(roundData.collectRoutesTime)
                  << std::setw(14) << String::musToString(roundData.scanRoutesTime)
                  << std::setw(14) << String::musToString(roundData.relaxTransfersTime)
                  << std::setw(14) << String::musToString(roundData.totalTime) << std::endl;
    }

    inline void printStatistics() const noexcept {
        RoundData total = statistics.front();
        printRow("init", total);
        for (size_t i = 1; i < statistics.size(); i++) {
            printRow(String::prettyInt(i - 1), statistics[i]);
            total += statistics[i];
        }
        printRow("total", total);
    }

private:
    Timer roundTimer;
    double roundTime;
    Timer phaseTimer;
    double phaseTime;

    std::vector<RoundData> statistics;

};

}
