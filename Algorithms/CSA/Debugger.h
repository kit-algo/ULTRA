#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

#include "../../DataStructures/CSA/Data.h"

#include "../../Helpers/Types.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/String/String.h"

namespace CSA {

class NoDebugger {

public:
    inline void initialize() const noexcept {}
    inline void initialize(const Data&) const noexcept {}

    inline void start() const noexcept {}
    inline void done() const noexcept {}

    inline void startClear() const noexcept {}
    inline void doneClear() const noexcept {}

    inline void startInitialization() const noexcept {}
    inline void doneInitialization() const noexcept {}

    inline void startConnectionScan() const noexcept {}
    inline void doneConnectionScan() const noexcept {}

    inline void scanConnection(const Connection&) const noexcept {}
    inline void relaxEdge(const Edge) const noexcept {};

    inline void updateStopByTrip(const StopId, const int) const noexcept {}
    inline void updateStopByTransfer(const StopId, const int) const noexcept {}
};

class TimeDebugger : public NoDebugger {

public:
    inline void initialize() noexcept {
        totalTime = 0;
        clearTime = 0;
        initializationTime = 0;
        connectionScanTime = 0;
        scannedConnections = 0;
        relaxedTransfers = 0;
        updatedStopsByTrip = 0;
        updatedStopsByTransfer = 0;
    }

    inline void initialize(const Data&) noexcept {
        initialize();
    }

    inline void start() noexcept {
        initialize();
        totalTimer.restart();
    }

    inline void done() noexcept {
        totalTime = totalTimer.elapsedMicroseconds();
        printStatistics();
    }

    inline void startClear() noexcept {
        phaseTimer.restart();
    }

    inline void doneClear() noexcept {
        clearTime = phaseTimer.elapsedMicroseconds();
    }

    inline void startInitialization() noexcept {
        phaseTimer.restart();
    }

    inline void doneInitialization() noexcept {
        initializationTime = phaseTimer.elapsedMicroseconds();
    }

    inline void startConnectionScan() noexcept {
        phaseTimer.restart();
    }

    inline void doneConnectionScan() noexcept {
        connectionScanTime = phaseTimer.elapsedMicroseconds();
    }

    inline void scanConnection(const Connection&) noexcept {
        scannedConnections++;
    }

    inline void relaxEdge(const Edge) noexcept {
        relaxedTransfers++;
    }

    inline void updateStopByTrip(const StopId, const int) noexcept {
        updatedStopsByTrip++;
    }

    inline void updateStopByTransfer(const StopId, const int) noexcept {
        updatedStopsByTransfer++;
    }

    inline double getClearTime() const noexcept {
        return clearTime;
    }

    inline double getInitializationTime() const noexcept {
        return initializationTime;
    }

    inline double getConnectionScanTime() const noexcept {
        return connectionScanTime;
    }

    inline double getTotalTime() const noexcept {
        return totalTime;
    }

    inline size_t getNumberOfScannedConnections() const noexcept {
        return scannedConnections;
    }

    inline size_t getNumberOfRelaxedTransfers() const noexcept {
        return relaxedTransfers;
    }

    inline size_t getNumberOfUpdatedStopsByTrip() const noexcept {
        return updatedStopsByTrip;
    }

    inline size_t getNumberOfUpdatedStopsByTransfer() const noexcept {
        return updatedStopsByTransfer;
    }

private:
    inline void printStatistics() const noexcept {
        std::cout << std::endl;
        std::cout << "Total time: " << String::musToString(totalTime) << std::endl;
        std::cout << "\tClear: " << String::musToString(clearTime) << std::endl;
        std::cout << "\tInitialization: " << String::musToString(initializationTime) << std::endl;
        std::cout << "\tConnection scan: " << String::musToString(connectionScanTime) << std::endl;
        std::cout << std::endl;

        std::cout << "Scanned connections: " << String::prettyInt(scannedConnections) << std::endl;
        std::cout << "Relaxed transfers: " << String::prettyInt(relaxedTransfers) << std::endl;
        std::cout << "Updated stops by trip: " << String::prettyInt(updatedStopsByTrip) << std::endl;
        std::cout << "Updated stops by transfer: " << String::prettyInt(updatedStopsByTransfer) << std::endl;
        std::cout << std::endl;
    }

private:
    Timer totalTimer;
    double totalTime;
    Timer phaseTimer;
    double clearTime;
    double initializationTime;
    double connectionScanTime;

    size_t scannedConnections;
    size_t relaxedTransfers;
    size_t updatedStopsByTrip;
    size_t updatedStopsByTransfer;
};

}
