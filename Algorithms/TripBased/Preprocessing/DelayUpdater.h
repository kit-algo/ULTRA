#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "../../../DataStructures/Graph/Graph.h"

#include "../../../DataStructures/TripBased/Data.h"
#include "../../../DataStructures/TripBased/Delay.h"
#include "../../../DataStructures/TripBased/DelayData.h"
#include "../../../DataStructures/TripBased/DelayInfo.h"
#include "../../../DataStructures/TripBased/DelayUpdateData.h"

#include "../../../Helpers/IO/File.h"
#include "../../../Helpers/String/String.h"
#include "../../../Helpers/Vector/Permutation.h"
#include "../../../Helpers/Vector/Vector.h"
#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/Timer.h"

#include "../../RAPTOR/InitialTransfers.h"
#include "ReplacementSearchBuilder.h"

namespace TripBased {

template<bool SET_EDGE_DEFAULTS, typename GRAPH>
inline void mergeShortcuts(GRAPH& graph, const std::vector<ShortcutInfo>& newShortcuts) noexcept {
    GRAPH temp;
    temp.reserve(graph.numVertices(), graph.numEdges() + newShortcuts.size());
    size_t j(0);
    for (const Vertex from : graph.vertices()) {
        temp.addVertex();
        Edge i = graph.beginEdgeFrom(from);
        while (i < graph.endEdgeFrom(from) && j < newShortcuts.size() && newShortcuts[j].from == from) {
            const Vertex iTo = graph.get(ToVertex, i);
            const Vertex jTo = newShortcuts[j].to;
            if (iTo == jTo) {
                if constexpr (SET_EDGE_DEFAULTS) {
                    const Edge edge = temp.addEdge(from, iTo, graph.edgeRecord(i));
                    temp.set(MinOriginDelay, edge, 0);
                    temp.set(MaxOriginDelay, edge, INFTY);
                    i++;
                    j++;
                } else {
                    Ensure(false, "Replacement search added a duplicate shortcut!");
                }
            } else if (iTo < jTo) {
                temp.addEdge(from, iTo, graph.edgeRecord(i));
                i++;
            } else {
                const Edge edge = temp.addEdge(from, jTo).set(TravelTime, newShortcuts[j].travelTime);
                if constexpr (SET_EDGE_DEFAULTS) {
                    temp.set(MinOriginDelay, edge, 0);
                    temp.set(MaxOriginDelay, edge, INFTY);
                } else {
                    suppressUnusedParameterWarning(edge);
                }
                j++;
            }
        }
        for (; i < graph.endEdgeFrom(from); i++) {
            temp.addEdge(from, graph.get(ToVertex, i), graph.edgeRecord(i));
        }
        for (; j < newShortcuts.size() && newShortcuts[j].from == from; j++) {
            const Edge edge = temp.addEdge(from, newShortcuts[j].to).set(TravelTime, newShortcuts[j].travelTime);
            if constexpr (SET_EDGE_DEFAULTS) {
                temp.set(MinOriginDelay, edge, 0);
                temp.set(MaxOriginDelay, edge, INFTY);
            } else {
                suppressUnusedParameterWarning(edge);
            }
        }
    }
    Graph::move(std::move(temp), graph);
}

inline void mergeShortcutsIntoDelayData(DelayData& delayData, std::vector<ShortcutInfo>& newShortcuts) noexcept {
    std::sort(newShortcuts.begin(), newShortcuts.end());
    mergeShortcuts<true>(delayData.stopEventGraph, newShortcuts);
}

struct UpdateLogEntry {
    UpdateLogEntry() {}

    UpdateLogEntry(const int finishTime, const std::vector<DelayUpdate>& updates, const std::vector<ShortcutInfo>& replacementShortcuts) :
        finishTime(finishTime),
        updates(updates),
        replacementShortcuts(replacementShortcuts) {
    }

    UpdateLogEntry(const int finishTime, const std::vector<DelayUpdate>& updates) :
        finishTime(finishTime),
        updates(updates) {
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(finishTime, updates, replacementShortcuts);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(finishTime, updates, replacementShortcuts);
    }

    inline void operator+=(const UpdateLogEntry& other) noexcept {
        updates += other.updates;
        replacementShortcuts += other.replacementShortcuts;
    }

    int finishTime;
    std::vector<DelayUpdate> updates;
    std::vector<ShortcutInfo> replacementShortcuts;
};

inline void applyAllReplacementShortcutsFromUpdateLog(DelayData& delayData, const std::string& fileName) noexcept {
    std::vector<UpdateLogEntry> updateLog;
    IO::deserialize(fileName, updateLog);
    std::vector<ShortcutInfo> replacementShortcuts;
    for (const UpdateLogEntry& entry : updateLog) {
        replacementShortcuts += entry.replacementShortcuts;
    }
    mergeShortcutsIntoDelayData(delayData, replacementShortcuts);
}

class DelayUpdaterWithReplacement {
private:
    struct UpdateStatistics {
        std::vector<int> startTimes;
        std::vector<int> finishTimes;
        std::vector<size_t> numUpdates;
        std::vector<size_t> numDelayedEvents;
        std::vector<size_t> numHighlyDelayedEvents;
        std::vector<size_t> numFilteredShortcuts;
        std::vector<size_t> numInfeasibleShortcuts;
        std::vector<size_t> numAddedShortcuts;
        std::vector<size_t> numQueryShortcuts;
        std::vector<double> dataUpdateTime;
        std::vector<double> searchTime;
        std::vector<double> originSearchTime;
        std::vector<double> targetSearchTime;
        std::vector<double> mergeTime;
        std::vector<double> totalTime;

        inline void writeToFile(const std::string& filename) const noexcept {
            IO::OFStream file(filename);
            file << "Start time,Finish time,Processed updates,Delayed events,Highly delayed events,Filtered shortcuts,Infeasible shortcuts,Replacement shortcuts,Query shortcuts,Data update time,Replacement search time,Origin search time,Target search time,Shortcut merge time,Total time\n";
            for (size_t i = 0; i < numUpdates.size(); i++) {
                file << String::secToTime(startTimes[i], true) << "," << String::secToTime(finishTimes[i], true) << "," << numUpdates[i] << "," << numDelayedEvents[i] << "," << numHighlyDelayedEvents[i] << "," << numFilteredShortcuts[i] << "," << numInfeasibleShortcuts[i] << "," << numAddedShortcuts[i] << "," << numQueryShortcuts[i] << "," << dataUpdateTime[i] << "," << searchTime[i] << "," << originSearchTime[i] << "," << targetSearchTime[i] << "," << mergeTime[i] << "," << totalTime[i] << "\n";
            }
        }

        inline void finalize() noexcept {
            numAddedShortcuts.emplace_back(numQueryShortcuts.back() - numFilteredShortcuts.back());
            totalTime.emplace_back(dataUpdateTime.back() + searchTime.back() + mergeTime.back());
        }

        inline friend std::ostream& operator<<(std::ostream& out, const UpdateStatistics& statistics) noexcept {
            out << "Avg. updates: " << String::prettyDouble(Vector::mean(statistics.numUpdates)) << std::endl;
            out << "Avg. delayed events: " << String::prettyDouble(Vector::mean(statistics.numDelayedEvents)) << std::endl;
            out << "Avg. highly delayed events: " << String::prettyDouble(Vector::mean(statistics.numHighlyDelayedEvents)) << std::endl;
            out << "Avg. query shortcuts: " << String::prettyDouble(Vector::mean(statistics.numQueryShortcuts)) << std::endl;
            out << "Total added shortcuts: " << String::prettyInt(Vector::sum(statistics.numAddedShortcuts)) << std::endl;
            out << "Max. data update time: " << String::musToString(Vector::max(statistics.dataUpdateTime)) << std::endl;
            out << "Avg. data update time: " << String::musToString(Vector::mean(statistics.dataUpdateTime)) << std::endl;
            out << "Max. search time: " << String::musToString(Vector::max(statistics.searchTime)) << std::endl;
            out << "Avg. search time: " << String::musToString(Vector::mean(statistics.searchTime)) << std::endl;
            out << "Max. origin time: " << String::musToString(Vector::max(statistics.originSearchTime)) << std::endl;
            out << "Avg. origin time: " << String::musToString(Vector::mean(statistics.originSearchTime)) << std::endl;
            out << "Max. target time: " << String::musToString(Vector::max(statistics.targetSearchTime)) << std::endl;
            out << "Avg. target time: " << String::musToString(Vector::mean(statistics.targetSearchTime)) << std::endl;
            out << "Max. merge time: " << String::musToString(Vector::max(statistics.mergeTime)) << std::endl;
            out << "Avg. merge time: " << String::musToString(Vector::mean(statistics.mergeTime)) << std::endl;
            out << "Max. total time: " << String::musToString(Vector::max(statistics.totalTime)) << std::endl;
            out << "Avg. total time: " << String::musToString(Vector::mean(statistics.totalTime)) << std::endl;
            return out;
        }
    };

public:
    DelayUpdaterWithReplacement(DelayData& delayData, const RAPTOR::BucketCHInitialTransfers& initialTransfers, const ThreadPinning& threadPinning, const int targetSlack) :
        delayData(delayData),
        delayUpdateData(delayData.createUpdateData(initialTransfers)),
        delayInfo(delayData),
        builder(delayUpdateData, threadPinning, targetSlack) {
    }

    inline void preRunUpdate(const std::vector<DelayUpdate>& updates, const bool allowDepartureDelays, const bool replaceDelayedTargets) noexcept {
        delayData.applyDelayUpdates(updates, allowDepartureDelays);
        delayData.refreshUpdateData(delayUpdateData);
        delayInfo.update(updates);
        delayInfo.applyPermutation(delayUpdateData.originalToInternal);

        builder.run(delayInfo, replaceDelayedTargets);
        std::vector<ShortcutInfo> replacementShortcuts = builder.getShortcuts();
        if (!replacementShortcuts.empty()) {
            mergeShortcuts<false>(delayUpdateData.tripData.stopEventGraph, replacementShortcuts);
            for (ShortcutInfo& shortcut : replacementShortcuts) {
                shortcut.applyVertexPermutation(delayUpdateData.internalToOriginal);
            }
            mergeShortcutsIntoDelayData(delayData, replacementShortcuts);
        }
        updateLog.emplace_back(-INFTY, updates, replacementShortcuts);
    }

    inline void runUpdates(const std::vector<DelayUpdate>& updates, const bool allowDepartureDelays, const bool replaceDelayedTargets) noexcept {
        logTimer.restart();
        size_t i = 0;
        Progress progress(updates.size());
        while (i < updates.size()) {
            int startTime = getLogTime();
            if (updates[i].revealTime > startTime) {
                advanceLogTimer(updates[i].revealTime);
                startTime = updates[i].revealTime;
            }
            std::vector<DelayUpdate> currentUpdates;
            while (i < updates.size() && updates[i].revealTime <= startTime) {
                currentUpdates.emplace_back(updates[i]);
                i++;
                progress++;
            }
            runUpdate(currentUpdates, allowDepartureDelays, replaceDelayedTargets, startTime);
        }
    }

    inline void runUpdate(const std::vector<DelayUpdate>& updates, const bool allowDepartureDelays, const bool replaceDelayedTargets, const int startTime) noexcept {
        statistics.startTimes.emplace_back(startTime);
        statistics.numUpdates.emplace_back(updates.size());

        phaseTimer.restart();
        delayData.applyDelayUpdates(updates, allowDepartureDelays);
        delayData.refreshUpdateData(delayUpdateData);
        delayInfo.update(updates);
        delayInfo.applyPermutation(delayUpdateData.originalToInternal);
        statistics.dataUpdateTime.emplace_back(phaseTimer.elapsedMicroseconds());
        statistics.numDelayedEvents.emplace_back(delayInfo.getDelayedEvents().size());
        statistics.numHighlyDelayedEvents.emplace_back(delayInfo.getHighlyDelayedEvents().size());
        statistics.numFilteredShortcuts.emplace_back(delayUpdateData.tripData.stopEventGraph.numEdges());
        statistics.numInfeasibleShortcuts.emplace_back(delayUpdateData.infeasibleShortcuts.numEdges());

        phaseTimer.restart();
        builder.run(delayInfo, replaceDelayedTargets);
        statistics.searchTime.emplace_back(phaseTimer.elapsedMicroseconds());
        statistics.originSearchTime.emplace_back(builder.getOriginSearchTime());
        statistics.targetSearchTime.emplace_back(builder.getTargetSearchTime());
        phaseTimer.restart();
        std::vector<ShortcutInfo> replacementShortcuts = builder.getShortcuts();
        if (!replacementShortcuts.empty()) {
            mergeShortcuts<false>(delayUpdateData.tripData.stopEventGraph, replacementShortcuts);
            for (ShortcutInfo& shortcut : replacementShortcuts) {
                shortcut.applyVertexPermutation(delayUpdateData.internalToOriginal);
            }
            mergeShortcutsIntoDelayData(delayData, replacementShortcuts);
        }
        statistics.mergeTime.emplace_back(phaseTimer.elapsedMicroseconds());
        statistics.numQueryShortcuts.emplace_back(delayUpdateData.tripData.stopEventGraph.numEdges());
        statistics.finishTimes.emplace_back(getLogTime());
        statistics.finalize();
        updateLog.emplace_back(statistics.finishTimes.back(), updates, replacementShortcuts);
    }

    inline void writeStatistics(const std::string& fileName) const noexcept {
        std::cout << statistics << std::endl;
        statistics.writeToFile(fileName);
    }

    inline void writeLog(const std::string& fileName) const noexcept {
        IO::serialize(fileName, updateLog);
    }

private:
    inline int getLogTime() noexcept {
        return logTimer.elapsedMilliseconds() / 1000;
    }

    inline void advanceLogTimer(const int targetTime) noexcept {
        logTimer.advance(static_cast<double>(targetTime) * 1000000);
    }

private:
    DelayData& delayData;
    DelayUpdateData delayUpdateData;
    DelayInfo delayInfo;
    ReplacementSearchBuilder builder;
    UpdateStatistics statistics;
    std::vector<UpdateLogEntry> updateLog;
    Timer logTimer;
    Timer phaseTimer;
};

class DelayUpdaterWithoutReplacement {
private:
    struct UpdateStatistics {
        std::vector<int> startTimes;
        std::vector<int> finishTimes;
        std::vector<size_t> numUpdates;
        std::vector<size_t> numQueryShortcuts;
        std::vector<double> totalTime;

        inline void writeToFile(const std::string& filename) const noexcept {
            IO::OFStream file(filename);
            file << "Start time,Finish time,Processed updates,Query shortcuts,Total time\n";
            for (size_t i = 0; i < numUpdates.size(); i++) {
                file << String::secToTime(startTimes[i], true) << "," << String::secToTime(finishTimes[i], true) << "," << numUpdates[i] << "," << numQueryShortcuts[i] << "," << totalTime[i] << "\n";
            }
        }

        inline friend std::ostream& operator<<(std::ostream& out, const UpdateStatistics& statistics) noexcept {
            out << "Avg. updates: " << String::prettyDouble(Vector::mean(statistics.numUpdates)) << std::endl;
            out << "Avg. query shortcuts: " << String::prettyDouble(Vector::mean(statistics.numQueryShortcuts)) << std::endl;
            out << "Max. total time: " << String::musToString(Vector::max(statistics.totalTime)) << std::endl;
            out << "Avg. total time: " << String::musToString(Vector::mean(statistics.totalTime)) << std::endl;
            return out;
        }
    };

public:
    DelayUpdaterWithoutReplacement(DelayData& delayData) :
        delayData(delayData),
        delayQueryData(delayData.createQueryData()) {
    }

    inline void preRunUpdate(const std::vector<DelayUpdate>& updates, const bool allowDepartureDelays) noexcept {
        delayData.applyDelayUpdates(updates, allowDepartureDelays);
        delayData.refreshQueryData(delayQueryData);
        updateLog.emplace_back(-INFTY, updates);
    }

    inline void runUpdates(const std::vector<DelayUpdate>& updates, const bool allowDepartureDelays) noexcept {
        logTimer.restart();
        size_t i = 0;
        Progress progress(updates.size());
        while (i < updates.size()) {
            int startTime = getLogTime();
            if (updates[i].revealTime > startTime) {
                advanceLogTimer(updates[i].revealTime);
                startTime = updates[i].revealTime;
            }
            std::vector<DelayUpdate> currentUpdates;
            while (i < updates.size() && updates[i].revealTime <= startTime) {
                currentUpdates.emplace_back(updates[i]);
                i++;
                progress++;
            }
            runUpdate(currentUpdates, allowDepartureDelays, startTime);
        }
    }

    inline void runUpdate(const std::vector<DelayUpdate>& updates, const bool allowDepartureDelays, const int startTime) noexcept {
        statistics.startTimes.emplace_back(startTime);
        statistics.numUpdates.emplace_back(updates.size());
        phaseTimer.restart();
        delayData.applyDelayUpdates(updates, allowDepartureDelays);
        delayData.refreshQueryData(delayQueryData);
        statistics.totalTime.emplace_back(phaseTimer.elapsedMicroseconds());
        statistics.numQueryShortcuts.emplace_back(delayQueryData.tripData.stopEventGraph.numEdges());
        statistics.finishTimes.emplace_back(getLogTime());
        updateLog.emplace_back(statistics.finishTimes.back(), updates);
    }

    inline void writeStatistics(const std::string& fileName) const noexcept {
        std::cout << statistics << std::endl;
        statistics.writeToFile(fileName);
    }

    inline void writeLog(const std::string& fileName) const noexcept {
        IO::serialize(fileName, updateLog);
    }

private:
    inline int getLogTime() noexcept {
        return logTimer.elapsedMilliseconds() / 1000;
    }

    inline void advanceLogTimer(const int targetTime) noexcept {
        logTimer.advance(static_cast<double>(targetTime) * 1000000);
    }

private:
    DelayData& delayData;
    DelayQueryData delayQueryData;
    UpdateStatistics statistics;
    std::vector<UpdateLogEntry> updateLog;
    Timer logTimer;
    Timer phaseTimer;
};

class DelayUpdateSimulator {
public:
    DelayUpdateSimulator(DelayData& delayData, const std::string& updateLogFile) :
        delayData(delayData),
        queryData(delayData.createQueryData()),
        nextLogEntry(0) {
        readLog(updateLogFile);
    }

    inline void applyNextUpdate(const bool allowDepartureDelays) noexcept {
        AssertMsg(nextLogEntry < updateLog.size(), "No update log entries left!");
        applyUpdate(updateLog[nextLogEntry], allowDepartureDelays);
        nextLogEntry++;
    }

    inline void applyUpdatesUntil(const int finishTime, const bool allowDepartureDelays) noexcept {
        UpdateLogEntry aggregatedEntry;
        while (nextLogEntry < updateLog.size() && updateLog[nextLogEntry].finishTime <= finishTime) {
            aggregatedEntry += updateLog[nextLogEntry];
            nextLogEntry++;
        }
        if (!aggregatedEntry.updates.empty()) {
            applyUpdate(aggregatedEntry, allowDepartureDelays);
        }
    }

    inline int nextUpdateFinishTime() const noexcept {
        AssertMsg(nextLogEntry < updateLog.size(), "No update log entries left!");
        return updateLog[nextLogEntry].finishTime;
    }

    inline size_t numUpdatePhases() const noexcept {
        return updateLog.size();
    }

    inline const DelayData& getDelayData() const noexcept {
        return delayData;
    }

    inline const DelayQueryData& getQueryData() const noexcept {
        return queryData;
    }

private:
    inline void readLog(const std::string& fileName) noexcept {
        IO::deserialize(fileName, updateLog);
    }

    inline void applyUpdate(UpdateLogEntry& logEntry, const bool allowDepartureDelays) noexcept {
        delayData.applyDelayUpdates(logEntry.updates, allowDepartureDelays);
        if (!logEntry.replacementShortcuts.empty()) {
            mergeShortcutsIntoDelayData(delayData, logEntry.replacementShortcuts);
        }
        delayData.refreshQueryData(queryData);
    }

    DelayData& delayData;
    DelayQueryData queryData;
    std::vector<UpdateLogEntry> updateLog;
    size_t nextLogEntry;
};

}
