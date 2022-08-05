#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

#include "../../DataStructures/RAPTOR/Data.h"

#include "../../Helpers/Timer.h"

namespace RAPTOR {

inline constexpr static int MetricWidth = 13;
inline constexpr static int TimeWidth = 15;

typedef enum {
    EXTRA_ROUND_CLEAR,
    EXTRA_ROUND_INITIALIZATION,
    EXTRA_ROUND_FINAL_TRANSFERS,
    EXTRA_ROUND_FORWARD_PRUNING,
    EXTRA_ROUND_BACKWARD_PRUNING,
    NUM_EXTRA_ROUNDS
} ExtraRound;

constexpr const char* ExtraRoundNames[] = {
    "clear",
    "init",
    "final",
    "forward",
    "backward",
};

typedef enum {
    PHASE_INITIALIZATION,
    PHASE_COLLECT,
    PHASE_SCAN,
    PHASE_TRANSFERS,
    PHASE_TRANSFERS_WALKING,
    PHASE_TRANSFERS_SCOOTER,
    PHASE_TRANSFERS_BICYCLE,
    PHASE_FINAL_TRANSFERS,
    NUM_PHASES
} Phase;

inline Phase getProfilerTransferPhase(const size_t mode) noexcept {
    return Phase(PHASE_TRANSFERS_WALKING + mode);
}

constexpr const char* PhaseNames[] = {
    "Init",
    "Collect",
    "Scan",
    "Transfers",
    "Transfers (Walking)",
    "Transfers (Scooter)",
    "Transfers (Bicycle)",
    "Final",
};

typedef enum {
    METRIC_ROUTES,
    METRIC_ROUTE_SEGMENTS,
    METRIC_VERTICES,
    METRIC_EDGES,
    METRIC_STOPS_BY_TRIP,
    METRIC_STOPS_BY_TRANSFER,
    NUM_METRICS
} Metric;

constexpr const char* MetricNames[] = {
    "Routes",
    "Segments",
    "Vertices",
    "Edges",
    "Stops (trip)",
    "Stops (transfer)",
};

class NoProfiler {

public:
    inline void registerExtraRounds(const std::initializer_list<ExtraRound>&) const noexcept {}
    inline void registerPhases(const std::initializer_list<Phase>&) const noexcept {}
    inline void registerMetrics(const std::initializer_list<Metric>&) const noexcept {}

    inline void initialize() const noexcept {}

    inline void start() const noexcept {}
    inline void done() const noexcept {}

    inline void startRound() const noexcept {}
    inline void startExtraRound(const ExtraRound) const noexcept {}
    inline void doneRound() const noexcept {}

    inline void startPhase() const noexcept {}
    inline void donePhase(const Phase) const noexcept {}

    inline void countMetric(const Metric) const noexcept {}
};


class BasicProfiler : public NoProfiler {

public:
    BasicProfiler() :
        numQueries(0),
        metricValue(NUM_METRICS, 0),
        roundCount(0),
        initialTime(0),
        measureInitialTime(false),
        totalTime(0) {
    }

public:
    inline void reset() noexcept {
        numQueries = 0;
        Vector::fill(metricValue, (long long)0);
        roundCount = 0;
        initialTime = 0.0;
        measureInitialTime = false;
        totalTime = 0.0;
    }

    inline void start() noexcept {
        roundCount++;
        totalTimer.restart();
    }

    inline void done() noexcept {
        totalTime += totalTimer.elapsedMicroseconds();
    }

    inline void startRound() noexcept {
        roundCount++;
    }

    inline void startExtraRound(const ExtraRound extraRound) noexcept {
        if (extraRound == EXTRA_ROUND_INITIALIZATION) {
            initialTimer.restart();
            measureInitialTime = true;
        } else {
            measureInitialTime = false;
        }
    }

    inline void stopRound() noexcept {
        if (measureInitialTime) {
            initialTime += initialTimer.elapsedMicroseconds();
        }
    }

    inline void countMetric(const Metric metric) noexcept {
        metricValue[metric]++;
    }

    inline void printStatistics() const noexcept {
        std::cout << "Number of scanned routes: " << String::prettyDouble(metricValue[METRIC_ROUTES] / numQueries, 0) << std::endl;
        std::cout << "Number of settled vertices: " << String::prettyDouble(metricValue[METRIC_VERTICES] / numQueries, 0) << std::endl;
        std::cout << "Number of rounds: " << String::prettyDouble(roundCount / numQueries, 2) << std::endl;
        std::cout << "Initial transfers time: " << String::musToString(initialTime / numQueries) << std::endl;
        std::cout << "Total time: " << String::musToString(totalTime / numQueries) << std::endl;
    }

private:
    size_t numQueries;
    std::vector<long long> metricValue;
    size_t roundCount;

    Timer initialTimer;
    double initialTime;
    bool measureInitialTime;

    Timer totalTimer;
    double totalTime;

};

struct RoundData {
    RoundData() :
        metricValue(NUM_METRICS, 0),
        phaseTime(NUM_PHASES, 0.0),
        totalTime(0.0) {
    }

    inline RoundData& operator+=(const RoundData& other) noexcept {
        for (size_t metric = 0; metric < NUM_METRICS; metric++) {
            metricValue[metric] += other.metricValue[metric];
        }
        for (size_t phase = 0; phase < NUM_PHASES; phase++) {
            phaseTime[phase] += other.phaseTime[phase];
        }
        totalTime += other.totalTime;
        return *this;
    }

    inline RoundData& operator/=(size_t numQueries) noexcept {
        for (long long& metric : metricValue) {
            metric /= numQueries;
        }
        for (double& phase : phaseTime) {
            phase /= numQueries;
        }
        totalTime /= numQueries;
        return *this;
    }

    inline static int metricLength(const Metric metric) noexcept {
        return std::max((int)std::string(MetricNames[metric]).length() + 1, MetricWidth);
    }

    inline static int phaseLength(const Phase phase) noexcept {
        return std::max((int)std::string(PhaseNames[phase]).length() + 1, TimeWidth);
    }

    inline static void printHeader(const std::vector<Metric>& metrics, const std::vector<Phase>& phases) noexcept {
        std::cout << std::endl << "Statistics:" << std::endl << std::setw(8) << "Round";
        for (const Metric metric : metrics) {
            std::cout << std::setw(metricLength(metric)) << MetricNames[metric];
        }
        for (const Phase phase : phases) {
            std::cout << std::setw(phaseLength(phase)) << PhaseNames[phase];
        }
        std::cout << std::setw(TimeWidth) << "Total" << std::endl;
    }

    inline void print(const std::vector<Metric>& metrics, const std::vector<Phase>& phases, const std::string& name) const noexcept {
        std::cout << std::setw(8) << name;
        for (const Metric metric : metrics) {
            std::cout << std::setw(metricLength(metric)) << String::prettyInt(metricValue[metric]);
        }
        for (const Phase phase : phases) {
            std::cout << std::setw(phaseLength(phase) + 1) << String::musToString(phaseTime[phase]);
        }
        std::cout << std::setw(TimeWidth + 1) << String::musToString(totalTime) << std::endl;
    }

    inline void print(const std::vector<Metric>& metrics, const std::vector<Phase>& phases, const size_t round) const noexcept {
        print(metrics, phases, String::prettyInt(round));
    }

    std::vector<long long> metricValue;
    std::vector<double> phaseTime;
    double totalTime;
};

class SimpleProfiler : public NoProfiler {

public:
    SimpleProfiler() :
        totalTime(0.0),
        extraRoundData(NUM_EXTRA_ROUNDS),
        currentRoundData(NULL) {
    }

    inline void registerExtraRounds(const std::initializer_list<ExtraRound>& extraRoundList) noexcept {
        for (const ExtraRound extraRound : extraRoundList) {
            extraRounds.push_back(extraRound);
        }
    }

    inline void registerPhases(const std::initializer_list<Phase>& phaseList) noexcept {
        for (const Phase phase : phaseList) {
            phases.push_back(phase);
        }
    }

    inline void registerMetrics(const std::initializer_list<Metric>& metricList) noexcept {
        for (const Metric metric : metricList) {
            metrics.push_back(metric);
        }
    }

    inline void initialize() noexcept {
        totalTime = 0.0;
        roundData.clear();
        std::vector<RoundData>(NUM_EXTRA_ROUNDS).swap(extraRoundData);
        currentRoundData = NULL;
    }

    inline void start() noexcept {
        initialize();
        totalTimer.restart();
    }

    inline void done() noexcept {
        totalTime += totalTimer.elapsedMicroseconds();
        printStatistics();
    }

    inline void startRound() noexcept {
        roundData.emplace_back();
        currentRoundData = &roundData.back();
        roundTimer.restart();
    }

    inline void startExtraRound(const ExtraRound extraRound) noexcept {
        currentRoundData = &extraRoundData[extraRound];
        roundTimer.restart();
    }

    inline void doneRound() noexcept {
        currentRoundData->totalTime += roundTimer.elapsedMicroseconds();
    }

    inline void startPhase() noexcept {
        phaseTimer.restart();
    }

    inline void donePhase(const Phase phase) noexcept {
        currentRoundData->phaseTime[phase] += phaseTimer.elapsedMicroseconds();
    }

    inline void countMetric(const Metric metric) const noexcept {
        currentRoundData->metricValue[metric]++;
    }

    inline double getTotalTime() const noexcept {
        return totalTime;
    }

    inline double getExtraRoundTime(const ExtraRound extraRound) const noexcept {
        return extraRoundData[extraRound].totalTime;
    }

    inline double getPhaseTime(const Phase phase) const noexcept {
        double result = Vector::sum<long long>(roundData, [&](const RoundData& data) {
            return data.phaseTime[phase];
        });
        for (const ExtraRound extraRound : extraRounds) {
            result += extraRoundData[extraRound].phaseTime[phase];
        }
        return result;
    }

    inline double getPhaseTimeInExtraRound(const Phase phase, const ExtraRound extraRound) const noexcept {
        return extraRoundData[extraRound].phaseTime[phase];
    }

    inline long long getMetric(const Metric metric) const noexcept {
        long long result = Vector::sum<long long>(roundData, [&](const RoundData& data) {
            return data.metricValue[metric];
        });
        for (const ExtraRound extraRound : extraRounds) {
            result += extraRoundData[extraRound].metricValue[metric];
        }
        return result;
    }

    inline SimpleProfiler& operator+=(const SimpleProfiler& other) noexcept {
        totalTime += other.totalTime;
        if (roundData.size() < other.roundData.size()) {
            roundData.resize(other.roundData.size());
        }
        for (size_t i = 0; i < other.roundData.size(); i++) {
            roundData[i] += other.roundData[i];
        }
        for (size_t i = 0; i < NUM_EXTRA_ROUNDS; i++) {
            extraRoundData[i] += other.extraRoundData[i];
        }
        return *this;
    }

private:
    inline void printStatistics() const noexcept {
        RoundData::printHeader(metrics, phases);
        RoundData total;
        for (const ExtraRound extraRound : extraRounds) {
            extraRoundData[extraRound].print(metrics, phases, ExtraRoundNames[extraRound]);
            total += extraRoundData[extraRound];
        }
        for (size_t i = 0; i < roundData.size(); i++) {
            roundData[i].print(metrics, phases, i);
            total += roundData[i];
        }
        total.print(metrics, phases, "total");
        std::cout << "Total time: " << String::musToString(totalTime) << std::endl;
    }

    Timer totalTimer;
    double totalTime;
    Timer roundTimer;
    Timer phaseTimer;
    std::vector<Phase> phases;
    std::vector<Metric> metrics;
    std::vector<RoundData> roundData;
    std::vector<ExtraRound> extraRounds;
    std::vector<RoundData> extraRoundData;
    RoundData* currentRoundData;
};

class AggregateProfiler : public NoProfiler {

public:
    AggregateProfiler() :
        totalTime(0.0),
        extraRoundData(NUM_EXTRA_ROUNDS),
        currentRoundData(NULL),
        inExtraRound(false),
        numQueries(0),
        numRounds(0),
        totalNumRounds(0) {
    }

    inline void registerExtraRounds(const std::initializer_list<ExtraRound>& extraRoundList) noexcept {
        for (const ExtraRound extraRound : extraRoundList) {
            extraRounds.push_back(extraRound);
        }
    }

    inline void registerPhases(const std::initializer_list<Phase>& phaseList) noexcept {
        for (const Phase phase : phaseList) {
            phases.push_back(phase);
        }
    }

    inline void registerMetrics(const std::initializer_list<Metric>& metricList) noexcept {
        for (const Metric metric : metricList) {
            metrics.push_back(metric);
        }
    }

    inline void initialize() noexcept {
        totalTime = 0.0;
        roundData.clear();
        std::vector<RoundData>(NUM_EXTRA_ROUNDS).swap(extraRoundData);
        currentRoundData = NULL;
        inExtraRound = false;
        numQueries = 0;
        numRounds = 0;
        totalNumRounds = 0;
    }

    inline void start() noexcept {
        currentRoundData = NULL;
        numRounds = 0;
        inExtraRound = false;
        totalTimer.restart();
    }

    inline void done() noexcept {
        totalTime += totalTimer.elapsedMicroseconds();
        numQueries++;
    }

    inline void startRound() noexcept {
        if (numRounds >= roundData.size()) {
            roundData.emplace_back();
        }
        inExtraRound = false;
        currentRoundData = &roundData[numRounds];
        roundTimer.restart();
    }

    inline void startExtraRound(const ExtraRound extraRound) noexcept {
        inExtraRound = true;
        currentRoundData = &extraRoundData[extraRound];
        roundTimer.restart();
    }

    inline void doneRound() noexcept {
        currentRoundData->totalTime += roundTimer.elapsedMicroseconds();
        if (!inExtraRound) {
            numRounds++;
            totalNumRounds++;
        }
    }

    inline void startPhase() noexcept {
        phaseTimer.restart();
    }

    inline void donePhase(const Phase phase) noexcept {
        currentRoundData->phaseTime[phase] += phaseTimer.elapsedMicroseconds();
    }

    inline void countMetric(const Metric metric) const noexcept {
        currentRoundData->metricValue[metric]++;
    }

    inline double getTotalTime() const noexcept {
        return totalTime/numQueries;
    }

    inline double getExtraRoundTime(const ExtraRound extraRound) const noexcept {
        return extraRoundData[extraRound].totalTime/numQueries;
    }

    inline double getPhaseTime(const Phase phase) const noexcept {
        double result = Vector::sum<long long>(roundData, [&](const RoundData& data) {
            return data.phaseTime[phase];
        });
        for (const ExtraRound extraRound : extraRounds) {
            result += extraRoundData[extraRound].phaseTime[phase];
        }
        return result/numQueries;
    }

    inline double getPhaseTimeInExtraRound(const Phase phase, const ExtraRound extraRound) const noexcept {
        return extraRoundData[extraRound].phaseTime[phase]/numQueries;
    }

    inline double getMetric(const Metric metric) const noexcept {
        double result = Vector::sum<long long>(roundData, [&](const RoundData& data) {
            return data.metricValue[metric];
        });
        for (const ExtraRound extraRound : extraRounds) {
            result += extraRoundData[extraRound].metricValue[metric];
        }
        return result/numQueries;
    }

    inline void printStatistics() const noexcept {
        RoundData::printHeader(metrics, phases);
        RoundData total;
        for (const ExtraRound extraRound : extraRounds) {
            RoundData data = extraRoundData[extraRound];
            data /= numQueries;
            data.print(metrics, phases, ExtraRoundNames[extraRound]);
            total += data;
        }
        for (size_t i = 0; i < roundData.size(); i++) {
            RoundData data = roundData[i];
            data /= numQueries;
            data.print(metrics, phases, i);
            total += data;
        }
        total.print(metrics, phases, "total");
        std::cout << "Total time: " << String::musToString(totalTime/numQueries) << std::endl;
        std::cout << "Avg. rounds: " << String::prettyDouble(totalNumRounds/static_cast<double>(numQueries)) << std::endl;
    }

    inline AggregateProfiler& operator+=(const AggregateProfiler& other) noexcept {
        totalTime += other.totalTime;
        if (roundData.size() < other.roundData.size()) {
            roundData.resize(other.roundData.size());
        }
        for (size_t i = 0; i < other.roundData.size(); i++) {
            roundData[i] += other.roundData[i];
        }
        for (size_t i = 0; i < NUM_EXTRA_ROUNDS; i++) {
            extraRoundData[i] += other.extraRoundData[i];
        }
        numQueries += other.numQueries;
        return *this;
    }

    Timer totalTimer;
    double totalTime;
    Timer roundTimer;
    Timer phaseTimer;
    std::vector<Phase> phases;
    std::vector<Metric> metrics;
    std::vector<RoundData> roundData;
    std::vector<ExtraRound> extraRounds;
    std::vector<RoundData> extraRoundData;
    RoundData* currentRoundData;
    bool inExtraRound;
    size_t numQueries;
    size_t numRounds;
    size_t totalNumRounds;
};

}
