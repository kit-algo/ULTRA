#pragma once

#include "../../../DataStructures/TripBased/Data.h"

namespace TripBased {

class StopArrivalTimes {

public:
    StopArrivalTimes(const Data& data) :
        data(data),
        defaultLabels(data.numberOfStops(), INFTY),
        currentRound(0) {
    }

public:
    inline void clear() noexcept {
        labels.resize(1);
        labels[0] = defaultLabels;
        currentRound = 0;
    }

    inline void startNewRound() noexcept {
        currentRound = labels.size();
        labels.emplace_back(labels.back());
    }

    inline void startNewRound(const size_t round) noexcept {
        while (round >= labels.size()) {
            labels.emplace_back(labels.back());
        }
        currentRound = round;
    }

    inline int operator()(const StopId stop, const size_t round) const noexcept {
        const size_t trueRound = std::min(round, labels.size() - 1);
        return labels[trueRound][stop];
    }

    inline void update(const StopEventId stopEvent) noexcept {
        const StopId stop = data.arrivalEvents[stopEvent].stop;
        const int arrivalTime = data.arrivalEvents[stopEvent].arrivalTime;
        labels[currentRound][stop] = std::min(labels[currentRound][stop], arrivalTime);
    }

    inline void updateCopyForward(const StopEventId stopEvent) noexcept {
        const StopId stop = data.arrivalEvents[stopEvent].stop;
        const int arrivalTime = data.arrivalEvents[stopEvent].arrivalTime;
        for (size_t round = currentRound; round < labels.size(); round++) {
            if (labels[round][stop] <= arrivalTime) break;
            labels[round][stop] = arrivalTime;
        }
    }

private:
    const Data& data;

    std::vector<std::vector<int>> labels;
    std::vector<int> defaultLabels;

    size_t currentRound;
};

}
