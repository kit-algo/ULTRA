#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../TransferModes.h"

namespace RAPTOR {

class ArrivalLabel {

public:
    ArrivalLabel(const int arrivalTime = never, const size_t numberOfTrips = -1) :
        arrivalTime(arrivalTime),
        numberOfTrips(numberOfTrips) {
    }

    inline int travelTime(const int departureTime) const noexcept {
        return arrivalTime - departureTime;
    }

    inline bool operator<(const ArrivalLabel& other) const noexcept {
        return (arrivalTime < other.arrivalTime) || ((arrivalTime == other.arrivalTime) && (numberOfTrips < other.numberOfTrips));
    }

    inline bool operator==(const ArrivalLabel& other) const noexcept {
        return (arrivalTime == other.arrivalTime) && (numberOfTrips == other.numberOfTrips);
    }

    inline bool operator!=(const ArrivalLabel& other) const noexcept {
        return !(*this == other);
    }

    inline bool dominates(const ArrivalLabel& other) const noexcept {
        return arrivalTime <= other.arrivalTime && numberOfTrips <= other.numberOfTrips;
    }

    inline friend std::ostream& operator<<(std::ostream& out, const ArrivalLabel& label) noexcept {
        return out << "arrivalTime: " << label.arrivalTime << ", numberOfTrips: " << label.numberOfTrips;
    }

public:
    int arrivalTime;
    size_t numberOfTrips;

};

struct WalkingParetoLabel {
    inline static constexpr int NumberOfCriteria = 3;

    WalkingParetoLabel(const int arrivalTime = never, const int walkingDistance = INFTY, const int numberOfTrips = -1) :
        arrivalTime(arrivalTime),
        walkingDistance(walkingDistance),
        numberOfTrips(numberOfTrips) {
    }

    template<typename LABEL>
    WalkingParetoLabel(const LABEL& label, const size_t numberOfTrips) :
        arrivalTime(label.arrivalTime),
        walkingDistance(label.walkingDistance),
        numberOfTrips(numberOfTrips) {
    }

    inline int travelTime(const int departureTime) const noexcept {
        return arrivalTime - departureTime;
    }

    inline bool operator<(const WalkingParetoLabel& other) const noexcept {
        return std::tie(numberOfTrips, arrivalTime, walkingDistance) < std::tie(other.numberOfTrips, other.arrivalTime, other.walkingDistance);
    }

    inline bool operator==(const WalkingParetoLabel& other) const noexcept {
        return std::tie(numberOfTrips, arrivalTime, walkingDistance) == std::tie(other.numberOfTrips, other.arrivalTime, other.walkingDistance);
    }

    inline bool dominates(const WalkingParetoLabel& other) const noexcept {
        return arrivalTime <= other.arrivalTime && walkingDistance <= other.walkingDistance && numberOfTrips <= other.numberOfTrips;
    }

    inline bool isWithinSlack(const std::vector<ArrivalLabel>& anchorLabels, const int departureTime, const double arrivalSlack, const double tripSlack) const noexcept {
        for (const ArrivalLabel& anchorLabel : anchorLabels) {
            if (isWithinSlack(anchorLabel, departureTime, arrivalSlack, tripSlack)) return true;
            if (anchorLabel.numberOfTrips <= numberOfTrips) break;
        }
        return false;
    }

    inline bool isWithinSlack(const ArrivalLabel& anchorLabel, const int departureTime, const double arrivalSlack, const double tripSlack) const noexcept {
        if (travelTime(departureTime) > anchorLabel.travelTime(departureTime) * arrivalSlack) return false;
        if (numberOfTrips > std::ceil(anchorLabel.numberOfTrips * tripSlack)) return false;
        return true;
    }

    inline friend std::ostream& operator<<(std::ostream& out, const WalkingParetoLabel& label) noexcept {
        return out << "arrivalTime: " << label.arrivalTime << ", walkingDistance: " << label.walkingDistance << ", numberOfTrips: " << label.numberOfTrips;
    }

    inline int getCriterion(const size_t i) const noexcept {
        AssertMsg(i < NumberOfCriteria, "Invalid criterion " << i << "!");
        if (i == 0) {
            return arrivalTime;
        } else if (i == 1) {
            return numberOfTrips;
        } else {
            return walkingDistance;
        }
    }

    int arrivalTime;
    int walkingDistance;
    size_t numberOfTrips;
};

enum TransferTimeBuckets {
    BUCKET_NONE = 0,
    BUCKET_LITTLE = 1,
    BUCKET_MEDIUM = 2,
    BUCKET_MUCH = 3,
    BUCKET_EXTREME = 4,
    NUM_TRANSFER_TIME_BUCKETS = 5,
};

int transferTimeBucketThreshold[NUM_TRANSFER_TIME_BUCKETS - 1] = { 0, 600, 1200, 2400 };

inline int getTransferTimeBucketValue(const int transferTime) noexcept {
    for (size_t i = 0; i < NUM_TRANSFER_TIME_BUCKETS - 1; i++) {
        if (transferTime <= transferTimeBucketThreshold[i]) return i;
    }
    return BUCKET_EXTREME;
}

template<size_t NUM_MODES>
struct MultimodalParetoLabel {
    inline static constexpr size_t NumTransferModes = NUM_MODES;
    inline static constexpr size_t NumberOfCriteria = NumTransferModes + 2;

    MultimodalParetoLabel() : arrivalTime(INFTY), numberOfTrips(-1) {}

    template<typename LABEL>
    MultimodalParetoLabel(const LABEL& label, const size_t numberOfTrips) :
        arrivalTime(label.arrivalTime),
        numberOfTrips(numberOfTrips) {
        std::copy(std::begin(label.transferTime), std::end(label.transferTime), std::begin(transferTime));
    }

    inline int travelTime(const int departureTime) const noexcept {
        return arrivalTime - departureTime;
    }

    inline bool isWithinSlack(const std::vector<ArrivalLabel>& anchorLabels, const int departureTime, const double arrivalSlack, const double tripSlack) const noexcept {
        for (const ArrivalLabel& anchorLabel : anchorLabels) {
            if (isWithinSlack(anchorLabel, departureTime, arrivalSlack, tripSlack)) return true;
            if (anchorLabel.numberOfTrips <= numberOfTrips) break;
        }
        return false;
    }

    inline bool isWithinSlack(const ArrivalLabel& anchorLabel, const int departureTime, const double arrivalSlack, const double tripSlack) const noexcept {
        if (travelTime(departureTime) > anchorLabel.travelTime(departureTime) * arrivalSlack) return false;
        if (numberOfTrips > std::ceil(anchorLabel.numberOfTrips * tripSlack)) return false;
        return true;
    }

    inline MultimodalParetoLabel bucketize(const int factor = 1) const noexcept {
        MultimodalParetoLabel result = *this;
        for (size_t i = 0; i < NumTransferModes; i++) {
            result.transferTime[i] = factor * getTransferTimeBucketValue(transferTime[i]);
        }
        return result;
    }

    inline bool dominates(const MultimodalParetoLabel& other) const noexcept {
        if (arrivalTime > other.arrivalTime) return false;
        if (numberOfTrips > other.numberOfTrips) return false;
        for (size_t i = 0; i < NumTransferModes; i++) {
            if (transferTime[i] > other.transferTime[i]) return false;
        }
        return true;
    }

    inline bool operator<(const MultimodalParetoLabel& other) const noexcept {
           return std::tie(numberOfTrips, arrivalTime, transferTime[0]) < std::tie(other.numberOfTrips, other.arrivalTime, other.transferTime[0]);
    }

    inline bool operator==(const MultimodalParetoLabel& other) const noexcept {
        if (arrivalTime != other.arrivalTime) return false;
        if (numberOfTrips != other.numberOfTrips) return false;
        for (size_t i = 0; i < NumTransferModes; i++) {
            if (transferTime[i] != other.transferTime[i]) return false;
        }
        return true;
    }

    inline friend std::ostream& operator<<(std::ostream& out, const MultimodalParetoLabel& label) noexcept {
        out << "arrivalTime: " << label.arrivalTime;
        for (size_t i = 0; i < NumTransferModes; i++) {
            out << ", " << "mode" << i << "Time: " << label.transferTime[i];
        }
        return out << ", numberOfTrips: " << label.numberOfTrips;
    }

    inline int getCriterion(const size_t i) const noexcept {
        AssertMsg(i < NumberOfCriteria, "Invalid criterion " << i << "!");
        if (i == 0) {
            return arrivalTime;
        } else if (i == 1) {
            return numberOfTrips;
        } else {
            return transferTime[i - 2];
        }
    }

    int arrivalTime;
    int transferTime[NumTransferModes];
    size_t numberOfTrips;
};

}
