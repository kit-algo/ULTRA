#pragma once

#include <iostream>
#include <vector>
#include <string>

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

    inline friend std::ostream& operator<<(std::ostream& out, const ArrivalLabel& label) noexcept {
        return out << "arrivalTime: " << label.arrivalTime << ", numberOfTrips: " << label.numberOfTrips;
    }

public:
    int arrivalTime;
    size_t numberOfTrips;

};

struct WalkingParetoLabel {
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

    int arrivalTime;
    int walkingDistance;
    size_t numberOfTrips;
};

}
