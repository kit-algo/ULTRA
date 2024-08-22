#pragma once

#include <random>
#include <vector>

#include "Data.h"
#include "../Queries/Queries.h"

namespace TripBased {

struct DelayIncident {
    DelayIncident(const TripId trip = noTripId, const StopIndex startIndex = noStopIndex) :
        trip(trip),
        startIndex(startIndex),
        delay(generateRandomDelay()) {
    }

    inline static int generateRandomDelay() noexcept {
        const int val = rand() % 1000;
        if (val < 500) return 0;
        if (val < 800) return (rand() % 180) + 1;
        if (val < 900) return (rand() % 120) + 181;
        if (val < 960) return (rand() % 300) + 301;
        if (val < 980) return (rand() % 300) + 601;
        if (val < 990) return (rand() % 300) + 901;
        if (val < 996) return (rand() % 600) + 1201;
        return (rand() % 1800) + 1801;
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(trip, startIndex, delay);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(trip, startIndex, delay);
    }

    TripId trip;
    StopIndex startIndex;
    int delay;
};

struct DelayUpdate {
    DelayUpdate() {}

    DelayUpdate(const Data& data, const DelayIncident incident, const int revealTime) :
        firstEvent(data.getStopEventId(incident.trip, incident.startIndex)),
        revealTime(revealTime) {
        const size_t numEvents = data.numberOfStopsInTrip(incident.trip) - incident.startIndex;
        arrivalDelay.resize(numEvents, incident.delay);
        departureDelay.resize(numEvents, incident.delay);
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(firstEvent, revealTime, arrivalDelay, departureDelay);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(firstEvent, revealTime, arrivalDelay, departureDelay);
    }

    StopEventId firstEvent;
    int revealTime;
    std::vector<int> arrivalDelay;
    std::vector<int> departureDelay;
};

class DelayScenario {
public:
    DelayScenario(const Data& data, const int startTime = -INFTY, const int endTime = INFTY, const int seed = 42) :
        data(data),
        startTime(startTime),
        endTime(endTime) {
        srand(seed);
        for (const TripId trip : data.trips()) {
            StopIndex index = StopIndex(rand() % data.numberOfStopsInTrip(trip));
            DelayIncident incident(trip, index);
            if (incident.delay == 0) continue;
            const int revealTime = data.getStopEvent(trip, index).arrivalTime;
            if (revealTime >= endTime) continue;
            if (revealTime < startTime) {
                const StopIndex lastIndex = StopIndex(data.numberOfStopsInTrip(trip) - 1);
                if (data.getStopEvent(trip, lastIndex).arrivalTime < startTime) continue;
                while (data.getStopEvent(trip, incident.startIndex).arrivalTime < startTime) incident.startIndex++;
                preStartIncidents.emplace_back(incident);
                preStartUpdates.emplace_back(data, incident, startTime - 1);
            } else {
                incidents.emplace_back(incident);
                revealTimes.emplace_back(revealTime);
                updates.emplace_back(data, incident, revealTime);
            }
        }
        const Order sortedOrder(Construct::Sort, revealTimes);
        sortedOrder.order(incidents);
        sortedOrder.order(revealTimes);
        sortedOrder.order(updates);
    }

    DelayScenario(const Data& data, const std::string& fileName) :
        data(data) {
        deserialize(fileName);
    }

    inline std::vector<DelayIncident> getAllIncidents() const noexcept {
        return preStartIncidents + incidents;
    }

    inline std::vector<DelayIncident> getDelayIncidentsUntil(const VertexQuery& query) const noexcept {
        std::vector<TripBased::DelayIncident> result = preStartIncidents;
        for (size_t i = 0; i < incidents.size(); i++) {
            if (revealTimes[i] > query.departureTime) break;
            result.emplace_back(incidents[i]);
        }
        return result;
    }

    inline std::vector<std::vector<TripBased::DelayIncident>> groupDelayIncidentsByQuery(const std::vector<VertexQuery>& queries) const noexcept {
        Assert(!queries.empty(), "No queries supplied!");
        std::vector<std::vector<TripBased::DelayIncident>> result(queries.size());
        result[0] = preStartIncidents;
        size_t d = 0;
        for (size_t i = 0; i < queries.size(); i++) {
            while (d < incidents.size() && revealTimes[d] <= queries[i].departureTime) {
                result[i].emplace_back(incidents[d]);
                d++;
            }
            if (d == incidents.size()) break;
        }
        return result;
    }

    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, startTime, endTime, preStartIncidents, incidents, revealTimes, preStartUpdates, updates);
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, startTime, endTime, preStartIncidents, incidents, revealTimes, preStartUpdates, updates);
    }

public:
    const Data& data;
    int startTime;
    int endTime;
    std::vector<DelayIncident> preStartIncidents;
    std::vector<DelayIncident> incidents;
    std::vector<int> revealTimes;
    std::vector<DelayUpdate> preStartUpdates;
    std::vector<DelayUpdate> updates;
};

}
