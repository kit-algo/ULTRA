#pragma once

#include "../../Helpers/Types.h"
#include "../../Helpers/IO/Serialization.h"

namespace TripBased {

struct Shortcut {
    Shortcut(const StopEventId origin, const StopEventId destination, const int walkingDistance = 0) :
        origin(origin),
        destination(destination),
        walkingDistance(walkingDistance) {
    }

    StopEventId origin;
    StopEventId destination;
    int walkingDistance;
};

struct DelayShortcut {
    DelayShortcut(const StopEventId destination, const int travelTime, const int minOriginDelay, const int maxOriginDelay) :
        destination(destination),
        travelTime(travelTime),
        minOriginDelay(minOriginDelay),
        maxOriginDelay(maxOriginDelay) {
    }

    inline void merge(const DelayShortcut& other) noexcept {
        minOriginDelay = std::min(minOriginDelay, other.minOriginDelay);
        maxOriginDelay = std::max(maxOriginDelay, other.maxOriginDelay);
    }

    inline bool operator<(const DelayShortcut& other) const noexcept {
        return destination > other.destination;
    }

    StopEventId destination;
    int travelTime;
    int minOriginDelay;
    int maxOriginDelay;
};

struct SlackData {
    SlackData(const StopEventId event = noStopEvent, const int slack = INFTY) :
        event(event),
        slack(slack) {
    }

    StopEventId event;
    int slack;

    inline bool isFeasible(const std::vector<int>& arrivalDelay) const noexcept {
        AssertMsg(event < arrivalDelay.size(), "Event is out of bounds!");
        return slack > arrivalDelay[event];
    }

    inline bool isValid() const noexcept {
        return event != noStopEvent;
    }

    inline bool operator<(const SlackData& other) const noexcept {
        return event < other.event || (event == other.event && slack < other.slack);
    }
};

struct UndelayedWitnessData {
    UndelayedWitnessData(const StopEventId event1 = noStopEvent, const StopEventId event2 = noStopEvent, const int slack = INFTY) :
        event1(event1),
        event2(event2),
        slack(slack) {
    }

    StopEventId event1;
    StopEventId event2;
    int slack;
};

struct CandidateData {
    CandidateData() {}

    CandidateData(const UndelayedWitnessData& data, const int event2Slack) :
        event1Data(data.event1, data.slack),
        event2Data(data.event2, event2Slack) {
    }

    SlackData event1Data;
    SlackData event2Data;
};


struct Candidate {
    Candidate() : origin(noStopEvent), destination(noStopEvent) {}

    Candidate(const StopEventId origin, const StopEventId destination, const CandidateData& data) :
        origin(origin),
        destination(destination),
        data(data) {
    }

    StopEventId origin;
    StopEventId destination;
    CandidateData data;

    inline bool operator<(const Candidate& other) const noexcept {
        return origin < other.origin || (origin == other.origin && destination < other.destination);
    }
};

struct ShortcutWitnessData {
    std::vector<SlackData> events;

    inline void add(const CandidateData& candidate) noexcept {
        if (!candidate.event1Data.isValid()) return;
        events.emplace_back(candidate.event1Data);
        if (candidate.event2Data.isValid()) {
            events.back().slack++;
            events.emplace_back(candidate.event2Data);
        }
    }

    inline void removeDuplicates() noexcept {
        std::vector<SlackData> newEvents;
        std::sort(events.begin(), events.end());
        StopEventId lastEvent = noStopEvent;
        for (const SlackData& d : events) {
            if (lastEvent == d.event) continue;
            lastEvent = d.event;
            newEvents.emplace_back(d);
        }
        newEvents.swap(events);
    }
};

struct WitnessData {
    std::vector<StopEventId> events;
    std::vector<int> slacks;
    std::vector<size_t> firstEvent;

    inline void build(const std::vector<Candidate>& candidates, const DelayGraph& stopEventGraph) noexcept {
        std::vector<ShortcutWitnessData> perShortcutData(stopEventGraph.numEdges());
        StopEventId origin = noStopEvent;
        StopEventId destination = noStopEvent;
        Edge edge = noEdge;
        for (const Candidate& candidate : candidates) {
            if (origin != candidate.origin || destination != candidate.destination) {
                if (edge != noEdge) perShortcutData[edge].removeDuplicates();
                origin = candidate.origin;
                destination = candidate.destination;
                edge = stopEventGraph.findEdge(Vertex(origin), Vertex(destination));
                AssertMsg(edge != noEdge, "No edge found for candidate!");
            }
            perShortcutData[edge].add(candidate.data);
        }
        for (const Edge edge : stopEventGraph.edges()) {
            firstEvent.emplace_back(events.size());
            for (const SlackData& d : perShortcutData[edge].events) {
                events.emplace_back(d.event);
                slacks.emplace_back(d.slack);
            }
        }
        firstEvent.emplace_back(events.size());
    }

    inline size_t size() const noexcept {
        return events.size();
    }

    inline bool isShortcutNeeded(const Edge edge, const std::vector<int>& arrivalDelay) const noexcept {
        for (size_t i = firstEvent[edge]; i < firstEvent[edge + 1]; i++) {
            if (slacks[i] <= arrivalDelay[events[i]]) return true;
        }
        return false;
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(events, slacks, firstEvent);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(events, slacks, firstEvent);
    }
};

}
