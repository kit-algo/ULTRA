#pragma once

#include <vector>

#include "Shortcut.h"
#include "../../Helpers/Types.h"

namespace TripBased {

class ShortcutCollection {

public:
    ShortcutCollection(const size_t numberOfOrigins) :
        shortcuts(numberOfOrigins) {
    }

    inline void add(const StopEventId origin, const DelayShortcut& shortcut) noexcept {
        unmergedOrigins.emplace_back(origin);
        unmergedShortcuts.emplace_back(shortcut);
    }

    inline void mergeUnmerged() noexcept {
        if (unmergedShortcuts.empty()) return;
        StopEventId lastOrigin = unmergedOrigins[0];
        size_t originBegin = 0;
        for (size_t i = 1; i < unmergedShortcuts.size(); i++) {
            if (unmergedOrigins[i] == lastOrigin) continue;
            merge(unmergedShortcuts, originBegin, i, lastOrigin);
            originBegin = i;
            lastOrigin = unmergedOrigins[i];
        }
        merge(unmergedShortcuts, originBegin, unmergedShortcuts.size(), lastOrigin);
        unmergedOrigins.clear();
        unmergedShortcuts.clear();
    }

    inline void merge(const ShortcutCollection& other) noexcept {
        AssertMsg(shortcuts.size() == other.shortcuts.size(), "Number of origins is different!");
        for (StopEventId origin(0); origin < shortcuts.size(); origin++) {
            merge(other.shortcuts[origin], origin);
        }
    }

    inline DynamicDelayGraph getGraph() const noexcept {
        DynamicDelayGraph result;
        result.addVertices(shortcuts.size());
        for (Vertex fromVertex(0); fromVertex < shortcuts.size(); fromVertex++) {
            for (const DelayShortcut& shortcut : shortcuts[fromVertex]) {
                const Edge edge = result.addEdge(fromVertex, Vertex(shortcut.destination));
                result.set(TravelTime, edge, shortcut.travelTime);
                result.set(MinOriginDelay, edge, shortcut.minOriginDelay);
                result.set(MaxOriginDelay, edge, shortcut.maxOriginDelay);
            }
        }
        result.sortEdges(ToVertex);
        return result;
    }

    inline long long memoryUsageInBytes() const noexcept {
        long long result = 0;
        result += Vector::memoryUsageInBytes(unmergedOrigins);
        result += Vector::memoryUsageInBytes(unmergedShortcuts);
        result += Vector::memoryUsageInBytes(shortcuts);
        result += Vector::memoryUsageInBytes(tempShortcuts);
        return result;
    }

    inline size_t numberOfShortcuts() const noexcept {
        size_t result = 0;
        for (const std::vector<DelayShortcut>& s : shortcuts) {
            result += s.size();
        }
        return result;
    }

    inline void clear() noexcept {
        std::vector<StopEventId>().swap(unmergedOrigins);
        std::vector<DelayShortcut>().swap(unmergedShortcuts);
        std::vector<std::vector<DelayShortcut>>(shortcuts.size()).swap(shortcuts);
        std::vector<DelayShortcut>().swap(tempShortcuts);
    }

private:
    inline void merge(const std::vector<DelayShortcut>& container, const StopEventId origin) noexcept {
        merge(container, 0, container.size(), origin);
    }

    inline void merge(const std::vector<DelayShortcut>& container, const size_t begin, const size_t end, const StopEventId origin) noexcept {
        size_t i = begin;
        size_t j = 0;
        while (i < end && j < shortcuts[origin].size()) {
            if (container[i].destination == shortcuts[origin][j].destination) {
                shortcuts[origin][j].merge(container[i]);
                i++;
            } else if (container[i].destination > shortcuts[origin][j].destination) {
                tempShortcuts.emplace_back(container[i]);
                i++;
            } else {
                tempShortcuts.emplace_back(shortcuts[origin][j]);
                j++;
            }
        }
        tempShortcuts.insert(tempShortcuts.end(), container.begin() + i, container.begin() + end);
        tempShortcuts.insert(tempShortcuts.end(), shortcuts[origin].begin() + j, shortcuts[origin].end());
        shortcuts[origin].swap(tempShortcuts);
        tempShortcuts.clear();
    }

    std::vector<StopEventId> unmergedOrigins;
    std::vector<DelayShortcut> unmergedShortcuts;
    std::vector<std::vector<DelayShortcut>> shortcuts;
    std::vector<DelayShortcut> tempShortcuts;
};

}
