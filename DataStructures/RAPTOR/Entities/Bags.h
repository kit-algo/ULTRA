#pragma once

#include <vector>

#include "../../../Helpers/Assert.h"
#include "../../Container/ExternalKHeap.h"

namespace RAPTOR {

template<typename LABEL>
struct Bag {
    using Label = LABEL;
    using Iterator = typename std::vector<Label>::const_iterator;

    Bag() {}

    Bag(const std::vector<Label>& labelVector) {
        for (const Label& label : labelVector) {
            merge(label);
        }
    }

    template<typename OTHER_LABEL>
    inline bool dominates(const OTHER_LABEL& newLabel) const noexcept {
        for (const Label& label : labels) {
            if (label.dominates(newLabel)) return true;
        }
        return false;
    }

    template<typename OTHER_LABEL>
    inline bool dominatesStrongly(const OTHER_LABEL& newLabel) const noexcept {
        for (const Label& label : labels) {
            if (label.dominatesStrongly(newLabel)) return true;
        }
        return false;
    }

    template<typename OTHER_LABEL>
    inline bool dominates(const Bag<OTHER_LABEL>& other) const noexcept {
        for (const OTHER_LABEL& label : other.labels) {
            if (!dominates(label)) return false;
        }
        return true;
    }

    inline bool merge(const Label& newLabel) noexcept {
        size_t removedLabels = 0;
        for (size_t i = 0; i < labels.size(); i++) {
            if (labels[i].dominates(newLabel)) return false;
            if (newLabel.dominates(labels[i])) {
                removedLabels++;
                continue;
            }
            labels[i - removedLabels] = labels[i];
        }
        labels.resize(labels.size() - removedLabels + 1);
        labels.back() = newLabel;
        return true;
    }

    inline bool mergeWithStrongDominance(const Label& newLabel) noexcept {
        size_t removedLabels = 0;
        for (size_t i = 0; i < labels.size(); i++) {
            if (labels[i].dominatesStrongly(newLabel)) return false;
            if (newLabel.dominates(labels[i])) {
                removedLabels++;
                continue;
            }
            labels[i - removedLabels] = labels[i];
        }
        labels.resize(labels.size() - removedLabels + 1);
        labels.back() = newLabel;
        return true;
    }

    inline void mergeUndominated(const Label& newLabel) noexcept {
        AssertMsg(!dominates(newLabel), "Trying to merge dominated label!");
        size_t removedLabels = 0;
        for (size_t i = 0; i < labels.size(); i++) {
            if (newLabel.dominates(labels[i])) {
                removedLabels++;
                continue;
            }
            labels[i - removedLabels] = labels[i];
        }
        labels.resize(labels.size() - removedLabels + 1);
        labels.back() = newLabel;
    }

    inline bool mergeUndominatedUnlessEqual(const Label& newLabel) noexcept {
        AssertMsg(!dominatesStrongly(newLabel), "Trying to merge dominated label!");
        size_t removedLabels = 0;
        for (size_t i = 0; i < labels.size(); i++) {
            if (labels[i] == newLabel) return false;
            if (newLabel.dominatesStrongly(labels[i])) {
                removedLabels++;
                continue;
            }
            labels[i - removedLabels] = labels[i];
        }
        labels.resize(labels.size() - removedLabels + 1);
        labels.back() = newLabel;
        return true;
    }

    inline size_t size() const noexcept {
        return labels.size();
    }

    inline void resize(const size_t newSize) noexcept {
        labels.resize(newSize);
    }

    inline Label& operator[](const size_t i) noexcept {
        return labels[i];
    }

    inline const Label& operator[](const size_t i) const noexcept {
        return labels[i];
    }

    inline Iterator begin() const noexcept {
        return labels.begin();
    }

    inline Iterator end() const noexcept {
        return labels.end();
    }

    inline void clear() noexcept {
        labels.clear();
    }

    std::vector<Label> labels;
};

template<typename ROUTE_LABEL>
struct RouteBag {
    using RouteLabel = ROUTE_LABEL;

    inline void merge(const RouteLabel& newLabel) noexcept {
        size_t removedLabels = 0;
        for (size_t i = 0; i < labels.size(); i++) {
            if (labels[i].dominates(newLabel)) return;
            if (newLabel.dominates(labels[i])) {
                removedLabels++;
                continue;
            }
            labels[i - removedLabels] = labels[i];
        }
        labels.resize(labels.size() - removedLabels + 1);
        labels.back() = newLabel;
        return;
    }

    inline size_t size() const noexcept {
        return labels.size();
    }

    inline RouteLabel& operator[](const size_t i) noexcept {
        return labels[i];
    }

    inline const RouteLabel& operator[](const size_t i) const noexcept {
        return labels[i];
    }

    std::vector<RouteLabel> labels;
};

template<typename DIJKSTRA_LABEL>
class DijkstraBag : public ExternalKHeapElement {
public:
    using DijkstraLabel = DIJKSTRA_LABEL;
    static constexpr int logK = 2;
    static constexpr int K = 1 << logK;

    DijkstraBag() : heapSize(0) {}

    template<typename OTHER_LABEL>
    inline bool dominates(const OTHER_LABEL& newLabel) const noexcept {
        for (const DijkstraLabel& label : labels) {
            if (label.dominates(newLabel)) return true;
        }
        return false;
    }

    template<typename OTHER_LABEL>
    inline bool dominates(const Bag<OTHER_LABEL>& other) const noexcept {
        for (const OTHER_LABEL& label : other.labels) {
            if (!dominates(label)) return false;
        }
        return true;
    }

    inline size_t size() const noexcept {
        return labels.size();
    }

    inline bool empty() const noexcept {
        return labels.empty();
    }

    inline bool heapEmpty() const noexcept {
        return heapSize == 0;
    }

    inline const DijkstraLabel& front() const noexcept {
        AssertMsg(!empty(), "An empty heap has no front!");
        AssertMsg(heapSize > 0, "An empty heap has no front!");
        return labels[0];
    }

    inline int getKey() const noexcept {
        return front().getKey();
    }

    inline bool hasSmallerKey(const DijkstraBag* const other) const noexcept {
        return getKey() < other->getKey();
    }

    inline const DijkstraLabel& extractFront() noexcept {
        AssertMsg(!empty(), "An empty heap has no front!");
        AssertMsg(heapSize > 0, "An empty heap has no front!");
        heapSize--;
        if (heapSize > 0) {
            std::swap(labels[0], labels[heapSize]);
            siftDown(0);
        }
        return labels[heapSize];
    }

    template<bool ONTO_HEAP>
    inline bool merge(const DijkstraLabel& newLabel) noexcept {
        size_t removedLabels = 0;
        size_t removedHeapLabels = 0;
        for (size_t i = 0; i < labels.size(); i++) {
            if (labels[i].dominates(newLabel)) return false;
            if (newLabel.dominates(labels[i])) {
                removedLabels++;
                if (i < heapSize) removedHeapLabels++;
                continue;
            }
            labels[i - removedLabels] = labels[i];
        }
        heapSize -= removedHeapLabels;
        labels.resize(labels.size() - removedLabels + 1);
        labels.back() = newLabel;
        if constexpr (ONTO_HEAP) {
            std::swap(labels.back(), labels[heapSize]);
            heapSize++;
            heapify();
        } else {
            if (removedHeapLabels > 0) heapify();
        }
        return true;
    }

    inline void initialize(const DijkstraLabel& label) noexcept {
        AssertMsg(labels.empty(), "Trying to initialize non-empty bag!");
        AssertMsg(heapSize == 0, "Trying to initialize non-empty bag!");
        labels.emplace_back(label);
    }

private:
    inline void heapify() noexcept {
        if (heapSize <= 1) return;
        for (size_t i = parent(heapSize - 1); i != size_t(-1); i--) {
            siftDown(i);
        }
    }

    inline void siftDown(size_t i) noexcept {
        AssertMsg(i < heapSize, "siftDown index out of range!");
        while (true) {
            size_t minIndex = i;
            const size_t childrenStart = firstChild(i);
            const size_t childrenEnd = std::min(childrenStart + K, heapSize);
            for (size_t j = childrenStart; j < childrenEnd; j++) {
                if (labels[j].hasSmallerKey(&labels[minIndex])) {
                    minIndex = j;
                }
            }
            if (minIndex == i) break;
            std::swap(labels[i], labels[minIndex]);
            i = minIndex;
        }
    }

    inline size_t parent(const size_t i) const noexcept {
        return (i - 1) >> logK;
    }

    inline size_t firstChild(const size_t i) const noexcept {
        return (i << logK) + 1;
    }

    std::vector<DijkstraLabel> labels;
    size_t heapSize;
};

}
