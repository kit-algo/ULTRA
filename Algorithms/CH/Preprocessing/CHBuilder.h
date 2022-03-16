#pragma once

#include <set>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>

#include "BidirectionalWitnessSearch.h"
#include "CHData.h"
#include "KeyFunction.h"
#include "StopCriterion.h"

#include "../../../DataStructures/Graph/Graph.h"
#include "../../../DataStructures/Container/ExternalKHeap.h"

#include "../../../Helpers/Timer.h"

namespace CH {

template<typename WITNESS_SEARCH = BidirectionalWitnessSearch<CHConstructionGraph>, typename KEY_FUNCTION = GreedyKey<WITNESS_SEARCH>, typename STOP_CRITERION = NoStopCriterion, bool BUILD_Q_LINEAR = false, bool BREAK_KEY_TIES_BY_ID = false, bool SORT_SHORTCUTS = false>
class Builder {

public:
    using WitnessSearch = WITNESS_SEARCH;
    using KeyFunction = KEY_FUNCTION;
    using StopCriterion = STOP_CRITERION;
    constexpr static bool BuildQLinear = BUILD_Q_LINEAR;
    constexpr static bool BreakKeyTiesById = BREAK_KEY_TIES_BY_ID;
    constexpr static bool SortShortcuts = SORT_SHORTCUTS;
    using Type = Builder<WitnessSearch, KeyFunction, StopCriterion, BuildQLinear, BreakKeyTiesById, SortShortcuts>;

private:
    using KeyType = typename KEY_FUNCTION::KeyType;

private:
    struct VertexLabel : public ExternalKHeapElement {
        VertexLabel() : key(0) {}
        inline bool hasSmallerKey(const VertexLabel* const other) const noexcept {
            if (BreakKeyTiesById) {
                return (key < other->key) || ((key == other->key) && (this < other));
            } else {
                return key < other->key;
            }
        }
        KeyType key;
    };

    struct Shortcut {
        Vertex from;
        Vertex to;
        int weight;
        inline bool operator<(const Shortcut& other) const {return weight < other.weight;}
    };

public:
    template<typename GRAPH, typename WEIGHT>
    Builder(GRAPH&& graph, const WEIGHT& weight, const KeyFunction& keyFunction = KeyFunction(), const StopCriterion& stopCriterion = StopCriterion(), const WitnessSearch& witnessSearch = WitnessSearch()) :
        data(std::move(graph), weight),
        keyFunction(keyFunction),
        witnessSearch(witnessSearch),
        stopCriterion(stopCriterion),
        Q(data.numVertices),
        label(data.numVertices) {
    }

    Builder(CHCoreGraph&& graph, const KeyFunction& keyFunction = KeyFunction(), const StopCriterion& stopCriterion = StopCriterion(), const WitnessSearch& witnessSearch = WitnessSearch()) :
        data(std::move(graph)),
        keyFunction(keyFunction),
        witnessSearch(witnessSearch),
        stopCriterion(stopCriterion),
        Q(data.numVertices),
        label(data.numVertices) {
    }

    Builder(Data&& originalData, const KeyFunction& keyFunction = KeyFunction(), const StopCriterion& stopCriterion = StopCriterion(), const WitnessSearch& witnessSearch = WitnessSearch()) :
        data(std::move(originalData)),
        keyFunction(keyFunction),
        witnessSearch(witnessSearch),
        stopCriterion(stopCriterion),
        Q(data.numVertices),
        label(data.numVertices) {
    }

    inline void run() {
        Timer timer;
        initialize<true>();
        buildQ<true>();
        contractQVertices();
        std::cout << "Building CH took " << String::msToString(timer.elapsedMilliseconds()) << std::endl;
    }

    inline void changeKey(const KeyFunction& keyFunction) noexcept {
        this->keyFunction = keyFunction;
        std::vector<Vertex> vertices;
        while (!Q.empty()) {
            VertexLabel* vLabel = Q.extractFront();
            Vertex v = vLabel - &(label[0]);
            vertices.push_back(v);
            label[v].key = getKey(v);
        }
        for (const Vertex v : vertices) {
            Q.Update(&(label[v]));
        }
    }

    inline void reKey(const Vertex vertex) noexcept {
        label[vertex].key = getKey(vertex);
        Q.update(&(label[vertex]));
    }

    inline void copyCoreToCH() noexcept {
        while (!Q.empty()) {
            VertexLabel* vLabel = Q.extractFront();
            Vertex v = Vertex(vLabel - &(label[0]));
            for (Edge edge : data.core.edgesFrom(v)) {
                data.forwardCH.addEdge(v, data.core.get(ToVertex, edge)).set(ViaVertex, data.core.get(ViaVertex, edge)).set(Weight, data.core.get(Weight, edge));
                data.backwardCH.addEdge(data.core.get(ToVertex, edge), v).set(ViaVertex, data.core.get(ViaVertex, edge)).set(Weight, data.core.get(Weight, edge));
            }
        }
    }

    inline const CHCoreGraph& getCore() const noexcept {
        return data.core;
    }

    inline CHCoreGraph& getCore() noexcept {
        return data.core;
    }

    inline const std::vector<Vertex>& getOrder() const noexcept {
        return data.order;
    }

    inline std::vector<Vertex>& getOrder() noexcept {
        return data.order;
    }

    inline const Data& getData() const noexcept {
        return data;
    }

    inline Data& getData() noexcept {
        return data;
    }

private:
    template<bool RESET_DATA>
    inline void initialize() noexcept {
        if constexpr (RESET_DATA) {
            data.order.clear();
            std::vector<VertexLabel>(data.numVertices).swap(label);
            data.forwardCH.reserve(data.numVertices, 1.5 * data.core.numEdges());
            data.backwardCH.reserve(data.numVertices, 1.5 * data.core.numEdges());
        }
        witnessSearch.initialize(&(data.core), &(data.core[Weight]));
        stopCriterion.initialize(&data);
        keyFunction.initialize(&data, &witnessSearch);
    }

    inline KeyType getKey(const Vertex vertex) noexcept {
        return keyFunction(vertex);
    }

    template<bool RESET_DATA>
    inline void buildQ() noexcept {
        Q.clear();
        std::vector<bool> alreadyContracted;
        if (!RESET_DATA) {
            alreadyContracted.resize(data.core.numVertices(), false);
            for (const Vertex vertex : data.order) {
                alreadyContracted[vertex] = true;
            }
        }
        for (Vertex vertex : data.core.vertices()) {
            if constexpr (RESET_DATA) {
                data.level[vertex] = 0;
            } else {
                if (alreadyContracted[vertex]) continue;
            }
            label[vertex].key = getKey(vertex);
            if constexpr (!(BuildQLinear && RESET_DATA)) {
                Q.update(&(label[vertex]));
            }
        }
        if constexpr (BuildQLinear && RESET_DATA) {
            Q.build(label);
        }
    }

    inline void contractQVertices() noexcept {
        while (!Q.empty()) {
            keyFunction.update(*this);
            if (stopCriterion(Q)) {break;}
            VertexLabel* vLabel = Q.extractFront();
            Vertex v = Vertex(vLabel - &(label[0]));
            contract(v);
        }
    }

    inline void contract(const Vertex vertex) noexcept {
        data.order.push_back(vertex);
        std::vector<Shortcut> shortcuts;
        for (Edge first : data.core.edgesTo(vertex)) {
            Vertex from = data.core.get(FromVertex, first);
            for (Edge second : data.core.edgesFrom(vertex)) {
                Vertex to = data.core.get(ToVertex, second);
                if (from == to) continue;
                shortcuts.push_back(Shortcut({from, to, data.core.get(Weight, first) + data.core.get(Weight, second)}));
            }
        }
        if constexpr (SortShortcuts) {
            std::sort(shortcuts.begin(), shortcuts.end());
        }
        for (Shortcut shortcut : shortcuts) {
            if (witnessSearch.shortcutIsNecessary(shortcut.from, shortcut.to, vertex, shortcut.weight)) {
                addShortcut(shortcut.from, shortcut.to, vertex, shortcut.weight);
            }
        }
        std::set<Vertex> neighbors;
        for (Edge edge : data.core.edgesFrom(vertex)) {
            Vertex to = data.core.get(ToVertex, edge);
            data.forwardCH.addEdge(vertex, to).set(ViaVertex, data.core.get(ViaVertex, edge)).set(Weight, data.core.get(Weight, edge));
            neighbors.insert(to);
        }
        for (Edge edge : data.core.edgesTo(vertex)) {
            Vertex from = data.core.get(FromVertex, edge);
            data.backwardCH.addEdge(vertex, from).set(ViaVertex, data.core.get(ViaVertex, edge)).set(Weight, data.core.get(Weight, edge));
            neighbors.insert(from);
        }
        data.core.isolateVertex(vertex);
        const uint16_t level = data.level[vertex] + 1;
        for (Vertex neighbor : neighbors) {
            data.level[neighbor] = std::max(data.level[neighbor], level);
            label[neighbor].key = getKey(neighbor);
            Q.update(&(label[neighbor]));
        }
    }

    inline void addShortcut(const Vertex from, const Vertex to, const Vertex via, const int shortcutWeight) noexcept {
        Edge shortcut = data.core.findEdge(from, to);
        if (data.core.isEdge(shortcut)) {
            if (data.core.get(Weight, shortcut) > shortcutWeight) {
                data.core.set(ViaVertex, shortcut, via);
                data.core.set(Weight, shortcut, shortcutWeight);
            }
        } else {
            data.core.addEdge(from, to).set(ViaVertex, via).set(Weight, shortcutWeight);
        }
    }

private:
    Data data;
    KeyFunction keyFunction;
    WitnessSearch witnessSearch;
    StopCriterion stopCriterion;

    ExternalKHeap<2, VertexLabel> Q;
    std::vector<VertexLabel> label;

};

}
