#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "CHData.h"
#include "WitnessSearch.h"

#include "../../../Helpers/Assert.h"
#include "../../../Helpers/Ranges/Range.h"
#include "../../../Helpers/Vector/Permutation.h"

namespace CH {

template<typename WITNESS_SEARCH>
class GreedyKey {

public:
    using WitnessSearch = WITNESS_SEARCH;
    using KeyType = int;
    using Type = GreedyKey<WitnessSearch>;

public:
    GreedyKey(const int shortcutWeight = 1024, const int levelWeight = 1024, const int degreeWeight = 0) :
        data(nullptr),
        witnessSearch(nullptr),
        shortcutWeight(shortcutWeight),
        levelWeight(levelWeight),
        degreeWeight(degreeWeight) {
    }

    inline KeyType operator() (const Vertex vertex) noexcept {
        const int inDegree = data->core.inDegree(vertex);
        const int outDegree = data->core.outDegree(vertex);
        if (inDegree <= 2 && outDegree <= 2) {
            const int degree = data->core.degree(vertex);
            if (degree <= 1) return data->level[vertex] - 1000;
            if (degree == 2) return data->level[vertex] - 100000;
        }
        const int shortcutsAdded = simulateContract(vertex);
        const int key = ((shortcutWeight * shortcutsAdded) / (inDegree + outDegree))
                      + ((levelWeight * data->level[vertex]))
                      + ((degreeWeight * inDegree * outDegree));
        Assert(key >= 0);
        return key;
    }

    template<typename T> inline void update(T&) noexcept {}

    inline void initialize(const Data* data, WITNESS_SEARCH* witnessSearch) {
        this->data = data;
        this->witnessSearch = witnessSearch;
    }

private:
    inline int simulateContract(const Vertex vertex) noexcept {
        int shortcutsAdded = 0;
        for (Edge first : data->core.edgesTo(vertex)) {
            Vertex from = data->core.get(FromVertex, first);
            for (Edge second : data->core.edgesFrom(vertex)) {
                Vertex to = data->core.get(ToVertex, second);
                if (from == to) continue;
                if (witnessSearch->shortcutIsNecessary(from, to, vertex, data->core.get(Weight, first) + data->core.get(Weight, second))) {
                    shortcutsAdded++;
                }
            }
        }
        return shortcutsAdded;
    }

private:
    const Data* data;
    WITNESS_SEARCH* witnessSearch;
    const int shortcutWeight;
    const int levelWeight;
    const int degreeWeight;

};

template<typename WITNESS_SEARCH>
class PermutationKey {

public:
    using WitnessSearch = WITNESS_SEARCH;
    using KeyType = int;
    using Type = PermutationKey<WitnessSearch>;

public:
    PermutationKey() {Assert(false);}
    PermutationKey(const Permutation& permutation) :
        permutation(permutation) {
    }

    inline KeyType operator() (const Vertex vertex) noexcept {
        return permutation[vertex];
    }

    template<typename T> inline void update(T&) noexcept {}

    inline void initialize(const Data* data, WitnessSearch*) noexcept {
        AssertMsg(permutation.size() == data->numVertices, "Permutation of size " << permutation.size() << " cannot be used for a graph with " << data->numVertices << " vertices!");
    }

private:
    const Permutation permutation;

};

template<typename WITNESS_SEARCH>
class OrderKey : public PermutationKey<WITNESS_SEARCH> {

public:
    OrderKey() : PermutationKey<WITNESS_SEARCH>() {Assert(false);}
    OrderKey(const Order& order) : PermutationKey<WITNESS_SEARCH>(Permutation(Construct::Invert, order)) {}
    OrderKey(Order&& order) : PermutationKey<WITNESS_SEARCH>(Permutation(Construct::Invert, std::move(order))) {}

};

template<typename WITNESS_SEARCH, typename KEY_FUNCTION = GreedyKey<WITNESS_SEARCH>>
class PartialKey {

public:
    using WitnessSearch = WITNESS_SEARCH;
    using KeyFunction = KEY_FUNCTION;
    using KeyType = typename KeyFunction::KeyType;
    using Type = PartialKey<WitnessSearch, KeyFunction>;

public:
    PartialKey(const std::vector<bool>& contractable, const size_t minOrderIndex, const KeyFunction& keyFunction = KeyFunction()) :
        data(nullptr),
        contractable(contractable),
        keyFunction(keyFunction),
        minOrderIndex(minOrderIndex) {
    }
    PartialKey(const std::vector<bool>& contractable, const KeyFunction& keyFunction = KeyFunction()) :
        PartialKey(contractable, contractable.size() + 1, keyFunction) {
    }

    inline KeyType operator() (const Vertex vertex) noexcept {
        return contractable[vertex] ? keyFunction(vertex) : intMax;
    }

    template<typename T>
    inline void update(T& t) noexcept {
        if (data->order.size() >= minOrderIndex) {
            for (Vertex v = Vertex(contractable.size() - 1); v < contractable.size(); v--) {
                if (!contractable[v]) {
                    contractable[v] = true;
                    t.reKey(v);
                }
            }
            minOrderIndex = contractable.size() + 1;
        }
    }

    inline void initialize(const Data* data, WitnessSearch* witnessSearch) noexcept {
        this->data = data;
        keyFunction.initialize(data, witnessSearch);
    }

private:
    const Data* data;
    std::vector<bool> contractable;
    KeyFunction keyFunction;
    size_t minOrderIndex;
};

template<typename WITNESS_SEARCH, typename KEY_FUNCTION = GreedyKey<WITNESS_SEARCH>>
class StaggeredKey {

public:
    using WitnessSearch = WITNESS_SEARCH;
    using KeyFunction = KEY_FUNCTION;
    using KeyType = typename KeyFunction::KeyType;
    using Type = StaggeredKey<WitnessSearch, KeyFunction>;

public:
    StaggeredKey(const std::vector<size_t>& firstContractableRound, const std::vector<size_t>& coreSizes, const KeyFunction& keyFunction = KeyFunction()) :
        data(nullptr),
        firstContractableRound(firstContractableRound),
        keyFunction(keyFunction),
        coreSizes(coreSizes),
        round(0) {
    }

    inline KeyType operator() (const Vertex vertex) noexcept {
        return (firstContractableRound[vertex] <= round) ? keyFunction(vertex) : intMax;
    }

    template<typename T>
    inline void update(T& t) noexcept {
        if (round < coreSizes.size() && data->coreSize() <= coreSizes[round]) {
            round++;
            for (Vertex v = Vertex(firstContractableRound.size() - 1); v < firstContractableRound.size(); v--) {
                if (firstContractableRound[v] == round) {
                    t.reKey(v);
                }
            }
        }
    }

    inline void initialize(const Data* data, WitnessSearch* witnessSearch) noexcept {
        this->data = data;
        keyFunction.initialize(data, witnessSearch);
    }

private:
    const Data* data;
    std::vector<size_t> firstContractableRound;
    KeyFunction keyFunction;
    std::vector<size_t> coreSizes;
    size_t round;
};

template<typename WITNESS_SEARCH, typename KEY_FUNCTION = GreedyKey<WITNESS_SEARCH>>
class FactorKey {

public:
    using WitnessSearch = WITNESS_SEARCH;
    using KeyFunction = KEY_FUNCTION;
    using KeyType = typename KeyFunction::KeyType;
    using Type = FactorKey<WitnessSearch, KeyFunction>;

public:
    FactorKey(std::vector<float> factor, const KeyFunction& keyFunction = KeyFunction()) :
        data(nullptr),
        factor(factor),
        keyFunction(keyFunction) {
    }

    inline KeyType operator() (const Vertex vertex) noexcept {
        KeyType key = keyFunction(vertex);
        if (key > 0) {
            key *= factor[vertex];
            if (key < 0) key = intMax;
        }
        return key;
    }

    template<typename T> inline void update(T&) noexcept {}

    inline void initialize(const Data* data, WitnessSearch* witnessSearch) noexcept {
        this->data = data;
        keyFunction.initialize(data, witnessSearch);
    }

private:
    const Data* data;
    std::vector<float> factor;
    KeyFunction keyFunction;

};

}
