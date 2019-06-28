/**********************************************************************************

 Copyright (c) 2019 Tobias Zündorf

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
 modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************************/

#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "CHData.h"

#include "../../../assert.h"
#include "../../../Helpers/Ranges/Range.h"

namespace CH {

template<typename WITNESS_SEARCH, int SHORTCUT_WEIGHT = 1024, int LEVEL_WEIGHT = 1024, int DEGREE_WEIGHT = 0>
class GreedyKey {

public:
    using WitnessSearch = WITNESS_SEARCH;
    constexpr static int shortcutWeight = SHORTCUT_WEIGHT;
    constexpr static int levelWeight = LEVEL_WEIGHT;
    constexpr static int degreeWeight = DEGREE_WEIGHT;
    using KeyType = int;
    using Type = GreedyKey<WitnessSearch, shortcutWeight, levelWeight>;

public:
    GreedyKey() :
        data(nullptr),
        witnessSearch(nullptr) {
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
        AssertMsg(key >= 0, "Key may not be negative!");
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

}
