#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "CHData.h"

#include "../../../assert.h"

namespace CH {

class NoStopCriterion {

public:
    NoStopCriterion() {}
    inline void initialize(const Data*) noexcept {}
    template<typename QUEUE>
    inline bool operator() (QUEUE&) noexcept {return false;}

};

class CoreCriterion {

public:
    CoreCriterion(const size_t minCoreSize = 0, const double maxCoreDegree = 0) :
        minCoreSize(minCoreSize),
        maxCoreDegree(maxCoreDegree) {
    }
    inline void initialize(const Data* data) noexcept {this->data = data;}
    template<typename QUEUE>
    inline bool operator() (QUEUE&) noexcept {
        return (data->coreSize() <= minCoreSize) || (data->core.numEdges() / double(data->coreSize()) >= maxCoreDegree);
    }

private:
    const Data* data;
    size_t minCoreSize;
    double maxCoreDegree;

};

}
