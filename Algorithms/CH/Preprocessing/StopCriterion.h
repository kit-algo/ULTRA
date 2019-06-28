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
