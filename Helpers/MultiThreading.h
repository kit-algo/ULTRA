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
#include <iomanip>
#include <vector>
#include <thread>
#include <cmath>

#include "Assert.h"
#include "Helpers.h"

#include <sched.h>
#include <numa.h>

#include <omp.h>

#include "Assert.h"

inline void pinThreadToCoreId(const size_t coreId) noexcept {
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(coreId, &mask);
    sched_setaffinity(0, sizeof(mask), &mask);
}

inline size_t numberOfCores() noexcept {
    return std::thread::hardware_concurrency();
}

class ThreadPinning {

public:
    ThreadPinning(const size_t numberOfThreads, const size_t pinMultiplier) :
        numberOfThreads(numberOfThreads),
        pinMultiplier(pinMultiplier) {
    }

    inline void pinThread() const noexcept {
        pinThreadToCoreId((omp_get_thread_num() * pinMultiplier) % numberOfCores());
        AssertMsg(static_cast<size_t>(omp_get_num_threads()) == numberOfThreads, "Number of threads is " << omp_get_num_threads() << ", but should be " << numberOfThreads << "!");
    }

    size_t numberOfThreads;
    size_t pinMultiplier;

};
