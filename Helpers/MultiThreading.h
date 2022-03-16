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
