#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <thread>
#include <cmath>

#include "Assert.h"

#include <sched.h>
#include <numa.h>

#include <omp.h>

#include "Assert.h"

class ThreadScheduler {

public:
    ThreadScheduler(const std::string strategy, const size_t nt = omp_get_num_procs()) :
        numLogicalCpus(omp_get_max_threads()),
        numThreads(nt),
        numNumaNodes(1),
        threadToLogicalCpu(numThreads, 0),
        threadToNumaNode(numThreads, 0),
        threadToNumaMaster(numThreads, false),
        numaNodeToLogicalCpus(numNumaNodes) {
        Assert(1 != -1);
        initialize(strategy);
    }

    inline size_t numThreadsUsed() const {return numThreads;}
    inline size_t getNumNumaNodesUsed() const {return numNumaNodesUsed;}

    inline size_t getNumaNodeFromThreadId(const size_t threadId) const {
        Assert(threadId < threadToNumaNode.size());
        return threadToNumaNode[threadId];
    }

    inline size_t getLogicalCpuFromThreadId(const size_t threadId) const {
        Assert(threadId < threadToLogicalCpu.size());
        return threadToLogicalCpu[threadId];
    }

    inline bool isNumaNodeMaster(const size_t threadId) const {
        Assert(threadId < threadToNumaMaster.size());
        return threadToNumaMaster[threadId];
    }

    inline void pinThread(const size_t threadId) {
        Assert(threadId < threadToLogicalCpu.size());
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(threadToLogicalCpu[threadId], &mask);
        sched_setaffinity(0, sizeof(mask), &mask);
    }

protected:
    void initialize(const std::string strategy, bool verbose = true) {
        if (verbose) std::cout << "NUMA nodes in the system: " << numNumaNodes << std::endl;
        const int width = ceil(log10(double(numLogicalCpus)));
        for (size_t n = 0; n < numNumaNodes; ++n) {
            std::cout << "Node " << n << ": " << std::flush;
            bitmask *bmp = numa_allocate_cpumask();
            numa_node_to_cpus(n, bmp);
            for (size_t l = 0; l < numLogicalCpus; ++l) {
                if (numa_bitmask_isbitset(bmp, l)) {
                    numaNodeToLogicalCpus[n].push_back(l);
                    std::cout << std::setw(width) << std::right << l << " " << std::flush;
                }
            }
            numa_free_cpumask(bmp);
            std::cout << std::endl;
        }
        if (verbose) std::cout << "Distributing threads according to strategy: " << strategy << std::endl;
        if (strategy == "R") distributeRoundRobin();
        if (strategy == "F") distributeFill();
        numNumaNodesUsed = 0;
        for (size_t threadId = 0; threadId < numThreads; ++threadId) {
            threadToNumaNode[threadId] = numa_node_of_cpu(threadToLogicalCpu[threadId]);
            numNumaNodesUsed = std::max(numNumaNodesUsed, threadToNumaNode[threadId]);
        }
        numNumaNodesUsed++;

        std::vector<bool> numaAssigned(numNumaNodes, false);
        for (size_t threadId = 0; threadId < numThreads; ++threadId) {
            size_t currentNumaNode = threadToNumaNode[threadId];
            if (numaAssigned[currentNumaNode]) continue;
            numaAssigned[currentNumaNode] = true;
            threadToNumaMaster[threadId] = true;
        }

        if (verbose) {
            std::cout << "Logical CPUs (NUMA nodes) being used: " << std::flush;
            for (size_t t = 0; t < numThreads; ++t) {
                std::cout << threadToLogicalCpu[t] << "(" << threadToNumaNode[t] << ")  " << std::flush;
            }
            std::cout << std::endl;
            std::cout << "# NUMA nodes used in this setting: " << numNumaNodesUsed << std::endl;
        }
        if (verbose) std::cout << std::endl;
    }

    inline void distributeRoundRobin() {
        std::vector<size_t> currentLogicalCpuIndex(numNumaNodes, 0);
        size_t currentNumaNode(0);
        for (size_t t = 0; t < numThreads; ++t) {
            // Skip numa nodes that are 'full'
            while(currentLogicalCpuIndex[currentNumaNode] >= numaNodeToLogicalCpus[currentNumaNode].size())
                currentNumaNode = (currentNumaNode + 1) % numNumaNodes;

            threadToLogicalCpu[t] = numaNodeToLogicalCpus[currentNumaNode][currentLogicalCpuIndex[currentNumaNode]++];

            // Go to next numa node
            currentNumaNode = (currentNumaNode + 1) % numNumaNodes; // Next node
        }
    }

    inline void distributeFill() {
        for (size_t t = 0; t < numThreads; ++t) {
            threadToLogicalCpu[t] = t;
        }
    }

private:
    size_t numLogicalCpus;
    size_t numThreads;
    size_t numNumaNodes;
    std::vector<size_t> threadToLogicalCpu;
    std::vector<size_t> threadToNumaNode;
    std::vector<bool> threadToNumaMaster;
    std::vector<std::vector<size_t>> numaNodeToLogicalCpus;
    size_t numNumaNodesUsed;

};

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
