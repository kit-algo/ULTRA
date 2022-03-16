#pragma once

#include <iostream>
#include <ostream>

#include "../Assert.h"

class ProgressBar {

public:
    ProgressBar(const long long n, const bool v = true, std::ostream &o = std::cout) : verbose(v), os(o), percentOutputStep(20), dotOutputStep(5) {
        init(n);
    }

    void init(long long n) {
        numSteps         = n;
        stepsDone        = 0;
        lastDrawnPercent = 0;
        if (verbose) os << "0% " << std::flush;
    }

    void iterate() {
        ++stepsDone;
        unsigned int until = (stepsDone*100) / numSteps;
        draw(until);
    }

    void iterateTo(long long target) {
        Assert(target >= stepsDone);
        stepsDone = target;
        unsigned int until = (stepsDone*100) / numSteps;
        draw(until);
    }

    void operator+=(long long addend) {
        Assert(addend >= 0);
        stepsDone += addend;
        unsigned int until = (stepsDone*100) / numSteps;
        draw(until);
    }

    void operator++() {iterate();}
    void operator++(int) {iterate();}
    inline void SetDotOutputStep(const int d) { dotOutputStep = d; }
    inline void SetPercentOutputStep(const int p) { percentOutputStep = p; }

protected:
    inline void draw(unsigned int until) {
        if (verbose) {
            for (unsigned short i = (lastDrawnPercent + 1); i <= until; ++i) {
                if (i % percentOutputStep == 0) {
                    if (i > 0) os << " " << i << "% " << std::flush;
                } else {
                    if (i % dotOutputStep == 0) os << "." << std::flush;
                }
            }
        }
        lastDrawnPercent = until;
    }

private:
    bool verbose;
    std::ostream &os;
    long long numSteps;
    long long stepsDone;
    unsigned short lastDrawnPercent;
    int percentOutputStep;
    int dotOutputStep;

};
