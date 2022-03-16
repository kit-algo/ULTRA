#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../Assert.h"
#include "../Timer.h"
#include "../String/String.h"

class Progress {

public:
    Progress(const long long n, const bool v = true, std::ostream &o = std::cout) :
        verbose(v),
        os(o),
        numSteps(n),
        stepsDone(0),
        lastDrawnStep(0),
        lastDrawnTime(0.0),
        checkTimeStep(n / 1000),
        redrawTime(500),
        redrawStep(std::max<int>(1, n / 20)) {
        init(n);
    }

    inline void init(long long n) {
        numSteps = n;
        stepsDone = 0;
        lastDrawnStep = 0;
        timer.restart();
        lastDrawnTime = timer.elapsedMilliseconds();
        if (verbose) os << "0% " << std::flush;
    }

    inline void iterate() {
        ++stepsDone;
        checkDraw();
    }

    inline void iterateTo(long long target) {
        Assert(target >= stepsDone);
        stepsDone = target;
        checkDraw();
    }

    inline void operator+=(long long addend) {
        Assert(addend >= 0);
        stepsDone += addend;
        checkDraw();
    }

    inline void operator++() {iterate();}
    inline void operator++(int) {iterate();}

    inline void finished() {
        stepsDone = numSteps;
        draw();
        if (verbose) os << std::endl;
    }

    inline void SetCheckTimeStep(const int s) {checkTimeStep = s;}
    inline void SetRedrawTime(const double t) {redrawTime = t;}
    inline void SetRedrawStep(const int s) {redrawStep = std::max(1, s);}

    inline int getNumberOfStepsDone() const noexcept {
        return stepsDone;
    }


protected:
    inline void checkDraw() {
        if (numSteps <= stepsDone) {
            draw();
        } else if ((lastDrawnStep / redrawStep) < (stepsDone / redrawStep)) {
            draw();
        } else if (stepsDone - lastDrawnStep > checkTimeStep) {
            double time = timer.elapsedMilliseconds();
            if (time - lastDrawnTime > redrawTime) {
                draw();
            }
        }
    }

    inline void draw() {
        if (verbose) os << "\r                                                \r";
        double percent = stepsDone / static_cast<double>(numSteps);
        if (verbose) os << String::percent(percent);
        if (stepsDone < numSteps) {
            const double time = timer.elapsedMilliseconds();
            const double estimate = time * (1 - percent) / percent;
            if (verbose) os << " (" << String::msToString(estimate) << ")" << std::flush;
            lastDrawnStep = stepsDone;
            lastDrawnTime = time;
        } else {
            const double time = timer.elapsedMilliseconds();
            if (verbose) os << " (" << String::msToString(time) << ")" << std::flush;
        }
    }

private:
    bool verbose;
    std::ostream &os;
    Timer timer;
    long long numSteps;
    long long stepsDone;

    int lastDrawnStep;
    double lastDrawnTime;

    int checkTimeStep;
    double redrawTime;
    int redrawStep;

};
