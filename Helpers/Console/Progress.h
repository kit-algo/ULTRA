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

    void init(long long n) {
        numSteps = n;
        stepsDone = 0;
        lastDrawnStep = 0;
        timer.restart();
        lastDrawnTime = timer.elapsedMilliseconds();
        if (verbose) os << "0% " << std::flush;
    }

    void iterate() {
        ++stepsDone;
        checkDraw();
    }

    void iterateTo(long long target) {
        AssertMsg(target >= stepsDone, "Target is too small!");
        stepsDone = target;
        checkDraw();
    }

    void operator+=(long long addend) {
        AssertMsg(addend >= 0, "Addend may not be negative!");
        stepsDone += addend;
        checkDraw();
    }

    void operator++() {iterate();}
    void operator++(int) {iterate();}

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
