#pragma once

#include <sys/time.h>

class Timer {

public:
    Timer() : start(timestamp()) {}

    inline void restart() noexcept {
        start = timestamp();
    }

    inline double elapsedMicroseconds() const noexcept {
        double cur = timestamp();
        return cur - start;
    }

    inline double elapsedMilliseconds() const noexcept {
        double cur = timestamp();
        return (cur - start) / 1000.0;
    }

    inline void advance(double targetTime) noexcept {
        double cur = timestamp();
        start = cur - targetTime;
    }

private:
    inline static double timestamp() noexcept {
        timeval tp;
        gettimeofday(&tp, nullptr);
        double mus = static_cast<double>(tp.tv_usec);
        return (tp.tv_sec * 1000000.0) + mus;
    }

private:
    double start;

};

