#pragma once

#include <chrono>

class Timer {

public:
    using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;

    Timer() : start(timestamp()) {}

    inline void restart() noexcept {
        start = timestamp();
    }

    [[nodiscard]] inline double elapsedMicroseconds() const noexcept {
        return elapsedTime<std::chrono::microseconds>();
    }

    [[nodiscard]] inline double elapsedMilliseconds() const noexcept {
        return elapsedTime<std::chrono::milliseconds>();
    }

    template<typename UNIT>
    [[nodiscard]] inline double elapsedTime() const noexcept {
        return static_cast<double>(std::chrono::duration_cast<UNIT>(timestamp() - start).count());
    }

    inline void advance(const int targetTimeInSeconds) noexcept {
        const TimePoint cur = timestamp();
        const std::chrono::seconds target(targetTimeInSeconds);
        start = cur - target;
    }

private:
    inline static TimePoint timestamp() noexcept {
        return std::chrono::steady_clock::now();
    }

private:
    TimePoint start;

};