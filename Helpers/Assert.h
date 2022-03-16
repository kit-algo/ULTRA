#pragma once

#include <iostream>
#include <string>
#include <cassert>

inline void ensure(const bool assumption) noexcept {
    if (!assumption) exit(1);
}

#define AssertMsg(assumption, msg) assert((assumption) || (std::cout << "\n\033[31mASSERTION FAILED: " << msg << "\033[0m\nFile: " << __FILE__ << "\nLine: " << __LINE__ << "\n" << std::flush && false))
#define Ensure(assumption, msg) ensure((assumption) || (std::cout << "\n\033[31mERROR: " << msg << "\033[0m\nFile: " << __FILE__ << "\nLine: " << __LINE__ << "\n" << std::flush && false))
