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
#include <string>
#include <cassert>

inline void ensure(const bool assumption) noexcept {
    if (!assumption) exit(1);
}

#define AssertMsg(assumption, msg) assert((assumption) || (std::cout << "\n\033[31mASSERTION FAILED: " << msg << "\033[0m\nFile: " << __FILE__ << "\nLine: " << __LINE__ << "\n" << std::flush && false))
#define Ensure(assumption, msg) ensure((assumption) || (std::cout << "\n\033[31mERROR: " << msg << "\033[0m\nFile: " << __FILE__ << "\nLine: " << __LINE__ << "\n" << std::flush && false))
