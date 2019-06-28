/**********************************************************************************

 Copyright (c) 2019 Jonas Sauer, Tobias Zündorf

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

#include <ctime>
#include <cmath>
#include <vector>
#include <string>
#include <bitset>
#include <sstream>
#include <cstdlib>
#include <iostream>
#include <limits.h>
#include <algorithm>

#include "../Assert.h"
#include "../Ranges/ReverseRange.h"

namespace String {

    inline std::string toLower(const std::string& s) {
        std::string out;
        std::transform(s.begin(), s.end(), back_inserter(out), ::tolower);
        return out;
    }

    inline std::string firstToLower(const std::string& s) {
        if (s.empty()) return s;
        std::string result = s;
        if (s[0] >= 'A' && s[0] <= 'Z') {
            result[0] = result[0] - 'A' + 'a';
        }
        return result;
    }

    inline bool containsSubString(const std::string& fullString, const size_t index, const std::string& subString) noexcept {
        if (index + subString.size() > fullString.size()) return false;
        for (size_t i = 0; i < subString.size(); i++) {
            if (fullString[index + i] != subString[i]) return false;
        }
        return true;
    }

    inline bool containsSubString(const std::string& fullString, const std::string& subString) noexcept {
        if (subString.size() > fullString.size()) return false;
        for (size_t i = 0; i <= fullString.size() - subString.size(); i++) {
            if (containsSubString(fullString, i, subString)) return true;
        }
        return false;
    }

    inline size_t firstIndexOf(const std::string& fullString, const std::string& subString) noexcept {
        if (subString.size() > fullString.size()) return fullString.size();
        for (size_t i = 0; i <= fullString.size() - subString.size(); i++) {
            if (containsSubString(fullString, i, subString)) return i;
        }
        return fullString.size();
    }

    inline size_t lastIndexOf(const std::string& fullString, const std::string& subString) noexcept {
        if (subString.size() > fullString.size()) return fullString.size();
        for (size_t i = fullString.size() - subString.size(); i < fullString.size(); i--) {
            if (containsSubString(fullString, i, subString)) return i;
        }
        return fullString.size();
    }

    inline bool endsWith(const std::string& fullString, const std::string& endString) {
        if (fullString.size() < endString.size()) return false;
        return fullString.compare(fullString.size() - endString.size(), endString.size(), endString) == 0;
    }

    inline bool beginsWith(const std::string& fullString, const std::string& endString) {
        if (fullString.size() < endString.size()) return false;
        return fullString.compare(0, endString.size(), endString) == 0;
    }

    template<typename U, typename T>
    inline U lexicalCast(const T& in) {
        std::stringstream ss;
        ss << in;
        U out;
        ss >> out;
        return out;
    }
    template<>
    inline bool lexicalCast(const std::string& in) {
        return (in == "1") || (toLower(in) == "true");
    }
    template<>
    inline int lexicalCast(const std::string& in) {return atoi(in.c_str());}
    template<>
    inline double lexicalCast(const std::string& in) {return atof(in.c_str());}
    template<>
    inline std::string lexicalCast(const std::string& in) {return in;}

    inline std::vector<std::string>& split(const std::string& s, const char delim, std::vector<std::string>& elems) {
        std::stringstream ss(s);
        std::string item;
        while(std::getline(ss, item, delim)) {
            elems.push_back(item);
        }
        return elems;
    }
    inline std::vector<std::string> split(const std::string& s, const char delim) {
        std::vector<std::string> elems;
        return split(s, delim, elems);
    }

    inline bool isWhiteSpace(const char c) {
        return (c == ' ') || (c == '\t') || (c == '\n');
    }

    inline std::string percent(double p) {
        if (p != p) return "100.00%";
        int a = p * 100;
        int b = ((int)(p * 10000)) % 100;
        if (b < 10) {
            return std::to_string(a) + ".0" + std::to_string(b) + "%";
        } else {
            return std::to_string(a) + "." + std::to_string(b) + "%";
        }
    }

    template<typename T>
    inline std::string prettyInt(T i) {
        if (i >= std::numeric_limits<T>::max()) return "infinity";
        std::vector<std::string> tokens;
        std::stringstream ss;
        if (i < 0) {
            ss << "-";
            i = -i;
        }
        if (i >= std::numeric_limits<T>::max()) return "-infinity";
        while (i >= 1000) {
            const short block = i % 1000;
            i = i / 1000;
            tokens.push_back(std::to_string(block));
            if (block < 10) {
                tokens.push_back("00");
            } else if (block < 100) {
                tokens.push_back("0");
            }
            tokens.push_back(",");
        }
        tokens.push_back(std::to_string(static_cast<short>(i)));
        for (const std::string& s : descending(tokens)) ss << s;
        return ss.str();
    }

    inline std::string prettyInt(double i) {return prettyInt<int>(i);}

    inline std::string prettyDouble(const double d, const unsigned precision = 2, const bool includePreDecimals = false) {
        if (d > std::numeric_limits<long long>::max()) return std::to_string(d);
        double decimal = std::abs(d);
        const std::string preDecimals = ((d < 0) ? "-" : "") + prettyInt((long long)decimal);
        std::stringstream result;
        result << preDecimals << ".";
        decimal -= std::floor(decimal);
        const int maxI = includePreDecimals ? precision - preDecimals.size() : precision;
        for (int i = 0; i < maxI; i++) {
            decimal *= 10;
            int digit = decimal;
            decimal -= digit;
            result << std::to_string(digit);
        }
        return result.str();
    }

    template<typename T>
    inline std::string secToString(T t, T infinity = std::numeric_limits<T>::max()) {
        if (t >= infinity) return "infinity";
        if ((!(t >= 0)) && (t <= -infinity)) return "-infinity";
        const bool negative = (t < 0);
        t = negative ? -t : t;
        if (t == 0) return "0s";
        std::string result = std::to_string(t % 60) + "s";
        t = t / 60;
        if (t > 0) {
            result = std::to_string(t % 60) + "m " + result;
            t = t / 60;
            if (t > 0) {
                result = std::to_string(t % 24) + "h " + result;
                t = t / 24;
                if (t > 0) {
                    result = std::to_string(t) + "d " + result;
                }
            }
        }
        if (negative) result = "- " + result;
        return result;
    }

    inline std::string secToString(double t) {return secToString<int>(t);}
    inline std::string secToString(double t, int infinity) {return secToString<int>(t, infinity);}

    template<typename T>
    inline std::string secToTime(T t, T infinity = std::numeric_limits<T>::max(), bool displaySeconds = false) {
        if (t >= infinity) return "infinity";
        if ((!(t >= 0)) && (t <= -infinity)) return "-infinity";
        const bool negative = (t < 0);
        t = negative ? -t : t;
        if (t == 0) return "0:00";
        T s = t % 60;
        T m = (t / 60) % 60;
        T h = (t / 3600) % 24;
        T d = t / 86400;
        std::string result;
        if (d != 0) result += std::to_string(d) + "d ";
        result += std::to_string(h) + ":";
        if (m < 10) result += "0";
        result += std::to_string(m);
        if (s != 0 || displaySeconds) {
            result += ":";
            if (s < 10) result += "0";
            result += std::to_string(s);
        }
        if (negative) result = "- " + result;
        return result;
    }

    template<typename T>
    inline std::string secToTime(T t, bool displaySeconds) {return secToTime(t, std::numeric_limits<T>::max(), displaySeconds);}
    inline std::string secToTime(double t, int infinity, bool displaySeconds = false) {return secToTime<int>(t, infinity, displaySeconds);}
    inline std::string secToTime(double t, bool displaySeconds = false) {return secToTime<int>(t, std::numeric_limits<int>::max(), displaySeconds);}

    template<typename T>
    inline std::string msToString(T t, T infinity = std::numeric_limits<T>::max()) {
        if (t >= infinity) return "infinity";
        if ((!(t >= 0)) && (t <= -infinity)) return "-infinity";
        if (t == 0) return "0ms";
        if (t >= 1000) {
            return secToString(t / 1000) + " " + std::to_string(t % 1000) + "ms";
        } else {
            return std::to_string(t % 1000) + "ms";
        }
    }

    inline std::string msToString(double t) {return msToString<int>(t);}
    inline std::string msToString(double t, int infinity) {return msToString<int>(t, infinity);}

    template<typename T>
    inline std::string musToString(T t, T infinity = std::numeric_limits<T>::max()) {
        if (t >= infinity) return "infinity";
        if ((!(t >= 0)) && (t <= -infinity)) return "-infinity";
        if (t == 0) return "0µs";
        if (t >= 1000) {
            return msToString(t / 1000) + " "  + std::to_string(t % 1000) + "µs";
        } else {
            return std::to_string(t % 1000) + "µs";
        }
    }

    inline std::string musToString(double t) {return musToString<int>(t);}
    inline std::string musToString(double t, int infinity) {return musToString<int>(t, infinity);}

    inline std::string bytesToString(const long long bytes, const double k = 1000.0) {
        if (bytes <= 1) return prettyInt(bytes) + "Byte";
        if (bytes < k) return prettyInt(bytes) + "Bytes";
        double b = bytes / k;
        if (b < k) return prettyDouble(b) + "KB";
        b = b / k;
        if (b < k) return prettyDouble(b) + "MB";
        b = b / k;
        if (b < k) return prettyDouble(b) + "GB";
        b = b / k;
        return prettyDouble(b) + "TB";
    }

    std::string trim(const std::string& s) {
        std::stringstream ss;
        int end = s.size() - 1;
        while (end >= 0 && isWhiteSpace(s[end])) end--;
        bool omitWhiteSpace = true;
        for (int i = 0; i <= end; i++) {
            char c = s[i];
            if (isWhiteSpace(c)) {
                if (!omitWhiteSpace) {
                    ss << " ";
                    omitWhiteSpace = true;
                }
            } else {
                ss << c;
                omitWhiteSpace = false;
            }
        }
        return ss.str();
    }

    std::string replaceAll(const std::string& s, const char c, const std::string& replacement) {
        std::stringstream ss;
        for (size_t i = 0; i < s.size(); i++) {
            char cs = s[i];
            if (cs == c) {
                ss << replacement;
            } else {
                ss << cs;
            }
        }
        return ss.str();
    }

    std::string replaceAll(const std::string& s, const std::string& pattern, const std::string& replacement) {
        if (pattern.size() > s.size()) return s;
        std::stringstream ss;
        size_t i = 0;
        while (i <= s.size() - pattern.size()) {
            bool matches = true;
            for (size_t j = 0; j < pattern.size(); j++) {
                if (s[i + j] != pattern[j]) {
                    matches = false;
                    break;
                }
            }
            if (matches) {
                ss << replacement;
                i += pattern.size();
            } else {
                ss << s[i];
                i++;
            }
        }
        while (i < s.size()) {
            ss << s[i];
            i++;
        }
        return ss.str();
    }

}
