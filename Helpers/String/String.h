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

namespace std {

    std::string to_string(bool b) {
        if (b) {
            return "true";
        } else {
            return "false";
        }
    }

}

namespace String {

    inline std::string toUpper(const std::string& s) {
        std::string out;
        std::transform(s.begin(), s.end(), back_inserter(out), ::toupper);
        return out;
    }

    inline std::string toLower(const std::string& s) {
        std::string out;
        std::transform(s.begin(), s.end(), back_inserter(out), ::tolower);
        return out;
    }

    inline std::string firstToUpper(const std::string& s) {
        if (s.empty()) return s;
        std::string result = s;
        if (s[0] >= 'a' && s[0] <= 'z') {
            result[0] = result[0] - 'a' + 'A';
        }
        return result;
    }

    inline std::string firstToLower(const std::string& s) {
        if (s.empty()) return s;
        std::string result = s;
        if (s[0] >= 'A' && s[0] <= 'Z') {
            result[0] = result[0] - 'A' + 'a';
        }
        return result;
    }

    inline bool contains(const std::string& fullString, const char searchChar) noexcept {
        for (const char c : fullString) {
            if (c == searchChar) return true;
        }
        return false;
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

    template <typename CONTAINER_TYPE>
    inline std::string join(const CONTAINER_TYPE& container, const std::string& delimiter) {
        std::ostringstream s;
        for (typename CONTAINER_TYPE::const_iterator i = container.begin(); i != container.end(); ++i) {
            if (i != container.begin()) {
                s << delimiter;
            }
            s << *i;
        }
        return s.str();
    }

    template<typename TYPE>
    inline bool isNumber(const std::string& s) {
        TYPE n;
        return ((std::istringstream(s) >> n >> std::ws).eof());
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

    inline size_t count(const std::string& s, const char c) {
        size_t result = 0;
        for (const char x : s) {
            if (x == c) result++;
        }
        return result;
    }

    inline std::string whiteSpace(const int size) {
        std::stringstream result;
        for (int i = 0; i < size; i++) {
            result << " ";
        }
        return result.str();
    }

    inline bool isWhiteSpace(const char c) {
        return (c == ' ') || (c == '\t') || (c == '\n');
    }

    inline bool isDigit(const char c) {
        return (c >= '0') & (c <= '9');
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

    inline std::string prettyDouble(const double number, const int precision = 2, const bool includePreDecimals = false) {
        const std::string numberString = std::to_string(number);
        std::vector<char> digits;
        digits.reserve(numberString.size() + 1);
        int decimalPoint = 0;
        for (int i = numberString.size() - 1; i >= 0; i--) {
            if (numberString[i] == '.') {
                decimalPoint = digits.size();
            } else if (numberString[i] != '-') {
                digits.emplace_back(numberString[i]);
            }
        }
        const int numberOfDigits = digits.size();
        const int roundingPoint = (includePreDecimals) ? (numberOfDigits - precision - 1) : (decimalPoint - precision - 1);
        if (roundingPoint >= numberOfDigits) return "0";
        if (roundingPoint >= 0) {
            int lastDigit = roundingPoint + 1;
            if (digits[roundingPoint] >= '5') {
                digits.emplace_back('0');
                while (digits[lastDigit] == '9') {
                    lastDigit++;
                }
                digits[lastDigit]++;
            }
            for (int i = 0; i < lastDigit; i++) {
                digits[i] = '0';
            }
            while ((digits.back() == '0') && (decimalPoint + 1 < static_cast<int>(digits.size()))) {
                digits.pop_back();
            }
        }
        std::stringstream result;
        if (number < 0) result << '-';
        for (int i = digits.size() - 1; i > decimalPoint; i--) {
            result << digits[i];
            if (i % 3 == decimalPoint % 3) result << ',';
        }
        result << digits[decimalPoint];
        if (roundingPoint + 1 < decimalPoint) result << '.';
        for (int i = decimalPoint - 1; (i > roundingPoint) && (i >= 0); i--) {
            result << digits[i];
        }
        for (int i = -1; i > roundingPoint; i--) {
            result << '0';
        }
        return result.str();
    }

    template<typename T>
    inline std::string binary(const T& value) {
        std::stringstream result;
        const char* end = reinterpret_cast<const char*>(&value) - 1;
        const char* begin = end + sizeof(T);
        result << std::bitset<CHAR_BIT>(*begin--);
        while (begin != end) result << " " << std::bitset<CHAR_BIT>(*begin--);
        return result.str();
    }

    inline std::string colorToString(int r, int g, int b) {
        int rLow = r % 16;
        int rHigh = (r / 16) % 16;
        int gLow = g % 16;
        int gHigh = (g / 16) % 16;
        int bLow = b % 16;
        int bHigh = (b / 16) % 16;
        std::stringstream color;
        color << std::hex << rHigh << std::hex << rLow << std::hex << gHigh << std::hex << gLow << std::hex << bHigh << std::hex << bLow;
        return color.str();
    }

    inline bool isColor(const std::string& color) {
        if (color.size() != 6) return false;
        for (int i = 0; i < 6; i++) {
            const char c = color[i];
            if (c >= '0' && c <= '9') continue;
            if (c >= 'a' && c <= 'f') continue;
            if (c >= 'A' && c <= 'F') continue;
            return false;
        }
        return true;
    }

    template<typename T>
    inline std::string secToString(T t, T infinity = std::numeric_limits<T>::max()) {
        if (t >= infinity) return "infinity";
        if ((!(t >= 0)) && (t <= -infinity)) return "-infinity";
        const bool negative = (t < 0);
        t = negative ? -t : t;
        if (t == 0) return "0s";
        std::string result = std::to_string(t % 60) + "s";
        std::string fillIn = ((t % 60) < 10) ? " " : "";
        t = t / 60;
        if (t > 0) {
            result = std::to_string(t % 60) + "m " + fillIn + result;
            fillIn = ((t % 60) < 10) ? " " : "";
            t = t / 60;
            if (t > 0) {
                result = std::to_string(t % 24) + "h " + fillIn + result;
                fillIn = ((t % 60) < 10) ? " " : "";
                t = t / 24;
                if (t > 0) {
                    result = std::to_string(t) + "d " + fillIn + result;
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
        if (t == 0) return displaySeconds ? "0:00:00" : "0:00";
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
        std::string result = std::to_string(t % 1000) + "ms";
        if (t >= 1000) {
            std::string fillIn = "";
            if ((t % 1000) < 10) fillIn = "  ";
            else if ((t % 1000) < 100) fillIn = " ";
            result = secToString(t / 1000) + " " + fillIn + result;
        }
        return result;
    }

    inline std::string msToString(double t) {return msToString<int>(t);}
    inline std::string msToString(double t, int infinity) {return msToString<int>(t, infinity);}

    template<typename T>
    inline std::string musToString(T t, T infinity = std::numeric_limits<T>::max()) {
        if (t >= infinity) return "infinity";
        if ((!(t >= 0)) && (t <= -infinity)) return "-infinity";
        if (t == 0) return "0µs";
        std::string result = std::to_string(t % 1000) + "µs";
        if (t >= 1000) {
            std::string fillIn = "";
            if ((t % 1000) < 10) fillIn = "  ";
            else if ((t % 1000) < 100) fillIn = " ";
            result = msToString(t / 1000) + " " + fillIn + result;
        }
        return result;
    }

    inline std::string musToString(double t) {return musToString<int>(t);}
    inline std::string musToString(double t, int infinity) {return musToString<int>(t, infinity);}

    inline std::string timeString() {
        time_t now = time(0);
        tm *ltm = localtime(&now);
        const int seconds = (((ltm->tm_hour * 60) + ltm->tm_min) * 60) + ltm->tm_sec;
        return secToTime(seconds, true);
    }

    inline std::string dateString() {
        time_t now = time(0);
        tm *ltm = localtime(&now);
        const std::string year = std::to_string(ltm->tm_year + 1900);
        const std::string month = std::to_string(ltm->tm_mon + 1);
        const std::string day = std::to_string(ltm->tm_mday);
        std::stringstream ss;
        if (year.size() == 1) ss << "000";
        if (year.size() == 2) ss << "00";
        if (year.size() == 3) ss << '0';
        ss << year;
        if (month.size() == 1) ss << '0';
        ss << month;
        if (day.size() == 1) ss << '0';
        ss << day;
        return ss.str();
    }

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

    inline int parseSeconds(const std::string& time) {
        int seconds = 0;
        int value = 0;
        for (size_t i = 0; i < time.size(); i++) {
            if (String::isDigit(time[i])) {
                value = (value * 10) + static_cast<int>(time[i] - '0');
            } else if (time[i] == ':') {
                seconds = (seconds * 60) + value;
                value = 0;
            } else {
                error("The string " + time + " is not in the format HH:MM:SS");
                return -1;
            }
        }
        return (seconds * 60) + value;
    }

    inline int parseDay(const std::string& time) {
        if (time.size() != 8) error("The string " + time + " is not in the format YYYYMMDD");
        int year = lexicalCast<int>(time.substr(0, 4)) - 1900;
        int month = lexicalCast<int>(time.substr(4, 2)) - 1;
        int day = lexicalCast<int>(time.substr(6, 2));
        std::tm t = {0,0,12,day,month,year, 0, 0, 0, 0, 0};
        time_t seconds = std::mktime(&t);
        return (seconds < 0) ? (seconds / (60 * 60 * 24)) - 1 : (seconds / (60 * 60 * 24));
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

    inline std::string longestCommonSubstring(const std::string& str1, const std::string& str2) {
        if(str1.empty() || str2.empty()) return "";

        size_t* curr = new size_t[str2.size()];
        size_t* prev = new size_t[str2.size()];
        size_t maxSubstr = 0;
        size_t pos;

        for(size_t i = 0; i < str1.size(); ++i) {
            for(size_t j = 0; j < str2.size(); ++j) {
                if(str1[i] != str2[j]) {
                    curr[j] = 0;
                } else {
                    if(i == 0 || j == 0) {
                        curr[j] = 1;
                    } else {
                        curr[j] = prev[j - 1] + 1;
                    }
                    if (maxSubstr < curr[j]) {
                        maxSubstr = curr[j];
                        pos = j - curr[j] + 1;
                    }
                }
            }
            std::swap(curr, prev);
        }

        delete[] curr;
        delete[] prev;

        std::stringstream ss;
        for (size_t i = 0; i < maxSubstr; i++) {
            ss << str2[pos + i];
        }
        return ss.str();
    }

}
