#pragma once

#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <sys/ioctl.h>

#include "Command.h"

#include "../Helpers/FileSystem/FileSystem.h"
#include "../Helpers/String/String.h"
#include "../Helpers/Vector/Vector.h"

namespace Shell {

inline size_t getScreenWidth() {
    struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
    return w.ws_col;
}

inline size_t getScreenHeight() {
    struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
    return w.ws_row;
}

class LineBuffer {

private:
    struct CursorPosition {
        CursorPosition(const size_t x, const size_t y) : x(x), y(y) {}
        size_t x;
        size_t y;
    };

public:
    LineBuffer(std::ostream& out) :
        out(out),
        firstCharColumn(0),
        cursorCouldBeOffScreen(false) {
    }

    inline bool left() noexcept {
        if (prefix.empty()) {
            return false;
        } else {
            moveCursorLeft();
            suffix.emplace_back(prefix.back());
            prefix.pop_back();
            out << std::flush;
            return true;
        }
    }

    inline bool right() noexcept {
        if (suffix.empty()) {
            return false;
        } else {
            out << suffix.back();
            cursorCouldBeOffScreen = true;
            prefix.emplace_back(suffix.back());
            suffix.pop_back();
            fixCursorPosition();
            out << std::flush;
            return true;
        }
    }

    inline bool begin() noexcept {
        if (prefix.empty()) {
            return false;
        } else {
            while (!prefix.empty()) {
                moveCursorLeft();
                suffix.emplace_back(prefix.back());
                prefix.pop_back();
            }
            out << std::flush;
            return true;
        }
    }

    inline bool end() noexcept {
        if (suffix.empty()) {
            return false;
        } else {
            while (!suffix.empty()) {
                out << suffix.back();
                prefix.emplace_back(suffix.back());
                suffix.pop_back();
            }
            cursorCouldBeOffScreen = true;
            out << std::flush;
            return true;
        }
    }

    inline bool backspace() noexcept {
        if (prefix.empty()) {
            return false;
        } else {
            moveCursorLeft();
            out << getSuffix() << ' ';
            cursorCouldBeOffScreen = true;
            moveCursorLeft(prefix.size() + suffix.size(), suffix.size() + 1);
            prefix.pop_back();
            out << std::flush;
            return true;
        }
    }

    inline bool deleteChar() noexcept {
        if (suffix.empty()) {
            return false;
        } else {
            suffix.pop_back();
            out << getSuffix() << ' ';
            cursorCouldBeOffScreen = true;
            moveCursorLeft(prefix.size() + suffix.size() + 1, suffix.size() + 1);
            out << std::flush;
            return true;
        }
    }

    inline void operator<<(const char c) noexcept {
        prefix.emplace_back(c);
        out << c << getSuffix();
        cursorCouldBeOffScreen = true;
        moveCursorLeft(prefix.size() + suffix.size(), suffix.size());
        out << std::flush;
    }

    inline void operator<<(const std::string& s) noexcept {
        if (s.empty()) return;
        for (const char c : s) {
            prefix.emplace_back(c);
            out << c;
        }
        out << getSuffix();
        cursorCouldBeOffScreen = true;
        moveCursorLeft(prefix.size() + suffix.size(), suffix.size());
        out << std::flush;
    }

    inline void clear() noexcept {
        if (prefix.empty() && suffix.empty()) return;
        moveCursorLeft(prefix.size(), prefix.size());
        out << whiteSpace(prefix.size() + suffix.size());
        cursorCouldBeOffScreen = true;
        moveCursorLeft(prefix.size() + suffix.size(), prefix.size() + suffix.size());
        out << std::flush;
        prefix.clear();
        suffix.clear();
    }

    inline void setFirstCharColumn(const size_t newFirstCharColumn) noexcept {
        AssertMsg(prefix.size() + suffix.size() == 0, "Cannot change first char column if the buffer is not empty!");
        firstCharColumn = newFirstCharColumn;
    }

    inline std::string getPrefix() const noexcept {
        std::stringstream ss;
        for (const char c : prefix) {
            ss << c;
        }
        return ss.str();
    }

    inline void setPrefix(const std::string& newPrefix) noexcept {
        if (!prefix.empty()) {
            moveCursorLeft(prefix.size(), prefix.size());
            out << whiteSpace(prefix.size() + suffix.size());
            cursorCouldBeOffScreen = true;
            moveCursorLeft(prefix.size() + suffix.size(), prefix.size() + suffix.size());
            prefix.clear();
        }
        operator<<(newPrefix);
    }

    inline std::string getSuffix() const noexcept {
        std::stringstream ss;
        for (size_t i = suffix.size() - 1; i < suffix.size(); i--) {
            ss << suffix[i];
        }
        return ss.str();
    }

    inline std::string getString() const noexcept {
        return getPrefix() + getSuffix();
    }

    inline void setString(const std::string& s) noexcept {
        clear();
        operator<<(s);
    }

    inline std::string accept() noexcept {
        if (!suffix.empty()) {
            end();
        }
        std::string result = getString();
        firstCharColumn = 0;
        prefix.clear();
        suffix.clear();
        std::cout << std::endl;
        return result;
    }

    inline void rePrint() noexcept {
        out << getPrefix() << getSuffix();
        cursorCouldBeOffScreen = true;
        moveCursorLeft(prefix.size() + suffix.size(), suffix.size());
        out << std::flush;
    }

private:
    inline void cursorLeft() noexcept {
        out << "\x1b[1D";
    }

    inline void cursorRight() noexcept {
        out << "\x1b[1C";
    }

    inline void cursorUp() noexcept {
        out << "\x1b[1A";
    }

    inline void cursorDown() noexcept {
        out << "\x1b[1B";
    }

    inline void cursorToLineBegin() noexcept {
        out << '\r';
    }

    inline void moveCursorLeft(const size_t textSize, const size_t amount = 1) noexcept {
        if (amount == 0) return;
        AssertMsg(textSize >= amount, "Cannot move left for " << amount << " steps, because the buffer contains only " << textSize << " chars!");
        CursorPosition oldPosition = getCursorPosition(textSize, cursorCouldBeOffScreen);
        CursorPosition newPosition = getCursorPosition(textSize - amount, false);
        while (newPosition.y < oldPosition.y) {
            cursorUp();
            oldPosition.y--;
        }
        while (newPosition.x < oldPosition.x) {
            cursorLeft();
            oldPosition.x--;
        }
        while (newPosition.x > oldPosition.x) {
            cursorRight();
            oldPosition.x++;
        }
        cursorCouldBeOffScreen = false;
    }

    inline void moveCursorLeft() noexcept {
        moveCursorLeft(prefix.size());
    }

    inline std::string whiteSpace(size_t size) const noexcept {
        std::stringstream ss;
        for (size_t i = 0; i < size; i++) {
            ss << " ";
        }
        return ss.str();
    }

    inline CursorPosition getCursorPosition(const size_t textSize, const bool cursorCouldBeOffScreen) const noexcept {
        const size_t screenWidth = getScreenWidth();
        size_t x = (firstCharColumn + textSize) % screenWidth;
        size_t y = (firstCharColumn + textSize) / screenWidth;
        if (cursorCouldBeOffScreen && x == 0 && (firstCharColumn + textSize) > 0) {
            return CursorPosition(screenWidth, y - 1);
        } else {
            return CursorPosition(x, y);
        }
    }

    inline CursorPosition getCursorPosition() const noexcept {
        return getCursorPosition(prefix.size(), cursorCouldBeOffScreen);
    }

    inline void fixCursorPosition() noexcept {
        if (getCursorPosition().x == getScreenWidth() && !suffix.empty()) {
            cursorToLineBegin();
            cursorDown();
            cursorCouldBeOffScreen = false;
        }
    }

private:
    std::ostream& out;

    std::vector<char> prefix;
    std::vector<char> suffix;

    size_t firstCharColumn;

    bool cursorCouldBeOffScreen;

};

}
