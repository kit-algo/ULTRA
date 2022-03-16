#pragma once

#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <sys/ioctl.h>

#include "Command.h"

#include "LineBuffer.h"

#include "../Helpers/Timer.h"
#include "../Helpers/HighlightText.h"
#include "../Helpers/FileSystem/FileSystem.h"
#include "../Helpers/String/String.h"
#include "../Helpers/Vector/Vector.h"

namespace Shell {

const std::string newLine = "\n\r";

inline char getch() {
    char buf = 0;
    termios old;
    memset(&old, 0, sizeof(old));
    if (tcgetattr(0, &old) < 0) perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0) perror ("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) perror ("tcsetattr ~ICANON");
    return (buf);
}

class BasicShell {

public:
    BasicShell(std::string programName = "", std::string prompt = "> ", bool autosaveCache = true, std::string cacheFile = "readcache", std::ostream& out = std::cout) :
        programName(programName),
        prompt(prompt),
        out(out),
        shell(*this),
        running(false),
        cachePos(-1),
        cacheFile(cacheFile),
        autosaveCache(autosaveCache),
        lineBuffer(out),
        autoCompleteFiles(true),
        reportCommandTimes(false),
        reportParameters(false) {
        if (programName != "") this->cacheFile = programName + "." + cacheFile;
        dir = FileSystem::getWorkingDirectory();
        if (autosaveCache) loadCache();
    }

    ~BasicShell() {
        if (autosaveCache) saveCache();
        while (!commands.empty()) {
            delete commands.back();
            commands.pop_back();
        }
    }

    inline std::string readLine(const bool isPrompt) {
        printPrompt();
        lineBuffer.setFirstCharColumn(prompt.size());
        while (true) {
            char c = readChar(isPrompt);
            if (c == '\n') return acceptLineBuffer();
            lineBuffer << c;
        }
        return "";
    }

    inline char readChar(const bool isPrompt) {
        while (true) {
            uint64_t c = getch();
            if ((c >= 32 && c <= 126) || (c >= 128 && c <= 254)) { // Displayable character
                return (char)c;
            } else if (c == '\n') { // New line
                return '\n';
            } else if (c == 27) { // Escape sequence
                uint64_t c1 = getch();
                uint64_t c2 = getch();
                c = (c1 * 128) + c2;
                if (c2 >= 50 && c2 <= 55) {
                    uint64_t c3 = getch();
                    c = (c * 128) + c3;
                }
                if (c == 10184) { // Pos1 Key
                    lineBuffer.begin();
                } else if (c == 10182) { // End Key
                    lineBuffer.end();
                } else if (c == 11713) { // Up Key
                    cacheUp();
                } else if (c == 11714) { // Down Key
                    cacheDown();
                } else if (c == 11715) { // Right Key
                    if (!lineBuffer.right()) {
                        autoCompleteCache(lineBuffer.getPrefix());
                    }
                } else if (c == 11716) { // Left Key
                    lineBuffer.left();
                } else if (c == 1497598) { // Delete
                    lineBuffer.deleteChar();
                } else { // Other
                    shell << "\r\x1b[2K";
                    yellow("Unknown key '" + std::to_string((int)(c)) + "'");
                    shell << "\n";
                    printPrompt();
                    lineBuffer.rePrint();
                }
            } else if (c == 9) { // Tab
                autoComplete(isPrompt);
            } else if (c == 127) { // Backspace
                lineBuffer.backspace();
            } else if (c == 24) { // ctrl + x
                saveCache();
            } else { // Other
                shell << "\r\x1b[2K";
                yellow("Unknown key '" + std::to_string((int)(c)) + "'");
                shell << "\n";
                printPrompt();
                lineBuffer.rePrint();
            }
        }
        return '\n';
    }

    inline std::string acceptLineBuffer() {
        std::string s = lineBuffer.accept();
        addToCache(s);
        return s;
    }

    inline void autoComplete(const bool isPrompt) {
        std::vector<std::string> suggestions;
        std::string currentLine = lineBuffer.getPrefix();
        const bool endsWithSpace = currentLine.back() == ' ';
        currentLine = String::trim(currentLine);
        if (endsWithSpace) currentLine += ' ';
        const size_t firstSpacePos = currentLine.find(' ');
        const size_t lastSpacePos = currentLine.rfind(' ');
        if (!isPrompt) {
            if (firstSpacePos < currentLine.size()) {
                const std::string commandName = currentLine.substr(0, firstSpacePos);
                currentLine = currentLine.substr(lastSpacePos + 1);
                collectParameterSuggestions(commandName, currentLine, suggestions, String::count(String::trim(currentLine), ' ') - 1);
            } else if (!isPrompt) {
                collectCommandSuggestions(currentLine, suggestions);
            }
        }
        collectPathSuggestions(currentLine, suggestions);
        collectAdditionalSuggestions(currentLine, suggestions);
        autoComplete(currentLine, suggestions);
    }

    inline void cacheUp() {
        cachePos--;
        if (cachePos < 0) cachePos = cache.size() - 1;
        if (cachePos >= 0) lineBuffer.setString(cache[cachePos]);
    }

    inline void cacheDown() {
        if (cachePos < 0) return;
        cachePos++;
        if (size_t(cachePos) >= cache.size()) {
            cachePos = -1;
            lineBuffer.setString("");
        } else {
            lineBuffer.setString(cache[cachePos]);
        }
    }

    inline void addToCache(const std::string& s) {
        if (s.size() < 1) return;
        std::string line = String::trim(s);
        if (cache.empty() || cache.back() != line) cache.push_back(line);
        cachePos = -1;
    }

    inline Command* findCommand(const std::string& name) {
        for (Command* command : commands) {
            if (command->matches(name)) {
                return command;
            }
        }
        return nullptr;
    }

    inline const Command* findCommand(const std::string& name) const {
        for (const Command* command : commands) {
            if (command->matches(name)) {
                return command;
            }
        }
        return nullptr;
    }

    void run() {
        running = true;
        while (running) {
            interpretCommand(readLine(false));
        }
    }

    inline void interpretCommand(const std::string& line) noexcept {
        std::vector<std::string> tokens = splitCommand(line);
        if (tokens[0].size() < 1) return;
        Command* command = findCommand(tokens[0]);
        if (command != nullptr) {
            if (autosaveCache) saveCache();
            Timer commandTimer;
            command->execute(tokens[1]);
            if (reportCommandTimes) shell << grey("[Finished in ", String::msToString(commandTimer.elapsedMilliseconds()), "]") << newLine;
        } else {
            shell << "Unknown command: \"" << tokens[0] << "\", try using \"help\"" << newLine;
        }
    }

    template<typename T>
    inline T ask(const std::string& question) {
        std::string oldPrompt = prompt;
        prompt = question;
        std::string line = readLine(true);
        prompt = oldPrompt;
        return String::lexicalCast<T>(line);
    }

    inline std::string ask(const std::string& question) {
        return ask<std::string>(question);
    }

    template<typename T>
    inline T ask(const std::string& question, const std::vector<std::string>& suggestions) {
        additionalSuggestions = suggestions;
        std::string oldPrompt = prompt;
        prompt = question;
        std::string line = readLine(true);
        prompt = oldPrompt;
        additionalSuggestions.clear();
        return String::lexicalCast<T>(line);
    }

    inline std::string ask(const std::string& question, const std::vector<std::string>& suggestions) {
        return ask<std::string>(question, suggestions);
    }

    inline void stop() {
        running = false;
    }

    inline void execute(const std::string& line) noexcept {
        printPrompt() << line << newLine;
        addToCache(line);
        interpretCommand(line);
    }

    inline void loadCache() {
        std::ifstream inputFile(getCacheFile());
        if (!inputFile.is_open()) return;
        while (!inputFile.eof()) {
            std::string line;
            getline(inputFile, line);
            addToCache(line);
        }
        inputFile.close();
    }

    inline void saveCache() const {
        std::ofstream outputFile(getCacheFile());
        for (const std::string& s : cache) {
            outputFile << s << std::endl;
        }
        outputFile.close();
    }

    template<typename T>
    inline BasicShell& operator<<(const T& c) {
        out << c << std::flush;
        return *this;
    }

    inline BasicShell& flush() {
        out << std::flush;
        return *this;
    }

    inline BasicShell& endl() {
        out << std::endl;
        return *this;
    }

    template<typename... T>
    inline BasicShell& error(const T&... c)    {return shell << (std::string)::error(c...);}
    template<typename... T>
    inline BasicShell& red(const T&... c)      {return shell << (std::string)::red(c...);}
    template<typename... T>
    inline BasicShell& green(const T&... c)    {return shell << (std::string)::green(c...);}
    template<typename... T>
    inline BasicShell& yellow(const T&... c)   {return shell << (std::string)::yellow(c...);}
    template<typename... T>
    inline BasicShell& blue(const T&... c)     {return shell << (std::string)::blue(c...);}
    template<typename... T>
    inline BasicShell& magenta(const T&... c)  {return shell << (std::string)::magenta(c...);}
    template<typename... T>
    inline BasicShell& cyan(const T&... c)     {return shell << (std::string)::cyan(c...);}
    template<typename... T>
    inline BasicShell& white(const T&... c)    {return shell << (std::string)::white(c...);}


    inline BasicShell& printPrompt() {return blue(prompt);}

public:
    inline bool addCommand(Command* c) {
        if (Vector::contains(commands, c)) {
            return false;
        } else {
            commands.push_back(c);
            return true;
        }
    }

    inline const std::vector<Command*>& getCommands() const {
        return commands;
    }

    inline std::string getProgramName() const {
        return programName;
    }

    inline void setProgramName(const std::string& name) {
        programName = name;
    }

    inline std::string getPrompt() const {
        return prompt;
    }

    inline void setPrompt(const std::string& p) {
        prompt = p;
    }

    inline void clearLine() const {
        shell << "\r" << whiteSpace(getScreenWidth()) << "\r";
    }

    inline std::string getDir() const {
        return dir;
    }

    inline void setDir(const std::string& s) {
        dir = s;
        while(dir.rfind('/') == dir.size() - 1) {
            dir = dir.substr(0, dir.size() - 1);
        }
    }

    inline std::vector<std::string>& getReadCache() {
        return cache;
    }

    inline std::string getCacheFile() const {
        return cacheFile;
    }

    inline void setCacheFile(const std::string& filename) {
        cacheFile = filename;
    }

    inline bool getAutosaveCache() const {
        return autosaveCache;
    }

    inline void setAutosaveCache(const bool b) {
        autosaveCache = b;
    }

    inline bool getAutoCompleteFiles() const {
        return autoCompleteFiles;
    }

    inline void setAutoCompleteFiles(const bool b) {
        autoCompleteFiles = b;
    }

    inline bool getReportCommandTimes() const {
        return reportCommandTimes;
    }

    inline void setReportCommandTimes(const bool b) {
        reportCommandTimes = b;
    }

    inline bool getReportParameters() const {
        return reportParameters;
    }

    inline void setReportParameters(const bool b) {
        reportParameters = b;
    }

private:
    inline std::string whiteSpace(int size) const {
        std::stringstream ss;
        for (int i = 0; i < size; i++) {
            ss << " ";
        }
        return ss.str();
    }

    inline std::vector<std::string> splitCommand(const std::string& s) const {
        int splitPos = s.find(' ');
        std::vector<std::string> result;
        if (splitPos < 0) {
            result.push_back(s);
            result.push_back("");
        } else {
            result.push_back(s.substr(0, splitPos));
            result.push_back(s.substr(splitPos + 1));
        }
        return result;
    }

    inline void collectCommandSuggestions(const std::string& s, std::vector<std::string>& suggestions) {
        for (Command* command : commands) {
            if (command->name().find(s) == 0) {
                suggestions.push_back(command->name());
            }
        }
    }

    inline void collectAdditionalSuggestions(const std::string& s, std::vector<std::string>& suggestions) {
        for (const std::string& suggestion : additionalSuggestions) {
            if (suggestion.find(s) == 0) {
                suggestions.push_back(suggestion);
            }
        }
    }

    inline void collectPathSuggestions(const std::string& s, std::vector<std::string>& suggestions) {
        const int splitPos = s.rfind('/');
        if (splitPos < 0) return;
        const std::string path = FileSystem::extendPath(getDir(), s.substr(0, splitPos));
        const std::string file = s.substr(splitPos + 1);
        for (const std::string& fileName : FileSystem::getFiles(path)) {
            if (fileName.empty()) continue;
            if (fileName.find(file) == 0) {
                std::string suggestion = path + "/" + fileName;
                if (fileName[0] != '.' && FileSystem::isDirectory(suggestion)) suggestion += "/";
                suggestions.push_back(suggestion);
            }
        }
    }

    inline void collectCacheSuggestions(const std::string& s, std::vector<std::string>& suggestions) {
        std::string line = String::trim(s);
        if (s.size() > 0 && String::isWhiteSpace(s.back())) line += ' ';
        for (const std::string& entry : cache) {
            if (entry.find(line) == 0 && !Vector::contains(suggestions, entry)) {
                suggestions.push_back(entry);
            }
        }
    }

    inline void collectParameterSuggestions(const std::string& commandName, const std::string& s, std::vector<std::string>& suggestions) {
        Command* command = findCommand(commandName);
        if (command != nullptr) {
            for (const std::string& entry : command->parameterSuggestions()) {
                if (entry.find(s) == 0) {
                    suggestions.push_back(entry);
                }
            }
        }
    }

    inline void collectParameterSuggestions(const std::string& commandName, const std::string& s, std::vector<std::string>& suggestions, const size_t index) {
        Command* command = findCommand(commandName);
        if (command != nullptr) {
            for (const std::string& entry : command->parameterSuggestions(index)) {
                if (entry.find(s) == 0) {
                    suggestions.push_back(entry);
                }
            }
        }
    }


    inline void autoCompleteCache(const std::string& s) {
        std::vector<std::string> suggestions;
        collectCacheSuggestions(s, suggestions);
        autoComplete(s, suggestions);
    }

    inline void autoComplete(const std::string& s, std::vector<std::string>& suggestions) {
        if (suggestions.size() == 0) return;
        if (suggestions.size() == 1) {
            autoComplete(s, suggestions[0]);
            return;
        }
        std::sort(suggestions.begin(), suggestions.end());
        size_t minSize = suggestions[0].size();
        for (const std::string& suggestion : suggestions) {
            minSize = std::min(minSize, suggestion.size());
        }
        size_t i = s.size();
        bool equal = (i < minSize);
        while (i < minSize && equal) {
            const char c = suggestions[0][i];
            for (size_t j = 1; j < suggestions.size(); j++) {
                if (c != suggestions[j][i]) {
                    equal = false;
                    break;
                }
            }
            if (equal) i++;
        }
        if ((!equal) && (i <= s.size())) {
            shell << lineBuffer.getSuffix() << newLine;
            for (const std::string& n : suggestions) {
                shell << n << newLine;
            }
            printPrompt();
            lineBuffer.rePrint();
        } else {
            autoComplete(s, suggestions[0].substr(0, i));
        }
    }

    inline void autoComplete(const std::string& s, const std::string& suggestion) {
        if (s.empty()) return;
        std::string oldLine = lineBuffer.getPrefix();
        size_t index = oldLine.rfind(s);
        if (index >= oldLine.size()) return;
        std::string newLine = oldLine.substr(0, index) + suggestion;
        if (index + s.size() < oldLine.size()) newLine = newLine + oldLine.substr(index + s.size());
        lineBuffer.setPrefix(newLine);
    }

private:
    std::string programName;
    std::string prompt;
    std::ostream& out;
    BasicShell& shell;
    bool running;

    std::vector<Command*> commands;

    std::vector<std::string> cache;
    int cachePos;
    std::string cacheFile;
    bool autosaveCache;

    LineBuffer lineBuffer;

    std::string dir;
    bool autoCompleteFiles;

    bool reportCommandTimes;
    bool reportParameters;

    std::vector<std::string> additionalSuggestions;

};

}
