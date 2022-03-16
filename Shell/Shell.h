#pragma once

#include <vector>
#include <string>
#include <sstream>
#include <algorithm>

#include "BasicShell.h"
#include "ParameterizedCommand.h"

#include "../Helpers/Assert.h"
#include "../Helpers/FileSystem/FileSystem.h"

namespace Shell {

class Quit : public ParameterizedCommand {

public:
    Quit(BasicShell& shell) :
        ParameterizedCommand(shell, "quit", "Type ['Quit' | 'quit' | 'Q' | 'q' | 'Exit' | 'exit'] to terminate the application.") {
    }

    virtual bool matches(const std::string& s) const noexcept {
        std::string in = String::toLower(s);
        return (in == "q") || (in == "quit") || (in == "exit");
    }

    virtual void execute() noexcept {
        shell << newLine;
        shell.setReportCommandTimes(false);
        shell.stop();
        shell.getReadCache().pop_back();
    }
};

class Help : public ParameterizedCommand {

public:
    Help(BasicShell& shell) :
        ParameterizedCommand(shell, "help", "Type ['Help' | 'help' | 'H' | 'h'] to get an overview of available commands.", "Type ['Help' | 'help' | 'H' | 'h'] <Command> to get detailed help for a command.") {
        addParameter("Command", "");
    }

    virtual bool matches(const std::string& s) const noexcept {
        std::string in = String::toLower(s);
        return (in == "h") || (in == "help");
    }

    virtual void execute() noexcept {
        const std::string command = getParameter("Command");
        if (command == "") {
            const std::vector<Command*> commands = shell.getCommands();
            shell << "Available commands:" << newLine;
            for (const Command* com : commands) {
                shell << "    " << com->name() << newLine;
            }
        } else {
            const Command* com = shell.findCommand(command);
            if (com != nullptr) {
                shell << com->helpText() << newLine;
            } else {
                std::cout << "Unknown command: \"" << command << "\"." << newLine;
            }
        }
    }

    virtual std::vector<std::string> parameterSuggestions() const {
        std::vector<std::string> suggestions;
        for (Command* command : shell.getCommands()) {
            suggestions.push_back(command->name());
        }
        return suggestions;
    }
};

class Dir : public ParameterizedCommand {

public:
    Dir(BasicShell& shell) :
        ParameterizedCommand(shell, "dir", "Displays the current working directory.") {
    }

    virtual void execute() noexcept {
        shell << shell.getDir() << newLine;
    }

};

class Ls : public ParameterizedCommand {

public:
    Ls(BasicShell& shell) :
        ParameterizedCommand(shell, "ls", "Displays all files in the current working directory.") {
    }

    virtual void execute() noexcept {
        std::string path = shell.getDir();
        shell << shell.getDir() << newLine;
        DIR* dir;
        struct dirent* ent;
        if ((dir = opendir(path.c_str())) != nullptr) {
            std::vector<std::string> dirs;
            while ((ent = readdir(dir)) != nullptr) {
                dirs.push_back(ent->d_name);
            }
            closedir(dir);
            std::sort(dirs.begin(), dirs.end());
            for (const std::string& name : dirs) {
                shell << name << newLine;
            }
        } else {
            shell << "Could not open directory: \"" << path << "\"." << newLine;
        }
    }

};

class Cd : public ParameterizedCommand {

public:
    Cd(BasicShell& shell) :
        ParameterizedCommand(shell, "cd", "Changes the current working directory of the shell.") {
        addParameter("Directory");
    }

    virtual void execute() noexcept {
        std::string path = FileSystem::extendPath(shell.getDir(), getParameter("Directory"));
        if (FileSystem::isDirectory(path)) {
            shell.setDir(path);
            shell << shell.getDir() << newLine;
        } else {
            error(path);
        }
    }

private:
    inline void error(const std::string& s) {
        shell << "Unknown path / wrong syntax (" << s << ")." << newLine;
    }

};

class RunScript : public ParameterizedCommand {

public:
    RunScript(BasicShell& shell) :
        ParameterizedCommand(shell, "runScript", "Runs all the commands in the script file.") {
        addParameter("Script file");
    }

    virtual void execute() noexcept {
        const std::string filename = FileSystem::extendPath(shell.getDir(), getParameter("Script file"));
        std::ifstream script(filename);
        AssertMsg(script.is_open(), "cannot open file: " << filename);
        while (!script.eof()) {
            std::string line;
            getline(script, line);
            if (line == "") continue;
            shell.printPrompt();
            shell << line << newLine;
            shell.interpretCommand(line);
        }
    }

};

class ToggleCommandTimeReporting : public ParameterizedCommand {

public:
    ToggleCommandTimeReporting(BasicShell& shell) :
        ParameterizedCommand(shell, "toggleCommandTimeReporting", "Toggles whether the execution time of commands is reported or not.") {
        size_t toggleCount = 0;
        for (const std::string& s : shell.getReadCache()) {
            if (s == name()) {
                toggleCount++;
            }
        }
        if ((toggleCount % 2) != 0) {
            shell.setReportCommandTimes(!shell.getReportCommandTimes());
        }
    }

    virtual void execute() noexcept {
        if (shell.getReportCommandTimes()) {
            shell.setReportCommandTimes(false);
            shell << "Command execution times will no longer be reported!" << newLine;
        } else {
            shell.setReportCommandTimes(true);
            shell << "Command execution times will now be reported!" << newLine;
        }
    }

};

class ToggleParameterReporting : public ParameterizedCommand {

public:
    ToggleParameterReporting(BasicShell& shell) :
        ParameterizedCommand(shell, "toggleParameterReporting", "Toggles whether commands print their parameter values ore not.") {
        size_t toggleCount = 0;
        for (const std::string& s : shell.getReadCache()) {
            if (s == name()) {
                toggleCount++;
            }
        }
        if ((toggleCount % 2) != 0) {
            shell.setReportParameters(!shell.getReportParameters());
        }
    }

    virtual void execute() noexcept {
        if (shell.getReportParameters()) {
            shell.setReportParameters(false);
            shell << "Parameters and their values will no longer be reported!" << newLine;
        } else {
            shell.setReportParameters(true);
            shell << "Parameters and their values will now be reported!" << newLine;
        }
    }

};

class Shell : public BasicShell {

public:
    Shell(std::string programName = "", std::string prompt = "> ") : BasicShell(programName, prompt) {
        new Quit(*this);
        new Help(*this);
        new Dir(*this);
        new Ls(*this);
        new Cd(*this);
        new RunScript(*this);
        new ToggleCommandTimeReporting(*this);
        new ToggleParameterReporting(*this);
    }

};

}
