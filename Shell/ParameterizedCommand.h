#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include "BasicShell.h"
#include "Command.h"

#include "../Helpers/String/String.h"
#include "../Helpers/String/Enumeration.h"

namespace Shell {

class ParameterizedCommand : public Command {

public:
    const static inline std::string DescriptionLineBreak = "\n    ";

public:
    struct Parameter {
        Parameter(const std::string& name, const std::vector<std::string>& options = std::vector<std::string>()) :
            name(name),
            value(""),
            defaultValue(""),
            hasDefault(false),
            options(options) {
        }
        Parameter(const std::string& name, const std::string& defaultValue, const std::vector<std::string>& options = std::vector<std::string>()) :
            name(name),
            value(defaultValue),
            defaultValue(defaultValue),
            hasDefault(true),
            options(options) {
        }
        inline std::string optionsString() const noexcept {
            Enumeration enumeration;
            for (const std::string& string : options) {
                enumeration << string << sep;
            }
            return enumeration.str();
        }
        std::string name;
        std::string value;
        std::string defaultValue;
        bool hasDefault;
        std::vector<std::string> options;
    };

public:
    ParameterizedCommand(BasicShell& shell, const std::string& name = "", const std::string& description = "") :
        shell(shell) ,
        commandName(name),
        description(description) {
        shell.addCommand(this);
    }
    template<typename... T>
    ParameterizedCommand(BasicShell& shell, const std::string& name, const std::string& a, const std::string& b, const T&... c) :
        ParameterizedCommand(shell, name, a + DescriptionLineBreak + b, c...) {
    }

    virtual ~ParameterizedCommand() {}

    virtual std::string name() const noexcept {
        return commandName;
    }

    virtual std::string helpText() const noexcept {
        std::stringstream result;
        result << name();
        for (const Parameter& parameter : parameters) {
            result << " <" << parameter.name;
            if (parameter.hasDefault) {
                result << " = " << parameter.defaultValue;
            }
            result << ">";
        }
        result << DescriptionLineBreak << description;
        return result.str();
    }

    virtual void execute(const std::string& parameter) noexcept {
        std::vector<std::string> tokens = String::split(String::trim(parameter), ' ');
        bool addedParameters = false;
        for (size_t i = 0; i < parameters.size(); i++) {
            if (i < tokens.size()) {
                parameters[i].value = tokens[i];
            } else if (!parameters[i].hasDefault) {
                if (parameters[i].options.empty()) {
                    parameters[i].value = shell.ask(parameters[i].name + "> ", parameterSuggestions(i));
                } else {
                    parameters[i].value = shell.ask(parameters[i].name + "> ", parameters[i].options);
                }
                if (parameters[i].value == "") {
                    shell.error("Parameter <", parameters[i].name, "> was not specified!");
                    return;
                }
                addedParameters = true;
            }
            if ((!parameters[i].options.empty()) && (!Vector::contains(parameters[i].options, parameters[i].value))) {
                shell.error("Parameter <", parameters[i].name, "> must have one of the following values: ", parameters[i].optionsString());
                return;
            }
        }
        if (addedParameters) {
            std::stringstream command;
            command << name();
            for (const Parameter& parameter : parameters) {
                command << " " << parameter.value;
            }
            shell.addToCache(command.str());
            shell.saveCache();
        }
        if (shell.getReportParameters()) {
            for (const Parameter& parameter : parameters) {
                shell << grey(parameter.name, " = ", parameter.value) << newLine;
            }
        }
        execute();
    }

    virtual void execute() = 0;

    virtual std::vector<std::string> parameterSuggestions(const size_t index) const {
        if ((index < parameters.size()) && (!parameters[index].options.empty())) {
            return parameters[index].options;
        } else {
            return static_cast<const Command*>(this)->parameterSuggestions();
        }
    }

protected:
    inline void setName(const std::string& name) noexcept {
        commandName = name;
    }

    inline void setDescription(const std::string& newDescription) noexcept {
        description = newDescription;
    }

    inline void addParameter(const std::string& name, const std::vector<std::string>& options = std::vector<std::string>()) noexcept {
        parameters.emplace_back(name, options);
        addParameterToDescription(parameters.back());
    }

    inline void addParameter(const std::string& name, const std::initializer_list<std::string>& options) noexcept {
        addParameter(name, std::vector<std::string>(options));
    }

    inline void addParameter(const std::string& name, const std::string& defaultValue, const std::vector<std::string>& options = std::vector<std::string>()) noexcept {
        parameters.emplace_back(name, defaultValue, options);
        addParameterToDescription(parameters.back());
    }

    template<typename T>
    inline T getParameter(const std::string& name) const noexcept {
        for (const Parameter& parameter : parameters) {
            if (parameter.name == name) {
                return String::lexicalCast<T>(parameter.value);
            }
        }
        Ensure(false, "Parameter " << name << " is unknown!");
        return String::lexicalCast<T>("");
    }

    template<typename T>
    inline std::vector<T> getParameters(const std::string& name, const char delim = ',') const noexcept {
        for (const Parameter& parameter : parameters) {
            if (parameter.name == name) {
                return Vector::map(String::split(parameter.value, delim), [](const std::string& s){return String::lexicalCast<T>(s);});
            }
        }
        Ensure(false, "Parameter " << name << " is unknown!");
        return std::vector<T>();
    }

    inline std::string getParameter(const std::string& name) const noexcept {
        return getParameter<std::string>(name);
    }

    inline void addParameterToDescription(const Parameter& p) {
        if (!p.options.empty()) {
            description = description + DescriptionLineBreak + "    <" + p.name + ">: " + p.optionsString();
        }
    }

protected:
    BasicShell& shell;

private:
    std::string commandName;
    std::string description;

    std::vector<Parameter> parameters;

};

}
