#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../Helpers/String/String.h"

namespace Shell {

class Command {

public:
    Command() {}

    virtual ~Command() {}

    virtual bool matches(const std::string& s) const {
        return (String::toLower(s) == String::toLower(name()));
    };

    virtual std::string name() const = 0;

    virtual std::string helpText() const = 0;

    virtual void execute(const std::string& parameter) = 0;

    virtual std::vector<std::string> parameterSuggestions() const {
        return std::vector<std::string>();
    }

    virtual std::vector<std::string> parameterSuggestions(const size_t) const {
        return parameterSuggestions();
    }

};

}
