#pragma once

#include "common/keys.h"
#include "logger/logger.h"

#include <map>
#include <string>
#include <vector>

namespace ANSWER {
class ExeArguments {
public:
    static ExeArguments &GetInstance()
    {
        static ExeArguments instance;
        return instance;
    }
    bool Parse(int argc, char **argv);
    std::string GetArgument(const std::string &key) const;
    const std::vector<std::string> GetUsableArguments() const
    {
        std::vector<std::string> keys;
        for (const auto &pair : arguments_) {
            keys.push_back(pair.first);
        }
        return keys;
    }

    ExeArguments();
    ~ExeArguments();

private:
    std::map<std::string, std::string> arguments_;
};
}  // namespace ANSWER