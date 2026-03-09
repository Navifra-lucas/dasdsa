#pragma once

#include "logger/logger.h"

#include <Poco/Dynamic/Var.h>
#include <Poco/JSON/Array.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>

#include <fstream>
#include <map>
#include <memory>
#include <type_traits>

namespace ANSWER {

class Configurator {
public:
    static Configurator &GetInstance()
    {
        static Configurator instance;
        return instance;
    }

    bool LoadParameters(const std::string &filename);
    const Poco::JSON::Object::Ptr GetAllParams(const std::string &filename);

    const Poco::Dynamic::Var GetParamValue(
        const std::string &filename, const std::string &key,
        const std::string &subkey);

    const Poco::Dynamic::Var GetParamValue(
        const std::string &filename, const std::string &key);

    const Poco::JSON::Object::Ptr GetParamValuePtr(
        const std::string &filename, const std::string &key,
        const std::string &subkey);
    const Poco::JSON::Object::Ptr GetParamValuePtr(
        const std::string &filename, const std::string &key);
    bool HasParam(
        const std::string &filename, const std::string &key,
        const std::string &subkey);
    bool HasParam(const std::string &filename, const std::string &key);
    template <typename T>
    T GetParam(
        const std::string &filename, const std::string &key,
        const std::string &subkey, const T &default_value)
    {
        if (HasParam(filename, key, subkey)) {
            return GetParamValue(filename, key, subkey).convert<T>();
        }
        return default_value;
    }
    template <typename T>
    T GetParam(
        const std::string &filename, const std::string &key,
        const T &default_value)
    {
        if (HasParam(filename, key)) {
            return GetParamValue(filename, key).convert<T>();
        }
        return default_value;
    }

    std::map<std::string, Poco::JSON::Object::Ptr> config_json_map_;
    std::mutex config_mutex_;

    Configurator();  // Private constructor
    ~Configurator();  // Private constructor
    Configurator(const Configurator &) = delete;
    Configurator &operator=(const Configurator &) = delete;

private:
    // Add member variables to store parameters
};
}  // namespace ANSWER