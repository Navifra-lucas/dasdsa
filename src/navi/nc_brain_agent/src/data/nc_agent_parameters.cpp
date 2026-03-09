
#include "nc_brain_agent/nc_brain_agent.h"

#include <nc_brain_agent/data/nc_agent_parameters.h>

#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <type_traits>

using namespace NaviFra;
using Poco::File;
using Poco::Dynamic::Var;
using Poco::JSON::Array;
using Poco::JSON::Object;
namespace {
static Poco::SingletonHolder<NcAgentParameters> sh;
}

const std::string NcAgentParameters::KEY = "NcAgentParameters";

NcAgentParameters& NcAgentParameters::get()
{
    return *sh.get();
}

NcAgentParameters::NcAgentParameters()
{
    default_dirpath_ = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/install/share/core_launch/launch/";
    dirpath_ = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/";
    default_filepath_ = default_dirpath_ + "param.yaml";
    filepath_ = dirpath_ + "param.yaml";
}

NcAgentParameters::~NcAgentParameters()
{
}

YAML::Node NcAgentParameters::getParameters()
{
    NLOG(info) << "getParameters : " << default_filepath_;

    if (!File(default_filepath_).exists()) {
        throw Poco::Exception("File does not exist.");
    }

    std::ifstream fin(default_filepath_);
    YAML::Node yamlNode = YAML::Load(fin);

    return yamlNode;
}

YAML::Node NcAgentParameters::getParametersRobot()
{
    NLOG(info) << "getParametersRobot : " << filepath_;
    YAML::Node yamlNode;
    if (!File(filepath_).exists()) {
        NLOG(info) << "navifra_solution/navicore/configs/param.yaml not exists";
        std::ofstream outfile(filepath_);
    }
    else {
        std::ifstream fin(filepath_);
        yamlNode = YAML::Load(fin);
    }
    NLOG(info) << "getParametersRobot done";

    return yamlNode;
}

void NcAgentParameters::setParameters(std::string key, float value)
{
    std::ifstream fin(filepath_);
    YAML::Node yamlNode = YAML::Load(fin);

    if (yamlNode[key]) {
        yamlNode[key] = value;

        std::ofstream fout(filepath_);
        fout << yamlNode;
        fout.close();
    }
    else {
        LOG_ERROR("Parameter key not found");
    }
}
bool NcAgentParameters::isExits()
{
    if (!File(default_filepath_).exists()) {
        NLOG(info) << "getParameters : " << default_filepath_;
        return false;
    }
    return true;
}

std::string NcAgentParameters::typeName(std::string node)
{
    std::regex integerPattern("^-?\\d+$");  // 정수
    std::regex floatPattern("-?\\d+(\\.\\d+)?");  // 실수
    std::regex pattern("(true|false)$");  // bool

    if (std::regex_match(node, integerPattern)) {
        return "int";
    }
    else if (std::regex_match(node, floatPattern)) {
        return "float";
    }
    else if (std::regex_match(node, pattern)) {
        return "bool";
    }

    return "string";
}

Poco::JSON::Object NcAgentParameters::parserMaptoJSON(YAML::Node node)
{
    Poco::JSON::Object nodes;
    for (const auto& element : node) {
        if (node[element.first].IsScalar()) {
            if (typeName(element.second.as<std::string>()) == "bool") {
                nodes.set(element.first.as<std::string>(), element.second.as<bool>());
            }
            else if (typeName(element.second.as<std::string>()) == "float" || typeName(element.second.as<std::string>()) == "int") {
                nodes.set(element.first.as<std::string>(), element.second.as<float>());
            }
            else {
                nodes.set(element.first.as<std::string>(), element.second.as<std::string>());
            }
        }
        else if (node[element.first].IsSequence()) {
            nodes.set(element.first.as<std::string>(), parserSequencetoJSON(element.second));
        }
        else if (node[element.first].IsMap()) {
            nodes.set(element.first.as<std::string>(), parserMaptoJSON(element.second));
        }
    }
    return nodes;
}

Poco::JSON::Array NcAgentParameters::parserSequencetoJSON(YAML::Node node)
{
    Poco::JSON::Array nodes;
    for (const YAML::Node& element : node) {
        if (element.IsScalar()) {
            if (typeName(element.as<std::string>()) == "bool") {
                nodes.add(element.as<bool>());
            }
            else if (typeName(element.as<std::string>()) == "float" || typeName(element.as<std::string>()) == "int") {
                nodes.add(element.as<float>());
            }
            else {
                nodes.add(element.as<std::string>());
            }
        }
        else if (element.IsSequence()) {
            nodes.add(parserSequencetoJSON(element));
        }
        else if (element.IsMap()) {
            nodes.add(parserMaptoJSON(element));
        }
    }

    return nodes;
}

YAML::Node NcAgentParameters::parserMaptoYAML(Poco::JSON::Object obj)
{
    YAML::Node nodes;
    for (const auto& element : obj) {
        YAML::Node node;

        const std::string& key = element.first;
        const Poco::Dynamic::Var& value = element.second;

        if (obj.isArray(key)) {
            node = parserSequencetoYAML(*value.extract<Poco::JSON::Array::Ptr>());
        }
        else if (obj.isObject(key)) {
            node = parserMaptoYAML(*value.extract<Poco::JSON::Object::Ptr>());
        }
        else {
            if (value.isInteger()) {
                node = value.convert<int64_t>();
            }
            else if (value.isNumeric()) {
                node = value.convert<float>();
            }
            else if (value.isBoolean()) {
                node = value.convert<bool>();
            }
            else {
                node = value.convert<std::string>();
            }
        }

        nodes[element.first] = node;
    }
    return nodes;
}

YAML::Node NcAgentParameters::parserSequencetoYAML(Poco::JSON::Array obj)
{
    YAML::Node nodes;

    for (const auto& element : obj) {
        YAML::Node node;
        if (element.isArray()) {
            node = parserSequencetoYAML(element.extract<Poco::JSON::Array>());
        }
        else if (element.isStruct()) {
            node = parserMaptoYAML(element.extract<Poco::JSON::Object>());
        }
        else {
            // scalar
            if (element.isInteger()) {
                node = element.convert<int64_t>();
            }
            else if (element.isNumeric()) {
                node = element.convert<float>();
            }
            else if (element.isBoolean()) {
                node = element.convert<bool>();
            }
            else {
                node = element.convert<std::string>();
            }
            nodes.push_back(node);
        }
    }
    return nodes;
}

Poco::JSON::Object NcAgentParameters::toObject()
{
    std::string key;
    try {
        YAML::Node yamlnodes = getParameters();
        YAML::Node robot_yamlnodes = getParametersRobot();

        if (yamlnodes.size() == 0) {
            Object nodes;
            Array list;
            nodes.set("list", list);
            return nodes;
        }

        Object nodes;
        Array list;
        // install/param.yaml
        for (YAML::const_iterator it = yamlnodes.begin(); it != yamlnodes.end(); ++it) {
            key = it->first.as<std::string>();
            if (it->second.IsMap()) {
                for (YAML::const_iterator subIt = it->second.begin(); subIt != it->second.end(); ++subIt) {
                    std::string subKey = subIt->first.as<std::string>();
                    Object node;
                    node.set("category", key);
                    node.set("key", key + "/" + subKey);
                    node.set("title", subKey);
                    if (subIt->second.IsScalar()) {
                        std::string value = subIt->second.as<std::string>();
                        if (typeName(value) == "bool") {
                            node.set("type", typeName(value));
                            node.set("value", subIt->second.as<bool>());
                            node.set("default", subIt->second.as<bool>());
                        }
                        else if (typeName(value) == "float" || typeName(value) == "int") {
                            node.set("type", "number");
                            node.set("value", subIt->second.as<float>());
                            node.set("default", subIt->second.as<float>());
                        }
                        else {
                            node.set("type", "string");
                            node.set("value", value);
                            node.set("default", value);
                        }
                    }
                    else if (subIt->second.IsSequence()) {
                        Array values;
                        node.set("type", "list");
                        node.set("value", parserSequencetoJSON(yamlnodes[key][subKey]));
                        node.set("default", parserSequencetoJSON(yamlnodes[key][subKey]));
                    }
                    list.add(node);
                }
            }
        }
        if (robot_yamlnodes.size() > 0) {
            // navifra_solution/navicore/configs/param.yaml
            for (YAML::const_iterator it = robot_yamlnodes.begin(); it != robot_yamlnodes.end(); ++it) {
                key = it->first.as<std::string>();
                if (it->second.IsMap()) {
                    for (YAML::const_iterator subIt = it->second.begin(); subIt != it->second.end(); ++subIt) {
                        std::string subKey = subIt->first.as<std::string>();

                        Object node;
                        NLOG(info) << "start " << key + "/" + subKey;
                        int n_idx;
                        for (std::size_t i = 0; i < list.size(); ++i) {
                            Var var = list.get(i);
                            Object tmp = var.extract<Object>();
                            if (tmp.has("key")) {
                                // NLOG(info) << "iter " << tmp.getValue<std::string>("key");
                                if (tmp.getValue<std::string>("key") == key + "/" + subKey) {
                                    // NLOG(info) << "find " << key + "/" + subKey;
                                    node = tmp;
                                    n_idx = i;
                                    break;
                                }
                            }
                        }

                        if (node.has("key")) {
                            if (subIt->second.IsScalar()) {
                                std::string value = subIt->second.as<std::string>();
                                if (typeName(value) == "bool") {
                                    node.set("value", subIt->second.as<bool>());
                                }
                                else if (typeName(value) == "float" || typeName(value) == "int") {
                                    node.set("value", subIt->second.as<float>());
                                }
                                else {
                                    node.set("value", value);
                                }
                            }
                            else if (subIt->second.IsSequence()) {
                                Array values;
                                node.set("value", parserSequencetoJSON(subIt->second));
                            }
                            list.set(n_idx, node);
                        }
                        else {
                            NLOG(error) << "not find key " << key + "/" + subKey;
                        }
                    }
                }
            }
        }

        nodes.set("list", list);
        return nodes;
    }
    catch (Poco::Exception ex) {
        LOG_ERROR("%s", ex.displayText().c_str());
        return Object();
    }
}

void NcAgentParameters::updateParameters(Poco::JSON::Object::Ptr data)
{
    try {
        YAML::Node yamlnodes = getParameters();
        YAML::Node robot_yamlnodes = getParametersRobot();
        NLOG(info) << "updateParameters";
        if (data->has("list")) {
            Array::Ptr nodes = data->getArray("list");
            for (size_t index = 0; index < nodes->size(); index++) {
                Object::Ptr node = nodes->getObject(index);
                std::string key = node->get("key").convert<std::string>();
                Poco::StringTokenizer tokenizer(key, "/");

                if (tokenizer.count() == 2) {
                    std::string s_key1 = tokenizer[0];
                    std::string s_key2 = tokenizer[1];
                    if (yamlnodes[s_key1][s_key2].IsDefined()) {
                        if (yamlnodes[s_key1][s_key2].IsScalar()) {
                            if (node->get("type").convert<std::string>() == "bool") {
                                robot_yamlnodes[s_key1][s_key2] = node->get("value").convert<bool>();
                            }
                            else if (node->get("type").convert<std::string>() == "number") {
                                if (node->get("value").isNumeric() || node->get("value").isInteger()) {
                                    robot_yamlnodes[s_key1][s_key2] = node->get("value").convert<float>();
                                }
                            }
                            else if (node->get("type").convert<std::string>() == "string") {
                                robot_yamlnodes[s_key1][s_key2] = node->get("value").convert<std::string>();
                            }
                        }
                        else if (yamlnodes[s_key1][s_key2].IsSequence()) {
                            robot_yamlnodes[s_key1][s_key2] = parserSequencetoYAML(*node->getArray("value"));
                        }
                        else if (yamlnodes[s_key1][s_key2].IsMap()) {
                            robot_yamlnodes[s_key1][s_key2] = parserMaptoYAML(*node->getObject("value"));
                        }
                    }
                    else {
                        NLOG(error) << "IsDefined " << key;
                    }
                }
                else {
                    NLOG(error) << "key is not 2 depth " << key;
                }
            }
            // 새로운 Node 생성
            YAML::Node newNode;
            // 외부 map을 순회
            for (auto outerIt = robot_yamlnodes.begin(); outerIt != robot_yamlnodes.end(); ++outerIt) {
                std::string outerKey = outerIt->first.as<std::string>();
                YAML::Node innerNode = outerIt->second;

                // 내부 Node가 map인 경우에만 내부 키들을 필터링
                if (innerNode.IsMap()) {
                    // 내부 map을 순회하면서 삭제 대상 키가 아닌 것만 복사
                    for (auto innerIt = innerNode.begin(); innerIt != innerNode.end(); ++innerIt) {
                        std::string innerKey = innerIt->first.as<std::string>();
                        NLOG(info) << outerKey << "/" << innerKey << " " << yamlnodes[outerKey][innerKey] << " "
                                   << robot_yamlnodes[outerKey][innerKey] << " ? "
                                   << (yamlnodes[outerKey][innerKey] != robot_yamlnodes[outerKey][innerKey]);
                        if (innerIt->second.IsScalar()) {
                            std::string value = innerIt->second.as<std::string>();
                            if (typeName(value) == "bool") {
                                if (yamlnodes[outerKey][innerKey].as<bool>() != robot_yamlnodes[outerKey][innerKey].as<bool>()) {
                                    newNode[outerKey][innerKey] = robot_yamlnodes[outerKey][innerKey];
                                }
                            }
                            else if (typeName(value) == "float" || typeName(value) == "int") {
                                if (yamlnodes[outerKey][innerKey].as<float>() != robot_yamlnodes[outerKey][innerKey].as<float>()) {
                                    newNode[outerKey][innerKey] = robot_yamlnodes[outerKey][innerKey];
                                }
                            }
                            else {
                                if (yamlnodes[outerKey][innerKey].as<std::string>() !=
                                    robot_yamlnodes[outerKey][innerKey].as<std::string>()) {
                                    newNode[outerKey][innerKey] = robot_yamlnodes[outerKey][innerKey];
                                }
                            }
                        }
                        if (robot_yamlnodes[outerKey][innerKey].IsSequence()) {
                            // if (parserSequencetoJSON(yamlnodes[outerKey][innerKey]) !=
                            //     parserSequencetoJSON(robot_yamlnodes[outerKey][innerKey]))
                            {
                                newNode[outerKey][innerKey] = robot_yamlnodes[outerKey][innerKey];
                            }
                        }
                    }
                }
            }

            std::ofstream fout(filepath_);
            fout << YAML::Dump(newNode);
            fout.close();
        }
    }
    catch (Poco::Exception ex) {
        LOG_ERROR("%s", ex.displayText().c_str());
    }
}