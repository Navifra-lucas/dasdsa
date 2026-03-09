#include "nc_dcs/sequence_loader.h"

#include <ros/ros.h>

#include <fstream>

namespace nc_dcs {

// ActionParams helper methods implementation
std::string ActionParams::getString(const std::string& key, const std::string& default_val) const
{
    auto it = string_params.find(key);
    return (it != string_params.end()) ? it->second : default_val;
}

double ActionParams::getDouble(const std::string& key, double default_val) const
{
    auto it = double_params.find(key);
    return (it != double_params.end()) ? it->second : default_val;
}

bool ActionParams::getBool(const std::string& key, bool default_val) const
{
    auto it = bool_params.find(key);
    return (it != bool_params.end()) ? it->second : default_val;
}

void ActionParams::setString(const std::string& key, const std::string& value)
{
    string_params[key] = value;
}

void ActionParams::setDouble(const std::string& key, double value)
{
    double_params[key] = value;
}

void ActionParams::setBool(const std::string& key, bool value)
{
    bool_params[key] = value;
}

bool ActionParams::has(const std::string& key) const
{
    // boolean params
    if (bool_params.find(key) != bool_params.end()) {
        return true;
    }
    // double params
    if (double_params.find(key) != double_params.end()) {
        return true;
    }
    // string params
    if (string_params.find(key) != string_params.end()) {
        return true;
    }

    // 없으면 false
    return false;
}

// SequenceLoader implementation
SequenceLoader::SequenceLoader()
{
}

bool SequenceLoader::loadFromFile(const std::string& filepath)
{
    try {
        YAML::Node config = YAML::LoadFile(filepath);
        parseSequences(config);
        LOG_INFO("[SequenceLoader] Loaded sequences from file: %s", filepath.c_str());
        return true;
    }
    catch (const YAML::Exception& e) {
        LOG_ERROR("[SequenceLoader] Failed to load YAML file: %s - %s", filepath.c_str(), e.what());
        return false;
    }
}

bool SequenceLoader::loadFromParam(const std::string& param_name)
{
    ros::NodeHandle nh;
    std::string yaml_content;

    if (!nh.getParam(param_name, yaml_content)) {
        LOG_ERROR("[SequenceLoader] Failed to get parameter: %s", param_name.c_str());
        return false;
    }

    try {
        YAML::Node config = YAML::Load(yaml_content);
        parseSequences(config);
        LOG_INFO("[SequenceLoader] Loaded sequences from parameter: %s", param_name.c_str());
        return true;
    }
    catch (const YAML::Exception& e) {
        LOG_ERROR("[SequenceLoader] Failed to parse YAML parameter: %s", e.what());
        return false;
    }
}

void SequenceLoader::parseSequences(const YAML::Node& config)
{
    sequences_.clear();

    if (!config["sequences"]) {
        LOG_INFO("[SequenceLoader] No 'sequences' key found in YAML");
        return;
    }

    const YAML::Node& sequences = config["sequences"];

    for (YAML::const_iterator it = sequences.begin(); it != sequences.end(); ++it) {
        std::string seq_key = it->first.as<std::string>();
        const YAML::Node& seq_node = it->second;

        Sequence seq;
        seq.drive_type = seq_node["drive_type"].as<int>();
        seq.name = seq_node["name"].as<std::string>();
        seq.description = seq_node["description"].as<std::string>("");
        seq.tilt_correction_enabled = seq_node["tilt_correction_enabled"].as<bool>(false);  // Default: false
        seq.lift_adjustment_mode = seq_node["lift_adjustment_mode"].as<std::string>("continuous");  // Default: continuous

        if (seq_node["steps"]) {
            for (const auto& step_node : seq_node["steps"]) {
                seq.steps.push_back(parseStep(step_node));
            }
        }

        sequences_[seq.drive_type] = seq;
        LOG_INFO(
            "[SequenceLoader] Loaded sequence '%s' (drive_type=%d, tilt_correction=%s, lift_adjustment=%s) with %zu steps", 
            seq.name.c_str(), seq.drive_type, seq.tilt_correction_enabled ? "ON" : "OFF", seq.lift_adjustment_mode.c_str(), seq.steps.size());
    }
}

ActionParams SequenceLoader::parseParams(const YAML::Node& params_node)
{
    ActionParams params;

    if (!params_node || !params_node.IsMap()) {
        return params;
    }

    for (YAML::const_iterator it = params_node.begin(); it != params_node.end(); ++it) {
        std::string key = it->first.as<std::string>();
        const YAML::Node& value_node = it->second;

        // Try to determine type and store accordingly
        if (value_node.IsScalar()) {
            std::string value_str = value_node.as<std::string>();

            // Check if it's a boolean
            if (value_str == "true" || value_str == "false") {
                params.setBool(key, value_node.as<bool>());
            }
            // Check if it's a number
            else {
                try {
                    double value_double = value_node.as<double>();
                    params.setDouble(key, value_double);
                }
                catch (...) {
                    // If not a number, store as string
                    params.setString(key, value_str);
                }
            }
        }
    }

    return params;
}

SequenceStep SequenceLoader::parseStep(const YAML::Node& step_node)
{
    SequenceStep step;

    step.action_type = step_node["action_type"].as<std::string>("");
    step.wait_for = step_node["wait_for"].as<std::string>("");
    step.delay_sec = step_node["delay_sec"].as<double>(0.0);

    // Parse parameters
    if (step_node["params"]) {
        step.params = parseParams(step_node["params"]);
    }

    // Parse parallel actions
    if (step_node["parallel_actions"]) {
        for (const auto& action_node : step_node["parallel_actions"]) {
            step.parallel_actions.push_back(parseParallelAction(action_node));
        }
    }

    return step;
}

ParallelAction SequenceLoader::parseParallelAction(const YAML::Node& action_node)
{
    ParallelAction action;
    action.action_type = action_node["action_type"].as<std::string>("");

    if (action_node["params"]) {
        action.params = parseParams(action_node["params"]);
    }

    return action;
}

Sequence SequenceLoader::getSequence(int drive_type) const
{
    auto it = sequences_.find(drive_type);
    if (it != sequences_.end()) {
        return it->second;
    }
    LOG_INFO("[SequenceLoader] No sequence found for drive_type=%d", drive_type);
    return Sequence();
}

bool SequenceLoader::hasSequence(int drive_type) const
{
    return sequences_.find(drive_type) != sequences_.end();
}

std::vector<int> SequenceLoader::getAvailableDriveTypes() const
{
    std::vector<int> types;
    for (const auto& pair : sequences_) {
        types.push_back(pair.first);
    }
    return types;
}

}  // namespace nc_dcs
