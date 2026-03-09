#ifndef NC_DCS_SEQUENCE_LOADER_H
#define NC_DCS_SEQUENCE_LOADER_H

#include "util/logger.hpp"

#include <yaml-cpp/yaml.h>

#include <map>
#include <string>
#include <vector>

namespace nc_dcs {

// Action parameters structure (flexible key-value storage)
struct ActionParams {
    std::map<std::string, std::string> string_params;
    std::map<std::string, double> double_params;
    std::map<std::string, bool> bool_params;

    // Helper methods
    std::string getString(const std::string& key, const std::string& default_val = "") const;
    double getDouble(const std::string& key, double default_val = 0.0) const;
    bool getBool(const std::string& key, bool default_val = false) const;

    void setString(const std::string& key, const std::string& value);
    void setDouble(const std::string& key, double value);
    void setBool(const std::string& key, bool value);
    bool has(const std::string& key) const;
};

// Parallel action structure (for simultaneous actions)
struct ParallelAction {
    std::string action_type;  // fork_updown, fork_side, fork_tilt, move, perception
    ActionParams params;
};

// Sequence step structure
struct SequenceStep {
    std::string action_type;  // Main action type
    ActionParams params;  // Action parameters
    std::string wait_for;  // Wait condition: "completed", "none"
    double delay_sec;  // Fixed delay in seconds
    std::vector<ParallelAction> parallel_actions;  // Parallel actions

    SequenceStep()
        : delay_sec(0.0)
    {
    }
};

// Sequence structure
struct Sequence {
    int drive_type;
    std::string name;
    std::string description;
    std::vector<SequenceStep> steps;
    bool tilt_correction_enabled;  // Enable/disable automatic tilt correction based on perception
    std::string lift_adjustment_mode;  // "continuous" (default): keep adjusting height, "once": adjust only once at start

    Sequence()
        : tilt_correction_enabled(false)  // Default: disabled
        , lift_adjustment_mode("continuous")  // Default: continuous adjustment
    {
    }
};

class SequenceLoader {
public:
    SequenceLoader();
    bool loadFromFile(const std::string& filepath);
    bool loadFromParam(const std::string& param_name);

    Sequence getSequence(int drive_type) const;
    bool hasSequence(int drive_type) const;
    std::vector<int> getAvailableDriveTypes() const;

private:
    std::map<int, Sequence> sequences_;

    void parseSequences(const YAML::Node& config);
    SequenceStep parseStep(const YAML::Node& step_node);
    ParallelAction parseParallelAction(const YAML::Node& action_node);
    ActionParams parseParams(const YAML::Node& params_node);
};

}  // namespace nc_dcs

#endif  // NC_DCS_SEQUENCE_LOADER_H
