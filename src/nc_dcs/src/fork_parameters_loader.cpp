#include "nc_dcs/fork_parameters_loader.h"
#include <ros/package.h>

namespace nc_dcs {

ForkParametersLoader::ForkParametersLoader() {
    // Initialize with default values if needed
}

bool ForkParametersLoader::loadFromFile(const std::string& file_path) {
    try {
        YAML::Node config = YAML::LoadFile(file_path);
        
        if (!config["fork_parameters"]) {
            LOG_ERROR("[ForkParams] No 'fork_parameters' section found in %s", file_path.c_str());
            return false;
        }
        
        YAML::Node fork_params = config["fork_parameters"];
        if (!fork_params["rack_heights"]) {
            LOG_ERROR("[ForkParams] No 'rack_heights' section found in fork_parameters");
            return false;
        }
        
        // Load global parameters
        if (fork_params["docking_height_tolerance_mm"]) {
            docking_height_tolerance_mm_ = fork_params["docking_height_tolerance_mm"].as<int>();
            LOG_INFO("[ForkParams] docking_height_tolerance_mm: %d", docking_height_tolerance_mm_);
        }
        if (fork_params["lift_height_diff_speed_threshold_mm"]) {
            lift_height_diff_speed_threshold_mm_ = fork_params["lift_height_diff_speed_threshold_mm"].as<double>();
            LOG_INFO("[ForkParams] lift_height_diff_speed_threshold_mm: %.0f", lift_height_diff_speed_threshold_mm_);
        }
        if (fork_params["lift_height_diff_speed_percent"]) {
            lift_height_diff_speed_percent_ = fork_params["lift_height_diff_speed_percent"].as<double>();
            LOG_INFO("[ForkParams] lift_height_diff_speed_percent: %.0f", lift_height_diff_speed_percent_);
        }

        YAML::Node rack_heights = fork_params["rack_heights"];
        
        // Parse each rack configuration
        for (auto it = rack_heights.begin(); it != rack_heights.end(); ++it) {
            std::string rack_key = it->first.as<std::string>();
            parseRackConfig(it->second, rack_key);
        }
        
        LOG_INFO("[ForkParams] Successfully loaded %zu rack configurations from %s", 
                 rack_configs_.size(), file_path.c_str());
        
        // Log loaded configurations
        for (const auto& pair : rack_configs_) {
            const RackHeightConfig& config = pair.second;
            LOG_INFO("[ForkParams] Rack Type %d (%s): approach=%.0f, pickup=%.0f, carry=%.0f, place=%.0f, clearance=%.0f", 
                     config.rack_type, config.name.c_str(),
                     config.approach_height, config.pickup_height, config.carry_height,
                     config.place_height, config.clearance_height);
        }
        
        return true;
    } catch (const YAML::Exception& e) {
        LOG_ERROR("[ForkParams] YAML parsing error: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        LOG_ERROR("[ForkParams] Error loading fork parameters: %s", e.what());
        return false;
    }
}

void ForkParametersLoader::parseRackConfig(const YAML::Node& rack_node, const std::string& rack_key) {
    try {
        RackHeightConfig config;
        
        // Parse basic info
        config.rack_type = rack_node["rack_type"].as<int>();
        config.name = rack_node["name"].as<std::string>();
        config.description = rack_node["description"].as<std::string>();
        
        // Parse heights
        YAML::Node heights = rack_node["heights"];
        config.approach_height = heights["approach_height"].as<double>();
        config.pickup_height = heights["pickup_height"].as<double>();
        config.carry_height = heights["carry_height"].as<double>();
        config.place_height = heights["place_height"].as<double>();
        config.clearance_height = heights["clearance_height"].as<double>();
        
        // Store in map using rack_type as key
        rack_configs_[config.rack_type] = config;
        
        LOG_DEBUG("[ForkParams] Parsed rack config: %s (type %d)", 
                  config.name.c_str(), config.rack_type);
                  
    } catch (const YAML::Exception& e) {
        LOG_ERROR("[ForkParams] Error parsing rack config '%s': %s", 
                  rack_key.c_str(), e.what());
    }
}

double ForkParametersLoader::getApproachHeight(int rack_type) const {
    auto it = rack_configs_.find(rack_type);
    if (it != rack_configs_.end()) {
        return it->second.approach_height;
    }
    LOG_WARNING("[ForkParams] Rack type %d not found, returning 0", rack_type);
    return 0.0;
}

double ForkParametersLoader::getPickupHeight(int rack_type) const {
    auto it = rack_configs_.find(rack_type);
    if (it != rack_configs_.end()) {
        return it->second.pickup_height;
    }
    LOG_WARNING("[ForkParams] Rack type %d not found, returning 0", rack_type);
    return 0.0;
}

double ForkParametersLoader::getCarryHeight(int rack_type) const {
    auto it = rack_configs_.find(rack_type);
    if (it != rack_configs_.end()) {
        return it->second.carry_height;
    }
    LOG_WARNING("[ForkParams] Rack type %d not found, returning 0", rack_type);
    return 0.0;
}

double ForkParametersLoader::getPlaceHeight(int rack_type) const {
    auto it = rack_configs_.find(rack_type);
    if (it != rack_configs_.end()) {
        return it->second.place_height;
    }
    LOG_WARNING("[ForkParams] Rack type %d not found, returning 0", rack_type);
    return 0.0;
}

double ForkParametersLoader::getClearanceHeight(int rack_type) const {
    auto it = rack_configs_.find(rack_type);
    if (it != rack_configs_.end()) {
        return it->second.clearance_height;
    }
    LOG_WARNING("[ForkParams] Rack type %d not found, returning 0", rack_type);
    return 0.0;
}

double ForkParametersLoader::getHeight(int rack_type, const std::string& operation) const {
    auto it = rack_configs_.find(rack_type);
    if (it == rack_configs_.end()) {
        LOG_WARNING("[ForkParams] Rack type %d not found, returning 0", rack_type);
        return 0.0;
    }
    
    const RackHeightConfig& config = it->second;
    
    if (operation == "approach") {
        return config.approach_height;
    } else if (operation == "pickup") {
        return config.pickup_height;
    } else if (operation == "carry") {
        return config.carry_height;
    } else if (operation == "place") {
        return config.place_height;
    } else if (operation == "clearance") {
        return config.clearance_height;
    } else {
        LOG_WARNING("[ForkParams] Unknown operation '%s', returning 0", operation.c_str());
        return 0.0;
    }
}

bool ForkParametersLoader::hasRackType(int rack_type) const {
    return rack_configs_.find(rack_type) != rack_configs_.end();
}

const RackHeightConfig* ForkParametersLoader::getRackConfig(int rack_type) const {
    auto it = rack_configs_.find(rack_type);
    if (it != rack_configs_.end()) {
        return &(it->second);
    }
    return nullptr;
}

int ForkParametersLoader::getTargetValue(const std::string& action_type, const std::map<std::string, std::string> & params, int target_height) const
 {
    if(action_type == "fork_updown"){
        return target_height;
    }
    else if(action_type == "fork_side") {
        auto it = params.find("position");
        std::string position = (it != params.end()) ? it->second : "wide";
        return (position == "wide") ? 2 : 1;
    }
    else if(action_type == "fork_tilt"){
        auto it = params.find("direction");
        std::string direction = (it != params.end()) ? it->second : "up";
        return (direction == "up") ? 2 : 1;
    }
    return 0;
 }
} // namespace nc_dcs