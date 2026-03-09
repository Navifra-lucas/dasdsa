#ifndef NC_DCS_FORK_PARAMETERS_LOADER_H
#define NC_DCS_FORK_PARAMETERS_LOADER_H

#include <string>
#include <map>
#include <yaml-cpp/yaml.h>
#include "util/logger.hpp"

namespace nc_dcs {

struct RackHeightConfig {
    int rack_type;
    std::string name;
    std::string description;
    double approach_height;   // mm
    double pickup_height;     // mm
    double carry_height;      // mm
    double place_height;      // mm
    double clearance_height;  // mm
};

class ForkParametersLoader {
public:
    ForkParametersLoader();
    
    bool loadFromFile(const std::string& file_path);
    
    // Get height for specific rack type and operation
    double getApproachHeight(int rack_type) const;
    double getPickupHeight(int rack_type) const;
    double getCarryHeight(int rack_type) const;
    double getPlaceHeight(int rack_type) const;
    double getClearanceHeight(int rack_type) const;
    
    // Get height by operation name and rack type
    double getHeight(int rack_type, const std::string& operation) const;
    
    // Check if rack type is supported
    bool hasRackType(int rack_type) const;
    
    // Get rack configuration
    const RackHeightConfig* getRackConfig(int rack_type) const;
    
    // Get docking height tolerance (mm) for interlock check
    int getDockingHeightToleranceMm() const { return docking_height_tolerance_mm_; }

    // Get lift speed control parameters
    double getLiftHeightDiffSpeedThreshold() const { return lift_height_diff_speed_threshold_mm_; }
    double getLiftHeightDiffSpeedPercent() const { return lift_height_diff_speed_percent_; }

    // Get target value for fork action based on action type and parameters
    int getTargetValue(const std::string& action_type, 
                       const std::map<std::string, std::string>& params,
                       int target_height) const;
    
private:
    std::map<int, RackHeightConfig> rack_configs_;
    int docking_height_tolerance_mm_ = 50;  // Default 50mm
    double lift_height_diff_speed_threshold_mm_ = 200.0;  // Default 200mm
    double lift_height_diff_speed_percent_ = 50.0;         // Default 50%
    
    void parseRackConfig(const YAML::Node& rack_node, const std::string& rack_key);
};

} // namespace nc_dcs

#endif // NC_DCS_FORK_PARAMETERS_LOADER_H