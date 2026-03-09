#ifndef NC_DCS_MOCK_MSGS_H
#define NC_DCS_MOCK_MSGS_H

#include <geometry_msgs/PoseStamped.h>

#include <string>

namespace nc_dcs {

// Path Planning Module Messages
struct DockingCmd {
    geometry_msgs::PoseStamped target_pose;
};

// Lift Control Module Messages
struct LiftCmd {
    double height;
    std::string action_type;  // "UP", "DOWN", "STOP"
};

// Mission structure (from ForkLift message)
struct Mission {
    std::string type;  // Sequence name

    // From ForkLift.msg fields
    float f_current_x;
    float f_current_y;
    float f_current_deg;
    float f_target_x;
    float f_target_y;
    float f_target_deg;
    int n_rack_level;
    int n_target_level;
    int n_target_height;  // in mm
    int n_real_target_height;  // in mm
    int n_rack_type;
    int n_pallet_type;
    double intensity_height;  // Measured intensity height
    double f_perception_z;
    bool b_perception_valid;

    Mission()
        : f_current_x(0.0)
        , f_current_y(0.0)
        , f_current_deg(0.0)
        , f_target_x(0.0)
        , f_target_y(0.0)
        , f_target_deg(0.0)
        , n_rack_level(0)
        , n_target_level(0)
        , n_target_height(0)
        , n_rack_type(0)
        , n_real_target_height(0)
        , n_pallet_type(0)
        , intensity_height(0.0)
        , f_perception_z(0.0)
        , b_perception_valid(false)
    {
    }
};

}  // namespace nc_dcs

#endif  // NC_DCS_MOCK_MSGS_H