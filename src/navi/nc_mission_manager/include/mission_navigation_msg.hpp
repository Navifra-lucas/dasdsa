#ifndef MISSION_NAVIGATION_INFO_HPP_
#define MISSION_NAVIGATION_INFO_HPP_

#include "msg_information/parameter_information/motion_parameter.hpp"
#include "polygon/polygon.hpp"
#include "pos/pos.hpp"

#include <list>
#include <vector>

namespace NaviFra {
struct MissionNavigationMsg_t {
    int n_path_index;
    float f_collision_remained_sec;
    float f_node_to_node_percent;
    float f_max_linear_vel_of_path;

    float f_goal_pos_x_m;
    float f_goal_pos_y_m;
    float f_goal_pos_deg;

    float f_robot_pos_x_m;
    float f_robot_pos_y_m;
    float f_robot_pos_deg;
    float f_robot_current_linear_vel_x;
    float f_robot_current_angular_vel_w;

    float f_path_error_dist_m;
    float f_path_angle_deg;

    int n_stop_because_of_obstacle = 0;

    bool b_spin_turn;
    int n_avoid_state;
    int n_drive_type;
    std::string s_current_node;
    std::string s_current_node_id;
    std::string s_next_node;
    std::string s_next_node_id;
    std::string s_goal_node;
    std::string s_goal_node_id;
    std::string s_description_msg;
};

}  // namespace NaviFra

#endif
