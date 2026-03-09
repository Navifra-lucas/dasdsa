#ifndef AVOID_STRUCTURE_HPP
#define AVOID_STRUCTURE_HPP

#include "core_calculator/core_calculator.hpp"
#include "pos/pos.hpp"

#include <cmath>

namespace NaviFra {
// class AvoidStructure
// {
// public:
//     AvoidStructure(){};
//     virtual ~AvoidStructure(){};

struct Avoid_sector_t {
    int n_index = 0;
    int n_avoid_type = 0;
    int n_avoid_step_l_end = 0;
    int n_avoid_step_r_end = 0;
    Pos::LINE_TYPE e_curve_type = Pos::LINE_TYPE::LINE;

    bool operator==(const Avoid_sector_t& other) const
    {
        return n_avoid_type == other.n_avoid_type && n_avoid_step_l_end == other.n_avoid_step_l_end &&
            n_avoid_step_r_end == other.n_avoid_step_r_end && e_curve_type == other.e_curve_type;
    };

    bool operator!=(const Avoid_sector_t& other) const
    {
        return n_avoid_type != other.n_avoid_type || n_avoid_step_l_end != other.n_avoid_step_l_end ||
            n_avoid_step_r_end != other.n_avoid_step_r_end || e_curve_type != other.e_curve_type;
    };
};

struct Avoid_lane_t {
    std::vector<std::vector<Pos>> vec_avoid_path_left;
    std::vector<std::vector<Pos>> vec_avoid_path_right;
};

struct Avoid_remain_obs_t {
    Pos o_remain_obs;
    int n_avoid_condition = 0;
};

struct Lane_info_t {
    int n_right_max_num = 0;
    int n_left_max_num = 0;
    float f_lane_dist = 0;
};

struct Avoid_path_result {
    std::vector<NaviFra::Pos> vec_avoid_path;
    int n_target_lane_num;
};

struct Extend_path_result {
    std::vector<NaviFra::Pos> vec_extend_path;
    bool b_extend_available = true;
};

struct Avoid_sample_point {
    bool b_use = false;
    int n_s = 0;
    int n_d = 0;
    float f_s_m = 0;
    float f_d_m = 0;
    NaviFra::Pos o_pos;
};

struct Avoid_sample_edge {
    bool b_use = true;
    std::vector<NaviFra::Pos> edge;
};

// };
}  // namespace NaviFra
#endif