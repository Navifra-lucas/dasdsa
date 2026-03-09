#ifndef DISTANCE_RAY_CALCULATOR_HPP_
#define DISTANCE_RAY_CALCULATOR_HPP_

#include "core_msgs/MapDB.h"
#include "distance_transformer.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>
using namespace std;

namespace NaviFra {
class DistanchRayCalculator {
public:
    DistanchRayCalculator();
    ~DistanchRayCalculator(){};
    void Initialize(const core_msgs::MapDB::ConstPtr& o_map, float f_max_search_range_m);
    float CalcPointLength(int n_map_x_px, int n_map_y_px, float f_heading_angle_rad);
    void DistanceMap(const core_msgs::MapDB::ConstPtr& msg);

private:
    std::vector<std::vector<float>> vec_distance_map_;

    float f_map_max_x_ = 0;
    float f_map_min_x_ = 0;

    float f_map_max_y_ = 0;
    float f_map_min_y_ = 0;

    float f_dist_threshold_ = 5.0;
    float f_step_coeff_ = 0.999;
    float f_max_search_range_m_ = 0.f;
    float f_resolution_ = 0.01;
    bool b_start_make_map_ = false;
};
}  // namespace NaviFra

#endif
