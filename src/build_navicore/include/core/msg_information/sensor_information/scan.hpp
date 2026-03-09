#ifndef SCAN_T_HPP_
#define SCAN_T_HPP_

#include "extrinsic_parameter.hpp"

#include <vector>
namespace NaviFra {

struct Scan_t {
    ExtrinsicParameter_t st_extrinsic_parameter;
    float f_angle_min_deg;
    float f_angle_max_deg;
    float f_angle_increment_deg;
    float f_scan_time_sec;
    float f_min_range_m;
    float f_max_range_m;
    std::vector<float> vec_data_m;

    Scan_t()
    {
        f_angle_min_deg = 0.f;
        f_angle_max_deg = 0.f;
        f_angle_increment_deg = 0.f;
        f_scan_time_sec = 0.f;
        f_min_range_m = 0.f;
        f_max_range_m = 0.f;
    }
};

}  // namespace NaviFra

#endif