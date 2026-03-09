/*
 * @file	: ray_calculator.hpp
 * @date	: Mar 27, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: ray point calculator
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef RAY_CALCULATOR_HPP_
#define RAY_CALCULATOR_HPP_

#include <vector>

#include <opencv2/opencv.hpp>

#include "utils/pose.hpp"
#include "utils/grid_map/map_info.hpp"
#include "utils/grid_map/grid_struct.hpp"
#include "utils/grid_map/grid_map_calculator.hpp"

namespace NVFR {

namespace RayCalculator {

std::vector<Pos> mapToPoints(
  const std::vector<int8_t>& map, const MapInfo_t& st_map_info, const Pose& o_robot_pose,
  double d_start_angle_deg, double d_angle_range_deg,
  double d_ray_angle_res_deg, double d_max_ray_distance_m=10.0);

}  // namespace ImageLoader
namespace CR = RayCalculator;
}  // namespace NVFR

#endif
