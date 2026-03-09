/*
 * @file	: robot_config_param.hpp
 * @date	: Dec 20, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: gridmap param (global, local)
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_ROBOT_CONFIG_PARAM_HPP_
#define NAVIFRA_ROBOT_CONFIG_PARAM_HPP_

#include <vector>

#include "safety_area_param.hpp"

namespace NVFR {

struct RobotConfigParam_t
{
  // robot shape
  int n_robot_shape = 0; // robot shape (0: BOX, 1: CIRCLE)

  // box type config
  double d_front_m = 0.25; // [m] distance from base_link to front of robot body
  double d_rear_m = -0.25; // [m] distance from base_link to rear of robot body
  double d_left_m = 0.15; // [m] distance from base_link to left of robot body
  double d_right_m = -0.15; // [m] distance from base_link to right of robot body
  double d_body_resolution_m = 0.1; // [m] resolution of robot body

  // circle type config
  double d_radius_m = 0.5; // [m] radius of robot body

  // safety area
  std::vector<SafetyAreaParam_t> vec_safety_area;
};

} // namespace NVFR

#endif
