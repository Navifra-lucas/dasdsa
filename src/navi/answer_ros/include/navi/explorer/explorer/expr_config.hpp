/*
 * @file	: expr_config.hpp
 * @date	: Jun 12, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: edge contour detection from image
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef EXPLORER_EXPR_CONFIG_HPP_
#define EXPLORER_EXPR_CONFIG_HPP_

#include <iostream>
#include <sstream>

#include "utils/robot_config.hpp"

#include "explorer/param/sub/expr_weight_param.hpp"
#include "explorer/utils.hpp"

namespace NVFR {

struct ExprConfig_t
{
  // robot config
  RobotConfig o_robot_config;
  float f_min_robot_radius_m = 0.3f;

  // sensor info
  EUS::SensorInfo o_sensor_info;

  // collision margin
  float f_margin_m = 0.15f;

  // explore
  float f_start_path_distance_m = 0.5f;
  float f_min_unknown_length_m = 0.01f;

  // weight of explore cost
  ExprWeightParam_t st_expr_w;

};

} // namespace NVFR

#endif
