/*
 * @file	: expr_param.hpp
 * @date	: Jun 12, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: edge contour detection from image
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef EXPLORER_EXPR_PARAM_HPP_
#define EXPLORER_EXPR_PARAM_HPP_

#include "utils/param/sub/robot_config_param.hpp"
#include "explorer/param/sub/sensor_info_param.hpp"
#include "explorer/param/sub/expr_weight_param.hpp"

namespace NVFR {

struct ExprParam_t
{
public:
  // robot config
  RobotConfigParam_t st_robot_config_param;

  // sensor info
  SensorInfoParam_t st_sensor_info_param;

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
