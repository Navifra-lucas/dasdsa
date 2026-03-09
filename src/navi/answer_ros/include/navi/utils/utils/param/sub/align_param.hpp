/*
 * @file	: align_param.hpp
 * @date	: Jul 28, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: spin turn motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_ALIGN_PARAM_HPP_
#define NAVIFRA_ALIGN_PARAM_HPP_

namespace NVFR {

struct AlignParam_t
{
  // align controller
  double d_threshold_for_new_dyaw_deg = 0.5; // [deg]
  double d_threshold_for_end_deg = 0.5; // [deg]
  double d_threshold_for_start_motion_deg = 5.0; // [deg]
  double d_arround_target_min_vel_angle_deg = 1.0; // [deg]
  double d_min_vel_degs = 5.0; // [deg/s]
  double d_max_vel_degs = 30.0; // [deg/s]
  double d_accel_degss = 10.0; // [deg/ss]
  double d_decel_degss = 10.0; // [deg/ss]
  double d_max_decel_degss = 100.0; // [m/ss]
};

} // namespace NVFR

#endif
