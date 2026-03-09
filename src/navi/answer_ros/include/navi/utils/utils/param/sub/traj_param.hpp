/*
 * @file	: traj_param.hpp
 * @date	: Jul 28, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: purepersuit motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_TRAJ_PARAM_HPP_
#define NAVIFRA_TRAJ_PARAM_HPP_

namespace NVFR {

struct TrajParam_t
{
  // common traj info
  double d_max_curv = 3.0; // [m]

  // normal traj info
  double d_normal_min_vel_ms = 0.2; // [m/s]
  double d_normal_accel_mss = 0.7; // [m/ss]
  double d_normal_decel_mss = 0.37; // [m/ss]
  double d_normal_jerk_msss = 0.3; // [m/s]
  double d_normal_max_ang_vel_rads = 0.3; // [rad/s]
  double d_normal_threshold_for_end_m = 0.005; // [m]
  double d_normal_arround_goal_min_vel_dist_m = 0.05; // [m]

  // slow traj info
  double d_slow_min_vel_ms = 0.02; // [m/s]
  double d_slow_accel_mss = 0.3; // [m/ss]
  double d_slow_decel_mss = 0.25; // [m/ss]
  double d_slow_jerk_msss = 0.2; // [m/s]
  double d_slow_max_ang_vel_rads = 0.3; // [rad/s]
  double d_slow_threshold_for_end_m = 0.002; // [m]
  double d_slow_arround_goal_min_vel_dist_m = 0.1; // [m]

  // smooth curv
  bool b_smooth_curv = true;
  int n_smooth_curv_size = 5;
};

} // namespace NVFR

#endif
