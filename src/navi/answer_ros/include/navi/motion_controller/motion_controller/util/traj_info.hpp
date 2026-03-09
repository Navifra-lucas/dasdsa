/*
 * @file	: traj_info.hpp
 * @date	: Jul 25, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: trajectory information (parameters)
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef TRAJ_INFO_HPP_
#define TRAJ_INFO_HPP_

namespace NVFR {

struct TrajInfo_t
{
  // common param
  bool b_smooth_stop;
  double d_max_vel_ms;
  double d_max_curv;

  // distance param
  double d_threshold_for_end_m;
  double d_arround_goal_min_vel_dist_m;

  // control param
  double d_min_vel_ms;
  double d_accel_mss;
  double d_decel_mss;
  double d_jerk_msss;
  double d_max_ang_vel_rads;

};

} // namespace NVFR

#endif
