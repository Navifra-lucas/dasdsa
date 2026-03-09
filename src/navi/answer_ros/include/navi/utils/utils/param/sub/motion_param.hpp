/*
 * @file	: motion_param.hpp
 * @date	: Dec 20, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: purepersuit motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_MOTION_PARAM_HPP_
#define NAVIFRA_MOTION_PARAM_HPP_

#include "traj_param.hpp"
#include "align_param.hpp"
#include "spd_opt_param.hpp"
#include "purepersuit_param.hpp"
#include "mekf_param.hpp"
#include "hec_param.hpp"
#include "stanley_param.hpp"
#include "mpc_param.hpp"
#include "docking_param.hpp"

namespace NVFR {

struct MotionParam_t
{
  // motion system
  int64_t n_info_period_msec=50UL; // [msec]
  double d_motion_period_sec=0.04; // [sec]
  int n_motion_pred_step=50; // [N] (used on speed_optimizer & mpc)

  // motion info
  double d_motion_min_vel_ms = 0.02; // [m/s]
  double d_motion_max_vel_ms = 2.0; // [m/s]
  double d_motion_max_accel_mss = 5.0; // [m/ss]
  double d_motion_max_decel_mss = 10.0; // [m/ss]
  double d_motion_max_jerk_msss = 1.0; // [m/sss]
  double d_motion_max_ang_vel_rads = 0.35; // [rad/s]
  double d_motion_max_curv = 3.0; // [1/m]
  double d_motion_max_centri_acc = 0.1; // [m/ss]
  double d_motion_max_centri_jrk = 0.3; // [m/sss]

  // reduce speed from lateral error
  double d_allowed_lateral_error_m = 0.1; // [m]
  double d_max_lateral_error_m = 0.5; // [m]

  // traj info
  TrajParam_t st_traj_param;

  // align controller
  AlignParam_t st_align_param;

  // speed optimizer
  SpdOptParam_t st_spd_opt_param;

  // controller
  PurepersuitParam_t st_pp_param;
  MekfParam_t st_mekf_param;
  HecParam_t st_hec_param;
  StanleyParam_t st_stanley_param;
  MPCParam_t st_mpc_param;

  // docking
  DockingParam_t st_docking_param;

};

} // namespace NVFR

#endif
