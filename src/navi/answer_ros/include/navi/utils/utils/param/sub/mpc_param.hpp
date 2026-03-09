/*
 * @file	: mpc_param.hpp
 * @date	: Dec 20, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: purepersuit motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_MPC_PARAM_HPP_
#define NAVIFRA_MPC_PARAM_HPP_

namespace NVFR {

struct MPCParam_t
{
  // osqp param
  unsigned int un_max_iter=100; // [N]
  int n_time_limit_msec=100; // [micro sec]
  double d_eps=1e-5;
  double d_alpha=0.5;
  double d_sigma=0.5;
  double d_rho=0.5;

  // ackermann-steer
  double d_sd_wheel_distance_m=2.0;

  // constraints
  double d_max_lin_vel=2.0;
  double d_min_lin_vel=0.02;
  double d_max_ang_vel=0.5;
  double d_max_phi_vel=0.5;

  double d_max_lin_acc=10.0;
  double d_min_lin_acc=-20.0;
  double d_max_ang_acc=2.0;
  double d_max_phi_acc=2.0;

  double d_max_lin_jerk=0.3;
  double d_max_ang_jerk=0.05236;

  // cost weight
  double d_position_w=30.0;
  double d_angle_w=2.0;
  double d_phi_w=2.0;
  double d_lin_vel_w=1.0;
  double d_ang_vel_w=0.1;
  double d_phi_vel_w=0.1;

  double d_lin_acc_w=1.0;
  double d_ang_acc_w=1.0;

  double d_fposition_w=1.0;
  double d_fangle_w=1.0;
  double d_fphi_w=1.0;
  double d_fang_vel_w=1.0;
  double d_fang_acc_w=1.0;
  double d_fphi_vel_w=1.0;

  double d_hposition_w=1.0;
  double d_hangle_w=1.0;
  double d_hphi_w=1.0;
  double d_hang_vel_w=1.0;
  double d_hang_acc_w=1.0;
  double d_hphi_vel_w=1.0;

  // change weight
  double d_start_percise_motion_dist_m=2.0;
  double d_end_percise_motion_dist_m=0.5;
  double d_high_speed_m_s=1.0;

  // relax position weight
  bool b_relax_pos_w=true;
  double d_relax_pos_w=10.0;

  // maximum distance of additional path on the end waypoint
  double d_max_additional_dist_m=1.0;
};

} // namespace NVFR

#endif
