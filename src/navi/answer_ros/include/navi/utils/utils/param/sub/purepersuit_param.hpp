/*
 * @file	: purepersuit_param.hpp
 * @date	: Dec 20, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: purepersuit motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_PUREPERSUIT_PARAM_HPP_
#define NAVIFRA_PUREPERSUIT_PARAM_HPP_

#include "pid_param.hpp"

namespace NVFR {

struct PurepersuitParam_t
{
  double d_lh_default_dist_m = 0.5; // [m]
  double d_lh_vel_1st_order = 1.0; // [1/s]
  double d_lh_curv_w = 0.5;

  // maximum distance of additional path on the end waypoint
  double d_max_additional_dist_m=1.0;

  // pid param
  PidParam_t st_pid_a;
  PidParam_t st_pid_w;
};

} // namespace NVFR

#endif
