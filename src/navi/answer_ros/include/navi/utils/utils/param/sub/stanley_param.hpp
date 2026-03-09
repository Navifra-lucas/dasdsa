/*
 * @file	: stanley_param.hpp
 * @date	: Dec 20, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: purepersuit motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_STANLEY_PARAM_HPP_
#define NAVIFRA_STANLEY_PARAM_HPP_

#include "pid_param.hpp"

namespace NVFR {

struct StanleyParam_t
{
  double d_virtual_length_m = 1.0; // [m]
  double d_min_speed_ms = 0.02; // [m/s]
  double d_min_dist_m = 0.1; // [m]
  double d_lat_err_w = 0.7; // [m/s]

  // maximum distance of additional path on the end waypoint
  double d_max_additional_dist_m=1.0;

  // pid param
  PidParam_t st_pid_a;
  PidParam_t st_pid_w;
};

} // namespace NVFR

#endif
