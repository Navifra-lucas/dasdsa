/*
 * @file	: docking_param.hpp
 * @date	: Dec 20, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: purepersuit motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_DOCKING_PARAM_HPP_
#define NAVIFRA_DOCKING_PARAM_HPP_

#include "dekf_param.hpp"

namespace NVFR {

struct DockingParam_t
{
  // latency
  int n_delay_msec = 0;

  // tf (robot to sensor)
  double d_robot2sensor_x_m = 0.0;
  double d_robot2sensor_y_m = 0.0;
  double d_robot2sensor_deg = 0.0;

  // ekf
  DekfParam_t st_dekf_param;
};

} // namespace NVFR

#endif
