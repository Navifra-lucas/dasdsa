/*
 * @file	: control_info.hpp
 * @date	: Dec 20, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: purepersuit motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef CONTROL_INFO_HPP_
#define CONTROL_INFO_HPP_

namespace NVFR {

struct ControlInfo_t
{
  // current robot info in the path
  double d_remain_m;
  double d_pred_remain_m;
  int n_pred_idx;

  // constraints
  double d_max_vel_ms;

  // robot states
  double d_linear_spd;
  double d_angular_spd;
  double d_linear_acc;
  double d_angular_acc;

};

} // namespace NVFR

#endif
