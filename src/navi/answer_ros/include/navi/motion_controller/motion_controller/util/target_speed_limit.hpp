/*
 * @file	: target_speed_limit.hpp
 * @date	: Oct 24, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: target speed limit structure
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef TARGET_SPEED_LIMIT_HPP_
#define TARGET_SPEED_LIMIT_HPP_

namespace NVFR {

struct TargetSpeedLimit_t
{
  // target speed ratio
  int n_align_speed_percent=100;
  int n_drive_speed_percent=100;

  // target speed limit
  double d_align_limit_rad_s=10.0;
  double d_drive_limit_m_s=10.0;

};

} // namespace NVFR

#endif
