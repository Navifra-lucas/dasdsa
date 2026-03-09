/*
 * @file	: safety_area_param.hpp
 * @date	: Feb 28, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Collision detector
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef SAFETY_AREA_PARAM_HPP_
#define SAFETY_AREA_PARAM_HPP_

namespace NVFR {

struct SafetyAreaParam_t
{
  double d_dist_m=0.0;
  int n_lin_spd_limit_cm_s=0;
  int n_ang_spd_limit_deg_s=0;
};

} // namespace NVFR

#endif
