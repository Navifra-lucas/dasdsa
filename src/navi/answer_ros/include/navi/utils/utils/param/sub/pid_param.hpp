/*
 * @file	: pid_param.hpp
 * @date	: Dec 20, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: pid param structure
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_PID_PARAM_HPP_
#define NAVIFRA_PID_PARAM_HPP_

namespace NVFR {

struct PidParam_t
{
  double d_Kp = 1.0;
  double d_Ki = 0.0;
  double d_Kd = 0.0;
  double d_cutoff = 1.0;
};

} // namespace NVFR

#endif
