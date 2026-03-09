/*
 * @file	: spd_opt_param.hpp
 * @date	: Dec 20, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: purepersuit motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_SPD_OPT_PARAM_HPP_
#define NAVIFRA_SPD_OPT_PARAM_HPP_

#include <stdint.h>

namespace NVFR {

struct SpdOptParam_t
{
  unsigned int un_max_iter=100; // [N]
  int n_time_limit_msec=200; // [msec]
  double d_eps=1e-5;
  double d_alpha=0.5;
  double d_beta=0.25;

  // weight
  double d_w_v = 100.0;

};

} // namespace NVFR

#endif
