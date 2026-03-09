/*
 * @file	: mekf_param.hpp
 * @date	: Dec 20, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	:
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_MEKF_PARAM_HPP_
#define NAVIFRA_MEKF_PARAM_HPP_

namespace NVFR {

struct MekfParam_t
{
  // threshold of convergence
  double d_cvg_cov_d_thr = 0.01;
  double d_cvg_cov_a_thr = 0.01;
  // covariance of ekf (Q:model, R:noise)
  double d_cov_Qu_d_sigma = 0.5;
  double d_cov_Qu_a_sigma = 0.5;
  double d_cov_R_d_sigma = 0.005;
  double d_cov_R_a_sigma = 0.01;
};

} // namespace NVFR

#endif
