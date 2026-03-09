/*
 * @file	: dekf_param.hpp
 * @date	: Dec 20, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: purepersuit motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_DEKF_PARAM_HPP_
#define NAVIFRA_DEKF_PARAM_HPP_

namespace NVFR {

struct DekfParam_t
{
  // observer
  int n_max_buffer_size = 1;
  // perception constraints
  double d_max_error_dist = 1.0;
  double d_max_error_deg = 10.0;
  // threshold of convergence
  double d_cvg_cov_d_thr = 0.0005;
  double d_cvg_cov_a_thr = 0.0005;
  // covariance of ekf (Q:model, R:noise)
  double d_cov_Q0_d_weight = 0.0; // 0.002
  double d_cov_Q0_a_weight = 0.0; // 0.002
  double d_cov_Qm_dd_weight = 0.0; // 0.002
  double d_cov_Qm_da_weight = 0.0; // 0.02
  double d_cov_Qm_ad_weight = 0.0; // 0.001
  double d_cov_Qm_aa_weight = 0.0; // 0.01
  double d_cov_R0_d_weight = 0.0; // 0.002
  double d_cov_R0_a_weight = 0.0; // 0.002
  double d_cov_Rm_dd_weight = 0.0; // 0.01
  double d_cov_Rm_da_weight = 0.0;
  double d_cov_Rm_ad_weight = 0.0;
  double d_cov_Rm_aa_weight = 0.0; // 0.015
};

} // namespace NVFR

#endif
