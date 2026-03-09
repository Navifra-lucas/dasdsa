/*
 * @file	: pcl_cluster_param.hpp
 * @date	: Dec 20, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: purepersuit motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_PCL_CLUSTER_PARAM_HPP_
#define NAVIFRA_PCL_CLUSTER_PARAM_HPP_

namespace NVFR {

struct PclClusterParam_t
{
  // observer
  int n_method = 0;
  // perception constraints
  int n_ror_min_neighbor_num = 5;
  double d_ror_radius_m = 0.1;
  // threshold of convergence
  int n_sor_neighbor_num = 10;
  double d_sor_cov_thr = 1.0;
};

} // namespace NVFR

#endif
