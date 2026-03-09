/*
 * @file	: girdmap_param.hpp
 * @date	: Dec 20, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: gridmap param (global, local)
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_GRID_MAP_PARAM_HPP_
#define NAVIFRA_GRID_MAP_PARAM_HPP_

namespace NVFR {

struct GridMapParam_t
{
  // global map
  double d_global_map_resolution_m = 0.1; // [m] resolution
  double d_global_map_padding_m = 0.15; // [m] padding size
  double d_global_cost_dist_m = 0.35; // [m] cost distance

  // local map
  int n_local_map_period_msec = 80; // [msec]
  int n_local_map_size_x_px = 300; // map x size
  int n_local_map_size_y_px = 300; // map y size
  double d_local_map_resolution_m = 0.05; // [m] resolution
  double d_local_map_padding_m = 0.03; // [m] padding size
  double d_local_cost_dist_m = 0.3; // [m] cost distance
};

} // namespace NVFR

#endif
