
/*
 * @file	: grid_map_calculator.hpp
 * @date	: Feb. 24, 2025
 * @author	: "HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: grid map (map_info, map)
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_GRID_MAP_GRID_MAP_CALCULATOR_HPP_
#define NAVIFRA_GRID_MAP_GRID_MAP_CALCULATOR_HPP_

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <stdint.h>

#include "logger/logger.h"

#include "utils/pose.hpp"
#include "utils/grid_map/map_info.hpp"
#include "utils/grid_map/grid_struct.hpp"
#include "utils/motion_planner_calculator.hpp"

namespace NVFR {

namespace GridMapCalculator {

// converter grid pos[px] <-> map index
inline int GridPos2MapIdx(int n_x_px, int n_y_px, const MapInfo_t& st_map_info)
{
  return n_x_px + n_y_px * st_map_info.n_size_x_px;
}
inline int GridPos2MapIdx(const GridPos& st_grid_pos, const MapInfo_t& st_map_info)
{
  return st_grid_pos.n_x_px + st_grid_pos.n_y_px * st_map_info.n_size_x_px;

}
inline GridPos MapIdx2GridMap(int n_map_idx, const MapInfo_t& st_map_info)
{
  return GridPos(n_map_idx % st_map_info.n_size_x_px, n_map_idx / st_map_info.n_size_y_px);
}

// check out of map
inline bool OutOfMap(int n_x_px, int n_y_px, const MapInfo_t& st_map_info)
{
  return ( n_x_px < 0 || n_x_px >= st_map_info.n_size_x_px || n_y_px < 0 || n_y_px >= st_map_info.n_size_y_px );
}
inline bool OutOfMap(const GridPos& st_grid_pos, const MapInfo_t& st_map_info)
{
  return ( st_grid_pos.n_x_px < 0 || st_grid_pos.n_x_px >= st_map_info.n_size_x_px || st_grid_pos.n_y_px < 0 || st_grid_pos.n_y_px >= st_map_info.n_size_y_px );
}

// converter dist <-> cost
inline int8_t CalcCost(double d_distance_m, double d_cost_dist_m)
{
  double d_r = d_distance_m / d_cost_dist_m;
  int8_t n_cost = static_cast<int8_t>( 100.0 * (1.0 - d_r) * (1.0 - d_r) ); 
  if (n_cost < 1) n_cost = 1;
  else if (n_cost > 98) n_cost = 98;
  return n_cost;
}
inline double CalcDist(int8_t n_cost, double d_cost_dist_m, double d_resolution_m)
{
  switch (n_cost)
  {
    case GRID_STATE::UNKNOWN:
    case GRID_STATE::EMPTY:
      return d_cost_dist_m + 2 * d_resolution_m;
    case GRID_STATE::PADDING:
    case GRID_STATE::OCCUPIED:
      return 0.0;
    default:
      if (n_cost < 0 || n_cost > 100) return -1.0;
  }
  return d_cost_dist_m * (1.0 - 0.1 * std::sqrt( static_cast<double>(n_cost) ));
}

// converter world [m] <-> grid pos [px]
inline int Meter2Grid(double d_val_m, double d_origin_m, double d_resolution)
{
  return static_cast<int>((d_val_m + d_origin_m) / d_resolution);
}
inline double Grid2Meter(int n_val_px, double d_origin_m, double d_resolution)
{
  return static_cast<double>(n_val_px) * d_resolution - d_origin_m;
}
inline GridPos ConvertPos2GridPos(const Pos& o_pos, const MapInfo_t& st_map_info)
{
  GridPos st_grid_pos;

  // convert pose to grid pos
  st_grid_pos.n_x_px = Meter2Grid(o_pos.GetXm(), st_map_info.d_origin_x_m, st_map_info.d_resolution_m);
  st_grid_pos.n_y_px = Meter2Grid(o_pos.GetYm(), st_map_info.d_origin_y_m, st_map_info.d_resolution_m);

  return st_grid_pos;
}
inline GridPos ConvertPose2GridPos(const Pose& o_pose, const MapInfo_t& st_map_info)
{
  GridPos st_grid_pos;

  // convert pose to grid pos
  st_grid_pos.n_x_px = Meter2Grid(o_pose.GetXm(), st_map_info.d_origin_x_m, st_map_info.d_resolution_m);
  st_grid_pos.n_y_px = Meter2Grid(o_pose.GetYm(), st_map_info.d_origin_y_m, st_map_info.d_resolution_m);

  return st_grid_pos;
}
inline GridPos ConvertPose2GridPos(const Pose& o_pose, const Pose& o_center_pose, const MapInfo_t& st_map_info)
{
  GridPos st_grid_pos;

  // transform global pose to local pose
  Pose o_local_pose = CMP::TransformG2L(o_center_pose, o_pose);

  // convert local pose to grid pos
  st_grid_pos.n_x_px = Meter2Grid(o_local_pose.GetXm(), st_map_info.d_origin_x_m, st_map_info.d_resolution_m);
  st_grid_pos.n_y_px = Meter2Grid(o_local_pose.GetYm(), st_map_info.d_origin_y_m, st_map_info.d_resolution_m);

  return st_grid_pos;
}
inline int8_t ConvertPose2Cost(const Pose& o_pose, const std::vector<int8_t>& grid_map, const Pose& o_center_pose, const MapInfo_t& st_map_info)
{
  // transform global pose to local pose
  Pose o_local_pose = CMP::TransformG2L(o_center_pose, o_pose);

  // convert local pose to map index
  int n_x_px = Meter2Grid(o_local_pose.GetXm(), st_map_info.d_origin_x_m, st_map_info.d_resolution_m);
  int n_y_px = Meter2Grid(o_local_pose.GetYm(), st_map_info.d_origin_y_m, st_map_info.d_resolution_m);
  int n_map_idx = n_x_px + n_y_px * st_map_info.n_size_x_px;

  if (OutOfMap(n_x_px, n_y_px, st_map_info)) return -1;

  return grid_map[n_map_idx];
}
inline Pos ConvertGridPos2Pos(const GridPos& st_grid_pos, const MapInfo_t& st_map_info)
{
  Pos o_pos;

  // convert grid pos to pose
  o_pos.SetXm(Grid2Meter(st_grid_pos.n_x_px, st_map_info.d_origin_x_m, st_map_info.d_resolution_m));
  o_pos.SetYm(Grid2Meter(st_grid_pos.n_y_px, st_map_info.d_origin_y_m, st_map_info.d_resolution_m));

  return o_pos;
}
inline Pose ConvertGridPos2Pose(const GridPos& st_grid_pos, const MapInfo_t& st_map_info)
{
  Pose o_pose;

  // convert grid pos to pose
  o_pose.SetXm(Grid2Meter(st_grid_pos.n_x_px, st_map_info.d_origin_x_m, st_map_info.d_resolution_m));
  o_pose.SetYm(Grid2Meter(st_grid_pos.n_y_px, st_map_info.d_origin_y_m, st_map_info.d_resolution_m));

  return o_pose;
}

inline std::vector<GridPos> CreateTable(double d_padding_m, double d_cost_dist_m, double d_resolution_m)
{
  std::vector<GridPos> circle_table;
  double d_radius_m = d_padding_m + d_cost_dist_m;
  int n_R_px = static_cast<int>( (d_radius_m + 1e-5) / d_resolution_m ) + 1;
  for (int i = -n_R_px; i <= n_R_px; ++i)
  {
    for (int j = -n_R_px; j <= n_R_px; ++j)
    {
      GridPos st_grid_pos(i, j);
      int n_length_sq_px = st_grid_pos.n_x_px*st_grid_pos.n_x_px + st_grid_pos.n_y_px*st_grid_pos.n_y_px;
      double d_length_sq_px = static_cast<double>(n_length_sq_px);
      double d_length_m = std::sqrt(d_length_sq_px) * d_resolution_m;
      if (st_grid_pos.n_x_px == 0 && st_grid_pos.n_y_px == 0) {
        st_grid_pos.n_cost = GRID_STATE::OCCUPIED;
        circle_table.emplace_back(st_grid_pos);
      }
      else if (d_length_m < d_padding_m + 1e-5) {
        st_grid_pos.n_cost = GRID_STATE::PADDING;
        circle_table.emplace_back(st_grid_pos);
      }
      else if (d_length_m < d_padding_m + d_cost_dist_m + 1e-5) {
        st_grid_pos.n_cost = CalcCost(d_length_m - d_padding_m, d_cost_dist_m);
        if (st_grid_pos.n_cost != 0) {
          circle_table.emplace_back(st_grid_pos);
        }
      }
    }
  }
  return circle_table;
}

inline bool IsInvalid(const MapInfo_t& st_map_info)
{
  if (st_map_info.n_size_x_px < 1 || st_map_info.n_size_y_px < 1 || st_map_info.d_resolution_m < 0.0005) {
    return true;
  }
  return false;
}

inline bool IsInvalid(const std::vector<int8_t>& map, const MapInfo_t& st_map_info)
{
  if (static_cast<int>( map.size() ) != st_map_info.n_size_x_px * st_map_info.n_size_y_px || map.empty()) {
    return true;
  }
  return false;
}

inline int8_t DownSamplePixel(
  const std::vector<int8_t>& raw_map, const MapInfo_t& st_raw_map_info,
  int n_x_px, int n_y_px, const MapInfo_t& st_new_map_info)
{
  double d_half_res_m = 0.5 * st_new_map_info.d_resolution_m;
  double d_x_m = static_cast<double>(n_x_px) * st_new_map_info.d_resolution_m - 0.5 * st_new_map_info.d_size_x_m;
  double d_y_m = static_cast<double>(n_y_px) * st_new_map_info.d_resolution_m - 0.5 * st_new_map_info.d_size_y_m;
  int n_begin_x_px = CM::LUBnd(static_cast<int>((d_x_m - d_half_res_m + 0.5 * st_raw_map_info.d_size_x_m + 1e-6) / st_raw_map_info.d_resolution_m), 0, st_raw_map_info.n_size_x_px - 1);
  int n_end_x_px = CM::LUBnd(static_cast<int>((d_x_m + d_half_res_m + 0.5 * st_raw_map_info.d_size_x_m + 1e-6) / st_raw_map_info.d_resolution_m) + 1, 1, st_raw_map_info.n_size_x_px);
  int n_begin_y_px = CM::LUBnd(static_cast<int>((d_y_m - d_half_res_m + 0.5 * st_raw_map_info.d_size_y_m + 1e-6) / st_raw_map_info.d_resolution_m), 0, st_raw_map_info.n_size_y_px - 1);
  int n_end_y_px = CM::LUBnd(static_cast<int>((d_y_m + d_half_res_m + 0.5 * st_raw_map_info.d_size_y_m + 1e-6) / st_raw_map_info.d_resolution_m) + 1, 1, st_raw_map_info.n_size_y_px);
  int n_size_x_px = (n_end_x_px - n_begin_x_px);
  int n_sampling_size = n_size_x_px * (n_end_y_px - n_begin_y_px);
  int cnt=0;
  for (int n_idx = 0; n_idx < n_sampling_size; ++n_idx) {
    n_x_px = n_begin_x_px + n_idx % n_size_x_px;
    n_y_px = n_begin_y_px + n_idx / n_size_x_px;
    if (raw_map[n_x_px + n_y_px * st_raw_map_info.n_size_x_px] != GRID_STATE::EMPTY) {
      // ++cnt;
      // break;
      return GRID_STATE::OCCUPIED;
    }
  }
  float f_occ_ratio = static_cast<float>(cnt) / static_cast<float>(n_sampling_size);
  if(f_occ_ratio > 0.05f) {
    return GRID_STATE::OCCUPIED;
  }
  return GRID_STATE::EMPTY;
}

inline bool DownSampleMap(
  const std::vector<int8_t>& raw_map, MapInfo_t st_raw_map_info, double d_new_resolution_m,
  std::vector<int8_t>& new_map, MapInfo_t& st_new_map_info)
{
  // exceptions
  if (IsInvalid(st_raw_map_info)) {
    LOG_ERROR("[RemakeMap] Wrong raw map info : {}",
      st_raw_map_info.toStr().c_str());
    return false;
  }
  if (IsInvalid(raw_map, st_raw_map_info)) {
    LOG_ERROR("[RemakeMap] Wrong Map Config >> map size : {}, config : ({},{})",
      static_cast<int>(raw_map.size()),
      st_raw_map_info.n_size_x_px, st_raw_map_info.n_size_y_px);
    return false;
  }

  // check resolution
  st_raw_map_info.d_size_x_m = st_raw_map_info.n_size_x_px * st_raw_map_info.d_resolution_m;
  st_raw_map_info.d_size_y_m = st_raw_map_info.n_size_y_px * st_raw_map_info.d_resolution_m;
  st_new_map_info = st_raw_map_info;
  if (std::abs(st_raw_map_info.d_resolution_m - d_new_resolution_m) < 1e-5) {
    LOG_INFO("[RemakeMap]  Same resolution >> {}",
      st_new_map_info.toStr().c_str());
    new_map = raw_map;
    return true;
  }
  else if (st_raw_map_info.d_resolution_m > d_new_resolution_m) {
    LOG_INFO("[RemakeMap] Large resolution >> {}",
      st_new_map_info.toStr().c_str());
    new_map = raw_map;
    return true;
  }

  double d_res_ratio = d_new_resolution_m / st_raw_map_info.d_resolution_m - 1e-6;
  st_new_map_info.n_size_x_px = static_cast<int>(static_cast<double>(st_raw_map_info.n_size_x_px) / d_res_ratio);
  st_new_map_info.n_size_y_px = static_cast<int>(static_cast<double>(st_raw_map_info.n_size_y_px) / d_res_ratio);
  st_new_map_info.d_resolution_m = d_new_resolution_m;
  st_new_map_info.d_size_x_m = st_new_map_info.n_size_x_px * st_new_map_info.d_resolution_m;
  st_new_map_info.d_size_y_m = st_new_map_info.n_size_y_px * st_new_map_info.d_resolution_m;
  st_new_map_info.d_origin_x_m = st_raw_map_info.d_origin_x_m - 0.5 * st_raw_map_info.d_size_x_m + 0.5 * st_new_map_info.d_size_x_m;
  st_new_map_info.d_origin_y_m = st_raw_map_info.d_origin_y_m - 0.5 * st_raw_map_info.d_size_y_m + 0.5 * st_new_map_info.d_size_y_m;
  LOG_INFO("[RemakeMap] Small resolution >> {} -> {}",
    st_raw_map_info.toStr().c_str(), st_new_map_info.toStr().c_str());

  LOG_INFO("[RemakeMap] Downsaple map");
  int n_map_size = st_new_map_info.n_size_x_px * st_new_map_info.n_size_y_px;
  new_map.resize(n_map_size);
  for (int n_idx = 0; n_idx < n_map_size; ++n_idx) {
    new_map[n_idx] = DownSamplePixel(
      raw_map, st_raw_map_info,
      n_idx % st_new_map_info.n_size_x_px,
      n_idx / st_new_map_info.n_size_x_px,
      st_new_map_info);
  }

  return true;
}

inline bool CropMap(
  const std::vector<int8_t>& raw_map, const MapInfo_t& st_raw_map_info,
  int n_begin_x_px, int n_end_x_px, int n_begin_y_px, int n_end_y_px,
  std::vector<int8_t>& croped_map)
{
  // exceptions
  if (n_begin_x_px >= n_end_x_px || n_begin_y_px >= n_end_y_px) {
    LOG_ERROR("[CropMap] Wrong index data ({},{},{},{})",
      n_begin_x_px, n_end_x_px, n_begin_y_px, n_end_y_px);
    return false;
  }

  int n_real_begin_x_px = CM::LUBnd(n_begin_x_px, 0, st_raw_map_info.n_size_x_px - 1);
  int n_real_end_x_px = CM::LUBnd(n_end_x_px, 1, st_raw_map_info.n_size_x_px);
  int n_real_begin_y_px = CM::LUBnd(n_begin_y_px, 0, st_raw_map_info.n_size_y_px - 1);
  int n_real_end_y_px = CM::LUBnd(n_end_y_px, 1, st_raw_map_info.n_size_y_px);
  int n_jump_x_px = n_real_begin_x_px - n_begin_x_px;
  int n_crop_size_x_px = (n_end_x_px - n_begin_x_px);
  croped_map = std::vector<int8_t>(n_crop_size_x_px * (n_end_y_px - n_begin_y_px), GRID_STATE::OCCUPIED);

  for (int n_y_px = n_real_begin_y_px; n_y_px < n_real_end_y_px; ++n_y_px) {
    std::copy(
      raw_map.begin() + n_y_px * st_raw_map_info.n_size_x_px + n_real_begin_x_px,
      raw_map.begin() + n_y_px * st_raw_map_info.n_size_x_px + n_real_end_x_px,
      croped_map.begin() + (n_y_px - n_begin_y_px) * n_crop_size_x_px + n_jump_x_px
    );
  }

  return true;
}

}
namespace CGM = GridMapCalculator;
} // namespace NVFR

#endif
