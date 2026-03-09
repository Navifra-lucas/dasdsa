
/*
 * @file	: map_info.hpp
 * @date	: Feb. 24, 2025
 * @author	: "HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: grid map (map_info, map)
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_GRID_MAP_MAP_INFO_HPP_
#define NAVIFRA_GRID_MAP_MAP_INFO_HPP_

#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include <vector>
#include <cmath>
#include <stdint.h>

#include "utils/pose.hpp"
#include "utils/motion_planner_calculator.hpp"

namespace NVFR {

/**
 * @note type: int8_t
 * @param UNKNOWN, EMPTY, PADDING, OCCUPIED
*/
enum GRID_STATE : int8_t {
  UNKNOWN = -1,
  EMPTY = 0,
  // COST 1 ~ 98
  PADDING = 99,
  OCCUPIED = 100,
};

struct MapInfo_t
{
  int n_size_x_px = 0;
  int n_size_y_px = 0;
  double d_resolution_m = 0.0;

  double d_size_x_m = 0.0;
  double d_size_y_m = 0.0;
  double d_origin_x_m = 0.0;
  double d_origin_y_m = 0.0;

  bool operator==(const MapInfo_t& rhs) {
    return (rhs.n_size_x_px == n_size_x_px &&
            rhs.n_size_y_px == n_size_y_px &&
            std::fabs(rhs.d_resolution_m - d_resolution_m) < 1e-5);
  }
  bool operator!=(const MapInfo_t& rhs) {
    return !(rhs.n_size_x_px == n_size_x_px &&
            rhs.n_size_y_px == n_size_y_px &&
            std::fabs(rhs.d_resolution_m - d_resolution_m) < 1e-5);
  }

  std::string toStr() const;
  friend std::ostream& operator<<(std::ostream& os, const MapInfo_t& map_info);
};

struct Padding_t
{
  double d_padding_m = 0.0;
  double d_cost_dist_m = 0.0;

  bool operator==(const Padding_t& rhs) {
    return (std::fabs(rhs.d_padding_m - d_padding_m) < 1e-5 &&
            std::fabs(rhs.d_cost_dist_m - d_cost_dist_m) < 1e-5);
  }
  bool operator!=(const Padding_t& rhs) {
    return !(std::fabs(rhs.d_padding_m - d_padding_m) < 1e-5 &&
            std::fabs(rhs.d_cost_dist_m - d_cost_dist_m) < 1e-5);
  }

  std::string toStr() const;
  friend std::ostream& operator<<(std::ostream& os, const Padding_t& padding);
};

inline std::string MapInfo_t::toStr() const
{
  std::ostringstream oss;
  oss << std::fixed;
  oss.precision(2);
  oss <<
  "(" << n_size_x_px << "," << n_size_y_px << "," << d_resolution_m <<
  " | " << d_size_x_m << "," << d_size_y_m <<
  "," << d_origin_x_m << "," << d_origin_y_m <<
  ")";
  return oss.str();
}

inline std::ostream& operator<<(std::ostream& os, const MapInfo_t& map_info)
{
  std::cout << std::fixed;
  std::cout.precision(2);
  os <<
  "(" << map_info.n_size_x_px << "," << map_info.n_size_y_px << "," << map_info.d_resolution_m <<
  " | " << map_info.d_size_x_m << "," << map_info.d_size_y_m <<
  "," << map_info.d_origin_x_m << "," << map_info.d_origin_y_m <<
  ")";
  std::cout.unsetf(std::ios::fixed);
  return os;
}

inline std::string Padding_t::toStr() const
{
  std::ostringstream oss;
  oss << std::fixed;
  oss.precision(2);
  oss <<
  "(" << d_padding_m << "," << d_cost_dist_m << ")";
  return oss.str();
}

inline std::ostream& operator<<(std::ostream& os, const Padding_t& padding)
{
  std::cout << std::fixed;
  std::cout.precision(2);
  os <<
  "(" << padding.d_padding_m << "," << padding.d_cost_dist_m << ")";
  std::cout.unsetf(std::ios::fixed);
  return os;
}

} // namespace NVFR

#endif
