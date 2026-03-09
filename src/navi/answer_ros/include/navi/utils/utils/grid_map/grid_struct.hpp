
/*
 * @file	: grid_map_struct.hpp
 * @date	: Feb. 24, 2025
 * @author	: "HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: grid map (map_info, map)
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_GRID_MAP_GRID_STRUCT_HPP_
#define NAVIFRA_GRID_MAP_GRID_STRUCT_HPP_

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

struct GridPos
{
  GridPos():n_x_px(0),n_y_px(0),n_cost(0) {};
  GridPos(int _n_x_px, int _n_y_px):n_x_px(_n_x_px),n_y_px(_n_y_px),n_cost(0) {};
  GridPos(int _n_x_px, int _n_y_px, int8_t _n_cost):n_x_px(_n_x_px),n_y_px(_n_y_px),n_cost(_n_cost) {};
  int n_x_px;
  int n_y_px;
  int8_t n_cost;

  GridPos operator+(const GridPos& rhs) const;
  GridPos operator-(const GridPos& rhs) const;
  bool operator==(const GridPos& rhs) const;
  bool operator!=(const GridPos& rhs) const;

  std::string toStr() const;
  friend std::ostream& operator<<(std::ostream& os, const GridPos& p);
};

inline GridPos GridPos::operator+(const GridPos& rhs) const
{
  GridPos result;
  result.n_x_px = n_x_px + rhs.n_x_px;
  result.n_y_px = n_y_px + rhs.n_y_px;
  result.n_cost = n_cost + rhs.n_cost;
  return result;
}

inline GridPos GridPos::operator-(const GridPos& rhs) const
{
  GridPos result;
  result.n_x_px = n_x_px - rhs.n_x_px;
  result.n_y_px = n_y_px - rhs.n_y_px;
  result.n_cost = n_cost - rhs.n_cost;
  return result;
}

inline bool GridPos::operator==(const GridPos& rhs) const
{
  return (n_x_px == rhs.n_x_px) && (n_y_px == rhs.n_y_px);
}

inline bool GridPos::operator!=(const GridPos& rhs) const
{
  return (n_x_px != rhs.n_x_px) || (n_y_px != rhs.n_y_px);
}

inline std::string GridPos::toStr() const
{
  std::ostringstream oss;
  oss << "(grid px:" << n_x_px << "," << n_y_px << ")";
  return oss.str();
}

inline std::ostream& operator<<(std::ostream& os, const GridPos& p)
{
  os << "(grid px:" << p.n_x_px << "," << p.n_y_px << ")";
  return os;
}

} // namespace NVFR

#endif
