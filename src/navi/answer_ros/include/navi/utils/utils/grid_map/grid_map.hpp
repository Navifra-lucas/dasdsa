
/*
 * @file	: grid_map.hpp
 * @date	: Feb. 24, 2025
 * @author	: "HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: grid map (map_info, map)
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_GRID_MAP_GRID_MAP_HPP_
#define NAVIFRA_GRID_MAP_GRID_MAP_HPP_

#include <iostream>
#include <memory>
#include <vector>
#include <mutex>
#include <shared_mutex>

#include "utils/pose.hpp"
#include "utils/grid_map/map_info.hpp"
#include "utils/grid_map/grid_map_calculator.hpp"

namespace NVFR {

typedef std::shared_ptr< std::vector<int8_t> > MapPtr;

class GridMap
{
public:
  struct Data_t {
    const MapPtr grid_map_ptr;
    const Pose o_center_pose;
    const MapInfo_t st_map_info;
    const Padding_t st_padding;
  };
  GridMap() {};
  virtual ~GridMap() = default;

  GridMap::Data_t GetGridMap() const;

  bool SetPadding(Padding_t st_padding);

protected:
  MapInfo_t st_map_info_;
  Padding_t st_padding_;
  std::vector<GridPos> table_;
  mutable std::mutex mtx_param_;

  MapPtr grid_map_ptr_ = nullptr;
  Pose o_center_pose_;
  mutable std::shared_mutex smtx_grid_map_;

  bool UpdatePadding();

};

} // namespace NVFR

#endif
