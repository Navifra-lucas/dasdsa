
/*
 * @file	: grid_map_storage.hpp
 * @date	: Feb. 24, 2025
 * @author	: "HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: global map & local map handler
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_GRID_MAP_GRID_MAP_STORAGE_HPP_
#define NAVIFRA_GRID_MAP_GRID_MAP_STORAGE_HPP_

#include <iostream>
#include <vector>
#include <memory>
#include <mutex>
#include <shared_mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "global_map.hpp"
#include "local_map.hpp"

namespace NVFR {

class GridMapStorage
{
public:
  GridMapStorage() = default;
  virtual ~GridMapStorage() = default;

  GridMap::Data_t GetGlobalMap() const
  {
    return o_global_map_.GetGridMap();
  }
  GridMap::Data_t GetLocalMap() const
  {
    return o_local_map_.GetGridMap();
  }

protected:
  bool SetGlobalResolution(double d_resolution_m)
  {
    return o_global_map_.SetResolution(d_resolution_m);
  }
  bool SetGlobalPadding(Padding_t st_padding)
  {
    return o_global_map_.SetPadding(st_padding);
  }
  bool SetGlobalMap(const std::vector<int8_t>& map, const MapInfo_t& st_map_info)
  {
    return o_global_map_.SetGridMap(map, st_map_info);
  }
  bool SetLocalMapInfo(MapInfo_t st_map_info)
  {
    return o_local_map_.SetMapInfo(st_map_info);
  }
  bool SetLocalPadding(Padding_t st_padding)
  {
    return o_local_map_.SetPadding(st_padding);
  }
  bool SetLocalMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Pose& o_center_pose)
  {
    return o_local_map_.SetGridMap(cloud, o_center_pose);
  }

private:
  static GlobalMap o_global_map_;
  static LocalMap o_local_map_;

};

} // namespace NVFR

#endif
