
/*
 * @file	: local_map.hpp
 * @date	: Feb. 24, 2025
 * @author	: "HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: grid map (map_info, map)
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_GRID_MAP_LOCAL_MAP_HPP_
#define NAVIFRA_GRID_MAP_LOCAL_MAP_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "grid_map.hpp"

namespace NVFR {

class LocalMap : public GridMap
{
public:
  LocalMap():GridMap() {};
  virtual ~LocalMap() = default;

  bool SetMapInfo(MapInfo_t st_map_info);
  bool SetGridMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Pose& o_center_pose);

private:

};

typedef std::shared_ptr<LocalMap> LocalMapPtr;

} // namespace NVFR

#endif
