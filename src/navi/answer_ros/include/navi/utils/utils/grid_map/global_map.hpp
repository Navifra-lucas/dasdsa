
/*
 * @file	: global_map.hpp
 * @date	: Feb. 24, 2025
 * @author	: "HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: grid map (map_info, map)
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_GRID_MAP_GLOBAL_MAP_HPP_
#define NAVIFRA_GRID_MAP_GLOBAL_MAP_HPP_

#include "grid_map.hpp"

namespace NVFR {

class GlobalMap : public GridMap
{
public:
  GlobalMap():GridMap() {};
  virtual ~GlobalMap() = default;

  bool SetResolution(double d_resolution_m);
  bool SetGridMap(const std::vector<int8_t>& raw_map, const MapInfo_t& st_raw_map_info);

private:

};

typedef std::shared_ptr<GlobalMap> GlobalMapPtr;

} // namespace NVFR

#endif
