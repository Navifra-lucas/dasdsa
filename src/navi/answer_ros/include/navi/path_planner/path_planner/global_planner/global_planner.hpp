/*
 * @file	: global_planner.hpp
 * @date	: Feb 7, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Path Manager for current path state
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_HPP_
#define GLOBAL_PLANNER_GLOBAL_PLANNER_HPP_

#include <vector>
#include <queue>
#include <tuple>

#include "utils/param/sub/global_planner_param.hpp"
#include "utils/pose.hpp"
#include "utils/grid_map/grid_map.hpp"
#include "utils/debug_visualizer.hpp"

#include "path_planner/global_planner/utils/utils.hpp"

namespace NVFR {

namespace GlobalPlanner {

class Planner
{
public:
  explicit Planner(
    const std::vector<int8_t>& map,
    const MapInfo_t& st_map_info,
    const Padding_t& st_padding,
    const RobotConfig& o_robot_config,
    const GlobalPlannerParam_t& st_param,
    const std::vector<Node>& movements)
  : map_(map)
  , st_map_info_(st_map_info)
  , st_padding_(st_padding)
  , o_robot_config_(o_robot_config)
  , st_param_(st_param)
  , movements_(movements)
  {
    // exceptions
    int n_map_size = static_cast<int>(map_.size());
    if (n_map_size != st_map_info_.n_size_x_px * st_map_info_.n_size_y_px) {
      b_valid_data_ = false;
      LOG_ERROR("[Planner] map size: {}, map info: {}",
        n_map_size, st_map_info.toStr().c_str());
    }
    // else {
    //   DebugVisualizer::GetInstance()->PublishGlobalMap("planner_map", map, st_map_info);
    // }
  }
  virtual ~Planner() = default;

  virtual std::tuple< bool, std::vector<Node>, Path > Plan(
    const Pose& o_start_pose, const Pose& o_goal_pose) = 0;

protected:
  std::vector<int8_t> map_;
  const MapInfo_t st_map_info_;
  const Padding_t st_padding_;

  const RobotConfig o_robot_config_;

  const GlobalPlannerParam_t st_param_;

  const std::vector<Node> movements_;

  bool b_valid_data_ = true;

  virtual std::tuple< bool, std::vector<Node>, Path > MakePath(
    const Node* goal_node) const = 0;

};

} // namespace GlobalPlanner

} // namespace NVFR

#endif
