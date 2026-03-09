/*
 * @file	: global_planner_manager.hpp
 * @date	: Apr 21, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Decision Maker class for path planner and motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef GLOBAL_PLANNER_MANAGER_HPP_
#define GLOBAL_PLANNER_MANAGER_HPP_

#include <stdint.h>
#include <vector>
#include <memory>
#include <atomic>
#include <condition_variable>
#include <mutex>

#include "utils/debug_visualizer.hpp"
#include "utils/motion_planner_type_list.hpp"
#include "utils/pose.hpp"
#include "utils/robot_config.hpp"
#include "utils/mission_bundle.hpp"
#include "utils/grid_map/grid_map_storage.hpp"
#include "utils/param/sub/global_planner_param.hpp"

#include "path_planner/global_planner/global_planner_bundle.hpp"

namespace NVFR {

class GlobalPlannerManager : protected GridMapStorage
{
public:
  GlobalPlannerManager() = default;
  virtual ~GlobalPlannerManager() = default;

  bool ExploreMission(MissionBundle_t& st_mission_bundle, const GlobalPlannerParam_t& st_param) const;

private:
  GlobalPlanner::PathSmoother o_path_smoother_;

};

} // namespace NVFR

#endif
