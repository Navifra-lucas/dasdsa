
/*
 * @file	: navigator_param.hpp
 * @date	: Mar. 21, 2025
 * @author	: "HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: navigator parameters
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_NAVIGATOR_PARAM_HPP_
#define NAVIFRA_NAVIGATOR_PARAM_HPP_

#include "sub/gridmap_param.hpp"
#include "sub/robot_config_param.hpp"
#include "sub/global_planner_param.hpp"
#include "sub/pcl_cluster_param.hpp"
#include "sub/local_planner_param.hpp"
#include "sub/motion_param.hpp"

namespace NVFR {

struct NavigatorParam_t
{
  // sub param
  RobotConfigParam_t st_robot_config_param;
  PclClusterParam_t st_pcl_cluster_param;
  GridMapParam_t st_grid_map_param;
  GlobalPlannerParam_t st_global_planner_param;
  LocalPlannerParam_t st_local_planner_param;
  MotionParam_t st_motion_param;
};

} // namespace NVFR

#endif
