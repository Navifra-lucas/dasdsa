/*
 * @file	: astar_wfh.hpp
 * @date	: Feb 7, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Path Manager for current path state
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef GLOBAL_PLANNER_ASTAR_WFH_HPP_
#define GLOBAL_PLANNER_ASTAR_WFH_HPP_

#include "path_planner/global_planner/astar.hpp"

namespace NVFR {

namespace GlobalPlanner {

class AstarWFH : public Astar
{
public:
  explicit AstarWFH(
    const std::vector<int8_t>& map,
    const MapInfo_t& st_map_info,
    const Padding_t& st_padding,
    const RobotConfig& o_robot_config,
    const GlobalPlannerParam_t& st_param);
  virtual ~AstarWFH() = default;

  virtual std::tuple< bool, std::vector<Node>, Path > Plan(
    const Pose& o_start_pose, const Pose& o_goal_pose) override;

protected:

};

} // namespace GlobalPlanner

} // namespace NVFR

#endif
