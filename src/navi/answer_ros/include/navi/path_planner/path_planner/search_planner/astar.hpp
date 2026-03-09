/*
 * @file	: astar.hpp
 * @date	: Feb 7, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Path Manager for current path state
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef GLOBAL_PLANNER_ASTAR_HPP_
#define GLOBAL_PLANNER_ASTAR_HPP_

#include "path_planner/search_planner/search_planner.hpp"

namespace NVFR {

class Astar : public SearchPlanner
{
public:
  explicit Astar(
    const std::vector<int8_t>& map,
    const MapInfo_t& st_map_info,
    const Padding_t& st_padding,
    const RobotConfig& o_robot_config,
    const SearchPlanner::Param_t& st_param);
  virtual ~Astar() = default;

  virtual std::tuple< bool, std::vector<Node>, Path > Plan(
    const Pose& o_start_pose, const Pose& o_goal_pose) override;

protected:
  std::vector<Node> nodes_;

  virtual std::tuple< bool, std::vector<Node>, Path > MakePath(
    const Node* goal_node) const override;

  bool CheckCollision(
    const Node& node, double d_margin_m,
    bool b_backward=false) const;

};

} // namespace NVFR

#endif
