/*
 * @file	: hybrid_astar.hpp
 * @date	: Feb 7, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Path Manager for current path state
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef GLOBAL_PLANNER_HYBRID_ASTAR_HPP_
#define GLOBAL_PLANNER_HYBRID_ASTAR_HPP_

#include "path_planner/global_planner/global_planner.hpp"

namespace NVFR {

namespace GlobalPlanner {

class HybridAstar : public Planner
{
public:
  explicit HybridAstar(
    const std::vector<int8_t>& map,
    const MapInfo_t& st_map_info,
    const Padding_t& st_padding,
    const RobotConfig& o_robot_config,
    const GlobalPlannerParam_t& st_param,
    int n_theta_size = 36);
  virtual ~HybridAstar() = default;

  virtual std::tuple< bool, std::vector<Node>, Path > Plan(
    const Pose& o_start_pose, const Pose& o_goal_pose) override;

protected:
  const int THETA_SIZE;
  const float th2rad;
  const std::vector<float> GRID_COS;
  const std::vector<float> GRID_SIN;

  const std::vector<HNode> h_movements_;

  std::vector<HNode> nodes_;

  virtual std::tuple< bool, std::vector<Node>, Path > MakePath(
    const Node* goal_node) const override;

  HNode NextNode(const HNode& curr_node, const HNode& origin_curr_node,
    const HNode& movement) const;

  bool IsNearGoalNode(const HNode* curr_node, const HNode* origin_goal_node) const;

  bool CheckSpinCollision(const HNode& node, float f_margin_m) const;
  bool CheckCollision(const HNode& node, float f_margin_m, bool b_backward=false) const;

private:
  std::vector<float> CreateCOS(const int theta_size, const float dtheta) const;
  std::vector<float> CreateSIN(const int theta_size, const float dtheta) const;
  std::vector<HNode> CreateMovements(float dtheta) const;

};

} // namespace GlobalPlanner

} // namespace NVFR

#endif
