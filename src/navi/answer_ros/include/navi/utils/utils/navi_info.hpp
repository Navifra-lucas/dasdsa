/*
 * @file	: navi_info.hpp
 * @date	: Aug 20, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: class for navi information (mission, motion, ect)
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVI_INFO_HPP_
#define NAVI_INFO_HPP_

#include <string>

#include "utils/motion_planner_type_list.hpp"
#include "utils/pose.hpp"
#include "utils/mission_info.hpp"

namespace NVFR {

class NaviInfo
{
public:
  // type
  MPTL::CONTROLLER    e_controller  = MPTL::CONTROLLER::MPC;
  MPTL::AVOID         e_avoid       = MPTL::AVOID::NONE;
  MPTL::MISSION       e_mission     = MPTL::MISSION::IDLE;
  MPTL::ALIGNDIR      e_align_dir   = MPTL::ALIGNDIR::AUTO;
  MPTL::EDGE_PATH     e_path_type   = MPTL::EDGE_PATH::LINE;

  // drive info
  DriveInfo o_drive_info;

  // mission nodes info
  NodeInfo o_goal_node;
  NodeInfo o_target_node;
  NodeInfo o_curr_node;
  NodeInfo o_next_node;

  // mission progress
  double  d_align_target_deg                = 0;
  double  d_remain_deg                      = 0;
  double  d_total_length_curr_to_next_m     = 0;
  double  d_total_length_start_to_target_m    = 0;
  double  d_traveled_length_curr_to_next_m  = 0;
  double  d_traveled_length_start_to_target_m = 0;

  // motion status
  bool  b_spin_turn       = false;
  bool  b_pause           = false;
  bool  b_avoid           = false;
  bool  b_stop            = false;
  bool  b_obstacle        = false;
  bool  b_collision       = false;
  bool  b_away_from_path  = false;
  int  n_align_speed_percent            = 0;
  int  n_linear_speed_percent           = 0;
  double  d_target_linear_vel_on_path_m = 0;

  // motion error
  Pose  o_pose_error;

private:
};

} // namespace NVFR

#endif
