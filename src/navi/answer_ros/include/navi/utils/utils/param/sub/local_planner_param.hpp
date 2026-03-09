/*
 * @file	: local_planner_param.hpp
 * @date	: Dec 20, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: purepersuit motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_LOCAL_PLANNER_PARAM_HPP_
#define NAVIFRA_LOCAL_PLANNER_PARAM_HPP_

namespace NVFR {

struct LocalPlannerParam_t
{
  // duration
  double d_replan_period_sec = 0.1; // [sec]
  double d_avoid_delay_sec = 0.1; // [sec]

  // local planner
  double d_path_front_dist_m = 10.0; // [m] maintain length of current path
  double d_path_back_dist_m = 0.5; // [m] remain back waypoint with this length in current path
  double d_safe_stop_dist_m = 2.0; // [m] safe stop distance (before collision)
  double d_final_no_avoid_dist_m = 1.0; // [m] no avoidance distance arround goal of reference path
  double d_collision_margin_m = 0.02; // [m] collision margin in local planner

};

} // namespace NVFR

#endif
