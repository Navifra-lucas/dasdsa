/*
 * @file	: global_planner_param.hpp
 * @date	: Dec 20, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: purepersuit motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_GLOBAL_PLANNER_PARAM_HPP_
#define NAVIFRA_GLOBAL_PLANNER_PARAM_HPP_

namespace NVFR {

struct GlobalPlannerParam_t
{
  // contraints
  double d_max_curv = 3.0; // [1/m]

  // global planner
  double d_collision_margin_m = 0.05; // [m] collision margin in global planner

};

} // namespace NVFR

#endif
