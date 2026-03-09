/*
 * @file	: mission_bundle.hpp
 * @date	: Dec 12, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: class for drive information
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MISSION_BUNDLE_HPP_
#define MISSION_BUNDLE_HPP_

#include <iostream>
#include <string>

#include "utils/motion_planner_type_list.hpp"
#include "utils/mission_info.hpp"
#include "utils/mission_edge.hpp"
#include "utils/pose.hpp"

namespace NVFR {

struct MissionBundle_t
{
  // uuid
  std::string uuid;
  // mission type
  MPTL::MISSION e_mission_type = MPTL::MISSION::IDLE;
  // drive info
  DriveInfo o_drive_info;

  // align info
  AlignInfo o_align_info;

  // node path info
  bool b_arrive_align=false; // use o_align_info
  std::vector<MissionEdge> vec_mission_edge;

  // auto path info
  //

  // search path info
  //

  // explore info
  //

  // path
  Path vec_mission_path;

};

} // namespace NVFR

#endif
