/*
 * @file	: edge_array_generator.hpp
 * @date	: Feb 28, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Path Manager for current path state
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef EDGE_ARRAY_GENERATOR_HPP_
#define EDGE_ARRAY_GENERATOR_HPP_

#include <vector>

#include "utils/pose.hpp"
#include "utils/motion_planner_type_list.hpp"
#include "utils/mission_edge.hpp"
#include "utils/mission_bundle.hpp"

#include "path_planner/edge_planner/edge_generator.hpp"

namespace NVFR {

class EdgeArrayGenerator
{
public:
  EdgeArrayGenerator() {};
  virtual ~EdgeArrayGenerator() = default;

  static bool GeneratePath(
    MissionBundle_t& st_mission_bundle,
    double d_max_curv, double d_interval=0.01);

private:

};

} // namespace NVFR

#endif
