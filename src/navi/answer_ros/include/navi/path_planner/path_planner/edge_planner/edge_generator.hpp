/*
 * @file	: edge_generator.hpp
 * @date	: Feb 7, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Path Manager for current path state
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef EDGE_GENERATOR_HPP_
#define EDGE_GENERATOR_HPP_

#include "utils/pose.hpp"
#include "utils/mission_edge.hpp"

#include "path_planner/edge_planner/path_generator.hpp"
#include "path_planner/edge_planner/line_generator.hpp"
#include "path_planner/edge_planner/arc_generator.hpp"
#include "path_planner/edge_planner/bezier_generator.hpp"
#include "path_planner/edge_planner/poly_generator.hpp"

namespace NVFR {

class EdgeGenerator
{
public:
  EdgeGenerator() {};
  virtual ~EdgeGenerator() {};

  static PathResult GeneratePath(
    MissionEdge& o_mission_edge,
    double d_max_curv, double d_interval=0.01);

private:

};

} // namespace NVFR

#endif
