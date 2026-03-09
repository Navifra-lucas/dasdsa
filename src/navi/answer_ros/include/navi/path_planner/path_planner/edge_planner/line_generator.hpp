/*
 * @file	: line_generator.hpp
 * @date	: Feb 7, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Path Manager for current path state
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef LINE_GENERATOR_HPP_
#define LINE_GENERATOR_HPP_

#include "utils/pose.hpp"
#include "utils/mission_edge.hpp"
#include "path_planner/edge_planner/path_generator.hpp"

namespace NVFR {

class LineGenerator
{
public:
  LineGenerator() {};
  virtual ~LineGenerator() {};

  /**
   * @brief generate straight line path
   * @param o_start_pos (x,y)
   * @param o_end_pos (x,y)
   * @param d_target_spd [m/s]
   */
  static PathResult GeneratePath(
    const Pose& o_start_pos, const Pose& o_end_pos,
    double d_target_spd, double d_interval=0.01);

private:

};

} // namespace NVFR

#endif
