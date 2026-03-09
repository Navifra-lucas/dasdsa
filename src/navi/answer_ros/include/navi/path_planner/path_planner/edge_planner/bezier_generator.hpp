/*
 * @file	: bezier_generator.hpp
 * @date	: Feb 7, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Path Manager for current path state
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef BEZIER_GENERATOR_HPP_
#define BEZIER_GENERATOR_HPP_

#include "utils/pose.hpp"
#include "utils/mission_edge.hpp"
#include "path_planner/edge_planner/path_generator.hpp"

namespace NVFR {

class BezierGenerator
{
public:
  BezierGenerator() {};
  virtual ~BezierGenerator() {};

  /**
   * @brief generate polynomial path
   * @param o_start_pos (x,y)
   * @param o_start_control (x,y)
   * @param o_end_pos (x,y)
   * @param o_end_control (x,y)
   * @param d_target_spd [m/s]
   * @param d_max_curv [1/m]
   */
  static PathResult GeneratePath(
    const Pose& o_start_pos, const Pose& o_start_control,
    const Pose& o_end_pos, const Pose& o_end_control,
    double d_target_spd, double d_max_curv, double d_interval=0.01);

private:
  static void CalcPos(const Pose& P0, const Pose& P1, const Pose& P2, const Pose& P3, double t, Pose& Pt);
  static void CalcSpd(const Pose& P0, const Pose& P1, const Pose& P2, const Pose& P3, double t, Pose& Vt);
  static void CalcAcc(const Pose& P0, const Pose& P1, const Pose& P2, const Pose& P3, double t, Pose& At);

};

} // namespace NVFR

#endif
