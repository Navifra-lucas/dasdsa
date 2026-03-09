/*
 * @file	: arc_generator.hpp
 * @date	: Feb 7, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Path Manager for current path state
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef ARC_GENERATOR_HPP_
#define ARC_GENERATOR_HPP_

#include "utils/pose.hpp"
#include "utils/mission_edge.hpp"
#include "path_planner/edge_planner/path_generator.hpp"

namespace NVFR {

class ArcGenerator
{
public:
  ArcGenerator() {};
  virtual ~ArcGenerator() {};

  /**
   * @brief generate arc path
   * @param o_start_pos (x,y,yaw)
   * @param o_end_pos (x,y,yaw)
   * @param d_target_spd [m/s]
   * @param d_max_curv [1/m]
   */
  static PathResult GeneratePath(
    const Pose& o_start_pos, const Pose& o_end_pos,
    double d_target_spd, double d_max_curv, double d_interval=0.01);

private:
  /**
   * @note type: uint8_t
   * @param NO_ARC, ARC, START_LINE, END_LINE
  */
  enum ArcType : uint8_t
  {
    NO_ARC = 0,
    ARC = 1,
    START_LINE,
    END_LINE,
  };
  static ArcGenerator::ArcType CalcArc(const Pose& P0, const Pose& P1, Pose& Ps, Pose& Pc, double& R, double& angle_rad);

};

} // namespace NVFR

#endif
