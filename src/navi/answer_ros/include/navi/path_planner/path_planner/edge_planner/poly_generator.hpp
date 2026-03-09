/*
 * @file	: poly_generator.hpp
 * @date	: Feb 7, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Path Manager for current path state
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef POLY_GENERATOR_HPP_
#define POLY_GENERATOR_HPP_

#include "utils/pose.hpp"
#include "utils/mission_edge.hpp"
#include "path_planner/edge_planner/path_generator.hpp"

namespace NVFR {

class PolyGenerator
{
public:
  PolyGenerator() {};
  virtual ~PolyGenerator() {};

  /**
   * @brief generate polynomial path
   * @param o_start_pos (x,y,yaw,curv)
   * @param o_end_pos (x,y,yaw,curv)
   * @param f_T_control [sec]
   * @param f_target_spd [m/s]
   * @param f_max_curv [1/m]
   */
  static PathResult GeneratePath(
    const Pose& o_start_pos, const Pose& o_end_pos,
    double d_T_control, double d_target_spd, double d_max_curv, double d_interval=0.01);

private:
  static void SetTimeDomain(double t, double* poly_T);
  static void SolvePoly(const Pose& P0, const Pose& P1, double T, double* poly_x, double* poly_y);
  static void CalcPos(const double* poly_T, const double* poly_x, const double* poly_y, Pose& Pt);
  static void CalcSpd(const double* poly_T, const double* poly_x, const double* poly_y, Pose& Vt);
  static void CalcAcc(const double* poly_T, const double* poly_x, const double* poly_y, Pose& At);

};

} // namespace NVFR

#endif
