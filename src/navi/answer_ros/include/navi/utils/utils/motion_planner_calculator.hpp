
/*
 * @file	: motion_planner_calculator.hpp
 * @date	: Mar. 4, 2024
 * @author	: "HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: (pos,vel) calculator for motion controller, path planner and decision maker
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_MOTION_PLANNER_CALCULATOR_HPP_
#define NAVIFRA_MOTION_PLANNER_CALCULATOR_HPP_

#include <cmath>
#include <vector>

#include "utils/common_math.hpp"
#include "utils/pose.hpp"

namespace NVFR {
namespace MotionPlannerCalculator{

/**
 * @brief calculate curvature
 * @param dyaw difference from the angle of prev pos to the angle next pos [rad]
 * @param ds length from prev pos to next pos [m]
 * @return curvature (k) [double, 1/m]
*/
inline double CalcCurvature(double dyaw, double ds)
{
  return CM::NormRad(dyaw) / ds;
}

/**
 * @brief calculate curvature
 * @param o_prev_pos prev position
 * @param o_next_pos next position
 * @return curvature (k) [double, rad]
*/
inline double CalcCurvature(const Pose& o_prev_pos, const Pose& o_next_pos)
{
  return CM::NormRad(o_next_pos.GetRad() - o_prev_pos.GetRad()) / CM::Length(o_next_pos.GetXm() - o_prev_pos.GetXm(), o_next_pos.GetYm() - o_prev_pos.GetYm());
}

inline double CalcDistance(const Pos& o_p1, const Pos& o_p2)
{
  return CM::Length(o_p1.GetXm() - o_p2.GetXm(), o_p1.GetYm() - o_p2.GetYm());
}

inline double CalcDistance(const Pos& o_p1, const Pose& o_p2)
{
  return CM::Length(o_p1.GetXm() - o_p2.GetXm(), o_p1.GetYm() - o_p2.GetYm());
}

inline double CalcDistance(const Pose& o_p1, const Pos& o_p2)
{
  return CM::Length(o_p1.GetXm() - o_p2.GetXm(), o_p1.GetYm() - o_p2.GetYm());
}

inline double CalcDistance(const Pose& o_p1, const Pose& o_p2)
{
  return CM::Length(o_p1.GetXm() - o_p2.GetXm(), o_p1.GetYm() - o_p2.GetYm());
}

inline double CalcAtan2(const Pos& o_p1, const Pos& o_p2)
{
  return std::atan2(o_p2.GetYm() - o_p1.GetYm(), o_p2.GetXm() - o_p1.GetXm());
}

inline double CalcAtan2(const Pos& o_p1, const Pose& o_p2)
{
  return std::atan2(o_p2.GetYm() - o_p1.GetYm(), o_p2.GetXm() - o_p1.GetXm());
}

inline double CalcAtan2(const Pose& o_p1, const Pos& o_p2)
{
  return std::atan2(o_p2.GetYm() - o_p1.GetYm(), o_p2.GetXm() - o_p1.GetXm());
}

inline double CalcAtan2(const Pose& o_p1, const Pose& o_p2)
{
  return std::atan2(o_p2.GetYm() - o_p1.GetYm(), o_p2.GetXm() - o_p1.GetXm());
}

inline Pos TransformRef(const Pos& o_origin_pos, const Pos& o_pos)
{
  return 2 * o_origin_pos - o_pos;
}

inline Pose TransformRef(const Pose& o_origin_pos, const Pose& o_pos)
{
  return 2 * o_origin_pos - o_pos;
}

inline Pos TransformRef(const Pose& o_origin_pos, const Pos& o_pos)
{
  return 2 * o_origin_pos.GetPos() - o_pos;
}

inline Pose TransformRef(const Pos& o_origin_pos, const Pose& o_pos)
{
  return 2 * Pose(o_origin_pos) - o_pos;
}

/**
 * @brief calculate transformation (local to global)
 * @param o_origin_pos global point of the local origin
 * @param o_local_pos local point based on [o_origin_pos]
 * @return global point of [o_local_pos]
*/
inline Pose TransformL2G(const Pose& o_origin_pos, const Pose& o_local_pos)
{
  Pose result = o_local_pos;
  result.SetXm(o_origin_pos.GetXm() + o_local_pos.GetXm() * std::cos(o_origin_pos.GetRad()) - o_local_pos.GetYm() * std::sin(o_origin_pos.GetRad()));
  result.SetYm(o_origin_pos.GetYm() + o_local_pos.GetXm() * std::sin(o_origin_pos.GetRad()) + o_local_pos.GetYm() * std::cos(o_origin_pos.GetRad()));
  result.SetRad(o_origin_pos.GetRad() + o_local_pos.GetRad());
  return result;
}

/**
 * @brief calculate transformation (global to local)
 * @param o_origin_pos global point of the local origin
 * @param o_global_pos global point for transformation
 * @return local point of [o_local_pos] based on [o_origin_pos]
*/
inline Pose TransformG2L(const Pose& o_origin_pos, const Pose& o_global_pos)
{
  Pose result = o_global_pos;
  result.SetXm( (o_global_pos.GetXm() - o_origin_pos.GetXm()) * std::cos(o_origin_pos.GetRad()) + (o_global_pos.GetYm() - o_origin_pos.GetYm()) * std::sin(o_origin_pos.GetRad()));
  result.SetYm(-(o_global_pos.GetXm() - o_origin_pos.GetXm()) * std::sin(o_origin_pos.GetRad()) + (o_global_pos.GetYm() - o_origin_pos.GetYm()) * std::cos(o_origin_pos.GetRad()));
  result.SetRad(o_global_pos.GetRad() - o_origin_pos.GetRad());
  return result;
}

/**
 * @brief calculate longitudinal (x) length
 * @param o_origin_pos global point of the local origin
 * @param o_global_pos global point for transformation
 * @return longitudinal length of [o_local_pos] based on [o_origin_pos]
*/
inline double CalcLongitudinalLength(const Pose& o_origin_pos, const Pose& o_global_pos)
{
  return (o_global_pos.GetXm() - o_origin_pos.GetXm()) * std::cos(o_origin_pos.GetRad()) + (o_global_pos.GetYm() - o_origin_pos.GetYm()) * std::sin(o_origin_pos.GetRad());
}

/**
 * @brief calculate lateral (y) length
 * @param o_origin_pos global point of the local origin
 * @param o_global_pos global point for transformation
 * @return lateral length of [o_local_pos] based on [o_origin_pos]
*/
inline double CalcLateralLength(const Pose& o_origin_pos, const Pose& o_global_pos)
{
  return (o_global_pos.GetYm() - o_origin_pos.GetYm()) * std::cos(o_origin_pos.GetRad()) - (o_global_pos.GetXm() - o_origin_pos.GetXm()) * std::sin(o_origin_pos.GetRad());
}

/**
 * @brief rotate o_pos based on o_origin_pos
 * @param o_origin_pos center pos of rotation
 * @param o_pos pos before rotation
 * @param d_angle_rad rotation angle [rad]
 * @return rotated pos
*/
inline Pose RotatePose(const Pose& o_origin_pos, const Pose& o_pos, double d_angles_rad)
{
  Pose result = o_pos;
  result.SetXm( o_origin_pos.GetXm() + (o_pos.GetXm() - o_origin_pos.GetXm()) * std::cos(d_angles_rad) - (o_pos.GetYm() - o_origin_pos.GetYm()) * std::sin(d_angles_rad) );
  result.SetYm( o_origin_pos.GetYm() + (o_pos.GetXm() - o_origin_pos.GetXm()) * std::sin(d_angles_rad) + (o_pos.GetYm() - o_origin_pos.GetYm()) * std::cos(d_angles_rad) );
  result.SetRad(o_pos.GetRad() + d_angles_rad);
  return result;
}

/**
 * @brief find closest waypoint
 * @param path
 * @param o_robot_pos robot pos
 * @param n_start_idx (default: 0)
 * @return closest waypoint index [N]
*/
int FindClosestWaypointIdx(const Path& path, const Pose& o_robot_pos, int n_start_idx=0);

/**
 * @brief find closest waypoint and distance
 * @param path
 * @param o_robot_pos robot pos
 * @param d_closest_dist (&) distance from robot to closest waypoint
 * @param n_start_idx (default: 0)
 * @return closest waypoint index [N] and distance [m]
*/
int FindClosestWaypointIdxWithDist(const Path& path, const Pose& o_robot_pos, double& d_closest_dist, int n_start_idx=0);

/**
 * @brief find closest waypoint arround previous waypoint idx
 * @param path
 * @param o_robot_pos robot pos
 * @param n_start_idx (no default var)
 * @param n_end_idx range [n_start_idx - n_end_idx) (default: -1)
 * @return closest waypoint index [N]
*/
int FindClosestWaypointIdxOnRange(const Path& path, const Pose& o_robot_pos, int n_start_idx, int n_end_idx);

/**
 * @brief find closest waypoint arround previous waypoint idx
 * @param path
 * @param o_robot_pos robot pos
 * @param d_closest_dist (&) distance from robot to closest waypoint
 * @param n_start_idx (no default var)
 * @param n_end_idx range [n_start_idx - n_end_idx) (default: -1)
 * @return closest waypoint index [N] and distance [m]
*/
int FindClosestWaypointIdxOnRangeWithDist(const Path& path, const Pose& o_robot_pos, double& d_closest_dist, int n_start_idx, int n_end_idx);

/**
 * @brief find lookahead waypoint from the index
 * @param path
 * @param n_from_idx start index
 * @param d_lhd (lookahead distance)
 * @return lookahead waypoint index [N]
*/
int FindLookaheadWaypointIdx(const Path& path, int n_from_idx, double d_lhd);

/**
 * @brief find reverse lookahead from the index
 * @param path
 * @param n_from_idx start index
 * @param d_lhd (lookahead distance)
 * @return reverse lookahead waypoint index [N]
*/
int FindRevLookaheadWaypointIdx(const Path& path, int n_from_idx, double d_lhd);

/**
 * @brief calculate remain length from the robot to the final waypoint
 * @param path
 * @param o_robot_pos robot pos
 * @param n_closest_idx closest index (0 <= n_closest_idx < size of path)
 * @return length from the robot to the final waypoint [m]
*/
double CalcRemainLength(const Path& path, const Pose& o_robot_pos, int n_closest_idx);

/**
 * @brief calculate remain length from robot to final waypoint
 * @param path
 * @param o_robot_pos robot pos
 * @return reverse lookahead waypoint index [N]
*/
inline double CalcRemainLength(const Path& path, const Pose& o_robot_pos)
{
  return CalcRemainLength(path, o_robot_pos, FindClosestWaypointIdx(path, o_robot_pos));
}

} // namespace MotionPlannerCalculator
namespace CMP = MotionPlannerCalculator;
} // namespace NVFR

#endif
