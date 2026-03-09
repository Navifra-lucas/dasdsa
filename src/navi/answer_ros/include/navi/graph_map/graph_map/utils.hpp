/*
 * @file	: utils.hpp
 * @date	: Nov 6, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: utils for graph map
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef GRAPH_MAP_UTILS_HPP_
#define GRAPH_MAP_UTILS_HPP_

#include <cmath>
#include <vector>
#include <string>

#include "utils/pose.hpp"
#include "utils/common_math.hpp"
#include "utils/motion_planner_calculator.hpp"
#include "utils/motion_planner_type_list.hpp"

#include "graph_map/graph_node.hpp"

namespace NVFR {

inline double CalcDistanceFromLineSegment(const Pose& o_pose, const Pose& o_start_pose, const Pose& o_end_pose)
{
  double d_distance_m = 1E6;

  double d_yaw = CMP::CalcAtan2(o_start_pose, o_end_pose);
  double d_start_end_dist_m = CMP::CalcDistance(o_start_pose, o_end_pose);

  Pose o_local_pose = CMP::TransformG2L(o_start_pose, o_pose);

  if (o_local_pose.GetXm() < 0.0) {
    d_distance_m = CM::Length(o_local_pose.GetXm(), o_local_pose.GetYm());
  }
  else if (o_local_pose.GetXm() > d_start_end_dist_m) {
    double d_dx = o_local_pose.GetXm() - d_start_end_dist_m;
    d_distance_m = CM::Length(d_dx, o_local_pose.GetYm());
  }
  else {
    d_distance_m = std::abs(o_local_pose.GetYm());
  }

  return d_distance_m;
}

inline double CalcDistanceFromPath(const Pose& o_pose, const Path& path)
{
  double d_min_dist_m = 1E6;
  for (size_t i=1; i < path.size(); ++i) {
    double d_dist_m = CalcDistanceFromLineSegment(o_pose, path[i-1], path[i]);
    if (d_min_dist_m > d_dist_m) {
      d_min_dist_m = d_dist_m;
    }
  }
  return d_min_dist_m;
}

struct MNode {
  enum class STATUS {
    NONE,
    VISIT,
    CLOSED,
  } e_status = STATUS::NONE;
  int n_parent_node_num = -1;
  double d_gcost = 0.f;
  double d_hcost = 0.f;
};

} // namespace NVFR

#endif
