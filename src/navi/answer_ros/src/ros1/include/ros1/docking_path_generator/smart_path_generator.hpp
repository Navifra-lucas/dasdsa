#ifndef SMART_PATH_GENERATOR_HPP_
#define SMART_PATH_GENERATOR_HPP_

#include "utils/pose.hpp"
#include "utils/common_math.hpp"
#include "utils/motion_planner_calculator.hpp"

#include "path_planner/edge_planner/edge_generator.hpp"

#include "ros1/docking_path_generator/dpg_param.hpp"

namespace NVFR {

namespace SPG {

/**
 * @brief calculate virtual start pose
 * @param o_start_pose start position (x,y)
 * @param o_goal_pose goal pose (x,y,theta)
 * @return pose
 */
Pose CalculateVirtualStartPose(
    const Pose& o_start_pose,
    const Pose& o_goal_pose,
    const DPG_Param_t& st_dpg_param);

/**
 * @brief generate path (line), the foot of the perpendicular pose -> goal pose
 * @param o_start_pose start position (x,y)
 * @param o_goal_pose goal pose (x,y,theta)
 * @return path
 */
PathResult GenerateStraightPath(
    const Pose& o_start_pose,
    const Pose& o_goal_pose,
    const DPG_Param_t& st_dpg_param);

/**
 * @brief generate path (line | arc + line), start position -> goal pose
 * @param o_start_pose start position (x,y)
 * @param o_goal_pose goal pose (x,y,theta)
 * @return path
 */
PathResult GenerateEnterPath(
    const Pose& o_start_pose,
    const Pose& o_goal_pose,
    const DPG_Param_t& st_dpg_param);

}  // namespace SPG

}  // namespace NVFR

#endif
