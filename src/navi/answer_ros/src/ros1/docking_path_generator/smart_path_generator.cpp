#include "ros1/docking_path_generator/smart_path_generator.hpp"

#include "logger/logger.h"

namespace NVFR {

namespace SPG {

Pose CalculateVirtualStartPose(
    const Pose& o_start_pose,
    const Pose& o_goal_pose,
    const DPG_Param_t& st_dpg_param)
{
    Pose o_local_start_pose =
        o_start_pose / o_goal_pose;
    Pose o_local_virtual_start_pose(
        o_local_start_pose.GetXm(), 0.0);
    Pose o_virtual_start_pose =
        o_local_virtual_start_pose * o_goal_pose;
    return o_virtual_start_pose;
}

PathResult GenerateStraightPath(
    const Pose& o_start_pose,
    const Pose& o_goal_pose,
    const DPG_Param_t& st_dpg_param)
{
    PathResult path_result;

    // calculate the perpendicular pose (new start pose)
    double d_local_new_start_x_m = 0.0;
    Pose o_new_start_pose;
    {
        Pose o_local_start_pose = o_start_pose / o_goal_pose;
        d_local_new_start_x_m = -1.0 * o_local_start_pose.GetXm();
        Pose o_local_new_start_pose(o_local_start_pose.GetXm(), 0.0);
        o_new_start_pose = o_local_new_start_pose * o_goal_pose;
    }

    // exceptions
    double d_short_dist_m = 0.1;
    if (d_local_new_start_x_m < d_short_dist_m) {
        LOG_ERROR("d_local_new_start_x_m : {:.3f} < {:.3f} [m]",
            d_local_new_start_x_m, d_short_dist_m);
        path_result.first = PathGenerator::RESULT::SHORT;
    }
    else
    {
        path_result = LineGenerator::GeneratePath(
            o_new_start_pose, o_goal_pose, 0.5);
    }

    return path_result;
}

PathResult GenerateEnterPath(
    const Pose& o_start_pose,
    const Pose& o_goal_pose,
    const DPG_Param_t& st_dpg_param)
{
    PathResult path_result;

    path_result = GenerateStraightPath(o_start_pose, o_goal_pose, st_dpg_param);

    return path_result;
}

}  // namespace SPG

}  // namespace NVFR
