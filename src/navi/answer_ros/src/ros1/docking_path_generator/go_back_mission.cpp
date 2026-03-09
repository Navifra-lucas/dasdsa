#include "ros1/docking_path_generator/go_back_mission.hpp"

#include "logger/logger.h"

#include "utils/common_math.hpp"
#include "utils/motion_planner_calculator.hpp"

#include "path_planner/edge_planner/edge_generator.hpp"

namespace NVFR {

GoBackMission::GoBackMission()
    : st_dpg_param_()
{
    //
}

void GoBackMission::SetParam(const DPG_Param_t& st_dpg_param)
{
    LOG_DEBUG("set param");
    st_dpg_param_.Set(st_dpg_param);
}

void GoBackMission::SetNodeF(const Pose& o_pose)
{
    o_fNode_.Set(o_pose);
    LOG_DEBUG("[SetNodeF] {}, {}, {} [m,deg]",
        o_pose.GetXm(), o_pose.GetYm(), o_pose.GetDeg());
}

void GoBackMission::SetNodeN(const Pose& o_pose)
{
    o_nNode_.Set(o_pose);
    LOG_DEBUG("[SetNodeN] {}, {}, {} [m,deg]",
        o_pose.GetXm(), o_pose.GetYm(), o_pose.GetDeg());
}

void GoBackMission::SetNodeG(const Pose& o_pose)
{
    o_gNode_.Set(o_pose);
    LOG_DEBUG("[SetNodeG] {}, {}, {} [m,deg]",
        o_pose.GetXm(), o_pose.GetYm(), o_pose.GetDeg());
}

bool GoBackMission::GeneratePath(const Pose& o_robot_pose)
{
    // get param
    DPG_Param_t st_dpg_param = st_dpg_param_.Get();

    // get nodes
    Pose o_fNode = o_fNode_.Get();
    Pose o_nNode = o_nNode_.Get();
    Pose o_gNode = o_gNode_.Get();

    LOG_INFO("[GoBackMission] F node: ({:.3f}, {:.3f}) | N node: ({:.3f}, {:.3f}) | G node: ({:.3f}, {:.3f}) | robot pose: ({:.3f}, {:.3f}, {:.1f})",
        o_fNode.GetXm(), o_fNode.GetYm(),
        o_nNode.GetXm(), o_nNode.GetYm(),
        o_gNode.GetXm(), o_gNode.GetYm(),
        o_robot_pose.GetXm(), o_robot_pose.GetYm(), o_robot_pose.GetDeg());

    // exceptions
    if (Exceptions(o_fNode, o_nNode, o_gNode, o_robot_pose, st_dpg_param)) {
        LOG_ERROR("Exceptions: wrong nodes");
        return false;
    }

    o_gNode.SetRad(CMP::CalcAtan2(o_gNode, o_nNode));
    o_nNode.SetRad(o_gNode.GetRad());
    o_fNode.SetRad(CMP::CalcAtan2(o_nNode, o_fNode));

    PCM::Publish(PCM::KEY::RETURN_PATH,
        Path({o_gNode, o_nNode, o_fNode}));
    return true;
}

bool GoBackMission::Exceptions(
    const Pose& o_fNode,
    const Pose& o_nNode,
    const Pose& o_gNode,
    const Pose& o_robot_pose,
    const DPG_Param_t& st_dpg_param) const
{
    // check distance
    double d_distance_m = 0.0;
    // 1) N to F
    d_distance_m = CMP::CalcDistance(o_fNode, o_nNode);
    if (d_distance_m < 0.1)
    {
        LOG_ERROR("N node and F node are so closed ({:.3f})", d_distance_m);
        return true;
    }
    else if (d_distance_m > 10.0)
    {
        LOG_ERROR("N node and F node are so far ({:.3f})", d_distance_m);
        return true;
    }
    // 2) N to G
    d_distance_m = CMP::CalcDistance(o_nNode, o_gNode);
    if (d_distance_m < 0.1)
    {
        LOG_ERROR("N node and G node are so closed ({:.3f})", d_distance_m);
        return true;
    }
    else if (d_distance_m > 10.0)
    {
        LOG_ERROR("N node and G node are so far ({:.3f})", d_distance_m);
        return true;
    }

    // check the robot to be on the path
    // 1) N to F
    if (IsOnPath(o_nNode, o_fNode, o_robot_pose, st_dpg_param))
    {
        LOG_INFO("Robot is on the path between N node to F node");
    }
    // 2) N to G
    else if (IsOnPath(o_nNode, o_gNode, o_robot_pose, st_dpg_param))
    {
        LOG_INFO("Robot is on the path between N node to G node");
    }
    else
    {
        LOG_ERROR("Not on path (F - N - G)");
        return true;
    }

    LOG_DEBUG("Normal data");
    return false;
}

bool GoBackMission::IsOnPath(
    const Pose& o_p1,
    const Pose& o_p2,
    const Pose& o_pi,
    const DPG_Param_t& st_dpg_param) const
{
    // get param (GoBack uses separate tolerance for on-path check)
    double d_thr_dist_m = st_dpg_param.d_goback_on_path_dist_m;

    double d_1to2_distance_m = CMP::CalcDistance(o_p1, o_p2);
    Pose o_p0(
        o_p1.GetXm(),
        o_p1.GetYm(),
        CMP::CalcAtan2(o_p1, o_p2));
    Pose o_pi_based_p1 = o_pi / o_p0;
    if (o_pi_based_p1.GetYm() * o_pi_based_p1.GetYm() < d_thr_dist_m * d_thr_dist_m &&
        o_pi_based_p1.GetXm() > -d_thr_dist_m &&
        o_pi_based_p1.GetXm() <  d_thr_dist_m + d_1to2_distance_m)
    {
        return true;
    }
    return false;
}

}  // namespace NVFR
