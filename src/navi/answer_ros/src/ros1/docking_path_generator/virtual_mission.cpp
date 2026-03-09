#include "ros1/docking_path_generator/virtual_mission.hpp"

#include "logger/logger.h"

namespace NVFR {

VirtualMission::VirtualMission(
    bool b_local,
    const Pose& o_robot2sensor,
    const Pose& o_map2object,
    const Pose& o_object2goal,
    const Pose& o_fNode,
    const DPG_Param_t& st_dpg_param)
    : st_dpg_param_(st_dpg_param)
    , b_local_(b_local)
    , o_fNode_(o_fNode)
    , o_loop_thread_(std::bind(&VirtualMission::ExecuteDocking, this))
    , b_update_(false)
    , b_initial_(false)
{
    LOG_INFO("{} mission | F node : ({:.3f},{:.3f},{:.1f})",
        b_local_ ? "Local" : "Global",
        o_fNode_.GetXm(), o_fNode_.GetYm(), o_fNode_.GetDeg());

    o_loop_thread_.SetPeriod(
        static_cast<size_t>(st_dpg_param.n_replan_period_ms));
}

VirtualMission::~VirtualMission()
{
    o_loop_thread_.Stop([]{});
    o_docking_ptr_.reset();
}

DockingNodes_t VirtualMission::GetDockingNodes() const
{
    // get param
    DPG_Param_t st_dpg_param = st_dpg_param_.Get();

    Pose o_gNode = o_docking_ptr_->GetEstGoalPose();
    Pose o_nNode = Pose(-st_dpg_param.d_straight_enter_dist_m, 0.0) * o_gNode;
    return {o_fNode_, o_nNode, o_gNode};
}

void VirtualMission::SetRobotPose(const Pose& o_robot_pose)
{
    o_docking_ptr_->SetRobotPose(o_robot_pose);
    o_robot_pose_.Set(o_robot_pose);
}

bool VirtualMission::SetObjectPose(const Pose& o_object_pose)
{
    bool b_update = false;
    b_update = b_local_
        ? o_docking_ptr_->SetSensor2Object(o_object_pose)
        : o_docking_ptr_->SetMap2Object(o_object_pose);

    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (b_update_ == false && b_update == true)
        {
            b_update_ = true;
        }
    }

    return b_update;
}

void VirtualMission::Terminate()
{
    o_loop_thread_.Terminate();
}

void VirtualMission::Stop()
{
    o_loop_thread_.Stop([]{});
}

bool VirtualMission::DoFirstDocking()
{
    // get param
    DPG_Param_t st_dpg_param = st_dpg_param_.Get();

    // get docking nodes
    Pose o_gNode = o_docking_ptr_->GetEstGoalPose();

    // calculate virtual fNode
    Pose o_virtual_fNode =
        SPG::CalculateVirtualStartPose(
            o_fNode_, o_gNode, st_dpg_param);

    double d_lat_err_m = CMP::CalcDistance(o_fNode_, o_virtual_fNode);
    if (d_lat_err_m > st_dpg_param.d_far_from_path_dist_m)
    {
        LOG_WARN("[DoFirstDocking] Far from path: {} [m]",
            d_lat_err_m);
        Pose o_nNode = Pose(-st_dpg_param.d_straight_enter_dist_m, 0.0) * o_gNode;
        Pose o_nNode2fNode = o_fNode_ / o_nNode;
        int n_sign_D = CM::Sign(o_nNode2fNode.GetYm());
        double d_D = std::abs(o_nNode2fNode.GetYm());
        double d_L = -1.0 * o_nNode2fNode.GetXm();
        double d_R = 0.5 * (d_L * d_L + d_D * d_D) / d_D;
        // constraints
        LOG_INFO("Initial value:\n  F Node : {}, {}, {} [m,deg]\n  N Node : {}, {}, {} [m,deg]\n  G Node : {}, {}, {} [m,deg]\n  L={}, D={}, R={} [m]",
            o_fNode_.GetXm(), o_fNode_.GetYm(), o_fNode_.GetDeg(),
            o_nNode.GetXm(), o_nNode.GetYm(), o_nNode.GetDeg(),
            o_gNode.GetXm(), o_gNode.GetYm(), o_gNode.GetDeg(),
            d_L, d_D, d_R);
        if (d_L <= 0.0 || d_L <= d_D || d_R <= 1.0)
        {
            LOG_ERROR("[DoFirstDocking] Wrong initial value");
            // PCM::Publish(PCM::KEY::DOCKING_PATH,
            //     Path({}));
            // return false;
            double d_enter_rad = CMP::CalcAtan2(o_fNode_, o_virtual_fNode);
            Pose o_fNode_enter = o_fNode_;
            o_fNode_enter.SetRad(d_enter_rad);
            Pose o_virtual_fNode_enter = o_virtual_fNode;
            o_virtual_fNode_enter.SetRad(d_enter_rad);
            PCM::Publish(PCM::KEY::DOCKING_PATH,
                Path({o_fNode_enter, o_virtual_fNode_enter, o_virtual_fNode, o_gNode}));
            return true;
        }
        double d_fNode_yaw_rad =
            -1.0 * n_sign_D * std::atan2(d_L, d_R - d_D);
        o_virtual_fNode.SetPose(
            o_fNode_.GetXm(),
            o_fNode_.GetYm(),
            o_nNode.GetRad() + d_fNode_yaw_rad);

        PCM::Publish(PCM::KEY::DOCKING_PATH,
            Path({o_virtual_fNode, o_nNode, o_gNode}));
    }
    else
    {
        PCM::Publish(PCM::KEY::DOCKING_PATH,
            Path({o_virtual_fNode, o_gNode}));
    }

    return true;
}

void VirtualMission::ExecuteDocking()
{
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!b_update_)
        {
            LOG_TRACE("Not updated yet");
            return;
        }
        b_update_ = false;
    }

    if (!b_initial_)
    {
        bool b_success = DoFirstDocking();
        if (!b_success)
        {
            o_loop_thread_.Terminate();
            return;
        }
        b_initial_ = true;
    }

    // get param
    DPG_Param_t st_dpg_param = st_dpg_param_.Get();

    // get docking nodes
    Pose o_gNode = o_docking_ptr_->GetEstGoalPose();

    // check far from path
    double d_lat_err_m = CMP::CalcLateralLength(
        o_gNode, o_robot_pose_.Get());
    if (std::abs(d_lat_err_m) > st_dpg_param.d_far_from_path_dist_m)
    {
        LOG_ERROR("[ExecuteDocking] Far from path: {} [m]",
            d_lat_err_m);
        return;
    }

    // calculate virtual fNode
    Pose o_virtual_fNode =
        SPG::CalculateVirtualStartPose(
            o_fNode_, o_gNode, st_dpg_param);

    PCM::Publish(PCM::KEY::DOCKING_PATH,
        Path({o_virtual_fNode, o_gNode}));

}

}  // namespace NVFR
