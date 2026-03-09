#include "ros1/docking_path_generator/docking_path_generator_ros1.hpp"

#include "logger/logger.h"

#include "utils/common_math.hpp"

namespace NVFR {

void DockingPathGeneratorRos1::GetCmdDocking(
    const std_msgs::String::ConstPtr msg)
{
    // parse JSON
    DockingInfo o_docking_info;
    if (!o_docking_info.ParseJson(msg->data))
    {
        LOG_ERROR("Error: Parse DockingInfo");
        return;
    }

    n_mission_type_.Set(DockingInfo::CMD::NONE);
    std::this_thread::sleep_for(std::chrono::milliseconds(20UL));

    switch (o_docking_info.n_mission_type)
    {
        case DockingInfo::CMD::NONE:
        {
            LOG_INFO("DockingInfo/mission_type: NONE");
            DoResetMission();
            break;
        }

        case DockingInfo::CMD::PALLET:
        {
            LOG_INFO("DockingInfo/mission_type: PALLET");
            DoPalletMission(o_docking_info);
            break;
        }

        case DockingInfo::CMD::WINGBODY:
        {
            LOG_INFO("DockingInfo/mission_type: WINGBODY");
            DoWingBodyMission(o_docking_info);
            break;
        }

        case DockingInfo::CMD::GOBACK:
        {
            LOG_INFO("DockingInfo/mission_type: GOBACK");
            DoGoBackMission();
            break;
        }

        default:
            LOG_ERROR("Wrong mission type ({})",
                o_docking_info.n_mission_type);

    }
}

void DockingPathGeneratorRos1::DoResetMission()
{
    if (o_mission_ptr_)
    {
        o_mission_ptr_->Stop();
    }

    n_mission_type_.Set(DockingInfo::CMD::NONE);
    LOG_INFO("[GetCmdDocking] Reset docking mission");

}

void DockingPathGeneratorRos1::DoPalletMission(
    DockingInfo& o_docking_info)
{
    // get param
    DPG_Param_t st_dpg_param = st_dpg_param_.Get();

    // get robot pose
    Pose o_robot_pose(o_robot_pose_.Get());
    LOG_INFO("Robot pose: {}, {}, {} [m,deg]",
        o_robot_pose.GetXm(), o_robot_pose.GetYm(),
        o_robot_pose.GetDeg());

    // set tf (robot to sensor)
    Pose o_robot2sensor(
        st_dpg_param.st_pallet_docking_param.d_robot2sensor_x_m,
        st_dpg_param.st_pallet_docking_param.d_robot2sensor_y_m);
    o_robot2sensor.SetDeg(st_dpg_param.st_pallet_docking_param.d_robot2sensor_deg);
    LOG_INFO("Robot to Sensor: {}, {}, {} [m,deg]",
        o_robot2sensor.GetXm(), o_robot2sensor.GetYm(),
        o_robot2sensor.GetDeg());

    LOG_INFO("F Node: {}, {}, {} [m,deg]",
        o_docking_info.o_fNode.GetXm(), o_docking_info.o_fNode.GetYm(),
        o_docking_info.o_fNode.GetDeg());
    LOG_INFO("{} Object Pose: {}, {}, {} [m,deg]",
        o_docking_info.b_local ? "Local" : "Global",
        o_docking_info.o_pose.GetXm(), o_docking_info.o_pose.GetYm(),
        o_docking_info.o_pose.GetDeg());

    // set global pose
    Pose o_map2object = o_docking_info.b_local
        ? o_docking_info.o_pose * o_robot2sensor * o_robot_pose
        : o_docking_info.o_pose;
    LOG_INFO("Global Pallet: {}, {}, {} [m,deg]",
        o_map2object.GetXm(), o_map2object.GetYm(),
        o_map2object.GetDeg());

    o_mission_ptr_ = std::make_shared<PalletMission>(
        o_docking_info.b_local,
        o_robot2sensor,
        o_map2object,
        o_docking_info.o_object2goal,
        o_docking_info.o_fNode,
        st_dpg_param);
    auto nodes = o_mission_ptr_->GetDockingNodes();
    o_go_back_mission_.SetNodeF(nodes.o_fNode);
    o_go_back_mission_.SetNodeN(nodes.o_nNode);
    o_go_back_mission_.SetNodeG(nodes.o_gNode);

    n_mission_type_.Set(DockingInfo::CMD::PALLET);
    LOG_INFO("[GetCmdDocking] Run pallet docking mission");

}

void DockingPathGeneratorRos1::DoWingBodyMission(
    DockingInfo& o_docking_info)
{
    // get param
    DPG_Param_t st_dpg_param = st_dpg_param_.Get();

    // get robot pose
    Pose o_robot_pose(o_robot_pose_.Get());
    LOG_INFO("Robot pose: {}, {}, {} [m,deg]",
        o_robot_pose.GetXm(), o_robot_pose.GetYm(),
        o_robot_pose.GetDeg());

    // set tf (robot to sensor)
    Pose o_robot2sensor(
        st_dpg_param.st_wingbody_docking_param.d_robot2sensor_x_m,
        st_dpg_param.st_wingbody_docking_param.d_robot2sensor_y_m);
    o_robot2sensor.SetDeg(st_dpg_param.st_wingbody_docking_param.d_robot2sensor_deg);
    LOG_INFO("Robot to Sensor: {}, {}, {} [m,deg]",
        o_robot2sensor.GetXm(), o_robot2sensor.GetYm(),
        o_robot2sensor.GetDeg());

    LOG_INFO("F Node: {}, {}, {} [m,deg]",
        o_docking_info.o_fNode.GetXm(), o_docking_info.o_fNode.GetYm(),
        o_docking_info.o_fNode.GetDeg());
    LOG_INFO("{} Object Pose: {}, {}, {} [m,deg]",
        o_docking_info.b_local ? "Local" : "Global",
        o_docking_info.o_pose.GetXm(), o_docking_info.o_pose.GetYm(),
        o_docking_info.o_pose.GetDeg());

    // set global pose
    Pose o_map2object = o_docking_info.b_local
        ? o_docking_info.o_pose * o_robot2sensor * o_robot_pose
        : o_docking_info.o_pose;
    LOG_INFO("Global WingBody: {}, {}, {} [m,deg]",
        o_map2object.GetXm(), o_map2object.GetYm(),
        o_map2object.GetDeg());

    o_mission_ptr_ = std::make_shared<WingBodyMission>(
        o_docking_info.b_local,
        o_robot2sensor,
        o_map2object,
        o_docking_info.o_object2goal,
        o_docking_info.o_fNode,
        st_dpg_param);
    auto nodes = o_mission_ptr_->GetDockingNodes();
    o_go_back_mission_.SetNodeF(nodes.o_fNode);
    o_go_back_mission_.SetNodeN(nodes.o_nNode);
    o_go_back_mission_.SetNodeG(nodes.o_gNode);

    n_mission_type_.Set(DockingInfo::CMD::WINGBODY);
    LOG_INFO("[GetCmdDocking] Run wingbody docking mission");

}

void DockingPathGeneratorRos1::DoGoBackMission()
{
    // get robot pose
    Pose o_robot_pose(o_robot_pose_.Get());
    LOG_INFO("Robot pose: {}, {}, {} [m,deg]",
        o_robot_pose.GetXm(), o_robot_pose.GetYm(),
        o_robot_pose.GetDeg());

    if (o_mission_ptr_)
    {
        auto nodes = o_mission_ptr_->GetDockingNodes();
        o_go_back_mission_.SetNodeF(nodes.o_fNode);
        o_go_back_mission_.SetNodeN(nodes.o_nNode);
        o_go_back_mission_.SetNodeG(nodes.o_gNode);
        o_mission_ptr_->Terminate();
    }
    else
    {
        LOG_DEBUG("o_mission_ptr_ is nullptr");
    }

    bool b_go_back_mission = o_go_back_mission_.GeneratePath(o_robot_pose);
    if (b_go_back_mission)
    {
        LOG_INFO("[GetCmdDocking] Run goback mission");
    }

    n_mission_type_.Set(DockingInfo::CMD::GOBACK);

}

} // namespace NVFR
