#include "ros1/docking_path_generator/docking_path_generator_ros1.hpp"

#include "logger/logger.h"

#include "utils/common_math.hpp"

namespace NVFR {

void DockingPathGeneratorRos1::GetRobotPose(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
    // quat to euler
    double yaw_rad = CM::Quaternion2Yaw(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    Arr3d o_robot_pose = {
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        yaw_rad};
    o_robot_pose_.Set(o_robot_pose);
    LOG_TRACE(
        "[GetRobotPose] header: {},  robot_pose (x,y,yaw): {:.3f} [m], {:.3f} [m], {:.3f} [deg]",
        msg->header.frame_id.c_str(),
        msg->pose.pose.position.x, msg->pose.pose.position.y,
        yaw_rad * CM::Rad2Deg);

    DockingInfo::CMD n_mission_type = n_mission_type_.Get();
    switch (n_mission_type)
    {
        case DockingInfo::CMD::PALLET:
        case DockingInfo::CMD::WINGBODY:
            if (o_mission_ptr_)
            {
                o_mission_ptr_->SetRobotPose(Pose(o_robot_pose));
            }
            else
            {
                LOG_ERROR("o_mission_ptr_ is nullptr");
            }
            break;

        default:
            LOG_TRACE("Current mission type : {}",
                static_cast<int>(n_mission_type));
    }

}

} // namespace NVFR
