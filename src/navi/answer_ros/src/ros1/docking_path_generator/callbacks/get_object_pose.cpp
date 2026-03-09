#include "ros1/docking_path_generator/docking_path_generator_ros1.hpp"

#include "logger/logger.h"

#include "utils/common_math.hpp"

namespace NVFR {

void DockingPathGeneratorRos1::GetObjectPose(
    const geometry_msgs::PoseStamped::ConstPtr msg)
{
    // quat to euler
    double yaw_rad = CM::Quaternion2Yaw(
        msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w);
    Pose o_object_pose(
        msg->pose.position.x, msg->pose.position.y,
        msg->pose.position.z, yaw_rad);
    LOG_TRACE(
        "[GetObjectPose] header: {},  pallet pose (x,y,z,yaw): {:.3f} [m], {:.3f} [m], {:.3f} [m], {:.3f} [deg]",
        msg->header.frame_id.c_str(),
        msg->pose.position.x, msg->pose.position.y,
        msg->pose.position.z, yaw_rad * CM::Rad2Deg);

    DockingInfo::CMD n_mission_type = n_mission_type_.Get();
    switch (n_mission_type)
    {
        case DockingInfo::CMD::PALLET:
        case DockingInfo::CMD::WINGBODY:
            if (o_mission_ptr_)
            {
                o_mission_ptr_->SetObjectPose(o_object_pose);
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
