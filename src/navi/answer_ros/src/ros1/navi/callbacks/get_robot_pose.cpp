#include "ros1/navi/navigator_ros1.hpp"

#include "logger/logger.h"

namespace NVFR {

void NavigatorRos1::GetRobotPose(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
    // quat to euler
    double yaw_rad = CM::Quaternion2Yaw(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    o_navigator_ptr_->SetRobotPos(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        yaw_rad);
    LOG_TRACE(
        "[GetRobotPose] header: {},  robot_pos(x,y,yaw): {:.3f} [m], {:.3f} [m], {:.3f} [deg]",
        msg->header.frame_id.c_str(),
        msg->pose.pose.position.x, msg->pose.pose.position.y,
        yaw_rad * CM::Rad2Deg);
}

} // namespace NVFR
