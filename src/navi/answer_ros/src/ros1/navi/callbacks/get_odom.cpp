#include "ros1/navi/navigator_ros1.hpp"

#include "logger/logger.h"

namespace NVFR {

void NavigatorRos1::GetOdom(
    const nav_msgs::Odometry::ConstPtr msg)
{
    std::string s_frame_id = msg->header.frame_id;
    std::string s_child_id = msg->child_frame_id;
    double vx_m_s = msg->twist.twist.linear.x;
    double vy_m_s = msg->twist.twist.linear.y;
    double w_rad_s = msg->twist.twist.angular.z;
    o_navigator_ptr_->SetRobotVel(vx_m_s, vy_m_s, w_rad_s);
    LOG_TRACE("robot_vel(x,y,w): {:.3f} [m/s], {:.3f} [m/s], {:.3f} [deg/s]",
        vx_m_s, vy_m_s, w_rad_s * CM::Rad2Deg);
}

} // namespace NVFR
