#include "ros1/answer_ros1.h"
namespace ANSWER {
void AnswerRos1::GetOdometry2D(const nav_msgs::Odometry::ConstPtr msg)
{
    Pose2D current_odom_pose =
        ConvertROSToOdom2D<nav_msgs::Odometry::ConstPtr, ANSWER::Pose2D>(msg);
    SLAM2D::WheelOdometry::GetInstance().SetOdomPose(current_odom_pose);
    AnswerStatus::GetInstance().UpdateOdomTime();
    // AnswerStatus::GetInstance().GetMode()

    // if (AnswerStatus::GetInstance().GetStatus() == STATUS::SLAM) {
    //     // Do something for SLAM mode
    // }

    LocalizeCallback::odom_callback(current_odom_pose);
}
}  // namespace ANSWER