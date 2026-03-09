#include "ros1/answer_ros1.h"
namespace ANSWER {
void AnswerRos1::GetInitialPose(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
    auto initialpose = ConvertROSToPose2D<
        geometry_msgs::PoseWithCovarianceStamped::ConstPtr, ANSWER::Pose2D>(
        msg);

    LocalizeCallback::init_pose_callback(initialpose);
    LOG_INFO("Initial Pose Received");
}
}  // namespace ANSWER