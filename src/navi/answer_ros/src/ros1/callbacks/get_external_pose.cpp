#include "common/msg_converter.h"
#include "ros1/answer_ros1.h"
namespace ANSWER {
void AnswerRos1::GetExternalPose(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
    auto my_type = ConvertROSToPose2D<
        geometry_msgs::PoseWithCovarianceStamped::ConstPtr, Pose2D>(msg);
    localizer_->SetExternalPose(my_type, msg->header.frame_id);
}
}  // namespace ANSWER