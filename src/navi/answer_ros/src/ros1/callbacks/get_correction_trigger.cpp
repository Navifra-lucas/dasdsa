#include "ros1/answer_ros1.h"
namespace ANSWER {
void AnswerRos1::GetCorrectionTrigger(const std_msgs::String::ConstPtr msg)
{
    LOG_INFO("Correction Triggered");
    bool b_trigger = true;
    LocalizeCallback::trigger_find_pose_callback(b_trigger);
}
}  // namespace ANSWER
   // }