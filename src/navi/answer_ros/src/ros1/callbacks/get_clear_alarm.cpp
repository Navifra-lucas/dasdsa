#include "ros1/answer_ros1.h"
namespace ANSWER {
void AnswerRos1::GetClearAlarm(const std_msgs::Bool::ConstPtr msg)
{
    localizer_->ErrorClear();
}
}  // namespace ANSWER