#include "ros1/answer_ros1.h"
namespace ANSWER {
void AnswerRos1::GetRegisterReflector(const std_msgs::Bool::ConstPtr msg)
{
    LocalizeCallback::RegisterReflectorCallback();
}
}  // namespace ANSWER