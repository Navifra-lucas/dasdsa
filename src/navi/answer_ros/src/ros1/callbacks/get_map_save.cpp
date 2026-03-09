#include "ros1/answer_ros1.h"
namespace ANSWER {
void AnswerRos1::GetMapSave(const std_msgs::Int16::ConstPtr msg)
{
    bool save = true;
    SLAMCallback::map_save_callback(save);
}
}  // namespace ANSWER