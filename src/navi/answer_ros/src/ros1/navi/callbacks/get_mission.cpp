#include "ros1/navi/navigator_ros1.hpp"

#include <vector>

#include "logger/logger.h"

namespace NVFR {

void NavigatorRos1::GetMissionAlign(
    const answer_msgs::MissionAlign::ConstPtr msg)
{
    o_navigator_ptr_->PushMission(
        MsgConverter::Convert<answer_msgs::MissionAlign, MissionBundle_t>(*msg));

    SendMissionResponse(msg->uuid, true);
}

void NavigatorRos1::GetMissionNodePath(
    const answer_msgs::MissionNodePath::ConstPtr msg)
{
    if (msg->edge_array.empty()) {
        LOG_ERROR("[GetMission] Path is empty");
        SendMissionResponse(msg->uuid, false);
        return;
    }

    o_navigator_ptr_->PushMission(
        MsgConverter::Convert<answer_msgs::MissionNodePath, MissionBundle_t>(*msg));

    SendMissionResponse(msg->uuid, true);
}

void NavigatorRos1::GetMissionExplore(
    const answer_msgs::MissionExplore::ConstPtr msg)
{
    o_navigator_ptr_->PushMission(
        MsgConverter::Convert<answer_msgs::MissionExplore, MissionBundle_t>(*msg));

    SendMissionResponse(msg->uuid, true);
}

} // namespace NVFR
