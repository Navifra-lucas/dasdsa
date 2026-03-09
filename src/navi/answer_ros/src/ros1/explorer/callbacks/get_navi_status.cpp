#include "ros1/explorer/explorer_ros1.hpp"

#include "logger/logger.h"

namespace NVFR {

void ExplorerRos1::GetNaviMissionResponse(
    const answer_msgs::MissionResponse::ConstPtr msg)
{
    if (msg->uuid == "uuid-explore-path" && msg->b_received) {
        o_service_client_handler_.ReceiveResponse("FirstMission");
    }
    else if (msg->uuid == "uuid-explore-mission" && !msg->b_received) {
        LOG_ERROR("fail to receive explore mission, why ?");
    }
}

void ExplorerRos1::GetNaviStatus(
    const std_msgs::Int32::ConstPtr msg)
{
    switch (static_cast<MPTL::NAVI_STATUS>(msg->data)) {
        case MPTL::NAVI_STATUS::DONE:
        case MPTL::NAVI_STATUS::ABORTED:
            LOG_INFO("Explore");
            std::thread([&] () { o_explorer_ptr_->NewExplore(); }).detach();
            //o_explorer_ptr_->NewExplore();
            break;

        default:
            LOG_WARN("Navi status is wrong, Pause explorer ({})", msg->data);
            return;
    }
}

void ExplorerRos1::GetNaviError(
    const std_msgs::Int32::ConstPtr msg)
{
    switch (static_cast<MPTL::NAVI_ERROR>(msg->data)) {
        case MPTL::NAVI_ERROR::PATH_GENERATION:
            LOG_INFO("Fail to find explore path");
            o_explorer_ptr_->NextExplore();
            break;

        default:
            LOG_WARN("Navi error is wrong, Pause explorer ({})", msg->data);
            return;
    }
}

} // namespace NVFR
