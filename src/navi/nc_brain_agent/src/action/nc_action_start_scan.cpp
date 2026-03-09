#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <core_agent/manager/publish_manager.h>
#include <nc_brain_agent/action/nc_action_start_scan.h>
#include <nc_brain_agent/data/nc_status_channel.h>

using namespace NaviFra;

NcActionStartSCAN::NcActionStartSCAN()
{
}

NcActionStartSCAN::~NcActionStartSCAN()
{
}

std::string NcActionStartSCAN::implName()
{
    return "NcActionStartSCAN";
}

void NcActionStartSCAN::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    PublisherManager::instance().activate((int)PUBLISH_CHANNEL::CHANNEL_LIDAR);

    if (obj->has("data")) {
        auto data = obj->getObject("data");
        bool high = data->get("high").convert<bool>();
        std::string s_rq_data = high ? "high" : "low";
        ProcessResult result = requestStringSrv(action, s_rq_data);
        if (result.success) {
            LOG_INFO("Scan Start Mode  Change Success[ %s ] %s", s_rq_data.c_str(), action.c_str());
        }
        else {
            LOG_ERROR("Scan Start Mode Change Fail[ %s ] %s", s_rq_data.c_str(), action.c_str());
        }
    }

    sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
}
