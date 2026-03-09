#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <core_agent/manager/uuid_response_manager.h>
#include <nc_brain_agent/action/nc_action_avoid_permission.h>

using namespace NaviFra;

NcActionAvoidPerMission::NcActionAvoidPerMission()
{
}

NcActionAvoidPerMission::~NcActionAvoidPerMission()
{
}

std::string NcActionAvoidPerMission::implName()
{
    return "NcActionAvoidPerMission";
}

void NcActionAvoidPerMission::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string uuid = obj->get("uuid").convert<std::string>();
    if (UUIDResponseManager::instance().validateUUID(uuid)) {
        auto data = obj->getObject("data");
        responseAvoidance(data->has("avoid_permission") ? data->get("avoid_permission").convert<bool>() : false);
    }
}
