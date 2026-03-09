#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/ThreadPool.h>
#include <core_agent/data/robot_status.h>
#include <nc_brain_agent/action/async/nc_async_action_update_map.h>
#include <nc_brain_agent/action/nc_action_update_map.h>

using namespace NaviFra;

NcActionUpdateMap::NcActionUpdateMap()
{
}

NcActionUpdateMap::~NcActionUpdateMap()
{
}

std::string NcActionUpdateMap::implName()
{
    return "NcActionUpdateMap";
}

void NcActionUpdateMap::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    auto robotStatus = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY);
    auto r =
        new NcAsyncActionMapUpdate(robotStatus->getID(), obj->get("uuid").convert<std::string>(), robotStatus->getToken(), std::move(obj));
    Poco::ThreadPool::defaultPool().start(*r);
}
