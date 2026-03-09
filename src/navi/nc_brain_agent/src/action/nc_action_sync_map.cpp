#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/ThreadPool.h>
#include <core_agent/data/robot_status.h>
#include <nc_brain_agent/action/async/nc_async_action_sync_map.h>
#include <nc_brain_agent/action/nc_action_sync_map.h>

using namespace NaviFra;

NcActionSyncMap::NcActionSyncMap()
{
}

NcActionSyncMap::~NcActionSyncMap()
{
}

std::string NcActionSyncMap::implName()
{
    return "NcActionSyncMap";
}

void NcActionSyncMap::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    auto robotStatus = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY);
    auto r =
        new NcAsyncActionSyncMap(robotStatus->getID(), obj->get("uuid").convert<std::string>(), robotStatus->getToken(), std::move(obj));
    Poco::ThreadPool::defaultPool().start(*r);
}
