#include "nc_brain_agent/nc_brain_agent.h"

#include <nc_brain_agent/action/async/nc_async_action_slam_stop.h>
#include <nc_brain_agent/action/nc_action_get_slam_map.h>
#include <nc_brain_agent/data/nc_brain_map.h>

using namespace NaviFra;
NcActionGetSLAMMap::NcActionGetSLAMMap()
{
}

NcActionGetSLAMMap::~NcActionGetSLAMMap()
{
}

std::string NcActionGetSLAMMap::implName()
{
    return "NcActionGetSLAMMap";
}

void NcActionGetSLAMMap::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    auto r = new NcAsyncActionSLAMStop("");
    Poco::ThreadPool::defaultPool().start(*r);

    Poco::JSON::Object::Ptr mapObj = InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getSlamMap();
    sendResponseSuccessWithData(source, obj->get("uuid").convert<std::string>(), mapObj);
}
