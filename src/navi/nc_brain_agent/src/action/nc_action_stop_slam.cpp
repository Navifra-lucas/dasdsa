#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/async/nc_async_action_slam_stop.h>
#include <nc_brain_agent/action/nc_action_stop_slam.h>
#include <nc_brain_agent/message/nc_brain_command.h>
using namespace NaviFra;

NcActionStopSLAM::NcActionStopSLAM()
{
}

NcActionStopSLAM::~NcActionStopSLAM()
{
}

std::string NcActionStopSLAM::implName()
{
    return "NcActionStopSLAM";
}

void NcActionStopSLAM::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    Poco::JSON::Object::Ptr data = obj->getObject("data");

    if (data->get("save").convert<bool>() == false) {
        auto r = new NcAsyncActionSLAMStop(obj->get("uuid").convert<std::string>());
        Poco::ThreadPool::defaultPool().start(*r);
    }
    else {
        saveSLAM(1);
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
    }
}
