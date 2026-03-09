#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_stop.h>
#include <nc_brain_agent/message/nc_brain_command.h>

using namespace NaviFra;

NcActionStop::NcActionStop()
{
}

NcActionStop::~NcActionStop()
{
}

std::string NcActionStop::implName()
{
    return "NcActionStop";
}

void NcActionStop::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();
    sendResponseSuccess(source, obj->get("uuid").convert<std::string>());

    robotStop();
}
