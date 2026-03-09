#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_device_command.h>

using namespace NaviFra;

NcActionDeviceCommand::NcActionDeviceCommand()
{
}

NcActionDeviceCommand::~NcActionDeviceCommand()
{
}

std::string NcActionDeviceCommand::implName()
{
    return "NcActionDeviceCommand";
}

void NcActionDeviceCommand::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();
    if (obj->has("data")) {
        Poco::JSON::Object::Ptr data = obj->getObject("data");

        std::string command = data->has("command") ? data->get("command").convert<std::string>() : "";

        if (command.compare("") == 0) {
            LOG_ERROR("request data is not received on %s", action.c_str());
            sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
        }
        else {
            deviceCommand(command);
            sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
        }
    }
}
