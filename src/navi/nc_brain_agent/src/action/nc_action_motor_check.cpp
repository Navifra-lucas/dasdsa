#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_motor_check.h>

using namespace NaviFra;

NcActionMotorCheck::NcActionMotorCheck()
{
}

NcActionMotorCheck::~NcActionMotorCheck()
{
}

void NcActionMotorCheck::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    startMotorStatus();
    sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
}

std::string NcActionMotorCheck::implName()
{
    return "NcActionMotorCheck";
}
