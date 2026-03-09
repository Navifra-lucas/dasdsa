#include "nc_brain_agent/nc_brain_agent.h"

#include <nc_brain_agent/action/nc_action_get_parameters.h>
#include <nc_brain_agent/data/nc_agent_parameters.h>

using namespace NaviFra;

NcActionGetParameters::NcActionGetParameters()
{
}

NcActionGetParameters::~NcActionGetParameters()
{
}

std::string NcActionGetParameters::implName()
{
    return "NcActionGetParameters";
}

void NcActionGetParameters::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    if (!NcAgentParameters::get().isExits())
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail", "File does not exist.");
    else
        sendResponseSuccessWithData(source, obj->get("uuid").convert<std::string>(), NcAgentParameters::get().toObject());
}
