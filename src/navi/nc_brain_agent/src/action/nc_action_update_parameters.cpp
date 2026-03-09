#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_update_parameters.h>
#include <nc_brain_agent/data/nc_agent_parameters.h>

using namespace NaviFra;

NcActionUpdateParameters::NcActionUpdateParameters()
{
}

NcActionUpdateParameters::~NcActionUpdateParameters()
{
}

std::string NcActionUpdateParameters::implName()
{
    return "NcActionUpdateParameters";
}

void NcActionUpdateParameters::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();
    Poco::JSON::Object::Ptr data = obj->getObject("data");
    std::string strdata = obj->has("data") ? obj->get("data").convert<std::string>() : "";
    if (strdata.compare("") == 0) {
        LOG_ERROR("request data is not received on %s", action.c_str());
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
    }
    else {
        NcAgentParameters::get().updateParameters(obj->getObject("data"));
        updateParameters();

        Poco::JSON::Object::Ptr objPtr;
        objPtr.reset(new Poco::JSON::Object(NcAgentParameters::get().toObject()));
        sendResponseSuccessWithData(source, obj->get("uuid").convert<std::string>(), objPtr);
    }
}
