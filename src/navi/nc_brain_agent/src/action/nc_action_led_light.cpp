#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_led_light.h>

using namespace NaviFra;

NcActionLedLight::NcActionLedLight()
{
}

NcActionLedLight::~NcActionLedLight()
{
}

std::string NcActionLedLight::implName()
{
    return "NcActionLedLight";
}

void NcActionLedLight::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    Poco::JSON::Object::Ptr data = obj->getObject("data");

    if (obj->has("data")) {
        std::string color = data->get("color").convert<std::string>();

        fabcolor(color);
    }

    sendResponseSuccess(source, obj->get("uuid").extract<std::string>());
}