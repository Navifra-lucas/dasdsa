#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_sound.h>

using namespace NaviFra;

NcActionSound::NcActionSound()
{
}

NcActionSound::~NcActionSound()
{
}

std::string NcActionSound::implName()
{
    return "NcActionSound";
}

void NcActionSound::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    Poco::JSON::Object::Ptr data = obj->getObject("data");

    if (obj->has("data")) {
        std::string id = data->get("id").convert<std::string>();
        int repeat_num = data->get("repeat_num").convert<int>();
        fabsound(id, repeat_num);
    }

    sendResponseSuccess(source, obj->get("uuid").extract<std::string>());
}