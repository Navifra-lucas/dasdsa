#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_playback_seek.h>

using namespace NaviFra;

NcActionPlaybackSeek::NcActionPlaybackSeek()
{
}

NcActionPlaybackSeek::~NcActionPlaybackSeek()
{
}

void NcActionPlaybackSeek::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();
    Poco::JSON::Object::Ptr data = obj->getObject("data");
    std::string command;

    std::string strposition = data->has("position") ? data->get("position").convert<std::string>() : "";
    if (strposition.compare("") == 0) {
        LOG_ERROR("request data is not received on %s", action.c_str());
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
    }
    else {
        command = "seek/" + strposition;
        playBackCMD(command);
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
    }
}

std::string NcActionPlaybackSeek::implName()
{
    return "NcActionPlaybackSeek";
}
