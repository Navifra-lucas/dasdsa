#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_playback_pause.h>
#include <nc_brain_agent/service/nc_playback_service.h>

using Poco::DynamicStruct;
using namespace NaviFra;

NcActionPlaybackPause::NcActionPlaybackPause()
{
}

NcActionPlaybackPause::~NcActionPlaybackPause()
{
}

void NcActionPlaybackPause::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    NcPlaybackService::get().stop();
    playBackCMD("stop/");
    sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
}

std::string NcActionPlaybackPause::implName()
{
    return "NcActionPlaybackPause";
}
