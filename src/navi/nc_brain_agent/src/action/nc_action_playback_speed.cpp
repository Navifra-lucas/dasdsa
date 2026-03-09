#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_playback_speed.h>
#include <std_msgs/String.h>

using namespace NaviFra;

NcActionPlaybackSpeed::NcActionPlaybackSpeed()
{
}

NcActionPlaybackSpeed::~NcActionPlaybackSpeed()
{
}

void NcActionPlaybackSpeed::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();
    Poco::JSON::Object::Ptr data = obj->getObject("data");
    std_msgs::String command;
    std::string strspeed = data->has("speed") ? data->get("speed").convert<std::string>() : "";
    if (strspeed.compare("") == 0) {
        LOG_ERROR("request data is not received on %s", action.c_str());
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
    }
    else {
        playBackCMD("speed/" + strspeed);
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
    }
}

std::string NcActionPlaybackSpeed::implName()
{
    return "NcActionPlaybackSpeed";
}
