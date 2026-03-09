#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_playback_download.h>

using namespace NaviFra;

NcActionPlaybackDownload::NcActionPlaybackDownload()
{
}

NcActionPlaybackDownload::~NcActionPlaybackDownload()
{
}

void NcActionPlaybackDownload::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    auto data = obj->getObject("data");
    if (data->has("key") && data->has("log_id") && data->has("type")) {
        std::string command = Poco::format(
            "download/%s/%s/%s", data->get("key").convert<std::string>(), data->get("log_id").convert<std::string>(),
            data->get("type").convert<std::string>());
        playBackCMD(command);
    }
    else {
        std::ostringstream oss;
        obj->stringify(oss);

        NLOG(error) << "JSON Struct Error " << oss.str();
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail", "Data StructError");
    }
}

std::string NcActionPlaybackDownload::implName()
{
    return "NcActionPlaybackDownload";
}
