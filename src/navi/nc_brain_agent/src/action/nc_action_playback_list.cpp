#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/File.h>
#include <nc_brain_agent/action/nc_action_playback_list.h>

using namespace NaviFra;
NcActionPlaybackList::NcActionPlaybackList()
{
}

NcActionPlaybackList::~NcActionPlaybackList()
{
}

void NcActionPlaybackList::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string homePath = std::getenv("HOME");
    std::string core_root = std::getenv("CORE_ROOT") ? std::getenv("CORE_ROOT") : "";
    std::string playback_path = core_root.empty() != true ? core_root + "/bag/" : homePath + "/navifra_solution/navicore/configs/bag/";

    Poco::File dirPlay(playback_path);
    std::vector<std::string> filelist;
    dirPlay.list(filelist);

    Poco::JSON::Array array;

    for (const auto file : filelist) {
        if (file.find(".bag") != std::string::npos) {
            Poco::JSON::Object objFile;
            objFile.set("file", file);
            objFile.set("created_at", Poco::File(playback_path + file).getLastModified().raw() / 1000000);
            array.add(objFile);
        }
    }

    Poco::JSON::Object data;
    data.set("list", array);

    sendResponseSuccessWithData(source, obj->get("uuid").convert<std::string>(), data);
}

std::string NcActionPlaybackList::implName()
{
    return "NcActionPlaybackList";
}
