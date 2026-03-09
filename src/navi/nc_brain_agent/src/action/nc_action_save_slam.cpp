#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_save_slam.h>

using namespace NaviFra;
NcActionSaveSLAM::NcActionSaveSLAM()
{
}

NcActionSaveSLAM::~NcActionSaveSLAM()
{
}

std::string NcActionSaveSLAM::implName()
{
    return "NcActionSaveSLAM";
}

void NcActionSaveSLAM::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
    saveSLAM(1);
}
