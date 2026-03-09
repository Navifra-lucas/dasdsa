#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/DynamicStruct.h>
#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_cmdvel.h>

using namespace NaviFra;
using Poco::DynamicStruct;

NcActionCMDVel::NcActionCMDVel()
{
}

NcActionCMDVel::~NcActionCMDVel()
{
}

std::string NcActionCMDVel::implName()
{
    return "NcActionCMDVel";
}

void NcActionCMDVel::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    if (robotInfo->getStatus() == "running") {
        LOG_ERROR("robot status is running. can not action on %s", action.c_str());
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
        // sendResponseSuccess(source , obj->get("uuid").convert<std::string>(), "fail", "The robot cannot perform the task in running
        // status.");
        return;
    }
    DynamicStruct data = *obj;

    if ((float)data["data"]["linear"] == 0 && (float)data["data"]["angular"] == 0 && (float)data["data"]["linear_y"] == 0) {
        return;
    }
    sendResponseSuccess(source, data["uuid"].extract<std::string>());

    float linearX = (float)data["data"]["linear"];
    float linearY = (float)data["data"]["linear_y"];
    float angulaZ = (float)data["data"]["angular"] * 3.14 / 180;

    cmdVel(linearX, linearY, angulaZ);
}
