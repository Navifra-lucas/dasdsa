#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <core_agent/data/robot_status.h>
#include <math.h>
#include <nc_brain_agent/action/nc_action_action.h>
#include <nc_brain_agent/data/nc_brain_map.h>
#include <nc_brain_agent/nc_robot_agent.h>
#include <tf/tf.h>

using namespace NaviFra;

NcActionAction::NcActionAction()
{
}

NcActionAction::~NcActionAction()
{
}

void NcActionAction::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    if (robotInfo->getStatus() != "idle") {
        LOG_ERROR("robot status is not idle. can not action on %s", action.c_str());
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
        // sendResponseSuccess(source , obj->get("uuid").convert<std::string>(), "fail", "The robot cannot perform the task in running
        // status.");
        return;
    }
    NLOG(info) << "implonAction : " << action;
    node_action();
    sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
}

std::string NcActionAction::implName()
{
    return "NcActionAction";
}
