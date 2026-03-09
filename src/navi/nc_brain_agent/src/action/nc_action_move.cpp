#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <core_agent/data/robot_status.h>
#include <nc_brain_agent/action/nc_action_move.h>

using namespace NaviFra;

NcActionMove::NcActionMove()
{
}

NcActionMove::~NcActionMove()
{
}

std::string NcActionMove::implName()
{
    return "NcActionMove";
}

void NcActionMove::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    // if (robotInfo->getStatus() != "idle") {
    //     LOG_ERROR("robot status is not idle. can not action on %s", action.c_str());
    //     sendResponseSuccess(source , obj->get("uuid").convert<std::string>(), "fail");
    //     // sendResponseSuccess(source , obj->get("uuid").convert<std::string>(), "fail", "The robot cannot perform the task in running
    //     status."); return;
    // }

    Poco::JSON::Object::Ptr data = obj->getObject("data");
    if (!InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->isJobActive()) {
        std::string id = data->get("node_id").convert<std::string>();
        move(id, "Goal");

        sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
    }
    else {
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail", "fail! job is active!");
    }
}
