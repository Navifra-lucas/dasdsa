#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_device_goal.h>

using namespace NaviFra;

NcActionDeviceGoal::NcActionDeviceGoal()
{
}

NcActionDeviceGoal::~NcActionDeviceGoal()
{
}

std::string NcActionDeviceGoal::implName()
{
    return "NcActionDeviceGoal";
}

void NcActionDeviceGoal::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
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
    Poco::JSON::Object::Ptr data = obj->getObject("data");

    std::string id = data->has("device_id") ? data->get("device_id").convert<std::string>() : "";
    std::string name = data->has("node_name") ? data->get("node_name").convert<std::string>() : "";
    std::string type = "Goal";

    if (id.compare("") == 0 || name.compare("") == 0) {
        LOG_ERROR("request data is not received on %s", action.c_str());
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
    }
    else {
        deviceGoal(id, name, type);
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
    }
}
