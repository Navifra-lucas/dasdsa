#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <core_msgs/RepeatTestMsg.h>
#include <nc_brain_agent/action/nc_action_repeat_drive.h>
#include <nc_brain_agent/service/nc_repeat_drive_service.h>

using namespace NaviFra;

NcActionRepeatDrive::NcActionRepeatDrive()
{
}

NcActionRepeatDrive::~NcActionRepeatDrive()
{
}

std::string NcActionRepeatDrive::implName()
{
    return "NcActionRepeatDrive";
}

void NcActionRepeatDrive::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
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

    if (obj->has("data")) {
        Poco::JSON::Object::Ptr data = obj->getObject("data");

        if (data->has("command")) {
            std::string cmd = data->get("command");

            if (cmd == "start") {
                if (data->has("nodes") && data->has("iteration") && data->has("stop")) {
                    std::ostringstream oss;
                    data->stringify(oss);
                    repeatTestCMD(oss.str());
                    NcRepeatDriveService::get().start();
                }
            }
            else {
                std::ostringstream oss;
                data->stringify(oss);
                repeatTestCMD(oss.str());

                if (cmd == "resume")
                    NcRepeatDriveService::get().start();
                else
                    NcRepeatDriveService::get().stop();
            }

            sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
        }
        else {
            LOG_ERROR("request data is not received on %s", action.c_str());
            sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
        }
    }
    else {
        LOG_ERROR("request data is not received on %s", action.c_str());
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
    }
}
