#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <core_agent/data/robot_calibration.h>
#include <nc_brain_agent/action/nc_action_calibration.h>
#include <nc_brain_agent/nc_robot_agent.h>

using namespace NaviFra;

NcActionCalibration::NcActionCalibration()
{
}

NcActionCalibration::~NcActionCalibration()
{
}

std::string NcActionCalibration::implName()
{
    return "NcActionCalibration";
}

void NcActionCalibration::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    // if (robotInfo->getStatus() != "idle") {
    //     LOG_ERROR("robot status is not idle. can not action on %s", action.c_str());
    //     sendResponseSuccess(source , obj->get("uuid").convert<std::string>(), "fail");
    //     // sendResponseSuccess(source , obj->get("uuid").convert<std::string>(), "fail", "The robot cannot perform the task in running
    //     status."); return;
    // }

    if (obj->has("data")) {
        Poco::JSON::Object::Ptr data = obj->getObject("data");
        std::string strCaliType = data->has("type") ? data->get("type").convert<std::string>() : "";

        if (strCaliType.compare("") == 0) {
            LOG_ERROR("request data is not received on %s", action.c_str());
            sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
        }
        else {
            auto robotCalibration = InMemoryRepository::instance().get<RobotCalibration>(RobotCalibration::KEY);
            robotCalibration->setCurrentType(strCaliType);
            std::string calitype = robotCalibration->calibrationCommand(strCaliType);
            if (calitype != "") {
                calibration(calitype);
            }
        }
    }
    sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
}
