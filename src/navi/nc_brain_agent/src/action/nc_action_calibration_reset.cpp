#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <core_agent/data/robot_calibration.h>
#include <nc_brain_agent/action/nc_action_calibration_reset.h>
#include <nc_brain_agent/nc_robot_agent.h>

using namespace NaviFra;

NcActionCalibrationReset::NcActionCalibrationReset()
{
}

NcActionCalibrationReset::~NcActionCalibrationReset()
{
}

std::string NcActionCalibrationReset::implName()
{
    return "NcActionCalibrationReset";
}

void NcActionCalibrationReset::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    InMemoryRepository::instance().get<RobotCalibration>(RobotCalibration::KEY)->clear();
    std::string action = obj->get("action").convert<std::string>();

    calibration("clear");
    sendResponseSuccess(source, obj->get("uuid").extract<std::string>());
}
