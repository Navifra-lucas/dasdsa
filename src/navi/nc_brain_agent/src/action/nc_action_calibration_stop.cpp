#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_calibration_stop.h>

using namespace NaviFra;

NcActionCalibrationStop::NcActionCalibrationStop()
{
}

NcActionCalibrationStop::~NcActionCalibrationStop()
{
}

std::string NcActionCalibrationStop::implName()
{
    return "NcActionCalibrationStop";
}

void NcActionCalibrationStop::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();
    calibration("clear");

    sendResponseSuccess(source, obj->get("uuid").extract<std::string>());
}
