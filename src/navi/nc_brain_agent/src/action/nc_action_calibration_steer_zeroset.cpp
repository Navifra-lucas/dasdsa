#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_calibration_steer_zeroset.h>

using namespace NaviFra;

NcActionCalibrationSteerZeroSet::NcActionCalibrationSteerZeroSet()
{
}

NcActionCalibrationSteerZeroSet::~NcActionCalibrationSteerZeroSet()
{
}

std::string NcActionCalibrationSteerZeroSet::implName()
{
    return "NcActionCalibrationSteerZeroSet";
}

void NcActionCalibrationSteerZeroSet::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();

    calibrationSteerZeroSet(false);
    sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
}
