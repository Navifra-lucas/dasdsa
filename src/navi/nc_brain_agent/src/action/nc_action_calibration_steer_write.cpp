#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_calibration_steer_write.h>

using namespace NaviFra;

NcActionCalibrationSteerWrite::NcActionCalibrationSteerWrite()
{
}

NcActionCalibrationSteerWrite::~NcActionCalibrationSteerWrite()
{
}

std::string NcActionCalibrationSteerWrite::implName()
{
    return "NcActionCalibrationSteerWrite";
}

void NcActionCalibrationSteerWrite::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();
    if (obj->has("data")) {
        auto data = obj->getObject("data");
        auto write = data->getArray("write");
        std::string command;

        for (size_t index = 0; index < write->size(); index++) {
            if (index == 0)
                command += write->get(index).convert<std::string>();
            else
                command += "/" + write->get(index).convert<std::string>();
        }

        calibrationSteerWrite(command);
        sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
    }
}
