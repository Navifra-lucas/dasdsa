#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_brain_agent/action/nc_action_calibration_docking_save.h>
#include <nc_brain_agent/data/nc_agent_parameters.h>
#include <std_msgs/String.h>

using namespace NaviFra;

NcActionCalibrationDockingSave::NcActionCalibrationDockingSave()
{
}

NcActionCalibrationDockingSave::~NcActionCalibrationDockingSave()
{
}

std::string NcActionCalibrationDockingSave::implName()
{
    return "NcActionCalibrationDockingSave";
}

void NcActionCalibrationDockingSave::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();
    if (obj->has("data")) {
        Poco::JSON::Object::Ptr data = obj->getObject("data");

        std::string strresult = data->has("result") ? data->get("result").convert<std::string>() : "";
        if (strresult.compare("") == 0) {
            LOG_ERROR("request data is not received on %s", action.c_str());
            sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
        }
        else {
            calibrationDockingSave(strresult);
            sendResponseSuccess(source, obj->get("uuid").extract<std::string>());
        }
    }
}
