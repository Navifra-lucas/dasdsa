#include "nc_brain_agent/nc_brain_agent.h"

#include <nc_brain_agent/action/nc_action_calibration_docking_read.h>
#include <nc_brain_agent/data/nc_agent_parameters.h>

using namespace NaviFra;

NcActionCalibrationDockingRead::NcActionCalibrationDockingRead()
{
}

NcActionCalibrationDockingRead::~NcActionCalibrationDockingRead()
{
}

std::string NcActionCalibrationDockingRead::implName()
{
    return "NcActionCalibrationDockingRead";
}

void NcActionCalibrationDockingRead::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();
    if (obj->has("data")) {
        Poco::JSON::Object::Ptr data = obj->getObject("data");
        Poco::JSON::Object message;
        std::string strresult = data->has("type") ? data->get("type").convert<std::string>() : "";
        std::string str;

        if (strresult.compare("") == 0) {
            LOG_ERROR("request data is not received on %s", action.c_str());
            sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
        }
        else {
            float degOffset = 0.0;
            float xmOffset = 0.0;
            float ymOffset = 0.0;
            if (strresult == "front") {
                degOffset = NcAgentParameters::get().getParameters()["vmaker/f_front_v_marker_deg_offset"].as<double>();
                xmOffset = NcAgentParameters::get().getParameters()["vmaker/f_front_v_marker_xm_offset"].as<double>();
                ymOffset = NcAgentParameters::get().getParameters()["vmaker/f_front_v_marker_ym_offset"].as<double>();
            }
            else if (strresult == "rear") {
                degOffset = NcAgentParameters::get().getParameters()["vmaker/f_rear_v_marker_deg_offset"].as<double>();
                xmOffset = NcAgentParameters::get().getParameters()["vmaker/f_rear_v_marker_xm_offset"].as<double>();
                ymOffset = NcAgentParameters::get().getParameters()["vmaker/f_rear_v_marker_ym_offset"].as<double>();
            }

            message.set(
                "result", strresult + "/" + std::to_string(degOffset) + "/" + std::to_string(xmOffset) + "/" + std::to_string(ymOffset));

            sendResponseSuccessWithData(source, obj->get("uuid").extract<std::string>(), message);
        }
    }
}