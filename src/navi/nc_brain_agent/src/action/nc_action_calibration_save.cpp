#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/data/robot_calibration.h>
#include <core_agent/data/robot_info.h>
#include <core_agent/data/types.h>
#include <nc_brain_agent/action/nc_action_calibration_save.h>
#include <nc_brain_agent/data/nc_agent_parameters.h>
#include <nc_brain_agent/nc_robot_agent.h>

using namespace NaviFra;

NcActionCalibrationSave::NcActionCalibrationSave()
{
}

NcActionCalibrationSave::~NcActionCalibrationSave()
{
}

std::string NcActionCalibrationSave::implName()
{
    return "NcActionCalibrationSave";
}

void NcActionCalibrationSave::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    std::string action = obj->get("action").convert<std::string>();
    if (obj->has("data")) {
        Poco::JSON::Object::Ptr data = obj->getObject("data");

        std::string type = data->has("type") ? data->get("type").convert<std::string>() : "";
        float ratio = (float)data->get("ratio");
        float base = (float)data->get("base");  // 직진, SD

        if (type.compare("") == 0) {
            LOG_ERROR("request data is not received on %s", action.c_str());
            sendResponseSuccess(source, obj->get("uuid").convert<std::string>(), "fail");
        }
        else {
            LOG_TRACE(
                "type :%s , ratio: %f, base: %f  ======  strRobottype_:%s , type_RobotCalibration_: %s", type.c_str(), ratio, base,
                InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY)->getRobotType().c_str(),
                InMemoryRepository::instance().get<RobotCalibration>(RobotCalibration::KEY)->currentType().c_str());

            // if ((ratio != 0) && (InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY)->getRobotType().c_str() == type)) {
            //     if (InMemoryRepository::instance().get<RobotCalibration>(RobotCalibration::KEY)->currentType() == TypeCalibration::FOWARD ||
            //         InMemoryRepository::instance().get<RobotCalibration>(RobotCalibration::KEY)->currentType() ==
            //             TypeCalibration::BACKWARD) {
            //         // float front_steer = (float)data["data"]["front_steer"];// 직진, QD
            //         // float rear_steer = (float)data["data"]["rear_steer"];// 직진, QD
            //         if (type == "DD") {
            //             float fgetVal = 0.0;
            //             fgetVal = NcAgentParameters::get().getParameters()["driver_dd/f_dd_FL_encoder_pulse"].as<double>();
            //             NcAgentParameters::get().setParameters("driver_dd/f_dd_FL_encoder_pulse", fgetVal / ratio);
            //             fgetVal = NcAgentParameters::get().getParameters()["driver_dd/f_dd_RR_encoder_pulse"].as<double>();
            //             NcAgentParameters::get().setParameters("driver_dd/f_dd_RR_encoder_pulse", fgetVal / ratio);
            //         }
            //         else if (type == "QD") {
            //             float fgetVal = 0.0;
            //             fgetVal = NcAgentParameters::get().getParameters()["driver_qd/f_qd_FL_encoder_pulse"].as<double>();
            //             NcAgentParameters::get().setParameters("driver_qd/f_qd_FL_encoder_pulse", fgetVal / ratio);
            //             fgetVal = NcAgentParameters::get().getParameters()["driver_qd/f_qd_RR_encoder_pulse"].as<double>();
            //             NcAgentParameters::get().setParameters("driver_qd/f_qd_RR_encoder_pulse", fgetVal / ratio);
            //         }
            //         else if (type == "SD") {
            //             float fgetVal = 0.0;
            //             fgetVal = NcAgentParameters::get().getParameters()["driver_sd/f_sd_FL_encoder_pulse"].as<double>();
            //             NcAgentParameters::get().setParameters("driver_sd/f_sd_FL_encoder_pulse", fgetVal / ratio);
            //         }
            //     }

            //     else if (
            //         InMemoryRepository::instance().get<RobotCalibration>(RobotCalibration::KEY)->currentType() == TypeCalibration::LEFT ||
            //         InMemoryRepository::instance().get<RobotCalibration>(RobotCalibration::KEY)->currentType() == TypeCalibration::RIGHT) {
            //         if (type == "DD") {
            //             float fgetVal = 0.0;
            //             fgetVal = NcAgentParameters::get().getParameters()["driver_dd/f_dd_wheelbase_width_m"].as<double>();
            //             NcAgentParameters::get().setParameters("driver_dd/f_dd_wheelbase_width_m", fgetVal * ratio);
            //         }
            //         else if (type == "QD") {
            //             float fgetVal = 0.0;
            //             fgetVal = NcAgentParameters::get().getParameters()["driver_wheel/f_FL_wheel_x_m"].as<double>();
            //             NcAgentParameters::get().setParameters("driver_wheel/f_FL_wheel_x_m", fgetVal * ratio);
            //             fgetVal = NcAgentParameters::get().getParameters()["driver_wheel/f_FL_wheel_y_m"].as<double>();
            //             NcAgentParameters::get().setParameters("driver_wheel/f_FL_wheel_y_m", fgetVal * ratio);

            //             fgetVal = NcAgentParameters::get().getParameters()["driver_wheel/f_RR_wheel_x_m"].as<double>();
            //             NcAgentParameters::get().setParameters("driver_wheel/f_RR_wheel_x_m", fgetVal * ratio);
            //             fgetVal = NcAgentParameters::get().getParameters()["driver_wheel/f_RR_wheel_y_m"].as<double>();
            //             NcAgentParameters::get().setParameters("driver_wheel/f_RR_wheel_y_m", fgetVal * ratio);
            //         }
            //         else if (type == "SD") {
            //             float fgetVal = 0.0;
            //             fgetVal = NcAgentParameters::get().getParameters()["driver_wheel/f_FL_wheel_x_m"].as<double>();
            //             NcAgentParameters::get().setParameters("driver_wheel/f_FL_wheel_x_m", fgetVal / ratio);
            //             //  fgetVal = NcAgentParameters::get().getParameters()["driver_wheel/f_FL_wheel_y_m"].as<double>();
            //             //  NcAgentParameters::get().setParameters("driver_wheel/f_FL_wheel_y_m",fgetVal/ratio);
            //         }
            //     }
            // }
        }

        sendResponseSuccess(source, obj->get("uuid").convert<std::string>());
    }
}
