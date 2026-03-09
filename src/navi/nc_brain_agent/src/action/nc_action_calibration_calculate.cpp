#include "nc_brain_agent/nc_brain_agent.h"

#include <core_agent/data/robot_calibration.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/data/types.h>
#include <nc_brain_agent/action/nc_action_calibration_calculate.h>

using namespace NaviFra;

NcActionCalibrationCalculate::NcActionCalibrationCalculate()
{
}

NcActionCalibrationCalculate::~NcActionCalibrationCalculate()
{
}

std::string NcActionCalibrationCalculate::implName()
{
    return "NcActionCalibrationCalculate";
}

void NcActionCalibrationCalculate::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    auto dmCalibration = InMemoryRepository::instance().get<RobotCalibration>(RobotCalibration::KEY);
    if (dmCalibration->size() == 7) {
        Poco::JSON::Object message;
        message.set("id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());
        message.set("type", dmCalibration->currentType());
        if (dmCalibration->currentType() == TypeCalibration::FOWARD || dmCalibration->currentType() == TypeCalibration::LEFT) {
            message.set("percent", dmCalibration->at(6));
            message.set("x", dmCalibration->at(0));
            message.set("y", dmCalibration->at(1));
            message.set("deg", dmCalibration->at(2));
        }
        else if (dmCalibration->currentType() == TypeCalibration::BACKWARD || dmCalibration->currentType() == TypeCalibration::RIGHT) {
            message.set("percent", dmCalibration->at(6));
            message.set("x", dmCalibration->at(3));
            message.set("y", dmCalibration->at(4));
            message.set("deg", dmCalibration->at(5));
        }
        else if (dmCalibration->currentType() == TypeCalibration::LIDAR) {
            message.set("cmd", "move");
            message.set("completed", 10);  // grey : 라이다 코드 추가필요
            message.set("total", 10);  // grey : 라이다 코드 추가필요
        }
        else {
            LOG_ERROR("Can not found type of robot-calibration !");
        }
        sendResponseSuccessWithData(source, obj->get("uuid").extract<std::string>(), message);
    }
}
