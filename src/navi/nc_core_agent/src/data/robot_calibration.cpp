#include "core_agent/core_agent.h"

#include <core_agent/data/robot_calibration.h>
#include <core_agent/data/types.h>

using namespace NaviFra;
const std::string RobotCalibration::KEY = "RobotCalibration";

RobotCalibration::RobotCalibration()
{
}

RobotCalibration::~RobotCalibration()
{
    result_.clear();
}

void RobotCalibration::setCurrentType(std::string type)
{
    currnt_type_ = type;
}

std::string RobotCalibration::currentType()
{
    return currnt_type_;
}

std::string RobotCalibration::calibrationCommand(const std::string type)
{
    std::string strCaliType;  //
    if (type == TypeCalibration::FOWARD) {
        strCaliType = "v_f";
    }
    else if (type == TypeCalibration::BACKWARD) {
        strCaliType = "v_b";
    }
    else if (type == TypeCalibration::LEFT) {
        strCaliType = "w_p";
    }
    else if (type == TypeCalibration::RIGHT) {
        strCaliType = "w_m";
    }
    else if (type == TypeCalibration::LIDAR) {
        strCaliType = "lidarcal";
    }
    else {
        strCaliType = "";
    }  //
    return strCaliType;
}

void RobotCalibration::appendResult(float res)
{
    result_.push_back(res);
}

void RobotCalibration::clear()
{
    result_.clear();
}

size_t RobotCalibration::size()
{
    return result_.size();
}

float RobotCalibration::at(size_t index)
{
    try {
        return result_.at(index);
    }
    catch (...) {
        return 0.f;
    }
}