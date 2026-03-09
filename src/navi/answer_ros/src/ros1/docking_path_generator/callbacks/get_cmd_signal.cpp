#include "ros1/docking_path_generator/docking_path_generator_ros1.hpp"

#include "logger/logger.h"

namespace NVFR {

void DockingPathGeneratorRos1::GetParamReadSignal(
    const std_msgs::Bool::ConstPtr msg)
{
    LoadParam();
}

void DockingPathGeneratorRos1::GetLogLv(
    const std_msgs::String::ConstPtr msg)
{
    logger_->SetLogLevel(msg->data);
}

} // namespace NVFR
