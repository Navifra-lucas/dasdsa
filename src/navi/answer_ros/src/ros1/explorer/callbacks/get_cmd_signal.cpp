#include "ros1/explorer/explorer_ros1.hpp"

#include "logger/logger.h"

namespace NVFR {

void ExplorerRos1::GetParamReadSignal(
    const std_msgs::Bool::ConstPtr msg)
{
    LoadParam();
}

void ExplorerRos1::GetLogLv(
    const std_msgs::String::ConstPtr msg)
{
    logger_->SetLogLevel(msg->data);
}

} // namespace NVFR
