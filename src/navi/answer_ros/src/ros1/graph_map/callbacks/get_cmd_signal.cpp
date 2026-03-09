#include "ros1/graph_map/graph_map_ros1.hpp"

#include "logger/logger.h"

namespace NVFR {

void GraphMapRos1::GetParamReadSignal(
    const std_msgs::Bool::ConstPtr msg)
{
    LoadParam();
}

void GraphMapRos1::GetLogLv(
    const std_msgs::String::ConstPtr msg)
{
    logger_->SetLogLevel(msg->data);
}

} // namespace NVFR
