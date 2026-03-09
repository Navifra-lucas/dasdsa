#include "ros1/navi/navigator_ros1.hpp"

#include "logger/logger.h"

namespace NVFR {

void NavigatorRos1::GetCmdSignal(
    const std_msgs::String::ConstPtr msg)
{
    std::string s_signal = msg->data;
    LOG_INFO("Cmd Signal : {}", s_signal.c_str());
    if (s_signal == "cancel") {
        o_navigator_ptr_->Cancel();
    }
    else if (s_signal == "onecyclestop") {
        o_navigator_ptr_->OneCycleStop();
    }
    else if (s_signal == "pause") {
        o_navigator_ptr_->Pause();
    }
    else if (s_signal == "resume") {
        o_navigator_ptr_->Resume();
    }
}

void NavigatorRos1::GetParamReadSignal(
    const std_msgs::Bool::ConstPtr msg)
{
    LoadParam();
}

void NavigatorRos1::GetLogLv(
    const std_msgs::String::ConstPtr msg)
{
    logger_->SetLogLevel(msg->data);
}

} // namespace NVFR
