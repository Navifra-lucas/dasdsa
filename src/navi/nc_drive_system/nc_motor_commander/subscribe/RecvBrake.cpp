#include "nc_driver_main.hpp"

void Driver::RecvBrake(const std_msgs::Bool::ConstPtr& msg)
{
    {
        std::lock_guard<std::mutex> lock(mtx_brake_);
        checktime_brake_ = std::chrono::steady_clock::now();
    }
    o_controller_->SetBrake(msg->data);
}