#include "nc_driver_main.hpp"

void Driver::RecvQuadCmd(const std_msgs::String::ConstPtr& msg)
{
    checktime_hearbeat_ = std::chrono::steady_clock::now();
    quad_str_ = msg->data;
    // o_controller_->SetQuadCmd(str);
}