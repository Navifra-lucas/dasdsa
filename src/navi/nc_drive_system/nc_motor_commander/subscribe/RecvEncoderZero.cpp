#include "nc_driver_main.hpp"

void Driver::RecvEncoderZero(const std_msgs::String::ConstPtr& msg)
{
    string str = msg->data;
    o_controller_->EncoderZero(str);
}