#include "nc_driver_main.hpp"

void Driver::RecvMotorDriverInit(const std_msgs::String::ConstPtr& msg)
{
    string str = msg->data;
    o_controller_->MotorDriverInit(str);
}