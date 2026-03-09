#include "nc_driver_main.hpp"

void Driver::RecvErrorReset(const std_msgs::String::ConstPtr& msg)
{
    string str = msg->data;
    o_controller_->MotorErrorReset(str);
}