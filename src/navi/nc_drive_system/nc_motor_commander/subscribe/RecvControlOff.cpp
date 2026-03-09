#include "nc_driver_main.hpp"

void Driver::RecvControlOff(const std_msgs::Bool::ConstPtr& msg)
{
    o_controller_->ControlOff(msg->data);
}