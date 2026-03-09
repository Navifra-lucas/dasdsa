#include "nc_driver_main.hpp"

void Driver::RecvSteerOpen(const std_msgs::Bool::ConstPtr& msg)
{
    o_controller_->SteerAlotOpen(msg->data);
}