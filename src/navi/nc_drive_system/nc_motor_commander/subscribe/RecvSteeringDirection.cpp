#include "nc_driver_main.hpp"

void Driver::RecvSteeringDirection(const std_msgs::Int16::ConstPtr& msg)
{
    o_controller_->SpinturnSteerDirection(msg->data);
}