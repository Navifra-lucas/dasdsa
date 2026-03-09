#include "nc_driver_main.hpp"

void Driver::RecvESTOP(const std_msgs::Bool::ConstPtr& msg)
{
    b_estop_ = msg->data;
}
