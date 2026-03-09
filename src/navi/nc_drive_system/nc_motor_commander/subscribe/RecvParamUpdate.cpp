#include "nc_driver_main.hpp"

void Driver::RecvParamUpdate(const std_msgs::String::ConstPtr& msg)
{
    if (b_param_init_)
        Initialize(true);
}