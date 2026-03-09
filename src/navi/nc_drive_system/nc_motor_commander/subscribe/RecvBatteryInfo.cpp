#include "nc_driver_main.hpp"

void Driver::RecvBatteryInfo(const core_msgs::BatteryInfo::ConstPtr& msg)
{
    b_charging_ = msg->b_charging_state; 
}