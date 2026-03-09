#include "nc_driver_main.hpp"

void Driver::RecvAlarmUpdate(const core_msgs::NaviAlarm::ConstPtr& msg)
{
    if (msg->alarm == core_msgs::NaviAlarm::GOAL_ARRIVED)
        o_controller_->SpinturnSteerDirection(0);
}