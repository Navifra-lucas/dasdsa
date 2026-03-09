#include "nc_driver_main.hpp"

void Driver::RecvTurnBrake(const std_msgs::Bool::ConstPtr& msg)
{
    b_turn_feedback_brakeon_ = bool(msg->data);
}