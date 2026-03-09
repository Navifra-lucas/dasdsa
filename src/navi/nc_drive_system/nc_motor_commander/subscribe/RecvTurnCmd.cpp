#include "nc_driver_main.hpp"

void Driver::RecvTurnCmd(const std_msgs::Float32::ConstPtr& msg)
{
    f_turn_table_vel_ = float(msg->data);
    checktime_turn_table_hearbeat_ = std::chrono::steady_clock::now();
}