#include "nc_driver_main.hpp"

void Driver::RecvMotorDriverInterlock(const std_msgs::Bool::ConstPtr& msg)
{
    if (b_motor_driver_interlock_ != msg->data) {
        LOG_INFO("Subscribe motor driver interlock");
        b_motor_driver_interlock_ = msg->data;
        o_controller_->SetMotorInterLock(b_motor_driver_interlock_);
    }
}