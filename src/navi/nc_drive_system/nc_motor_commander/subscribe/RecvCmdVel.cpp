#include "nc_driver_main.hpp"

void Driver::RecvCmdvel(const geometry_msgs::Twist::ConstPtr& msg)
{
    if (b_charging_ == true) return;

    checktime_hearbeat_ = std::chrono::steady_clock::now();

    // 움직이는 속도값이 있을 때만 quad cmd 초기화
    float f_x = float(msg->linear.x);
    float f_y = float(msg->linear.y);
    float f_a = float(msg->angular.z);

    if (fabs(f_x) > 0.001 || fabs(f_y) > 0.001 || fabs(f_a) > 0.001) {
        string str = "";
        o_controller_->SetQuadCmd(str);
    }

    // 최대속도 제한
    float f_linear_vel = hypot(f_x, f_y);
    if (fabs(f_linear_vel) >= st_driver_param_.f_max_linear_vel_ms) {
        f_x = f_x * st_driver_param_.f_max_linear_vel_ms / fabs(f_linear_vel);
        f_y = f_y * st_driver_param_.f_max_linear_vel_ms / fabs(f_linear_vel);
    }
    if (fabs(f_a) >= st_driver_param_.f_max_angular_vel_degs) {
        f_a = f_a * st_driver_param_.f_max_angular_vel_degs / fabs(f_a);
    }

    {
        std::lock_guard<std::mutex> lock(mtx_cmd_);

        o_cmd_.SetVelX(f_x);
        o_cmd_.SetVelY(f_y);
        o_cmd_.SetVelYaw(f_a);

        if (float(msg->angular.x) < 0.1) {
            o_cmd_.SetDirectionFlag(0);  // 그냥 수동주행으로 명령온 경우
            if (fabs(f_linear_vel) > 0.001 || fabs(f_a) > 0.001)
                LOG_INFO("Manual Move Vel x : %.3f, y : %.3f, a : %.3f", f_x, f_y, f_a);
        }
        if (float(msg->angular.x) > 0.1 && float(msg->angular.x) < 1.1)
            o_cmd_.SetDirectionFlag(1);  // 자동주행으로 주행한 경우 forward
        if (float(msg->angular.x) > 1.1 && float(msg->angular.x) < 2.1)
            o_cmd_.SetDirectionFlag(2);  // 자동주행으로 주행한 경우 backward
        if (float(msg->angular.y) > 0.1)
            o_cmd_.SetDGFlag(true);  // y값으로 back flag 임시
        // LOG_DEBUG("o_cmd_ : %.3f %.3f %.3f %d ", o_cmd_.GetVelX(), o_cmd_.GetVelY(),
        //           o_cmd_.GetVelYaw(), o_cmd_.GetDirectionFlag());
    }
}