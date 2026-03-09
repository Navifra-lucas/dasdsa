#include "controllerDD.hpp"

#include "util/logger.hpp"

namespace NaviFra {
void ControllerDD::Initialize(bool b_only_param_update)
{
    LOG_INFO("ControllerDD Initialize");
    float f_wheelbase = st_driver_param_.st_wheel_param.f_FL_wheel_y_m - st_driver_param_.st_wheel_param.f_RR_wheel_y_m;
    LOG_INFO("ControllerDD f_wheelbase : %.3f", f_wheelbase);
    o_kinematic_calculator_.SetDDInfo(f_wheelbase);

}


void ControllerDD::WriteCommand(const Cmd& o_cmd)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    Cmd o_command = o_cmd;

    DD_Wheel_t st_info = o_kinematic_calculator_.ResolveDDCmd(o_command);
    st_wheel_cmd_.f_FL_target_rpm = st_info.f_FL_wheel_scalar * 60.0f / (st_driver_param_.st_wheel_param.f_FL_wheel_diameter_m * M_PI) *
        st_driver_param_.st_traction_param.f_FL_traction_gear_ratio;
    st_wheel_cmd_.f_RR_target_rpm = st_info.f_RR_wheel_scalar * 60.0f / (st_driver_param_.st_wheel_param.f_RR_wheel_diameter_m * M_PI) *
        st_driver_param_.st_traction_param.f_RR_traction_gear_ratio;
    st_wheel_cmd_.f_FL_target_rpm = LimitRPM(st_wheel_cmd_.f_FL_target_rpm, n_rpm_max[MOTOR_TRACTION_FL], n_rpm_min[MOTOR_TRACTION_FL]);
    st_wheel_cmd_.f_RR_target_rpm = LimitRPM(st_wheel_cmd_.f_RR_target_rpm, n_rpm_max[MOTOR_TRACTION_RR], n_rpm_min[MOTOR_TRACTION_RR]);

    if (!st_driver_param_.st_traction_param.b_FL_traction_target_rpmd)
        st_wheel_cmd_.f_FL_target_rpm *= -1;
    if (!st_driver_param_.st_traction_param.b_RR_traction_target_rpmd)
        st_wheel_cmd_.f_RR_target_rpm *= -1;
    
    // target rpm이 있다면 브레이크 풀기
    
    if(st_driver_param_.b_use_brake){
        BrakeControl();   
    }
    std::chrono::duration<double> brake_release_sec = std::chrono::steady_clock::now() - brake_release_checktime_;
    if (b_brake_on_feedback_ && brake_release_sec.count()*1000 < st_driver_param_.f_brake_release_time_ms)  // brake 걸려 있으면
    {
        if (st_wheel_cmd_.f_FL_target_rpm > 0.f)
            st_wheel_cmd_.f_FL_target_rpm = 0.1;
        if (st_wheel_cmd_.f_FL_target_rpm < 0.f)
            st_wheel_cmd_.f_FL_target_rpm = -0.1;

        if (st_wheel_cmd_.f_RR_target_rpm > 0.f)
            st_wheel_cmd_.f_RR_target_rpm = 0.1;
        if (st_wheel_cmd_.f_RR_target_rpm < 0.f)
            st_wheel_cmd_.f_RR_target_rpm = -0.1;
    }

    float f_FL_convert_target = ConvertFromRPM(MotorType::Traction, st_wheel_cmd_.f_FL_target_rpm);
    float f_RR_convert_target = ConvertFromRPM(MotorType::Traction, st_wheel_cmd_.f_RR_target_rpm);

    motor_msgs::MotorTarget motor_target;
    motor_msgs::MotorTargetInfo msg;

    motor_target.motor_id = MOTOR_TRACTION_FL;
    motor_target.operation_mode = 3;
    motor_target.target = f_FL_convert_target;
    msg.data.push_back(motor_target);
    motor_target.motor_id = MOTOR_TRACTION_RR;
    motor_target.target = f_RR_convert_target;
    msg.data.push_back(motor_target);

    pub_rpm_.publish(msg);

    std::chrono::duration<double> sec = std::chrono::steady_clock::now() - start_time;
    if (sec.count() > 0.1) {
        LOG_ERROR("%.3f millisec over", sec.count() * 1000);
    }
}

void ControllerDD::BrakeControl()
{
    static bool b_stop_check = false;
    static std::chrono::steady_clock::time_point stop_checktime = std::chrono::steady_clock::now();
    motor_msgs::MotorBrake motor_brake;
    motor_brake.b_brake_control = !st_driver_param_.b_external_brake_control;
    motor_brake.b_link_brake_and_servo = st_driver_param_.b_link_brake_and_servo;   
    if ((fabs(st_wheel_cmd_.f_FL_target_rpm) > 0.001f || fabs(st_wheel_cmd_.f_RR_target_rpm) > 0.001f) &&
        b_brake_on_feedback_) {
        brake_release_checktime_ = std::chrono::steady_clock::now();
        b_stop_check = false;
        b_brake_on_flag_ = false;
        motor_brake.b_brake_target = false;
        motor_brake.b_servo_on = true;   
        pub_brake_.publish(motor_brake);
    }
    else if (
        (fabs(st_wheel_cmd_.f_FL_target_rpm) < 0.001f &&
        fabs(st_wheel_cmd_.f_RR_target_rpm) < 0.001f)  // rpm이 없고, 피드백도 작아질때 brake on
        && (fabs(st_wheel_data_[MOTOR_TRACTION_FL].f_traction_feedback_rpm) < st_driver_param_.f_brake_threshold_rpm || 
            fabs(st_wheel_data_[MOTOR_TRACTION_RR].f_traction_feedback_rpm) < st_driver_param_.f_brake_threshold_rpm) &&
            !b_brake_on_feedback_) {
        if(!b_stop_check){
            stop_checktime = std::chrono::steady_clock::now();
            b_stop_check = true;
        }
        std::chrono::duration<double> brake_lock_sec = std::chrono::steady_clock::now() - stop_checktime;
        if(brake_lock_sec.count()*1000 > st_driver_param_.f_brake_lock_wait_time_ms){
            b_brake_on_flag_ = true;
            motor_brake.b_brake_target = true;
            motor_brake.b_servo_on = false;   
            pub_brake_.publish(motor_brake);
        }
    }
}

void ControllerDD::calculateOdom()
{
    /*
        오돔 업데이트
    */
    NaviFra::SimplePos o_dpos;
    WheelData_t st_data;

    double FL_distance_per_pulse = (st_driver_param_.st_wheel_param.f_FL_wheel_diameter_m * M_PI) /
        (st_driver_param_.st_traction_param.f_FL_traction_encoder_pulse * st_driver_param_.st_traction_param.f_FL_traction_gear_ratio);
    double RR_distance_per_pulse = (st_driver_param_.st_wheel_param.f_RR_wheel_diameter_m * M_PI) /
        (st_driver_param_.st_traction_param.f_RR_traction_encoder_pulse * st_driver_param_.st_traction_param.f_RR_traction_gear_ratio);

    st_data.f_FL_scalar = float(st_wheel_data_[MOTOR_TRACTION_FL].d_traction_encoder_acc * FL_distance_per_pulse);
    st_data.f_RR_scalar = float(st_wheel_data_[MOTOR_TRACTION_RR].d_traction_encoder_acc * RR_distance_per_pulse);
    o_dpos = o_kinematic_calculator_.CalcDposDDWheel(st_data);

    if (hypot(o_dpos.GetXm(), o_dpos.GetYm()) > 0.2 || fabs(o_dpos.GetDeg()) > 5) {
        LOG_ERROR(
            "odom large error %.3f, %.3f, %.3f / scalar F %.3f L %.3f / encoder acc F %.3f L %.3f", o_dpos.GetXm(), o_dpos.GetYm(),
            o_dpos.GetDeg(), st_data.f_FL_scalar, st_data.f_RR_scalar, st_wheel_data_[MOTOR_TRACTION_FL].d_traction_encoder_acc,
            st_wheel_data_[MOTOR_TRACTION_RR].d_traction_encoder_acc);
        o_dpos.SetXm(0);
        o_dpos.SetYm(0);
        o_dpos.SetDeg(0);

        std_msgs::String msg;
        msg.data = "[mt]odom_large";
        pub_warning_.publish(msg);
    }

    st_wheel_data_[MOTOR_TRACTION_FL].d_traction_encoder_acc = 0;
    st_wheel_data_[MOTOR_TRACTION_RR].d_traction_encoder_acc = 0;

    calculateVelocity(o_dpos);

    return;
}

void ControllerDD::calculateVelocity(const NaviFra::SimplePos& pose)
{
    // 속도 계산
    WheelData_t st_data_s;
    st_data_s.f_FL_scalar = st_wheel_data_[MOTOR_TRACTION_FL].f_traction_feedback_rpm / 60.0f /
        st_driver_param_.st_traction_param.f_FL_traction_gear_ratio * (st_driver_param_.st_wheel_param.f_FL_wheel_diameter_m * M_PI);
    st_data_s.f_RR_scalar = st_wheel_data_[MOTOR_TRACTION_RR].f_traction_feedback_rpm / 60.0f /
        st_driver_param_.st_traction_param.f_RR_traction_gear_ratio * (st_driver_param_.st_wheel_param.f_RR_wheel_diameter_m * M_PI);
    NaviFra::SimplePos o_vel = o_kinematic_calculator_.CalcCmdDDWheel(st_data_s);
    NaviFra::SimplePos o_pos = pose;
    CalOdomAndVel(o_pos, o_vel);
}

void ControllerDD::updateMotorData(const MotorDataMap& motor_data)
{
    for (const auto& data : motor_data) {
        MotorId motor_id = data.first;  // 키 (motor_id)
        motor_msgs::MotorData value = data.second;  // 값 (MotorData)
        if (motor_id == MOTOR_TRACTION_FL) {
            float f_feedback_rpm = ConvertToRPM(MotorType::Traction,value.feedback_velocity);
            st_wheel_data_[MOTOR_TRACTION_FL].f_traction_current = value.current;
            st_wheel_data_[MOTOR_TRACTION_FL].f_traction_voltage = value.voltage;
            st_wheel_data_[MOTOR_TRACTION_FL].f_traction_feedback_rpm = 
            f_feedback_rpm * (st_driver_param_.st_traction_param.b_FL_traction_feedback_rpmd ? 1 : -1);
                st_wheel_data_[MOTOR_TRACTION_FL].d_traction_encoder =
                value.encoder * (st_driver_param_.st_traction_param.b_FL_traction_encoderd ? 1 : -1);
            st_wheel_data_[MOTOR_TRACTION_FL].d_traction_encoder_acc =
                value.acc_encoder * (st_driver_param_.st_traction_param.b_FL_traction_encoderd ? 1 : -1);
            st_wheel_data_[MOTOR_TRACTION_FL].b_is_enable = value.is_enable;
            st_wheel_data_[MOTOR_TRACTION_FL].b_is_error = value.is_error;
            st_wheel_data_[MOTOR_TRACTION_FL].b_feedback_brake_on = value.feedback_brake_on;
            vec_f_FL_traction_motor_feedback_hz.emplace_back(std::chrono::steady_clock::now());
            if (vec_f_FL_traction_motor_feedback_hz.size() > 10)
                vec_f_FL_traction_motor_feedback_hz.erase(vec_f_FL_traction_motor_feedback_hz.begin());
        }
        else if (motor_id == MOTOR_TRACTION_RR) {
            float f_feedback_rpm = ConvertToRPM(MotorType::Traction,value.feedback_velocity);
            st_wheel_data_[MOTOR_TRACTION_RR].f_traction_current = value.current;
            st_wheel_data_[MOTOR_TRACTION_RR].f_traction_voltage = value.voltage;
            st_wheel_data_[MOTOR_TRACTION_RR].f_traction_feedback_rpm = 
            f_feedback_rpm * (st_driver_param_.st_traction_param.b_RR_traction_feedback_rpmd ? 1 : -1);
            st_wheel_data_[MOTOR_TRACTION_RR].d_traction_encoder =
                value.encoder * (st_driver_param_.st_traction_param.b_RR_traction_encoderd ? 1 : -1);
            st_wheel_data_[MOTOR_TRACTION_RR].d_traction_encoder_acc =
                value.acc_encoder * (st_driver_param_.st_traction_param.b_RR_traction_encoderd ? 1 : -1);
            st_wheel_data_[MOTOR_TRACTION_RR].b_is_enable = value.is_enable;
            st_wheel_data_[MOTOR_TRACTION_RR].b_is_error = value.is_error;
            st_wheel_data_[MOTOR_TRACTION_RR].b_feedback_brake_on = value.feedback_brake_on;
            vec_f_RR_traction_motor_feedback_hz.emplace_back(std::chrono::steady_clock::now());
            if (vec_f_RR_traction_motor_feedback_hz.size() > 10)
                vec_f_RR_traction_motor_feedback_hz.erase(vec_f_RR_traction_motor_feedback_hz.begin());
        }
    }
    // Motor Info pub
    core_msgs::MotorInfo o_motor_info;
    {
        o_motor_info.f_robot_type = 0;
        o_motor_info.f_FL_traction_motor_current = st_wheel_data_[MOTOR_TRACTION_FL].f_traction_current;
        o_motor_info.f_RR_traction_motor_current = st_wheel_data_[MOTOR_TRACTION_RR].f_traction_current;
        o_motor_info.f_FL_traction_motor_voltage = st_wheel_data_[MOTOR_TRACTION_FL].f_traction_voltage;
        o_motor_info.f_RR_traction_motor_voltage = st_wheel_data_[MOTOR_TRACTION_RR].f_traction_voltage;
        o_motor_info.f_FL_traction_motor_target_rpm = st_wheel_cmd_.f_FL_target_rpm;
        o_motor_info.f_RR_traction_motor_target_rpm = st_wheel_cmd_.f_RR_target_rpm;
        o_motor_info.f_FL_traction_motor_feedback_rpm = st_wheel_data_[MOTOR_TRACTION_FL].f_traction_feedback_rpm;
        o_motor_info.f_RR_traction_motor_feedback_rpm = st_wheel_data_[MOTOR_TRACTION_RR].f_traction_feedback_rpm;
        o_motor_info.f_FL_traction_motor_feedback_rpm_hz = GetMS(vec_f_FL_traction_motor_feedback_hz);
        o_motor_info.f_RR_traction_motor_feedback_rpm_hz = GetMS(vec_f_RR_traction_motor_feedback_hz);
        o_motor_info.f_FL_traction_motor_encoder = st_wheel_data_[MOTOR_TRACTION_FL].d_traction_encoder;
        o_motor_info.f_RR_traction_motor_encoder = st_wheel_data_[MOTOR_TRACTION_RR].d_traction_encoder;
        o_motor_info.f_FL_traction_motor_encoder_hz = GetMS(vec_f_FL_traction_motor_feedback_hz);
        o_motor_info.f_RR_traction_motor_encoder_hz = GetMS(vec_f_RR_traction_motor_feedback_hz);
        o_motor_info.b_brake_on_target = b_brake_on_flag_;
        if(st_driver_param_.b_external_brake_control)
            o_motor_info.b_brake_on_feedback = b_brake_on_external_feedback_;
        else
            o_motor_info.b_brake_on_feedback = st_wheel_data_[MOTOR_TRACTION_FL].b_feedback_brake_on && st_wheel_data_[MOTOR_TRACTION_RR].b_feedback_brake_on;
        b_brake_on_feedback_ = o_motor_info.b_brake_on_feedback;
    }

    Notify("motor_info_callback", o_motor_info);
}



bool ControllerDD::isEnable()
{
    return st_wheel_data_[MOTOR_TRACTION_FL].b_is_enable && st_wheel_data_[MOTOR_TRACTION_RR].b_is_enable;
}

void ControllerDD::Enable()
{
    if (!st_wheel_data_[MOTOR_TRACTION_FL].b_is_enable) {
        EnableMotor(MOTOR_TRACTION_FL);
    }
    if (!st_wheel_data_[MOTOR_TRACTION_RR].b_is_enable) {
        EnableMotor(MOTOR_TRACTION_RR);
    }
}

};  // namespace NaviFra
