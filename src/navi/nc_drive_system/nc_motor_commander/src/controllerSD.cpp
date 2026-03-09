#include "controllerSD.hpp"

namespace NaviFra {
void ControllerSD::Initialize(bool b_only_param_update)
{
    LOG_INFO(
        "ControllerSD wheel_pos F : (%.3f, %.3f, %.3f) ", st_driver_param_.st_wheel_param.f_FL_wheel_x_m,
        st_driver_param_.st_wheel_param.f_FL_wheel_y_m, st_driver_param_.st_wheel_param.f_FL_wheel_axis_offset_m);

    o_kinematic_calculator_.SetSDInfo(
        st_driver_param_.st_wheel_param.f_FL_wheel_x_m, st_driver_param_.st_wheel_param.f_FL_wheel_y_m,
        st_driver_param_.st_wheel_param.f_FL_wheel_axis_offset_m);
}

// void ControllerSD::EncoderZero(string& str_data)
// {

// }

// void ControllerSD::SteerAlotOpen(bool b_data)
// {

// }

void ControllerSD::SetQuadCmd(string& str_data)
{
    std::lock_guard<std::mutex> lock(mtx_quad_cmd_);  // 멀티스레드 안전
    quad_cmd_str_ = str_data;
}

string ControllerSD::GetQuadCmd()
{
    std::lock_guard<std::mutex> lock(mtx_quad_cmd_);  // 멀티스레드 안전
    return quad_cmd_str_;
}

// void ControllerSD::SetMotorGain(string& str_data)
// {
//     string str = str_data;
// }

void ControllerSD::SpinturnSteerDirection(int n_data)
{
    LOG_INFO("SpinturnSteerDirection : %d", n_data);  // 1이면 둘다 -90, 2면 둘다 +90

    o_kinematic_calculator_.SetSpinturnSteerDirection(n_data);
}

void ControllerSD::WriteCommand(const Cmd& o_cmd)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    Cmd o_command = o_cmd;
    string str_quad_cmd = GetQuadCmd();
    if (str_quad_cmd.size() > 0) {
        istringstream iss(str_quad_cmd);  // istringstream에 str을 담는다.
        string buffer;  // 구분자를 기준으로 절삭된 문자열이 담겨지는 버퍼
        vector<string> result;
        while (getline(iss, buffer, '/')) {
            result.push_back(buffer);  // 절삭된 문자열을 vector에 저장
        }
        if (result.size() != 2) {
            string str = "";
            SetQuadCmd(str);
        }
        float f_speed_ms = stof(result[0]);
        float f_angle_deg = stof(result[1]);

        int n_target_velocity_f = f_speed_ms * 60.0f / (st_driver_param_.st_wheel_param.f_FL_wheel_diameter_m * M_PI) *
            st_driver_param_.st_traction_param.f_FL_traction_gear_ratio;

        st_wheel_cmd_.f_FL_target_deg = f_angle_deg;

        o_kinematic_calculator_.st_qd_wheel_.f_FL_wheel_steer_angle_rad_pre = st_wheel_cmd_.f_FL_target_deg * DEGtoRAD;
        // quad cmd는 스티어링 원점 조절할때 쓰기 때문에, 이때는 스티어링 파워를 끄면 안되고 계속 제어해줘야함.
        st_wheel_cmd_.b_steer_control_always = true;
    }
    else {
        SD_Wheel_t st_info = o_kinematic_calculator_.ResolveSDCmd(o_command);

        st_wheel_cmd_.f_FL_target_rpm = st_info.f_FL_wheel_scalar * 60.0f / (st_driver_param_.st_wheel_param.f_FL_wheel_diameter_m * M_PI) *
            st_driver_param_.st_traction_param.f_FL_traction_gear_ratio;
        st_wheel_cmd_.f_FL_target_deg = st_info.f_FL_wheel_steer_angle_rad * RADtoDEG;
        st_wheel_cmd_.b_steer_control_always = false;
    }
    if (!st_driver_param_.st_traction_param.b_FL_traction_target_rpmd)
        st_wheel_cmd_.f_FL_target_rpm *= -1;
    if (!st_driver_param_.st_steer_param.b_FL_target_steer_angled)
        st_wheel_cmd_.f_FL_target_deg *= -1;
        st_wheel_cmd_.f_FL_target_deg += f_steer_offse_deg_;
    float f_target_angle = st_wheel_cmd_.f_FL_target_deg;
    float f_actual_angle = st_wheel_data_[MOTOR_STEER_FL].f_steer_angle_deg;  //수정필요
    static bool b_stop = true;
    float f_angle_gap_stop = fabs(f_target_angle - f_actual_angle);

    if ((o_command.GetVelX() == 0 && o_command.GetVelY() == 0 && 
        fabs(st_wheel_data_[MOTOR_TRACTION_FL].f_traction_feedback_rpm) < st_driver_param_.f_brake_threshold_rpm) ||
        (o_command.GetVelX() == 0 && o_command.GetVelY() == 0 && o_command.GetVelYaw() != 0)) {
        b_stop = true;  // 정지
    }
    else if (b_stop && (o_command.GetVelX() != 0 || o_command.GetVelY() != 0))  // 출발
    {
        if (f_angle_gap_stop < st_driver_param_.st_steer_param.f_steer_stop_thr_deg / 3)
            b_stop = false;
    }

    float MAX_ANGLE = st_driver_param_.st_steer_param.f_steer_start_thr_deg;
    if (b_stop) {
        MAX_ANGLE = st_driver_param_.st_steer_param.f_steer_stop_thr_deg;
        b_steer_align_ = true;
    }
    else {
        b_steer_align_ = false;
    }

    float f_angle_gap = min(fabs(f_target_angle - f_actual_angle), MAX_ANGLE + 0.1f);
    float f_rpm_ratio = 1;
    if (f_angle_gap >= MAX_ANGLE) {
        f_rpm_ratio = 0;
    }
    else if (f_angle_gap >= st_driver_param_.st_steer_param.f_steer_stop_thr_deg) {
        f_rpm_ratio = fabs(1 - f_angle_gap / MAX_ANGLE);
    }
    {
        st_wheel_cmd_.f_FL_target_rpm *= f_rpm_ratio;
    }

    st_wheel_cmd_.f_FL_target_rpm = LimitRPM(st_wheel_cmd_.f_FL_target_rpm, n_rpm_max[MOTOR_TRACTION_FL], n_rpm_min[MOTOR_TRACTION_FL]);

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
    }

    float f_FL_convert_target = ConvertFromRPM(MotorType::Traction, st_wheel_cmd_.f_FL_target_rpm);

    motor_msgs::MotorTarget motor_target;
    motor_msgs::MotorTargetInfo msg;

    motor_target.motor_id = MOTOR_TRACTION_FL;
    motor_target.operation_mode = 3;
    motor_target.target = f_FL_convert_target;
    msg.data.push_back(motor_target);
    motor_target.motor_id = MOTOR_STEER_FL;
    motor_target.operation_mode = 1;
    motor_target.target = st_wheel_cmd_.f_FL_target_deg;
    if (st_driver_param_.st_steer_param.n_steer_control_mode == 0) {
        float f_FL_steer_target_rpm = SteerDEGtoRPM(MOTOR_STEER_FL, st_wheel_cmd_.f_FL_target_deg, st_wheel_data_[MOTOR_STEER_FL].f_steer_angle_deg);
        motor_target.target = ConvertFromRPM(MotorType::Steer, f_FL_steer_target_rpm);
        motor_target.operation_mode = 3;
    }
    msg.data.push_back(motor_target);
    pub_rpm_.publish(msg);

    std::chrono::duration<double> sec = std::chrono::steady_clock::now() - start_time;
    if (sec.count() > 0.1) {
        LOG_ERROR("%.3f millisec over", sec.count() * 1000);
    }
}

void ControllerSD::BrakeControl()
{
    static bool b_stop_check = false;
    static std::chrono::steady_clock::time_point stop_checktime = std::chrono::steady_clock::now();
    motor_msgs::MotorBrake motor_brake;
    motor_brake.b_brake_control = !st_driver_param_.b_external_brake_control;
    motor_brake.b_link_brake_and_servo = st_driver_param_.b_link_brake_and_servo;   
    if (fabs(st_wheel_cmd_.f_FL_target_rpm) > 0.001f && b_brake_on_feedback_) {
        brake_release_checktime_ = std::chrono::steady_clock::now();
        b_stop_check = false;
        b_brake_on_flag_ = false;
        motor_brake.b_brake_target = false;
        motor_brake.b_servo_on = true;   
        pub_brake_.publish(motor_brake);
    }
    else if (
        fabs(st_wheel_cmd_.f_FL_target_rpm) < 0.001f &&
        fabs(st_wheel_data_[MOTOR_TRACTION_FL].f_traction_feedback_rpm) < st_driver_param_.f_brake_threshold_rpm &&
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

void ControllerSD::calculateOdom()
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    static std::chrono::system_clock::time_point tp_prev = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point tp_now = std::chrono::system_clock::now();
    std::chrono::duration<double> f_period = tp_now - tp_prev;
    tp_prev = tp_now;  // 다음 계산을 위한 갱신

    /*
        오돔 업데이트
    */
    NaviFra::SimplePos o_dpos;
    WheelData_t st_data;

    // double FL_distance_per_pulse = (st_driver_param_.st_wheel_param.f_FL_wheel_diameter_m * M_PI) /
        // (st_driver_param_.st_traction_param.f_FL_traction_encoder_pulse * st_driver_param_.st_traction_param.f_FL_traction_gear_ratio);
    // st_data.f_FL_scalar = float(st_wheel_data_[MOTOR_TRACTION_FL].d_traction_encoder_acc * FL_distance_per_pulse);
    st_data.f_FL_scalar = float(st_wheel_data_[MOTOR_TRACTION_FL].f_traction_feedback_rpm / 60.0f * f_period.count() * (st_driver_param_.st_wheel_param.f_FL_wheel_diameter_m * M_PI) / st_driver_param_.st_traction_param.f_FL_traction_gear_ratio);
    st_data.f_FL_angle_rad = st_wheel_data_[MOTOR_STEER_FL].f_steer_angle_deg * DEGtoRAD;
    o_dpos = o_kinematic_calculator_.CalcDposSDWheel(st_data);
    // NLOG(info) << "이동거리 계산: " << st_data.f_FL_scalar << ", feedback_rpm: " << st_wheel_data_[MOTOR_TRACTION_FL].f_traction_feedback_rpm 
    // << " , f_period: " << f_period.count() << " , 휠 지름: " << (st_driver_param_.st_wheel_param.f_FL_wheel_diameter_m * M_PI) << " , 기어비: " << st_driver_param_.st_traction_param.f_FL_traction_gear_ratio;
    if (hypot(o_dpos.GetXm(), o_dpos.GetYm()) > 0.2 || fabs(o_dpos.GetDeg()) > 10) {
        LOG_ERROR(
            "odom large error %.3f, %.3f, %.3f / scalar F %.3f / encoder acc F %.3f", o_dpos.GetXm(), o_dpos.GetYm(), o_dpos.GetDeg(),
            st_data.f_FL_scalar, st_wheel_data_[MOTOR_TRACTION_FL].d_traction_encoder_acc);
        o_dpos.SetXm(0);
        o_dpos.SetYm(0);
        o_dpos.SetDeg(0);
        std_msgs::String msg;
        msg.data = "[mt]odom_large";
        pub_warning_.publish(msg);
    }

    st_wheel_data_[MOTOR_TRACTION_FL].d_traction_encoder_acc = 0;

    calculateVelocity(o_dpos);
    std::chrono::duration<double> sec = std::chrono::steady_clock::now() - start_time;
    if (sec.count() > 0.1) {
        LOG_ERROR("%.3f millisec over", sec.count() * 1000);
    }
    return;
}

void ControllerSD::calculateVelocity(const NaviFra::SimplePos& pose)
{
    // 속도 계산
    WheelData_t st_data_s;
    st_wheel_data_[MOTOR_TRACTION_FL].f_traction_feedback_rpm = st_wheel_data_[MOTOR_TRACTION_FL].f_traction_feedback_rpm * (st_driver_param_.st_traction_param.b_FL_traction_feedback_rpmd ? 1 : -1);
    st_data_s.f_FL_scalar = st_wheel_data_[MOTOR_TRACTION_FL].f_traction_feedback_rpm / 60.0f /
        st_driver_param_.st_traction_param.f_FL_traction_gear_ratio * (st_driver_param_.st_wheel_param.f_FL_wheel_diameter_m * M_PI);
    st_data_s.f_FL_angle_rad = st_wheel_data_[MOTOR_STEER_FL].f_steer_angle_deg * DEGtoRAD;
    NaviFra::SimplePos o_vel = o_kinematic_calculator_.CalcCmdSDWheel(st_data_s);
    NaviFra::SimplePos o_pos = pose;
    CalOdomAndVel(o_pos, o_vel);
}

void ControllerSD::updateMotorData(const MotorDataMap& motor_data)
{
    for (const auto& data : motor_data) {
        MotorId motor_id = data.first;  // 키 (motor_id)
        float f_steer_feedback_deg;
        motor_msgs::MotorData value = data.second;  // 값 (MotorData)
        if (motor_id == MOTOR_TRACTION_FL) {
            float f_feedback_rpm = ConvertToRPM(MotorType::Traction,value.feedback_velocity);
            st_wheel_data_[MOTOR_TRACTION_FL].f_traction_current = value.current;
            st_wheel_data_[MOTOR_TRACTION_FL].f_traction_voltage = value.voltage;
            st_wheel_data_[MOTOR_TRACTION_FL].f_traction_feedback_rpm = f_feedback_rpm * (st_driver_param_.st_traction_param.b_FL_traction_feedback_rpmd ? 1 : -1);
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
        else if (motor_id == MOTOR_STEER_FL) {
            float f_feedback_rpm = ConvertToRPM(MotorType::Steer,value.feedback_velocity);
            st_wheel_data_[MOTOR_STEER_FL].f_steer_current = value.current;
            st_wheel_data_[MOTOR_STEER_FL].f_steer_feedback_rpm = f_feedback_rpm;
            if (st_driver_param_.st_steer_param.n_steer_control_mode == 0) {
                f_steer_feedback_deg = (double(value.encoder) / st_driver_param_.st_steer_param.f_FL_steer_encoder_pulse * 360 *
                     st_driver_param_.st_steer_param.f_FL_steer_gear_ratio + st_driver_param_.st_etc_param.f_FL_abssteer_offset);
            }
            else if (st_driver_param_.st_steer_param.n_steer_control_mode == 1) {
                f_steer_feedback_deg = value.feedback_angle;
            }
            st_wheel_data_[MOTOR_STEER_FL].f_steer_angle_deg = f_steer_feedback_deg * (st_driver_param_.st_steer_param.b_FL_feedback_steer_angled ? 1 : -1);
            st_wheel_data_[MOTOR_STEER_FL].b_is_enable = value.is_enable;
            st_wheel_data_[MOTOR_STEER_FL].b_is_error = value.is_error;
            vec_f_FL_steer_motor_feedback_hz.emplace_back(std::chrono::steady_clock::now());
            if (vec_f_FL_steer_motor_feedback_hz.size() > 10)
                vec_f_FL_steer_motor_feedback_hz.erase(vec_f_FL_steer_motor_feedback_hz.begin());
        }
        else if (motor_id == MOTOR_ABS_ENCODER_FL) {
            float f_angle = double(value.encoder);
            if (f_angle > 7200) {
                f_angle -= 14400;
            }
            f_angle /= -40;
            st_wheel_data_[MOTOR_STEER_FL].f_abssteer_angle_deg = f_angle * (st_driver_param_.st_etc_param.b_FL_steer_absfeedback_angled ? 1 : -1);
            // NLOG(info)<<"[JH] abs deg : "<<st_wheel_data_[MOTOR_STEER_FL].f_abssteer_angle_deg<<", encoder : "<<value.encoder;
                // (double(value.encoder) / st_driver_param_.st_etc_param.f_FL_abssteer_pulse * 360 *
                //      st_driver_param_.st_etc_param.f_FL_abssteer_gearratio +
                //  st_driver_param_.st_etc_param.f_FL_abssteer_offset) *
                // (st_driver_param_.st_etc_param.b_FL_steer_absfeedback_angled ? 1 : -1);
        }
        
    }
    // Motor Info pub
    core_msgs::MotorInfo o_motor_info;
    {
        o_motor_info.f_robot_type = 0;
        o_motor_info.f_FL_traction_motor_current = st_wheel_data_[MOTOR_TRACTION_FL].f_traction_current;
        o_motor_info.f_FL_traction_motor_voltage = st_wheel_data_[MOTOR_TRACTION_FL].f_traction_voltage;
        o_motor_info.f_FL_traction_motor_target_rpm = st_wheel_cmd_.f_FL_target_rpm;
        o_motor_info.f_FL_traction_motor_feedback_rpm = st_wheel_data_[MOTOR_TRACTION_FL].f_traction_feedback_rpm;
        o_motor_info.f_FL_traction_motor_feedback_rpm_hz = GetMS(vec_f_FL_traction_motor_feedback_hz);
        o_motor_info.f_FL_traction_motor_encoder = st_wheel_data_[MOTOR_TRACTION_FL].d_traction_encoder;
        o_motor_info.f_FL_traction_motor_encoder_hz = GetMS(vec_f_FL_traction_motor_feedback_hz);
        o_motor_info.f_FL_steer_motor_target_deg = st_wheel_cmd_.f_FL_target_deg;
        o_motor_info.f_FL_steer_motor_feedback_deg = st_wheel_data_[MOTOR_STEER_FL].f_steer_angle_deg;
        o_motor_info.f_FL_steer_motor_feedback_deg_hz = GetMS(vec_f_FL_steer_motor_feedback_hz);
        o_motor_info.f_FL_steer_motor_current = st_wheel_data_[MOTOR_STEER_FL].f_steer_current;
        o_motor_info.f_FL_steer_motor_target_rpm = st_wheel_data_[MOTOR_STEER_FL].f_steer_target_rpm;
        o_motor_info.f_FL_steer_motor_feedback_rpm = st_wheel_data_[MOTOR_STEER_FL].f_steer_feedback_rpm;
        o_motor_info.b_brake_on_target = b_brake_on_flag_;
        if(st_driver_param_.b_external_brake_control)
            o_motor_info.b_brake_on_feedback = b_brake_on_external_feedback_;
        else
            o_motor_info.b_brake_on_feedback = st_wheel_data_[MOTOR_TRACTION_FL].b_feedback_brake_on;
        if (st_driver_param_.st_etc_param.b_abs_steer_encoder_use) {
            f_steer_offse_deg_ = st_wheel_data_[MOTOR_STEER_FL].f_steer_angle_deg - st_wheel_data_[MOTOR_STEER_FL].f_abssteer_angle_deg;
            o_motor_info.f_FL_steer_motor_feedback_deg = st_wheel_data_[MOTOR_STEER_FL].f_abssteer_angle_deg;
            st_wheel_data_[MOTOR_STEER_FL].f_steer_angle_deg = st_wheel_data_[MOTOR_STEER_FL].f_abssteer_angle_deg;
            o_motor_info.f_FL_steer_absencoder_feedback_deg = st_wheel_data_[MOTOR_STEER_FL].f_abssteer_angle_deg;
        }
        b_brake_on_feedback_ = o_motor_info.b_brake_on_feedback;
        o_motor_info.b_steer_align = b_steer_align_;
    }

    Notify("motor_info_callback", o_motor_info);
}


bool ControllerSD::isEnable()
{
    return st_wheel_data_[MOTOR_TRACTION_FL].b_is_enable && st_wheel_data_[MOTOR_STEER_FL].b_is_enable;
}

void ControllerSD::Enable()
{
    if (!st_wheel_data_[MOTOR_TRACTION_FL].b_is_enable) {
        EnableMotor(MOTOR_TRACTION_FL);
    }
    if (!st_wheel_data_[MOTOR_STEER_FL].b_is_enable) {
        EnableMotor(MOTOR_STEER_FL);
    }
}



};  // namespace NaviFra
