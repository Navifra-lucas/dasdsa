#include "motor_sim_driver.hpp"

#include "core/util/logger.hpp"

using namespace NaviFra;

MotorSimDriver::MotorSimDriver(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    : nh_(nh)
    , nhp_(nhp)
{
    LOG_INFO("SIM DRIVER Create");
    Initialize(false);
}

MotorSimDriver::~MotorSimDriver()
{
    b_terminate_ = true;

    LOG_INFO("thread wait");
    th_readLoop_.join();
    LOG_INFO("destructor");
}

void MotorSimDriver::Initialize(bool b_only_param_update)
{
    b_init_ = false;
    o_param_.UpdateParam(nh_, nhp_, st_interface_param_, st_driver_param_);
    LOG_INFO("SetRobotType : %d", st_driver_param_.n_kinematics_type);
    b_init_ = true;
    if(b_only_param_update)
        return;
    pub_motor_data_ = nh_.advertise<motor_msgs::MotorDataInfo>("motor_data/info", 5);
    sub_motor_target_ = nh_.subscribe("motor_target", 5, &MotorSimDriver::SubscribeMotorTarget, this);
    sub_motor_brake_ = nh_.subscribe("motor_brake", 5, &MotorSimDriver::SubscribeMotorBrake, this);
    sub_motor_error_ = nh_.subscribe("motor_error_enable", 5, &MotorSimDriver::SubscribeMotorError, this);
    srv_motor_cmd_ = nh_.advertiseService("motor_cmd", &MotorSimDriver::MotorCmdCallback, this);

    LOG_INFO("Start_Loop");
    th_readLoop_ = std::thread(&MotorSimDriver::Start_Loop, this);
}

void MotorSimDriver::SubscribeMotorTarget(const motor_msgs::MotorTargetInfo::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_data_);  // 멀티스레드 안전
    o_motor_target_info_ = *msg;
    // for (const auto& motor_data : msg->data) {
    //     motor_msgs::MotorTarget o_motor_data = motor_data;  // 각 항목을 복사
    //     SetMotorData(o_motor_data.motor_id, o_motor_data);
    // }
}

void MotorSimDriver::SubscribeMotorBrake(const motor_msgs::MotorBrake::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_brake_);  // 멀티스레드 안전
    o_motor_brake_ = *msg;
}

void MotorSimDriver::SubscribeMotorError(const std_msgs::Bool::ConstPtr& msg)
{
    b_error_ = msg->data;
}

bool MotorSimDriver::MotorCmdCallback(motor_msgs::MotorCmd::Request& request, motor_msgs::MotorCmd::Response& response)
{
    NLOG(info) << "request.cmd " << request.s_cmd;
    response.b_is_response = true;
    return true;
}

float MotorSimDriver::ConvertToRPM(int n_type, float f_feedback_data)
{
    std::string s_target_unit = "rpm";
    int n_target_pulse = 1;
    float f_convert_feedback = f_feedback_data;
    if(n_type == MotorType::Traction) {
        s_target_unit = st_driver_param_.st_traction_param.s_target_unit;
        n_target_pulse = st_driver_param_.st_traction_param.n_target_pulse;
    }
    else if(n_type == MotorType::Steer) {
        s_target_unit = st_driver_param_.st_steer_param.s_target_unit;
        n_target_pulse = st_driver_param_.st_steer_param.n_target_pulse;
    }

    if (s_target_unit == "rpm" || s_target_unit == "RPM")
        return f_convert_feedback;
    else if (s_target_unit == "rps" || s_target_unit == "RPS")
        return f_convert_feedback * 60.0;
    else if (s_target_unit == "pps" || s_target_unit == "PPS")
        return f_convert_feedback * 60.0 / static_cast<float>(n_target_pulse);
    else 
        return f_convert_feedback;
}

void MotorSimDriver::Start_Loop()
{
    float f_period = 0.02;
    static double d_traction_encoder_fl = 0;
    static double d_traction_encoder_fr = 0;
    static double d_traction_encoder_rl = 0;
    static double d_traction_encoder_rr = 0;
    static double d_steer_encoder_fl = 0;
    static double d_steer_encoder_fr = 0;
    static double d_steer_encoder_rl = 0;
    static double d_steer_encoder_rr = 0;
    static float f_steer_angle_fl = 0;
    static float f_steer_angle_fr = 0;
    static float f_steer_angle_rl = 0;
    static float f_steer_angle_rr = 0;
    float f_up1 = 5;
    float f_up2 = 1;
    float f_up3 = 0.5;
    float f_up4 = 0.02;

    float f_std1 = 10;
    float f_std2 = 2;
    float f_std3 = 0.5;

    while (!b_terminate_) {
        motor_msgs::MotorDataInfo o_motor_data_info;
        motor_msgs::MotorTargetInfo o_motor_target_info;
        motor_msgs::MotorBrake o_motor_brake;
        {
            std::lock_guard<std::mutex> lock(mtx_data_);  // 멀티스레드 안전
            o_motor_target_info = o_motor_target_info_;
        }
        {
            std::lock_guard<std::mutex> lock(mtx_brake_);  // 멀티스레드 안전
            o_motor_brake = o_motor_brake_;
        }
        if(!b_init_)
            continue;
        if (st_driver_param_.n_kinematics_type == NaviFra::KINEMATICS::DD) {
            // mason 시뮬레이션 feedback rpm 딜레이
            // std::vector<float> vec_driving_motor_rpmt(2);
            // vec_driving_motor_rpmt[0] = st_cmd_.f_FL_target_rpm;
            // vec_driving_motor_rpmt[1] = st_cmd_.f_RR_target_rpm;

            // static std::vector<std::vector<float>> vec_driving_motor_rpm_s;
            // vec_driving_motor_rpm_s.insert(vec_driving_motor_rpm_s.begin(), vec_driving_motor_rpmt);
            // if(vec_driving_motor_rpm_s.size() > 20)
            // {
            //     st_data_.f_FL_feedback_rpm = vec_driving_motor_rpm_s[9][0];
            //     st_data_.f_RR_feedback_rpm = vec_driving_motor_rpm_s[9][1];
            //     vec_driving_motor_rpm_s.erase(vec_driving_motor_rpm_s.end()-1);
            // }
            for (const auto& motor_data : o_motor_target_info.data) {
                motor_msgs::MotorData o_motor_data;
                auto now = std::chrono::steady_clock::now();
                o_motor_data.status = 7;
                o_motor_data.x_status = "0x07";
                o_motor_data.s_status = "Operation";
                o_motor_data.is_enable = 1;
                if(b_error_){
                    o_motor_data.is_error = 1;
                    o_motor_data.error_code = 0x1111;
                }
                if(o_motor_brake.b_link_brake_and_servo) {
                    if(o_motor_brake.b_servo_on) {
                        o_motor_data.status = 7;
                        o_motor_data.x_status = "0x07";
                        o_motor_data.s_status = "Operation";
                        o_motor_data.is_enable = 1;
                    }
                    else {
                        o_motor_data.status = 0;
                        o_motor_data.x_status = "0x00";
                        o_motor_data.s_status = "Disable";
                        o_motor_data.is_enable = 0;
                    }
                }
                o_motor_data.bus_state = 5;
                o_motor_data.feedback_brake_on = false;
                if(o_motor_brake.b_brake_control)
                    o_motor_data.feedback_brake_on = o_motor_brake.b_brake_target;
                o_motor_data.last_update_time = std::chrono::duration<double>(now.time_since_epoch()).count();
                if (motor_data.motor_id == MOTOR::MOTOR_TRACTION_FL) {
                    o_motor_data.motor_id = MOTOR::MOTOR_TRACTION_FL;
                    o_motor_data.input_velocity = motor_data.target;
                    o_motor_data.feedback_velocity = motor_data.target;
                    o_motor_data.encoder = d_traction_encoder_fl +
                        ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_FL_traction_encoder_pulse * f_period;
                    o_motor_data.acc_encoder =
                        ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_FL_traction_encoder_pulse * f_period;
                    d_traction_encoder_fl +=
                        ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_FL_traction_encoder_pulse * f_period;
                }
                else if (motor_data.motor_id == MOTOR::MOTOR_TRACTION_RR) {
                    o_motor_data.motor_id = MOTOR::MOTOR_TRACTION_RR;
                    o_motor_data.input_velocity = motor_data.target;
                    o_motor_data.feedback_velocity = motor_data.target;
                    o_motor_data.encoder = d_traction_encoder_rr +
                        ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_RR_traction_encoder_pulse * f_period;
                    o_motor_data.acc_encoder =
                        ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_RR_traction_encoder_pulse * f_period;
                    d_traction_encoder_rr +=
                        ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_RR_traction_encoder_pulse * f_period;
                }
                o_motor_data_info.data.push_back(o_motor_data);
            }
            pub_motor_data_.publish(o_motor_data_info);
        }
        else if (st_driver_param_.n_kinematics_type == NaviFra::KINEMATICS::QD) {
            for (const auto& motor_data : o_motor_target_info.data) {
                motor_msgs::MotorData o_motor_data;
                auto now = std::chrono::steady_clock::now();
                o_motor_data.status = 7;
                o_motor_data.x_status = "0x07";
                o_motor_data.s_status = "Operation";
                o_motor_data.is_enable = 1;
                if(b_error_){
                    o_motor_data.is_error = 1;
                    o_motor_data.error_code = 0x1111;
                }
                if(o_motor_brake.b_link_brake_and_servo) {
                    if(o_motor_brake.b_servo_on) {
                        o_motor_data.status = 7;
                        o_motor_data.x_status = "0x07";
                        o_motor_data.s_status = "Operation";
                        o_motor_data.is_enable = 1;
                    }
                    else {
                        o_motor_data.status = 0;
                        o_motor_data.x_status = "0x00";
                        o_motor_data.s_status = "Disable";
                        o_motor_data.is_enable = 0;
                    }
                }
                o_motor_data.bus_state = 5;
                o_motor_data.feedback_brake_on = false;
                if(o_motor_brake.b_brake_control)
                    o_motor_data.feedback_brake_on = o_motor_brake.b_brake_target;
                o_motor_data.bus_state = 5;
                o_motor_data.last_update_time = std::chrono::duration<double>(now.time_since_epoch()).count();
                if(motor_data.motor_id == MOTOR::MOTOR_TRACTION_FL)
                {
                    o_motor_data.motor_id = MOTOR::MOTOR_TRACTION_FL;
                    o_motor_data.input_velocity = motor_data.target;
                    o_motor_data.feedback_velocity = motor_data.target;
                    o_motor_data.encoder = d_traction_encoder_fl + ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_FL_traction_encoder_pulse * f_period;
                    o_motor_data.acc_encoder = ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_FL_traction_encoder_pulse * f_period;
                    d_traction_encoder_fl += ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_FL_traction_encoder_pulse * f_period;
                }
                else if(motor_data.motor_id == MOTOR::MOTOR_TRACTION_RR)
                {
                    o_motor_data.motor_id = MOTOR::MOTOR_TRACTION_RR;
                    o_motor_data.input_velocity = motor_data.target;
                    o_motor_data.feedback_velocity = motor_data.target;
                    o_motor_data.encoder = d_traction_encoder_rr + ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_RR_traction_encoder_pulse * f_period;
                    o_motor_data.acc_encoder = ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_RR_traction_encoder_pulse * f_period;
                    d_traction_encoder_rr += ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_RR_traction_encoder_pulse * f_period;
                }
                else if(motor_data.motor_id == MOTOR::MOTOR_STEER_FL)
                {
                    if(!st_driver_param_.st_etc_param.b_abs_steer_encoder_use){
                        if (st_driver_param_.st_steer_param.n_steer_control_mode == 0) {
                            d_steer_encoder_fl += ConvertToRPM(MotorType::Steer, motor_data.target);
                            o_motor_data.encoder = d_steer_encoder_fl;
                            o_motor_data.input_velocity = motor_data.target;
                            o_motor_data.feedback_velocity = motor_data.target;
                        }
                        else if (st_driver_param_.st_steer_param.n_steer_control_mode == 1) {
                            float f_FL_gap = fabs(f_steer_angle_fl - motor_data.target);
                            if (f_steer_angle_fl < motor_data.target) {
                                if (f_FL_gap > f_std1)
                                    f_steer_angle_fl += f_up1;
                                else if (f_FL_gap > f_std2)
                                    f_steer_angle_fl += f_up2;
                                else if (f_FL_gap > f_std3)
                                    f_steer_angle_fl += f_up3;
                                else
                                    f_steer_angle_fl += f_up4;
                            }
                            else if (f_steer_angle_fl > motor_data.target) {
                                if (f_FL_gap > f_std1)
                                    f_steer_angle_fl -= f_up1;
                                else if (f_FL_gap > f_std2)
                                    f_steer_angle_fl -= f_up2;
                                else if (f_FL_gap > f_std3)
                                    f_steer_angle_fl -= f_up3;
                                else
                                    f_steer_angle_fl -= f_up4;
                            }
                            o_motor_data.input_angle = motor_data.target;
                            o_motor_data.feedback_angle = f_steer_angle_fl;
                        }
                    }
                    else{
                        d_steer_encoder_fl += ConvertToRPM(MotorType::Steer, motor_data.target);
                        o_motor_data.encoder = d_steer_encoder_fl;
                        o_motor_data.motor_id = MOTOR::MOTOR_ABS_ENCODER_FL;
                        o_motor_data_info.data.push_back(o_motor_data);
                        o_motor_data.input_velocity = motor_data.target;
                        o_motor_data.feedback_velocity = motor_data.target;
                    }
                    o_motor_data.motor_id = MOTOR::MOTOR_STEER_FL;
                }
                else if(motor_data.motor_id == MOTOR::MOTOR_STEER_RR)
                {
                    if(!st_driver_param_.st_etc_param.b_abs_steer_encoder_use){
                        if (st_driver_param_.st_steer_param.n_steer_control_mode == 0) {
                            d_steer_encoder_rr += ConvertToRPM(MotorType::Steer, motor_data.target);
                            o_motor_data.encoder = d_steer_encoder_rr;
                            o_motor_data.input_velocity = motor_data.target;
                            o_motor_data.feedback_velocity = motor_data.target;
                        }
                        else if (st_driver_param_.st_steer_param.n_steer_control_mode == 1) {
                            float f_RR_gap = fabs(f_steer_angle_rr - motor_data.target);
                            if (f_steer_angle_rr < motor_data.target) {
                                if (f_RR_gap > f_std1)
                                    f_steer_angle_rr += f_up1;
                                else if (f_RR_gap > f_std2)
                                    f_steer_angle_rr += f_up2;
                                else if (f_RR_gap > f_std3)
                                    f_steer_angle_rr += f_up3;
                                else
                                    f_steer_angle_rr += f_up4;
                            }
                            else if (f_steer_angle_rr > motor_data.target) {
                                if (f_RR_gap > f_std1)
                                    f_steer_angle_rr -= f_up1;
                                else if (f_RR_gap > f_std2)
                                    f_steer_angle_rr -= f_up2;
                                else if (f_RR_gap > f_std3)
                                    f_steer_angle_rr -= f_up3;
                                else
                                    f_steer_angle_rr -= f_up4;
                            }
                            o_motor_data.input_angle = motor_data.target;
                            o_motor_data.feedback_angle = f_steer_angle_rr;
                        }
                    }
                    else{
                        d_steer_encoder_rr += ConvertToRPM(MotorType::Steer, motor_data.target);
                        o_motor_data.encoder = d_steer_encoder_rr;
                        o_motor_data.motor_id = MOTOR::MOTOR_ABS_ENCODER_RR;
                        o_motor_data_info.data.push_back(o_motor_data);
                        o_motor_data.input_velocity = motor_data.target;
                        o_motor_data.feedback_velocity = motor_data.target;
                    }
                    o_motor_data.motor_id = MOTOR::MOTOR_STEER_RR;
                }
                o_motor_data_info.data.push_back(o_motor_data);
            }
            pub_motor_data_.publish(o_motor_data_info);
            // mason 시뮬레이션 feedback rpm 딜레이
            // std::vector<float> vec_driving_motor_rpmt(2);
            // vec_driving_motor_rpmt[0] = st_cmd_.f_FL_target_rpm;
            // vec_driving_motor_rpmt[1] = st_cmd_.f_RR_target_rpm;

            // static std::vector<std::vector<float>> vec_driving_motor_rpm_s;
            // vec_driving_motor_rpm_s.insert(vec_driving_motor_rpm_s.begin(), vec_driving_motor_rpmt);
            // if(vec_driving_motor_rpm_s.size() > 20)
            // {
            //     st_qd_data_.f_FL_traction_feedback_rpm = vec_driving_motor_rpm_s[9][0];
            //     st_qd_data_.f_RR_traction_feedback_rpm = vec_driving_motor_rpm_s[9][1];
            //     vec_driving_motor_rpm_s.erase(vec_driving_motor_rpm_s.end()-1);
            // }
        }
        else if (st_driver_param_.n_kinematics_type == NaviFra::KINEMATICS::OD) {
            for (const auto& motor_data : o_motor_target_info.data) {
                motor_msgs::MotorData o_motor_data;
                auto now = std::chrono::steady_clock::now();
                o_motor_data.status = 7;
                o_motor_data.x_status = "0x07";
                o_motor_data.s_status = "Operation";
                o_motor_data.is_enable = 1;
                if(b_error_){
                    o_motor_data.is_error = 1;
                    o_motor_data.error_code = 0x1111;
                }
                if(o_motor_brake.b_link_brake_and_servo) {
                    if(o_motor_brake.b_servo_on) {
                        o_motor_data.status = 7;
                        o_motor_data.x_status = "0x07";
                        o_motor_data.s_status = "Operation";
                        o_motor_data.is_enable = 1;
                    }
                    else {
                        o_motor_data.status = 0;
                        o_motor_data.x_status = "0x00";
                        o_motor_data.s_status = "Disable";
                        o_motor_data.is_enable = 0;
                    }
                }
                o_motor_data.bus_state = 5;
                o_motor_data.feedback_brake_on = false;
                if(o_motor_brake.b_brake_control)
                    o_motor_data.feedback_brake_on = o_motor_brake.b_brake_target;
                o_motor_data.bus_state = 5;
                o_motor_data.last_update_time = std::chrono::duration<double>(now.time_since_epoch()).count();
                if(motor_data.motor_id == MOTOR::MOTOR_TRACTION_FL)
                {
                    o_motor_data.motor_id = MOTOR::MOTOR_TRACTION_FL;
                    o_motor_data.input_velocity = motor_data.target;
                    o_motor_data.feedback_velocity = motor_data.target;
                    o_motor_data.encoder = d_traction_encoder_fl + ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_FL_traction_encoder_pulse * f_period;
                    o_motor_data.acc_encoder = ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_FL_traction_encoder_pulse * f_period;
                    d_traction_encoder_fl += ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_FL_traction_encoder_pulse * f_period;
                }
                if(motor_data.motor_id == MOTOR::MOTOR_TRACTION_FR)
                {
                    o_motor_data.motor_id = MOTOR::MOTOR_TRACTION_FR;
                    o_motor_data.input_velocity = motor_data.target;
                    o_motor_data.feedback_velocity = motor_data.target;
                    o_motor_data.encoder = d_traction_encoder_fr + ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_FR_traction_encoder_pulse * f_period;
                    o_motor_data.acc_encoder = ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_FR_traction_encoder_pulse * f_period;
                    d_traction_encoder_fr += ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_FR_traction_encoder_pulse * f_period;
                }
                if(motor_data.motor_id == MOTOR::MOTOR_TRACTION_RL)
                {
                    o_motor_data.motor_id = MOTOR::MOTOR_TRACTION_RL;
                    o_motor_data.input_velocity = motor_data.target;
                    o_motor_data.feedback_velocity = motor_data.target;
                    o_motor_data.encoder = d_traction_encoder_rl + ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_RL_traction_encoder_pulse * f_period;
                    o_motor_data.acc_encoder = ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_RL_traction_encoder_pulse * f_period;
                    d_traction_encoder_rl += ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_RL_traction_encoder_pulse * f_period;
                }
                else if(motor_data.motor_id == MOTOR::MOTOR_TRACTION_RR)
                {
                    o_motor_data.motor_id = MOTOR::MOTOR_TRACTION_RR;
                    o_motor_data.input_velocity = motor_data.target;
                    o_motor_data.feedback_velocity = motor_data.target;
                    o_motor_data.encoder = d_traction_encoder_rr + ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_RR_traction_encoder_pulse * f_period;
                    o_motor_data.acc_encoder = ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_RR_traction_encoder_pulse * f_period;
                    d_traction_encoder_rr += ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_RR_traction_encoder_pulse * f_period;
                }
                else if(motor_data.motor_id == MOTOR::MOTOR_STEER_FL)
                {
                    if(!st_driver_param_.st_etc_param.b_abs_steer_encoder_use){
                        if (st_driver_param_.st_steer_param.n_steer_control_mode == 0) {
                            d_steer_encoder_fl += ConvertToRPM(MotorType::Steer, motor_data.target);
                            o_motor_data.encoder = d_steer_encoder_fl;
                            o_motor_data.input_velocity = motor_data.target;
                            o_motor_data.feedback_velocity = motor_data.target;
                        }
                        else if (st_driver_param_.st_steer_param.n_steer_control_mode == 1) {
                            float f_FL_gap = fabs(f_steer_angle_fl - motor_data.target);
                            if (f_steer_angle_fl < motor_data.target) {
                                if (f_FL_gap > f_std1)
                                    f_steer_angle_fl += f_up1;
                                else if (f_FL_gap > f_std2)
                                    f_steer_angle_fl += f_up2;
                                else if (f_FL_gap > f_std3)
                                    f_steer_angle_fl += f_up3;
                                else
                                    f_steer_angle_fl += f_up4;
                            }
                            else if (f_steer_angle_fl > motor_data.target) {
                                if (f_FL_gap > f_std1)
                                    f_steer_angle_fl -= f_up1;
                                else if (f_FL_gap > f_std2)
                                    f_steer_angle_fl -= f_up2;
                                else if (f_FL_gap > f_std3)
                                    f_steer_angle_fl -= f_up3;
                                else
                                    f_steer_angle_fl -= f_up4;
                            }
                            o_motor_data.input_angle = motor_data.target;
                            o_motor_data.feedback_angle = f_steer_angle_fl;
                        }
                    }
                    else{
                        d_steer_encoder_fl += ConvertToRPM(MotorType::Steer, motor_data.target);
                        o_motor_data.encoder = d_steer_encoder_fl;
                        o_motor_data.motor_id = MOTOR::MOTOR_ABS_ENCODER_FL;
                        o_motor_data_info.data.push_back(o_motor_data);
                        o_motor_data.input_velocity = motor_data.target;
                        o_motor_data.feedback_velocity = motor_data.target;
                    }
                    o_motor_data.motor_id = MOTOR::MOTOR_STEER_FL;
                }
                else if(motor_data.motor_id == MOTOR::MOTOR_STEER_FR)
                {
                    if(!st_driver_param_.st_etc_param.b_abs_steer_encoder_use){
                        if (st_driver_param_.st_steer_param.n_steer_control_mode == 0) {
                            d_steer_encoder_fr += ConvertToRPM(MotorType::Steer, motor_data.target);
                            o_motor_data.encoder = d_steer_encoder_fr;
                            o_motor_data.input_velocity = motor_data.target;
                            o_motor_data.feedback_velocity = motor_data.target;
                        }
                        else if (st_driver_param_.st_steer_param.n_steer_control_mode == 1) {
                            float f_FR_gap = fabs(f_steer_angle_fr - motor_data.target);
                            if (f_steer_angle_fr < motor_data.target) {
                                if (f_FR_gap > f_std1)
                                    f_steer_angle_fr += f_up1;
                                else if (f_FR_gap > f_std2)
                                    f_steer_angle_fr += f_up2;
                                else if (f_FR_gap > f_std3)
                                    f_steer_angle_fr += f_up3;
                                else
                                    f_steer_angle_fr += f_up4;
                            }
                            else if (f_steer_angle_fr > motor_data.target) {
                                if (f_FR_gap > f_std1)
                                    f_steer_angle_fr -= f_up1;
                                else if (f_FR_gap > f_std2)
                                    f_steer_angle_fr -= f_up2;
                                else if (f_FR_gap > f_std3)
                                    f_steer_angle_fr -= f_up3;
                                else
                                    f_steer_angle_fr -= f_up4;
                            }
                            o_motor_data.input_angle = motor_data.target;
                            o_motor_data.feedback_angle = f_steer_angle_fr;
                        }
                    }
                    else{
                        d_steer_encoder_fr += ConvertToRPM(MotorType::Steer, motor_data.target);
                        o_motor_data.encoder = d_steer_encoder_fr;
                        o_motor_data.motor_id = MOTOR::MOTOR_ABS_ENCODER_FR;
                        o_motor_data_info.data.push_back(o_motor_data);
                        o_motor_data.input_velocity = motor_data.target;
                        o_motor_data.feedback_velocity = motor_data.target;
                    }
                    o_motor_data.motor_id = MOTOR::MOTOR_STEER_FR;
                }
                else if(motor_data.motor_id == MOTOR::MOTOR_STEER_RL)
                {
                    if(!st_driver_param_.st_etc_param.b_abs_steer_encoder_use){
                        if (st_driver_param_.st_steer_param.n_steer_control_mode == 0) {
                            d_steer_encoder_rl += ConvertToRPM(MotorType::Steer, motor_data.target);
                            o_motor_data.encoder = d_steer_encoder_rl;
                            o_motor_data.input_velocity = motor_data.target;
                            o_motor_data.feedback_velocity = motor_data.target;
                        }
                        else if (st_driver_param_.st_steer_param.n_steer_control_mode == 1) {
                            float f_RL_gap = fabs(f_steer_angle_rl - motor_data.target);
                            if (f_steer_angle_rl < motor_data.target) {
                                if (f_RL_gap > f_std1)
                                    f_steer_angle_rl += f_up1;
                                else if (f_RL_gap > f_std2)
                                    f_steer_angle_rl += f_up2;
                                else if (f_RL_gap > f_std3)
                                    f_steer_angle_rl += f_up3;
                                else
                                    f_steer_angle_rl += f_up4;
                            }
                            else if (f_steer_angle_rl > motor_data.target) {
                                if (f_RL_gap > f_std1)
                                    f_steer_angle_rl -= f_up1;
                                else if (f_RL_gap > f_std2)
                                    f_steer_angle_rl -= f_up2;
                                else if (f_RL_gap > f_std3)
                                    f_steer_angle_rl -= f_up3;
                                else
                                    f_steer_angle_rl -= f_up4;
                            }
                            o_motor_data.input_angle = motor_data.target;
                            o_motor_data.feedback_angle = f_steer_angle_rl;
                        }
                    }
                    else{
                        d_steer_encoder_rl += ConvertToRPM(MotorType::Steer, motor_data.target);
                        o_motor_data.encoder = d_steer_encoder_rl;
                        o_motor_data.motor_id = MOTOR::MOTOR_ABS_ENCODER_RL;
                        o_motor_data_info.data.push_back(o_motor_data);
                        o_motor_data.input_velocity = motor_data.target;
                        o_motor_data.feedback_velocity = motor_data.target;
                    }
                    o_motor_data.motor_id = MOTOR::MOTOR_STEER_RL;
                }
                else if(motor_data.motor_id == MOTOR::MOTOR_STEER_RR)
                {
                    if(!st_driver_param_.st_etc_param.b_abs_steer_encoder_use){
                        if (st_driver_param_.st_steer_param.n_steer_control_mode == 0) {
                            d_steer_encoder_rr += ConvertToRPM(MotorType::Steer, motor_data.target);
                            o_motor_data.encoder = d_steer_encoder_rr;
                            o_motor_data.input_velocity = motor_data.target;
                            o_motor_data.feedback_velocity = motor_data.target;
                        }
                        else if (st_driver_param_.st_steer_param.n_steer_control_mode == 1) {
                            float f_RR_gap = fabs(f_steer_angle_rr - motor_data.target);
                            if (f_steer_angle_rr < motor_data.target) {
                                if (f_RR_gap > f_std1)
                                    f_steer_angle_rr += f_up1;
                                else if (f_RR_gap > f_std2)
                                    f_steer_angle_rr += f_up2;
                                else if (f_RR_gap > f_std3)
                                    f_steer_angle_rr += f_up3;
                                else
                                    f_steer_angle_rr += f_up4;
                            }
                            else if (f_steer_angle_rr > motor_data.target) {
                                if (f_RR_gap > f_std1)
                                    f_steer_angle_rr -= f_up1;
                                else if (f_RR_gap > f_std2)
                                    f_steer_angle_rr -= f_up2;
                                else if (f_RR_gap > f_std3)
                                    f_steer_angle_rr -= f_up3;
                                else
                                    f_steer_angle_rr -= f_up4;
                            }
                            o_motor_data.input_angle = motor_data.target;
                            o_motor_data.feedback_angle = f_steer_angle_rr;
                        }
                    }
                    else{
                        d_steer_encoder_rr += ConvertToRPM(MotorType::Steer, motor_data.target);
                        o_motor_data.encoder = d_steer_encoder_rr;
                        o_motor_data.motor_id = MOTOR::MOTOR_ABS_ENCODER_RR;
                        o_motor_data_info.data.push_back(o_motor_data);
                        o_motor_data.input_velocity = motor_data.target;
                        o_motor_data.feedback_velocity = motor_data.target;
                    }
                    o_motor_data.motor_id = MOTOR::MOTOR_STEER_RR;
                }
                o_motor_data_info.data.push_back(o_motor_data);
            }
            pub_motor_data_.publish(o_motor_data_info);
            // mason 시뮬레이션 feedback rpm 딜레이
            // std::vector<float> vec_driving_motor_rpmt(2);
            // vec_driving_motor_rpmt[0] = st_cmd_.f_FL_target_rpm;
            // vec_driving_motor_rpmt[1] = st_cmd_.f_RR_target_rpm;

            // static std::vector<std::vector<float>> vec_driving_motor_rpm_s;
            // vec_driving_motor_rpm_s.insert(vec_driving_motor_rpm_s.begin(), vec_driving_motor_rpmt);
            // if(vec_driving_motor_rpm_s.size() > 20)
            // {
            //     st_qd_data_.f_FL_traction_feedback_rpm = vec_driving_motor_rpm_s[9][0];
            //     st_qd_data_.f_RR_traction_feedback_rpm = vec_driving_motor_rpm_s[9][1];
            //     vec_driving_motor_rpm_s.erase(vec_driving_motor_rpm_s.end()-1);
            // }
        }
        else if (st_driver_param_.n_kinematics_type == NaviFra::KINEMATICS::SD) {
            for (const auto& motor_data : o_motor_target_info.data) {
                motor_msgs::MotorData o_motor_data;
                auto now = std::chrono::steady_clock::now();
                o_motor_data.status = 7;
                o_motor_data.x_status = "0x07";
                o_motor_data.s_status = "Operation";
                o_motor_data.is_enable = 1;
                if(b_error_){
                    o_motor_data.is_error = 1;
                    o_motor_data.error_code = 0x1111;
                }
                if(o_motor_brake.b_link_brake_and_servo) {
                    if(o_motor_brake.b_servo_on) {
                        o_motor_data.status = 7;
                        o_motor_data.x_status = "0x07";
                        o_motor_data.s_status = "Operation";
                        o_motor_data.is_enable = 1;
                    }
                    else {
                        o_motor_data.status = 0;
                        o_motor_data.x_status = "0x00";
                        o_motor_data.s_status = "Disable";
                        o_motor_data.is_enable = 0;
                    }
                }
                o_motor_data.bus_state = 5;
                o_motor_data.feedback_brake_on = false;
                if(o_motor_brake.b_brake_control)
                    o_motor_data.feedback_brake_on = o_motor_brake.b_brake_target;
                o_motor_data.bus_state = 5;
                o_motor_data.last_update_time = std::chrono::duration<double>(now.time_since_epoch()).count();
                if(motor_data.motor_id == MOTOR::MOTOR_TRACTION_FL)
                {
                    o_motor_data.motor_id = MOTOR::MOTOR_TRACTION_FL;
                    o_motor_data.input_velocity = motor_data.target;
                    o_motor_data.feedback_velocity = motor_data.target;
                    o_motor_data.encoder = d_traction_encoder_fl + ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_FL_traction_encoder_pulse * f_period;
                    o_motor_data.acc_encoder = ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_FL_traction_encoder_pulse * f_period;
                    d_traction_encoder_fl += ConvertToRPM(MotorType::Traction, motor_data.target) / 60.0f * st_driver_param_.st_traction_param.f_FL_traction_encoder_pulse * f_period;
                }
                else if(motor_data.motor_id == MOTOR::MOTOR_STEER_FL)
                {
                    if(!st_driver_param_.st_etc_param.b_abs_steer_encoder_use){
                        if (st_driver_param_.st_steer_param.n_steer_control_mode == 0) {
                            d_steer_encoder_fl += ConvertToRPM(MotorType::Steer, motor_data.target);
                            o_motor_data.encoder = d_steer_encoder_fl;
                            o_motor_data.input_velocity = motor_data.target;
                            o_motor_data.feedback_velocity = motor_data.target;
                        }
                        else if (st_driver_param_.st_steer_param.n_steer_control_mode == 1) {
                            float f_FL_gap = fabs(f_steer_angle_fl - motor_data.target);
                            if (f_steer_angle_fl < motor_data.target) {
                                if (f_FL_gap > f_std1)
                                    f_steer_angle_fl += f_up1;
                                else if (f_FL_gap > f_std2)
                                    f_steer_angle_fl += f_up2;
                                else if (f_FL_gap > f_std3)
                                    f_steer_angle_fl += f_up3;
                                else
                                    f_steer_angle_fl += f_up4;
                            }
                            else if (f_steer_angle_fl > motor_data.target) {
                                if (f_FL_gap > f_std1)
                                    f_steer_angle_fl -= f_up1;
                                else if (f_FL_gap > f_std2)
                                    f_steer_angle_fl -= f_up2;
                                else if (f_FL_gap > f_std3)
                                    f_steer_angle_fl -= f_up3;
                                else
                                    f_steer_angle_fl -= f_up4;
                            }
                            o_motor_data.input_angle = motor_data.target;
                            o_motor_data.feedback_angle = f_steer_angle_fl;
                        }
                    }
                    else{
                        d_steer_encoder_fl += ConvertToRPM(MotorType::Steer, motor_data.target);
                        o_motor_data.encoder = d_steer_encoder_fl;
                        o_motor_data.motor_id = MOTOR::MOTOR_ABS_ENCODER_FL;
                        o_motor_data_info.data.push_back(o_motor_data);
                        o_motor_data.input_velocity = motor_data.target;
                        o_motor_data.feedback_velocity = motor_data.target;
                    }
                    o_motor_data.motor_id = MOTOR::MOTOR_STEER_FL;
                }
                o_motor_data_info.data.push_back(o_motor_data);
            }
            pub_motor_data_.publish(o_motor_data_info);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(int(f_period * 1000)));
    }
}
