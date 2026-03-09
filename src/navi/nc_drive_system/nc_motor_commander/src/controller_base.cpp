#include "controller_base.hpp"

using namespace std::chrono_literals;

namespace NaviFra {

void Controller::initialize(void)
{
    RegisterTalker();
    RegisterListener();
    SetMinMaxRPM();
    startThreadError();
    startThreadUpdate();

    try {
        NLOG(info) << "MotorCommander - Upper - Turntable : "
                   << ((st_driver_param_.st_upper_param.b_turntable_use == true) ? "USE" : "NOT USE");
        NLOG(info) << "Upper Turntable Params : ppr : " << st_driver_param_.st_upper_param.f_turntable_ppr
                   << ", gear_ratio : " << st_driver_param_.st_upper_param.f_turntable_gear_ratio << ", reference_position :"
                   << (st_driver_param_.st_upper_param.b_turntable_reference_position_is_initial_position == true
                           ? "initial"
                           : std::to_string(st_driver_param_.st_upper_param.n_turntable_reference_position));
        if (st_driver_param_.st_upper_param.b_turntable_use) {
            upper_turntable_ = (st_driver_param_.st_upper_param.b_turntable_reference_position_is_initial_position)
                ? std::make_unique<NaviFra::MotorCommander::Upper::Turntable>(
                      st_driver_param_.st_upper_param.f_turntable_ppr, st_driver_param_.st_upper_param.f_turntable_gear_ratio)
                : std::make_unique<NaviFra::MotorCommander::Upper::Turntable>(
                      st_driver_param_.st_upper_param.f_turntable_ppr, st_driver_param_.st_upper_param.f_turntable_gear_ratio,
                      st_driver_param_.st_upper_param.n_turntable_reference_position);
            upper_turntable_->initialize();
        }
    }
    catch (...) {
        NLOG(info) << "Upper Turntable Initilization failed.";
    }

    error_checker_ = std::make_unique<NaviFra::MotorCommander::ErrorChecker>(
        [this](const std::string& name, const std::any& var) { Notify(name, var); });
}

void Controller::finalize(void)
{
    error_checker_.reset();
    upper_turntable_.reset();

    stopThreadUpdate();
    stopThreadError();
}

void Controller::RegisterTalker()
{
    pub_rpm_ = nh_.advertise<motor_msgs::MotorTargetInfo>("motor_target", 5);
    pub_brake_ = nh_.advertise<motor_msgs::MotorBrake>("motor_brake", 5);
    pub_warning_ = nh_.advertise<std_msgs::String>("/navifra/warning", 5, false);
    pub_imu_reset_ = nh_.advertise<std_msgs::String>("/imu_reset", 5, false);

    motor_cmd_req_ = nh_.serviceClient<motor_msgs::MotorCmd>("motor_cmd");
}

void Controller::RegisterListener()
{
    sub_motor_data_ = nh_.subscribe("motor_data/info", 5, &Controller::RecvkMotorData, this, ros::TransportHints().tcpNoDelay(true));
    sub_imu_data_ = nh_.subscribe("imu_info", 5, &Controller::RecvImuData, this, ros::TransportHints().tcpNoDelay(true));
}

void Controller::RecvkMotorData(const motor_msgs::MotorDataInfo::ConstPtr& msg)
{
    // msg->data가 std::list 형식이라고 가정하고 복사
    for (const auto& motor_data : msg->data) {
        motor_msgs::MotorData o_motor_data = motor_data;  // 각 항목을 복사
        SetMotorData(static_cast<MotorId>(o_motor_data.motor_id), o_motor_data);
    }
}

void Controller::SetMinMaxRPM()
{
    n_rpm_max[MOTOR_TRACTION_FL] = st_driver_param_.st_traction_param.f_FL_traction_rpm_max;
    n_rpm_min[MOTOR_TRACTION_FL] = st_driver_param_.st_traction_param.f_FL_traction_rpm_min;

    n_rpm_max[MOTOR_TRACTION_FR] = st_driver_param_.st_traction_param.f_FR_traction_rpm_max;
    n_rpm_min[MOTOR_TRACTION_FR] = st_driver_param_.st_traction_param.f_FR_traction_rpm_min;

    n_rpm_max[MOTOR_TRACTION_RL] = st_driver_param_.st_traction_param.f_RL_traction_rpm_max;
    n_rpm_min[MOTOR_TRACTION_RL] = st_driver_param_.st_traction_param.f_RL_traction_rpm_min;

    n_rpm_max[MOTOR_TRACTION_RR] = st_driver_param_.st_traction_param.f_RR_traction_rpm_max;
    n_rpm_min[MOTOR_TRACTION_RR] = st_driver_param_.st_traction_param.f_RR_traction_rpm_min;

    n_rpm_max[MOTOR_STEER_FL] = st_driver_param_.st_steer_param.f_FL_steer_rpm_max;
    n_rpm_min[MOTOR_STEER_FL] = st_driver_param_.st_steer_param.f_FL_steer_rpm_min;

    n_rpm_max[MOTOR_STEER_FR] = st_driver_param_.st_steer_param.f_FR_steer_rpm_max;
    n_rpm_min[MOTOR_STEER_FR] = st_driver_param_.st_steer_param.f_FR_steer_rpm_min;

    n_rpm_max[MOTOR_STEER_RL] = st_driver_param_.st_steer_param.f_RL_steer_rpm_max;
    n_rpm_min[MOTOR_STEER_RL] = st_driver_param_.st_steer_param.f_RL_steer_rpm_min;

    n_rpm_max[MOTOR_STEER_RR] = st_driver_param_.st_steer_param.f_RR_steer_rpm_max;
    n_rpm_min[MOTOR_STEER_RR] = st_driver_param_.st_steer_param.f_RR_steer_rpm_min;
}

float Controller::LimitRPM(float f_target_rpm, int n_rpm_max, int n_rpm_min)
{
    if (f_target_rpm == 0.f)
        return 0.f;

    float f_abs_rpm = fabs(f_target_rpm);
    float f_limit_rpm = f_abs_rpm;

    if (f_abs_rpm < n_rpm_min)
        f_limit_rpm = n_rpm_min;
    else if (f_abs_rpm > n_rpm_max)
        f_limit_rpm = n_rpm_max;

    return (f_target_rpm >= 0.f ? f_limit_rpm : -f_limit_rpm);
}

double Controller::SteerDEGtoRPM(MotorId motor_id, float f_steer_target_angle, float f_steer_feedback_angle)
{
    float dt = st_driver_param_.f_control_period_ms;
    float f_gian_p = st_driver_param_.st_steer_param.f_steer_low_speed_pid_kp_gain;
    float f_gian_i = st_driver_param_.st_steer_param.f_steer_low_speed_pid_ki_gain;
    float f_gian_d = st_driver_param_.st_steer_param.f_steer_low_speed_pid_kd_gain;
    // if (st_cmd.f_robot_target_speed > st_driver_param_.st_steer_param.f_steer_speed_std_ms) {
    //     f_gian_p = st_driver_param_.st_steer_param.f_steer_high_speed_pid_kp_gain;
    //     f_gian_i = st_driver_param_.st_steer_param.f_steer_high_speed_pid_ki_gain;
    //     f_gian_d = st_driver_param_.st_steer_param.f_steer_high_speed_pid_kd_gain;
    // }
    float error = f_steer_target_angle - f_steer_feedback_angle;

    double pTerm = f_gian_p * error;
    integrals[motor_id] += error * dt;
    double iTerm = f_gian_i * integrals[motor_id];
    double dTerm = f_gian_d * (error - previous_errors[motor_id]) / dt;

    double controlInput = pTerm + iTerm + dTerm;

    int rpm_max = n_rpm_max[motor_id];
    int rpm_min = n_rpm_min[motor_id];
    if (controlInput != 0) {
        if (abs(controlInput) > rpm_max)
            controlInput = controlInput / abs(controlInput) * rpm_max;
        else if (abs(controlInput) < rpm_min)
            controlInput = controlInput / abs(controlInput) * rpm_min;
    }

    previous_errors[motor_id] = error;
    return controlInput;
}

float Controller::ConvertFromRPM(int n_type, float f_target_rpm)
{
    std::string s_target_unit = "rpm";
    int n_target_pulse = 1;
    float f_convert_target = f_target_rpm;
    if (n_type == MotorType::Traction) {
        s_target_unit = st_driver_param_.st_traction_param.s_target_unit;
        n_target_pulse = st_driver_param_.st_traction_param.n_target_pulse;
    }
    else if (n_type == MotorType::Steer) {
        s_target_unit = st_driver_param_.st_steer_param.s_target_unit;
        n_target_pulse = st_driver_param_.st_steer_param.n_target_pulse;
    }

    if (s_target_unit == "rpm" || s_target_unit == "RPM")
        return f_convert_target;
    else if (s_target_unit == "rps" || s_target_unit == "RPS")
        return f_convert_target / 60.0;
    else if (s_target_unit == "pps" || s_target_unit == "PPS")
        return f_convert_target / 60.0 * static_cast<float>(n_target_pulse);
    else
        return f_convert_target;
}

float Controller::ConvertToRPM(int n_type, float f_feedback_data)
{
    std::string s_target_unit = "rpm";
    int n_target_pulse = 1;
    float f_convert_feedback = f_feedback_data;
    if (n_type == MotorType::Traction) {
        s_target_unit = st_driver_param_.st_traction_param.s_target_unit;
        n_target_pulse = st_driver_param_.st_traction_param.n_target_pulse;
    }
    else if (n_type == MotorType::Steer) {
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

void Controller::CalOdomAndVel(SimplePos& o_d_pos, const SimplePos& o_vel)
{
    static int log_stamp = 0;
    std::chrono::duration<double> sec_imu = std::chrono::system_clock::now() - tp_imu_check_;
    if (sec_imu.count() > 1 && b_use_imu_)  // imu 1초이상 안들어오거나 정확히 0이면 false
    {
        LOG_ERROR("imu 1sec timeout, b_use_imu false");
        b_use_imu_ = false;
        f_imu_yaw_acc_deg_ = 0;
    }
    log_stamp++;

    if (log_stamp % 100 == 0) {
        if (b_use_imu_) {
            // LOG_ERROR("DIFF DEG %f", o_d_pos.GetDeg()-f_imu_yaw_acc_deg_);
            if (abs(o_d_pos.GetDeg() - f_imu_yaw_acc_deg_) > 1.45) {
                LOG_ERROR("ERROR SLIP DIFF DEG %f", o_d_pos.GetDeg() - f_imu_yaw_acc_deg_);

                NaviFra::Motor_ERROR st_error;
                st_error.b_etc_error_update = true;
                st_error.n_etc_error_code = core_msgs::NaviAlarm::ERROR_SLIP_PREDETECTED;
                st_error.s_etc_error_text = "ERROR_SLIP_PREDETECTED";
                Notify("error_callback", st_error);
            }
        }
        log_stamp = 0;
    }
    if (b_use_imu_) {
        o_d_pos.SetDeg(f_imu_yaw_acc_deg_);  // imu 쓸때는 encoder deg안씀
        f_imu_yaw_acc_deg_ = 0;
    }

    SimplePos o_pos_adding(
        o_d_pos.GetXm() * cos(o_odom_pos_.GetRad()) - o_d_pos.GetYm() * sin(o_odom_pos_.GetRad()),
        o_d_pos.GetXm() * sin(o_odom_pos_.GetRad()) + o_d_pos.GetYm() * cos(o_odom_pos_.GetRad()));
    o_pos_adding.SetRad(o_d_pos.GetRad());

    if (hypot(o_pos_adding.GetXm(), o_pos_adding.GetYm()) > 0.2 || fabs(o_pos_adding.GetDeg()) > 5) {
        NLOG(severity_level::error) << "CalOdomAndVel dodom large error " << o_pos_adding.GetXm() << " " << o_pos_adding.GetYm() << " "
                                    << o_pos_adding.GetDeg();
        o_pos_adding.SetXm(0);
        o_pos_adding.SetYm(0);
        o_pos_adding.SetRad(0);
    }

    o_odom_pos_.SetXm(o_odom_pos_.GetXm() + o_pos_adding.GetXm());
    o_odom_pos_.SetYm(o_odom_pos_.GetYm() + o_pos_adding.GetYm());
    o_odom_pos_.SetRad(o_odom_pos_.GetRad() + o_pos_adding.GetRad());

    std::vector<SimplePos> vec_odom(2);
    vec_odom[0] = o_odom_pos_;
    vec_odom[1] = o_vel;
    Notify("odom_callback", vec_odom);
}

void Controller::RecvImuData(const sensor_msgs::Imu::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_imu_data_);

    tf2::Quaternion quat;
    quat.setX(msg->orientation.x);
    quat.setY(msg->orientation.y);
    quat.setZ(msg->orientation.z);
    quat.setW(msg->orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    IMU_Data_t o_imu_data;
    o_imu_data.f_roll = roll * RADtoDEG;
    o_imu_data.f_pitch = pitch * RADtoDEG;
    o_imu_data.f_yaw = yaw * RADtoDEG;
    o_imu_data.f_angular_vel_x = msg->angular_velocity.x;
    o_imu_data.f_angular_vel_y = msg->angular_velocity.y;
    o_imu_data.f_angular_vel_z = msg->angular_velocity.z;
    o_imu_data.f_linear_acc_x = msg->linear_acceleration.x / 9.8;
    o_imu_data.f_linear_acc_y = msg->linear_acceleration.y / 9.8;
    o_imu_data.f_linear_acc_z = msg->linear_acceleration.z / 9.8;

    // check imu state
    static int n_error_count = 0;
    static int n_stop_count = 0;  // 정지상태
    bool b_stop = false;
    if (st_driver_param_.n_kinematics_type == 0 || st_driver_param_.n_kinematics_type == 1) {
        if ((fabs(st_wheel_cmd_.f_FL_target_rpm) < 0.001f && fabs(st_wheel_cmd_.f_RR_target_rpm) < 0.001f) &&
            (fabs(st_wheel_data_[MOTOR_TRACTION_FL].f_traction_feedback_rpm) < st_driver_param_.f_brake_threshold_rpm ||
             fabs(st_wheel_data_[MOTOR_TRACTION_RR].f_traction_feedback_rpm) < st_driver_param_.f_brake_threshold_rpm))
            b_stop = true;
    }
    else if (st_driver_param_.n_kinematics_type == 2) {
        if (fabs(st_wheel_cmd_.f_FL_target_rpm) < 0.001f &&
            fabs(st_wheel_data_[MOTOR_TRACTION_FL].f_traction_feedback_rpm) < st_driver_param_.f_brake_threshold_rpm)
            b_stop = true;
    }
    else if (st_driver_param_.n_kinematics_type == 3) {
        if ((fabs(st_wheel_cmd_.f_FL_target_rpm) < 0.001f && fabs(st_wheel_cmd_.f_FR_target_rpm) < 0.001f &&
             fabs(st_wheel_cmd_.f_RL_target_rpm) < 0.001f && fabs(st_wheel_cmd_.f_RR_target_rpm) < 0.001f) &&
            (fabs(st_wheel_data_[MOTOR_TRACTION_FL].f_traction_feedback_rpm) < st_driver_param_.f_brake_threshold_rpm ||
             fabs(st_wheel_data_[MOTOR_TRACTION_FR].f_traction_feedback_rpm) < st_driver_param_.f_brake_threshold_rpm ||
             fabs(st_wheel_data_[MOTOR_TRACTION_RL].f_traction_feedback_rpm) < st_driver_param_.f_brake_threshold_rpm ||
             fabs(st_wheel_data_[MOTOR_TRACTION_RR].f_traction_feedback_rpm) < st_driver_param_.f_brake_threshold_rpm))
            b_stop = true;
    }

    if (b_stop) {
        if (n_stop_count < 10000)
            n_stop_count++;
    }
    else {
        n_stop_count = 0;
        n_error_count = 0;
    }

    // IMU 오류 감지 및 리셋
    if (n_stop_count > 1000) {
        // 차량이 정지한 상태에서 imu 값이 이상한경우
        if (fabs(o_imu_data.f_angular_vel_x) > 1.5 || fabs(o_imu_data.f_angular_vel_y) > 1.5 || fabs(o_imu_data.f_angular_vel_z) > 1.5) {
            if (n_error_count < 10000)
                n_error_count++;
        }

        // IMU리셋
        if (n_error_count > 1000 && st_driver_param_.b_imuodom_enable == true) {
            n_error_count = 0;
            st_driver_param_.b_imuodom_enable = false;
            LOG_ERROR(
                "IMU ERROR not USE vel x %.3f, y %.3f, z %.3f", o_imu_data.f_angular_vel_x, o_imu_data.f_angular_vel_y,
                o_imu_data.f_angular_vel_z);

            tp_imu_reset_ = std::chrono::system_clock::now();
            std_msgs::String str_data;
            str_data.data = "";
            pub_imu_reset_.publish(str_data);
        }
        if (n_stop_count > 5000 && n_error_count < 10 && st_driver_param_.b_imuodom_enable == false) {
            st_driver_param_.b_imuodom_enable = true;
            LOG_ERROR("IMU USE");
        }
    }

    if (st_driver_param_.b_imuodom_enable) {
        float f_imu_deg_gap = (o_imu_data.f_yaw - f_imu_yaw_);
        std::chrono::duration<double> sec_imu = std::chrono::system_clock::now() - tp_imu_check_;
        static int n_imu_use_count = 0;
        if ((fabs(f_target_w_degs_) > 7 || fabs(f_feedback_w_degs_) > 7 || f_feedback_acc_mss_ > 0.4 || n_imu_use_count > 0) &&
            fabs(f_imu_deg_gap) < 1.0f && fabs(f_imu_yaw_) > 0.01) {
            if (fabs(f_target_w_degs_) > 7 || fabs(f_feedback_w_degs_) > 7 || f_feedback_acc_mss_ > 0.4)
                n_imu_use_count = 300;
            n_imu_use_count--;
            // 타겟 각속도, 피드백 각속도 있고, imu 갭이 충분히 신뢰할만 할 때 use imu true
            f_imu_yaw_acc_deg_ += f_imu_deg_gap;
            b_use_imu_ = true;
            tp_imu_use_ = std::chrono::system_clock::now();
        }
        else {
            b_use_imu_ = false;
            f_imu_yaw_acc_deg_ = 0;
        }
        o_imu_data.b_use_imu = b_use_imu_;
        static int n_log_count = 0;
        if (b_use_imu_)
            n_log_count++;
        if (n_log_count > 100) {
            LOG_INFO(
                "imu_fusion imu : %.3f, f_target_w_degs_ : %.3f, f_feedback_w_degs_ : %.3f, b_use_imu_ %d sec_imu %.3f f_imu_yaw_acc_deg_ %.3f",
                o_imu_data.f_yaw, f_target_w_degs_, f_feedback_w_degs_, b_use_imu_, sec_imu.count(), f_imu_yaw_acc_deg_);
            n_log_count = 0;
        }
    }

    Notify("imu_info_callback", o_imu_data);
    tp_imu_check_ = std::chrono::system_clock::now();
    f_imu_yaw_ = o_imu_data.f_yaw;
}

float Controller::GetHz(std::vector<std::chrono::steady_clock::time_point>& vec_time)
{
    if (vec_time.empty() || vec_time.size() == 1)
        return 0;
    std::chrono::duration<double> sec = vec_time.back() - vec_time.front();
    float f_ms = sec.count() / (vec_time.size() - 1);
    float f_hz = 0;
    if (f_ms > 0)
        f_hz = 1 / f_ms;
    return f_hz;
    // float f_sec_sum = 0;
    // int n_sec_cnt   = 0;
    // for (int i = 1; i < vec_time.size(); i++)
    // {
    //     std::chrono::duration<double> sec = vec_time[i] - vec_time[i - 1];
    //     f_sec_sum += sec.count();
    //     n_sec_cnt++;
    // }
    // return float(1 / (f_sec_sum / n_sec_cnt));
}

float Controller::GetMS(std::vector<std::chrono::steady_clock::time_point>& vec_time)
{
    if (vec_time.size() < 2)
        return 0;

    float f_sec_sum = 0;
    int n_sec_cnt = 0;
    for (int i = 1; i < vec_time.size(); i++) {
        std::chrono::duration<double> sec = vec_time[i] - vec_time[i - 1];
        f_sec_sum += sec.count();
        n_sec_cnt++;
    }
    float f_ms = float(f_sec_sum / n_sec_cnt);
    return ceil(f_ms * 1000);
}

void Controller::ReceivedError(const std::any& any_type_var)
{
    Motor_ERROR s_error = std::any_cast<Motor_ERROR>(any_type_var);
    Notify("error_callback", s_error);
}

void Controller::startThreadUpdate()
{
    if (update_running_handler_.load()) {
        NLOG(severity_level::warning) << "Update Handler thread is already running.";
        return;
    }

    update_running_handler_ = true;
    update_handler_thread_ = std::thread([this]() {
        NLOG(severity_level::info) << "Update loop started.";
        static MotorDataMap motor_data_pre = GetMotorData();
        while (update_running_handler_.load()) {
            try {
                MotorDataMap motor_data = GetMotorData();
                
                // 중요한 값들만 비교 (예: RPM, 에러 상태)
                bool has_significant_change = false;
                for (const auto& [id, data] : motor_data) {
                    auto prev_it = motor_data_pre.find(id);
                    if (prev_it == motor_data_pre.end() || data.last_update_time != prev_it->second.last_update_time) 
                    {
                        has_significant_change = true;
                        break;
                    }
                }

                if (!has_significant_change) {
                    std::this_thread::sleep_for(1ms);
                    continue;
                }

                // if (motor_data_pre == motor_data) {
                //     std::this_thread::sleep_for(1ms);
                //     continue;
                // }
                updateMotorData(motor_data);
                calculateOdom();

                has_error_ = error_checker_->checkErrorsWithType(motor_data, st_driver_param_);

                motor_data_pre = motor_data;
            }
            catch (...) {
                NLOG(severity_level::info) << "update motor data error";
            }
            std::this_thread::sleep_for(1ms);
        }

        NLOG(severity_level::info) << "Update loop endded.";
    });

    NLOG(severity_level::info) << "Update Handler thread started.";
}

void Controller::stopThreadUpdate()
{
    if (!update_running_handler_.load()) {
        NLOG(severity_level::error) << "Update Handler thread is not running.";
        return;
    }

    update_running_handler_ = false;
    if (update_handler_thread_.joinable()) {
        update_handler_thread_.join();
    }
    NLOG(severity_level::info) << "Update Handler thread stopped.";
}

void Controller::startThreadError()
{
    if (error_running_handler_.load()) {
        NLOG(severity_level::warning) << "Error Handler thread is already running.";
        return;
    }

    error_running_handler_ = true;
    error_handler_thread_ = std::thread([this]() {
        NLOG(severity_level::info) << "Error loop started.";

        while (error_running_handler_.load()) {
            if (has_error_ == false) {
                std::this_thread::sleep_for(100ms);

                continue;
            }

            try {
                auto error = error_checker_->getError();
                auto motor_data = GetMotorData();
                // Fault
                for (const auto& [motor_id, data] : motor_data) {
                    if (motor_id == MOTOR_TRACTION_FL) {
                        if(pre_error_t_ != data.error_code) {
                            if (data.is_error) {
                                NLOG(severity_level::error) << "traction motor error code : " << data.error_code;
                            }
                            else {
                                NLOG(severity_level::info) << "traction motor error code : " << data.error_code;
                            }
                            pre_error_t_ = data.error_code;
                        }

                    }
                    if (motor_id == MOTOR_STEER_FL) {
                        if(pre_error_s_ != data.error_code) {
                            if (data.is_error) {
                                NLOG(severity_level::error) << "steer motor error code : " << data.error_code;
                            }
                            else{
                                NLOG(severity_level::info) << "steer motor error code : " << data.error_code;
                            }
                            pre_error_s_ = data.error_code;
                        }
                    }
                }
            }
            catch (...) {
                //
            }
        }
        NLOG(severity_level::info) << "Error loop endded.";
    });

    NLOG(severity_level::info) << "Update Handler thread started.";
}

void Controller::stopThreadError()
{
    if (!error_running_handler_.load()) {
        NLOG(severity_level::error) << "Error Handler thread is not running.";
        return;
    }

    error_running_handler_ = false;
    if (error_handler_thread_.joinable()) {
        error_handler_thread_.join();
    }
    NLOG(severity_level::info) << "Error Handler thread stopped.";
}

void Controller::callMotorCmd(const std::string& command, MotorId motor_id)
{
    motor_msgs::MotorCmd srv;
    srv.request.s_cmd = command;
    srv.request.motor_id = motor_id;

    if (motor_cmd_req_.call(srv)) {
        if (!srv.response.b_is_response) {
            LOG_ERROR("Controller navican command [%s] failed for motor_id [%d]", command.c_str(), motor_id);
        }
    }
    else {
        LOG_ERROR("Failed to call service for command [%s] from [%d]", command.c_str(), motor_id);
    }
}

void Controller::StopCommand()
{
    callMotorCmd("shutdown", MOTOR_ALL);
    finalize();
}

bool Controller::Notify(const std::string& name, const std::any& var)
{
    if (callbacks_.find(name) == callbacks_.end()) {
        return false;
    }
    else {
        callbacks_[name](var);
        return true;
    }
}

bool Controller::RegisterCallbackFunc(const std::string& name, const std::function<void(const std::any&)>& func)
{
    if (callbacks_.find(name) == callbacks_.end()) {
        callbacks_[name] = func;
        return true;
    }
    else {
        return false;
    }
}

};  // namespace NaviFra
