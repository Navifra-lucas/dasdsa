#include "nc_driver_main.hpp"

#include "core/util/logger.hpp"
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <cmath>

Driver::Driver(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    : nh_(nh)
    , nhp_(nhp)
{
    LOG_INFO("Driver Create");
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    Initialize(false);
    b_param_init_ = true;

    const char* home_dir = getenv("HOME");
    if (home_dir != nullptr) {
        pose_dir_ = std::string(home_dir) + "/navifra_solution/navicore/configs";
    }
    else {
        pose_dir_ = "/home/navifra/navifra_solution/navicore/configs";
    }

    if (access(pose_dir_.c_str(), F_OK) == -1) {
        mkdir(pose_dir_.c_str(), 0777);
    }
}

Driver::~Driver()
{
    Shutdown();
}

void Driver::Shutdown()
{
    b_terminate_ = true;

    LOG_INFO("thread wait");
    th_heartbeat_.join();
    LOG_INFO("thread joined");

    if (o_sim_motor_driver_ != nullptr) {
        LOG_INFO("delete o_sim_motor_driver_ start");
        delete o_sim_motor_driver_;
        LOG_INFO("delete o_sim_motor_driver_ done");
    }
    if (o_controller_ != nullptr) {
        LOG_INFO("delete o_controller_ start");
        o_controller_->StopCommand();
        LOG_INFO("delete o_controller_ start2");
        delete o_controller_;
        LOG_INFO("delete o_controller_ done");
    }
    LOG_INFO("destructor");
}

void Driver::Initialize(bool b_only_param_update)
{
    LOG_INFO("Update Param start %d", b_only_param_update);
    o_param_.UpdateParam(nh_, nhp_, st_interface_param_, st_driver_param_);
    LOG_INFO("Update Param done");
    if (b_only_param_update && n_kinematics_type_pre_ != st_driver_param_.n_kinematics_type) {
        b_init_select_ = false;
        Shutdown();
        b_terminate_ = false;
    }
    if (st_driver_param_.n_kinematics_type == NaviFra::KINEMATICS::DD && !b_init_select_ && o_controller_ == nullptr) {
        o_controller_ = new ControllerDD;
    }
    else if (st_driver_param_.n_kinematics_type == NaviFra::KINEMATICS::QD && !b_init_select_ && o_controller_ == nullptr) {
        o_controller_ = new ControllerQD;
    }
    else if (st_driver_param_.n_kinematics_type == NaviFra::KINEMATICS::SD && !b_init_select_ && o_controller_ == nullptr) {
        o_controller_ = new ControllerSD;
    }
    else if (st_driver_param_.n_kinematics_type == NaviFra::KINEMATICS::OD && !b_init_select_ && o_controller_ == nullptr) {
        o_controller_ = new ControllerOD;
    }
    LOG_INFO("o_controller select done");

    if (!b_init_select_) {
        if (st_interface_param_.b_sim_use && o_sim_motor_driver_ == nullptr) {
            o_sim_motor_driver_ = new MotorSimDriver(nh_, nhp_);
        }
    }
    else if (o_sim_motor_driver_ != nullptr) {
        o_sim_motor_driver_->Initialize(b_only_param_update);
    }
    o_controller_->SetInterfaceParam(st_interface_param_);
    o_controller_->SetDriverParam(st_driver_param_);
    o_controller_->initialize();
    o_controller_->Initialize(b_only_param_update);

    f_linear_accel_vel_d_ = st_driver_param_.f_control_period_ms / 1000 * st_driver_param_.f_linear_max_accel_mss;
    f_linear_decel_vel_d_ = st_driver_param_.f_control_period_ms / 1000 * st_driver_param_.f_linear_max_decel_mss;
    f_angular_accde_vel_d_ = st_driver_param_.f_control_period_ms / 1000 * st_driver_param_.f_rot_max_accdecel_degss * M_PI / 180;

    checktime_hearbeat_ = std::chrono::steady_clock::now();
    checktime_odom_ = std::chrono::steady_clock::now();

    LOG_INFO("Regist done");
    if (!b_only_param_update) {
        RegistTalker();
        RegistListener();
    }
    if (!b_init_select_) {
        RegistCallback();
        th_heartbeat_ = std::thread(&Driver::HeartBeatLoop, this);
        b_init_select_ = true;
    }
    n_kinematics_type_pre_ = st_driver_param_.n_kinematics_type;
}

void Driver::RegistTalker()
{
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pub_motor_info_ = nh_.advertise<core_msgs::MotorInfo>("motor_info", 5);
    pub_estop_state_ = nh_.advertise<std_msgs::Bool>("estop_state", 5);
    pub_brake_cmd_ = nh_.advertise<std_msgs::Bool>("motor_brakeon_target", 5);
    pub_turntable_feed_ = nh_.advertise<std_msgs::Float32>("turntable_feedback_deg", 5);
    pub_turntable_speed_ = nh_.advertise<std_msgs::Float32>("turntable_feedback_deg_s", 5);
    pub_imu_info_ = nh_.advertise<sensor_msgs::Imu>("imu_info", 5);

    pub_mileage_ = nh_.advertise<std_msgs::Float64>("mileage", 5);
    pub_mileage_local_ = nh_.advertise<std_msgs::Float64>("mileage_local", 5);
}

void Driver::RegistListener()
{
    sub_cmd_vel_ = nh_.subscribe("cmd_vel", 5, &Driver::RecvCmdvel, this, ros::TransportHints().tcpNoDelay(true));
    sub_quad_cmd_ = nh_.subscribe("quad_cmd", 5, &Driver::RecvQuadCmd, this);
    sub_enc_zero_ = nh_.subscribe("encoder_zero", 5, &Driver::RecvEncoderZero, this);
    sub_param_ = nh_.subscribe("navifra/param_update", 5, &Driver::RecvParamUpdate, this);
    sub_turn_table_cmd_ = nh_.subscribe("turntable_target_deg_s", 5, &Driver::RecvTurnCmd, this);
    sub_turn_table_brake_ = nh_.subscribe("turntable/brakeon_feedback", 5, &Driver::RecvTurnBrake, this);
    sub_brake_ = nh_.subscribe("motor_brakeon_feedback", 5, &Driver::RecvBrake, this);
    sub_estop_ = nh_.subscribe("emergency", 5, &Driver::RecvESTOP, this);
    sub_control_off_ = nh_.subscribe("motor_control_off", 5, &Driver::RecvControlOff, this);
    sub_steer_open_ = nh_.subscribe("steer_alot_open", 5, &Driver::RecvSteerOpen, this);
    sub_steering_direction_ = nh_.subscribe("steering_direction", 5, &Driver::RecvSteeringDirection, this);
    sub_alarm_ = nh_.subscribe("navifra/alarm", 5, &Driver::RecvAlarmUpdate, this);
    sub_motor_driver_init_ = nh_.subscribe("motor_driver/init", 5, &Driver::RecvMotorDriverInit, this);
    sub_motor_error_reset_ = nh_.subscribe("motor_driver/error_reset", 5, &Driver::RecvErrorReset, this);
    sub_motor_driver_interlock_ = nh_.subscribe("motor_driver/interlock", 5, &Driver::RecvMotorDriverInterlock, this);
    sub_battery_info_ = nh_.subscribe("battery_info", 5, &Driver::RecvBatteryInfo, this);
}

void Driver::RegistCallback()
{
    o_controller_->RegisterCallbackFunc("odom_callback", std::bind(&Driver::ReceivedOdom, this, std::placeholders::_1));
    o_controller_->RegisterCallbackFunc("motor_info_callback", std::bind(&Driver::ReceivedMotorInfo, this, std::placeholders::_1));
    o_controller_->RegisterCallbackFunc("imu_info_callback", std::bind(&Driver::ReceivedImuInfo, this, std::placeholders::_1));
    o_controller_->RegisterCallbackFunc("error_callback", std::bind(&Driver::ReceivedError, this, std::placeholders::_1));
}

void Driver::HeartBeatLoop()
{
    static NaviFra::Cmd o_cmd_pre;
    std::chrono::steady_clock::time_point checktime_start;

    while (!b_terminate_) {
        std::chrono::steady_clock::time_point checktime_start = std::chrono::steady_clock::now();

        // interface check
        o_controller_->InterfaceCheck();

        // heartbeat check
        std::chrono::duration<double> sec = std::chrono::steady_clock::now() - checktime_hearbeat_;
        std::chrono::duration<double> turn_table_sec = std::chrono::steady_clock::now() - checktime_turn_table_hearbeat_;
        std::chrono::duration<double> brake_sec;

        {
            std::lock_guard<std::mutex> lock(mtx_brake_);
            brake_sec = std::chrono::steady_clock::now() - checktime_brake_;
        }
        NaviFra::Cmd o_cmd, o_cmd_new;
        {
            std::lock_guard<std::mutex> lock(mtx_cmd_);
            o_cmd = o_cmd_;
        }
        if (turn_table_sec.count() * 1000 > st_driver_param_.f_heartbeat_ms) {
            o_cmd.SetTurnTableVel(0);
        }
        else {
            o_cmd.SetTurnTableVel(f_turn_table_vel_);
        }

        if (st_driver_param_.b_use_brake == true && st_driver_param_.b_external_brake_control == true && brake_sec.count() > 3) {
            o_controller_->SetBrake(true);
            NaviFra::Motor_ERROR o_motor_error;
            o_motor_error.b_etc_error_update = true;
            o_motor_error.tp_etc_error = std::chrono::steady_clock::now();
            o_motor_error.n_etc_error_code = core_msgs::NaviAlarm::ERROR_BRAKE_SIGNAL_TIMEOUT;
            o_motor_error.s_etc_error_text = "ERROR_BRAKE_SIGNAL_TIMEOUT";
            SetMotorAlarm(o_motor_error);
        }

        if (sec.count() * 1000 > st_driver_param_.f_heartbeat_ms || b_error_)  // brake
        {
            o_cmd.SetVelX(0);
            o_cmd.SetVelY(0);
            o_cmd.SetVelYaw(0);
            o_cmd.SetStopFlag(true);
            quad_str_ = "";
            o_cmd_new = o_cmd;
            geometry_msgs::Twist temp;
            temp.linear.x = 0;
            temp.linear.y = 0;
            temp.angular.z = 0;
            pub_cmd_vel_.publish(temp);
        }
        else {
            o_cmd_new = o_cmd;
        }

        if (st_driver_param_.b_accdecel_enable) {
            o_cmd_new = GetSpeedLimit(o_cmd, o_cmd_pre);
        }

        if (b_estop_) {
            NaviFra::Cmd o_tmp;
            o_cmd_new = o_tmp;
        }
        o_controller_->SetQuadCmd(quad_str_);
        o_controller_->WriteCommand(o_cmd_new);

        static int n_count = 0;
        n_count++;
        if (n_count > 100 && (o_cmd.GetVelX() != 0 || o_cmd_new.GetVelX() != 0)) {
            n_count = 0;
            NLOG(info) << "o_cmd " << o_cmd.GetVelX() << "/" << o_cmd.GetVelY() << "/" << o_cmd.GetVelYaw() << " o_cmd_new "
                       << o_cmd_new.GetVelX() << "/" << o_cmd_new.GetVelY() << "/" << o_cmd_new.GetVelYaw();
        }

        o_cmd_pre = o_cmd_new;
        if (b_brake_on_feedback_) {
            NaviFra::Cmd o_tmp;
            o_cmd_pre = o_tmp;
        }
        std::chrono::duration<double> sec_end = std::chrono::steady_clock::now() - checktime_start;

        if (sec_end.count() > 0.1)
            LOG_ERROR("sec_end %.3f", sec_end.count());

        float f_remain_time = st_driver_param_.f_control_period_ms - sec_end.count() * 1000;
        if (f_remain_time < 0)
            f_remain_time = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(int(f_remain_time)));
    }
}

Cmd Driver::GetSpeedLimit(Cmd& o_new_cmd, Cmd& o_pre_cmd)
{
    // set profile
    float f_new_x = o_new_cmd.GetVelX();
    float f_new_y = o_new_cmd.GetVelY();
    float f_new_a = o_new_cmd.GetVelYaw();

    float f_pre_x = o_pre_cmd.GetVelX();
    float f_pre_y = o_pre_cmd.GetVelY();
    float f_pre_a = o_pre_cmd.GetVelYaw();

    float f_result_x = f_pre_x;
    float f_result_y = f_pre_y;
    float f_result_a = f_pre_a;

    float f_x_accdec_check = fabs(f_new_x) - fabs(f_pre_x);
    float f_x_vel_d = f_new_x - f_pre_x;
    float f_y_vel_d = f_new_y - f_pre_y;
    float f_min_acc_x = min(fabs(f_x_vel_d), f_linear_accel_vel_d_);
    float f_min_dec_x = min(fabs(f_x_vel_d), f_linear_decel_vel_d_);

    if (f_x_accdec_check > 0)  // linear 가속
    {
        if (f_x_vel_d > 0) {  // + 방향
            f_result_x += f_min_acc_x;
            if (f_y_vel_d != 0) {
                f_new_y = f_result_y + fabs(f_min_acc_x) * f_y_vel_d / fabs(f_x_vel_d);
            }
        }
        else if (f_x_vel_d < 0) {  // - 방향
            f_result_x -= f_min_acc_x;
            if (f_y_vel_d != 0) {
                f_new_y = f_result_y + fabs(f_min_acc_x) * f_y_vel_d / fabs(f_x_vel_d);
            }
        }
    }
    else if (f_x_accdec_check <= 0)  // linear 감속
    {
        if (f_x_vel_d > 0) {  // + 방향
            f_result_x += f_min_dec_x;
            if (f_y_vel_d != 0) {
                f_new_y = f_result_y + fabs(f_min_dec_x) * f_y_vel_d / fabs(f_x_vel_d);
            }
        }
        else if (f_x_vel_d < 0) {  // - 방향
            f_result_x -= f_min_dec_x;
            if (f_y_vel_d != 0) {
                f_new_y = f_result_y + fabs(f_min_dec_x) * f_y_vel_d / fabs(f_x_vel_d);
            }
        }
    }
    float f_y_accdec_check = fabs(f_new_y) - fabs(f_pre_y);
    float f_y_vel_d_2 = f_new_y - f_pre_y;
    float f_min_acc_y = min(fabs(f_y_vel_d_2), f_linear_accel_vel_d_);
    float f_min_dec_y = min(fabs(f_y_vel_d_2), f_linear_decel_vel_d_);
    if (f_y_accdec_check > 0)  // linear 가속
    {
        if (f_y_vel_d_2 > 0) {  // + 방향
            f_result_y += f_min_acc_y;
            if (f_x_vel_d != 0) {
                f_result_x = f_pre_x + fabs(f_min_acc_y) * f_x_vel_d / fabs(f_y_vel_d);
            }
        }
        else if (f_y_vel_d_2 < 0) {  // - 방향
            f_result_y -= f_min_acc_y;
            if (f_x_vel_d != 0) {
                f_result_x = f_pre_x + fabs(f_min_acc_y) * f_x_vel_d / fabs(f_y_vel_d);
            }
        }
    }
    else if (f_y_accdec_check <= 0)  // linear 감속
    {
        if (f_y_vel_d_2 > 0) {  // + 방향
            f_result_y += f_min_dec_y;
            if (f_x_vel_d != 0) {
                f_result_x = f_pre_x + fabs(f_min_dec_y) * f_x_vel_d / fabs(f_y_vel_d);
            }
        }
        else if (f_y_vel_d_2 < 0) {  // - 방향
            f_result_y -= f_min_dec_y;
            if (f_x_vel_d != 0) {
                f_result_x = f_pre_x + fabs(f_min_dec_y) * f_x_vel_d / fabs(f_y_vel_d);
            }
        }
    }
    //선속도 변화량 비율체크
    float f_x_vel_rate = 1;
    float f_y_vel_rate = 1;
    if (f_x_vel_d != 0)
        f_x_vel_rate = fabs((f_result_x - f_pre_x) / f_x_vel_d);
    if (f_y_vel_d != 0)
        f_y_vel_rate = fabs((f_result_y - f_pre_y) / f_y_vel_d);

    float f_min_accdec_linear = min(f_x_vel_rate, f_y_vel_rate);
    f_new_a = f_result_a + f_min_accdec_linear * (f_new_a - f_pre_a);
    float f_a_vel_d = f_new_a - f_pre_a;
    float f_a_accdec_check = fabs(f_new_a) - fabs(f_pre_a);
    float f_min_acc_a = min(fabs(f_a_vel_d), f_angular_accde_vel_d_);
    if (f_a_accdec_check > 0)  // angular 가속
    {
        if (f_a_vel_d > 0)  // + 방향
            f_result_a += f_min_acc_a;
        else if (f_a_vel_d < 0)  // - 방향
            f_result_a -= f_min_acc_a;
    }
    else if (f_a_accdec_check <= 0)  // angular 감속
    {
        if (f_a_vel_d > 0)  // + 방향
            f_result_a += f_min_acc_a;
        else if (f_a_vel_d < 0)  // - 방향
            f_result_a -= f_min_acc_a;
    }

    // when stop, continue turning radius (r)
    if (f_new_x == 0 && f_new_y == 0 && f_new_a == 0 && (f_pre_x != 0 || f_pre_y != 0) &&
        fabs(f_pre_a) > 0.01)  // 정지 명령이고 이전 선속도와 각속도가 0이 아닐 때
    {
        // previous turning radius
        float f_pre_rx = fabs(f_pre_x / f_pre_a);
        float f_pre_ry = fabs(f_pre_y / f_pre_a);
        // 곡률 유지를 위한 a
        float f_ax = 0;
        float f_ay = 0;
        if (f_pre_rx > 0.001)
            f_ax = fabs(f_result_x) / f_pre_rx * (f_pre_a / fabs(f_pre_a));
        if (f_pre_ry > 0.001)
            f_ay = fabs(f_result_y) / f_pre_ry * (f_pre_a / fabs(f_pre_a));
        // NLOG(info) << "result f_pre_x " << f_pre_x << " f_pre_y " << f_pre_y << " f_pre_a " << f_pre_a << " f_pre_rx " << f_pre_rx
        //            << " f_pre_ry " << f_pre_ry << " f_ax " << f_ax << " f_ay " << f_ay;

        f_result_a = fabs(f_ax) >= fabs(f_ay) ? f_ax : f_ay;
    }

    Cmd st_vel_result;
    st_vel_result = o_new_cmd;
    st_vel_result.SetDirectionFlag(o_new_cmd.GetDirectionFlag());
    st_vel_result.SetVelX(f_result_x);
    st_vel_result.SetVelY(f_result_y);
    st_vel_result.SetVelYaw(f_result_a);
    // NLOG(info) << "result " << f_result_x << " " << f_result_y << " " << f_result_a;
    // stop
    if (fabs(o_new_cmd.GetVelX()) < 0.001 && fabs(o_new_cmd.GetVelY()) < 0.001 && fabs(o_new_cmd.GetVelYaw()) < 0.001) {
        st_vel_result.SetStopFlag(true);
        st_vel_result.SetDirectionFlag(o_pre_cmd.GetDirectionFlag());
    }
    return st_vel_result;
}

void Driver::ReceivedOdom(const std::any& any_type_var)
{
    // odom 너무 빠르게 퍼블리시 하지 않도록 제한
    std::chrono::duration<double> sec = std::chrono::steady_clock::now() - checktime_odom_;
    if (sec.count() < 0.01)  // 10ms
        return;
    checktime_odom_ = std::chrono::steady_clock::now();

    std::vector<NaviFra::SimplePos> msg = std::any_cast<std::vector<NaviFra::SimplePos>>(any_type_var);

    if (msg.size() > 1) {
        NaviFra::SimplePos o_odom_pos = msg[0];
        NaviFra::SimplePos o_velocity = msg[1];

        static tf2_ros::TransformBroadcaster br;

        tf2::Quaternion odom_quat;
        odom_quat.setRPY(0, 0, o_odom_pos.GetRad());
        ros::Time current_time = ros::Time::now();

        // odom data publish
        nav_msgs::Odometry odom_ros;
        odom_ros.header.stamp = current_time;
        odom_ros.header.frame_id = st_driver_param_.s_target_frame_id.c_str();
        odom_ros.child_frame_id = st_driver_param_.s_child_frame_id.c_str();

        odom_ros.pose.pose.position.x = o_odom_pos.GetXm();
        odom_ros.pose.pose.position.y = o_odom_pos.GetYm();
        odom_ros.pose.pose.position.z = o_odom_pos.GetZm();

        odom_ros.pose.pose.orientation.x = odom_quat.x();
        odom_ros.pose.pose.orientation.y = odom_quat.y();
        odom_ros.pose.pose.orientation.z = odom_quat.z();
        odom_ros.pose.pose.orientation.w = odom_quat.w();

        odom_ros.twist.twist.linear.x = o_velocity.GetXm();
        odom_ros.twist.twist.linear.y = o_velocity.GetYm();
        odom_ros.twist.twist.linear.z = 0;
        odom_ros.twist.twist.angular.x = 0;
        odom_ros.twist.twist.angular.y = 0;
        odom_ros.twist.twist.angular.z = o_velocity.GetRad();

        // publish the message
        if (!b_gazebo_use_) {
            pub_odom_.publish(odom_ros);
        }

        // tf data publish
        if (st_driver_param_.b_tf_enable) {
            geometry_msgs::TransformStamped tf_ros;

            tf_ros.header.stamp = current_time;
            tf_ros.header.frame_id = st_driver_param_.s_target_frame_id.c_str();
            tf_ros.child_frame_id = st_driver_param_.s_child_frame_id.c_str();

            tf_ros.transform.translation.x = o_odom_pos.GetXm();
            tf_ros.transform.translation.y = o_odom_pos.GetYm();
            tf_ros.transform.translation.z = 0.1;

            tf_ros.transform.rotation.x = odom_quat.x();
            tf_ros.transform.rotation.y = odom_quat.y();
            tf_ros.transform.rotation.z = odom_quat.z();
            tf_ros.transform.rotation.w = odom_quat.w();
            br.sendTransform(tf_ros);
        }
    }
    // Mileage calculation
    if (msg.size() > 1) {
        NaviFra::SimplePos o_odom_pos = msg[0];
        count_mileage_++;
        if (count_mileage_ > 1) {
            count_mileage_ = 0;
            double odom_x = o_odom_pos.GetXm();
            double odom_y = o_odom_pos.GetYm();

            if (last_pos_.empty()) {
                last_pos_ = {odom_x, odom_y};
            }
            else {
                std::string mileage_path = pose_dir_ + "/mileage.txt";
                double mileage = 0.0;
                
                std::ifstream infile(mileage_path);
                if (infile.good()) {
                    std::string data;
                    if (std::getline(infile, data) && !data.empty()) {
                        try {
                            mileage = std::stod(data);
                        } catch (...) {
                            mileage = 0.0;
                        }
                    }
                    infile.close();

                    double dmile = std::hypot(odom_x - last_pos_[0], odom_y - last_pos_[1]);
                    if (dmile < 20) {
                        mileage += dmile / 1000.0;
                        mileage = std::round(mileage * 1000000.0) / 1000000.0;
                    }
                    std_msgs::Float64 mileage_msg;
                    mileage_msg.data = mileage;

                    std::ofstream outfile(mileage_path);
                    outfile << mileage;
                    outfile.close();

                    last_pos_ = {odom_x, odom_y};
                    pub_mileage_.publish(mileage_msg);
                }
                else {
                    std::ofstream outfile(mileage_path);
                    outfile << "0.0";
                    outfile.close();
                }

                std::string mileage_path_check = pose_dir_ + "/mileage_check.txt";
                std::ifstream infile_check(mileage_path_check);
                if (infile_check.good()) {
                    std::string data;
                    double check_mile = 0.0;
                    if (std::getline(infile_check, data) && !data.empty()) {
                         try {
                             check_mile = std::stod(data);
                         } catch (...) {
                             check_mile = 0.0;
                         }
                    }
                    infile_check.close();

                    std_msgs::Float64 mileage_local_msg;
                    mileage_local_msg.data = mileage - check_mile;
                    pub_mileage_local_.publish(mileage_local_msg);
                }
                else {
                    std::ofstream outfile_check(mileage_path_check);
                    outfile_check << "0.0";
                    outfile_check.close();
                }
            }
        }
    }

    std_msgs::Bool msg_estop;
    msg_estop.data = b_estop_;
    pub_estop_state_.publish(msg_estop);
}

void Driver::ReceivedMotorInfo(const std::any& any_type_var)
{
    std::chrono::duration<double> sec = std::chrono::steady_clock::now() - checktime_motorinfo_;
    if (sec.count() < 0.01)
        return;
    checktime_motorinfo_ = std::chrono::steady_clock::now();

    core_msgs::MotorInfo o_motor_info = std::any_cast<core_msgs::MotorInfo>(any_type_var);
    std_msgs::Bool msg_brake;
    msg_brake.data = o_motor_info.b_brake_on_target;
    pub_brake_cmd_.publish(msg_brake);
    b_brake_on_feedback_ = o_motor_info.b_brake_on_feedback;
    std_msgs::Float32 msg_turntable;
    msg_turntable.data = o_motor_info.f_turn_table_deg;
    if (fabs(o_motor_info.f_turn_table_deg) > 0.001 && st_driver_param_.n_kinematics_type == NaviFra::KINEMATICS::DD)
        pub_turntable_feed_.publish(msg_turntable);

    std_msgs::Float32 msg_turntable_speed;
    msg_turntable_speed.data = o_motor_info.f_turn_table_feedback_deg_s;
    if (fabs(o_motor_info.f_turn_table_feedback_deg_s) > 0.001 && st_driver_param_.n_kinematics_type == NaviFra::KINEMATICS::DD)
        pub_turntable_speed_.publish(msg_turntable_speed);

    NaviFra::Motor_ERROR o_motor_error = GetMotorAlarm();
    std::chrono::duration<double> sec_FL_error = std::chrono::steady_clock::now() - o_motor_error.tp_FL_error;
    std::chrono::duration<double> sec_FR_error = std::chrono::steady_clock::now() - o_motor_error.tp_FR_error;
    std::chrono::duration<double> sec_RL_error = std::chrono::steady_clock::now() - o_motor_error.tp_RL_error;
    std::chrono::duration<double> sec_RR_error = std::chrono::steady_clock::now() - o_motor_error.tp_RR_error;
    std::chrono::duration<double> sec_etc_error = std::chrono::steady_clock::now() - o_motor_error.tp_etc_error;

    float f_time_sec = 0.3;
    bool b_error = false;
    if (sec_FL_error.count() < f_time_sec) {
        b_error = true;
        o_motor_info.n_FL_error_code = o_motor_error.n_FL_error_code;
        o_motor_info.s_FL_error_text = o_motor_error.s_FL_error_text;
    }
    if (sec_FR_error.count() < f_time_sec) {
        b_error = true;
        o_motor_info.n_FR_error_code = o_motor_error.n_FR_error_code;
        o_motor_info.s_FR_error_text = o_motor_error.s_FR_error_text;
    }
    if (sec_RL_error.count() < f_time_sec) {
        b_error = true;
        o_motor_info.n_RL_error_code = o_motor_error.n_RL_error_code;
        o_motor_info.s_RL_error_text = o_motor_error.s_RL_error_text;
    }
    if (sec_RR_error.count() < f_time_sec) {
        b_error = true;
        o_motor_info.n_RR_error_code = o_motor_error.n_RR_error_code;
        o_motor_info.s_RR_error_text = o_motor_error.s_RR_error_text;
    }
    if (sec_etc_error.count() < f_time_sec) {
        b_error = true;
        o_motor_info.n_etc_error_code = o_motor_error.n_etc_error_code;
        o_motor_info.s_etc_error_text = o_motor_error.s_etc_error_text;
    }
    if (b_error)  // error가 하나라도 있으면 sto 비활성화
    {
        b_error_ = true;
    }
    else {
        b_error_ = false;
    }

    o_motor_info.f_turn_table_target_deg_s = f_turn_table_vel_;
    o_motor_info.b_turn_feedback_brakeon = b_turn_feedback_brakeon_;
    pub_motor_info_.publish(o_motor_info);
}

void Driver::ReceivedError(const std::any& any_type_var)
{
    std::lock_guard<std::mutex> lock(mtx_info_);
    NaviFra::Motor_ERROR s_error = std::any_cast<NaviFra::Motor_ERROR>(any_type_var);
    SetMotorAlarm(s_error);
}

void Driver::ReceivedImuInfo(const std::any& any_type_var)
{
    NaviFra::IMU_Data_t o_imu_data = std::any_cast<NaviFra::IMU_Data_t>(any_type_var);
    sensor_msgs::Imu msg;
    tf2::Quaternion quat;
    double roll_rad = o_imu_data.f_roll * DEGtoRAD;
    double pitch_rad = o_imu_data.f_pitch * DEGtoRAD;
    double yaw_rad = o_imu_data.f_yaw * DEGtoRAD;

    quat.setRPY(roll_rad, pitch_rad, yaw_rad);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "imu_link";
    msg.orientation.x = (float)quat.x();
    msg.orientation.y = (float)quat.y();
    msg.orientation.z = (float)quat.z();
    msg.orientation.w = (float)quat.w();
    msg.angular_velocity.x = o_imu_data.f_angular_vel_x;
    msg.angular_velocity.y = o_imu_data.f_angular_vel_y;
    msg.angular_velocity.z = o_imu_data.f_angular_vel_z;
    msg.linear_acceleration.x = o_imu_data.f_linear_acc_x * 9.8;
    msg.linear_acceleration.y = o_imu_data.f_linear_acc_y * 9.8;
    msg.linear_acceleration.z = o_imu_data.f_linear_acc_z * 9.8;
    pub_imu_info_.publish(msg);
}

void Driver::SetMotorAlarm(NaviFra::Motor_ERROR& o_motor_error)
{
    std::lock_guard<std::mutex> lock(mtx_error_);
    if (o_motor_error.b_FL_error_update) {
        o_motor_error_.tp_FL_error = std::chrono::steady_clock::now();
        o_motor_error_.n_FL_error_code = o_motor_error.n_FL_error_code;
        o_motor_error_.s_FL_error_text = o_motor_error.s_FL_error_text;
    }
    if (o_motor_error.b_FR_error_update) {
        o_motor_error_.tp_FR_error = std::chrono::steady_clock::now();
        o_motor_error_.n_FR_error_code = o_motor_error.n_FR_error_code;
        o_motor_error_.s_FR_error_text = o_motor_error.s_FR_error_text;
    }
    if (o_motor_error.b_RL_error_update) {
        o_motor_error_.tp_RL_error = std::chrono::steady_clock::now();
        o_motor_error_.n_RL_error_code = o_motor_error.n_RL_error_code;
        o_motor_error_.s_RL_error_text = o_motor_error.s_RL_error_text;
    }
    if (o_motor_error.b_RR_error_update) {
        o_motor_error_.tp_RR_error = std::chrono::steady_clock::now();
        o_motor_error_.n_RR_error_code = o_motor_error.n_RR_error_code;
        o_motor_error_.s_RR_error_text = o_motor_error.s_RR_error_text;
    }
    if (o_motor_error.b_etc_error_update) {
        o_motor_error_.tp_etc_error = std::chrono::steady_clock::now();
        o_motor_error_.n_etc_error_code = o_motor_error.n_etc_error_code;
        o_motor_error_.s_etc_error_text = o_motor_error.s_etc_error_text;
    }
}

NaviFra::Motor_ERROR Driver::GetMotorAlarm()
{
    std::lock_guard<std::mutex> lock(mtx_error_);
    return o_motor_error_;
}

void Driver::SetGazeboUse(bool val)
{
    b_gazebo_use_ = val;
    NLOG(info) << "[driver] gazebo set to " << b_gazebo_use_;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nc_motor_commander");

#ifdef NAVIFRA_LICENSE_ON

    License o_license_checker;
    if (o_license_checker.CheckLicence() == 0) {
        LOG_ERROR("************************************************");
        LOG_ERROR("DRIVER Your License Not Resiter. Terminate the process.");
        LOG_ERROR("************************************************");
        return 0;
    }
    else {
        LOG_INFO("DRIVER License is Resitered.");
    }

#else  // license 옵션 안건 경우
    LOG_WARNING("************************************************");
    LOG_WARNING("DRIVER License Not Checked!  Please Check your License.");
    LOG_WARNING("************************************************");
#endif

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    bool gazebo_flag = false;
    nhp.getParam("b_gazebo_use", gazebo_flag);

    Driver motor_driver(nh, nhp);
    motor_driver.SetGazeboUse(gazebo_flag);
    ros::spin();

    return 0;
}