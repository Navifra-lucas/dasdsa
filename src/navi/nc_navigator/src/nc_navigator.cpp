
#include "nc_navigator/nc_navigator_pch.h"
// #include <nc_navigator/nc_error_alarm.hpp>
#include <nc_navigator/nc_navigator.hpp>

using namespace std;
using namespace NaviFra;

ncNavigator::ncNavigator()
{
    o_param_server_.RegisteCallbackFunc("param_callback", std::bind(&ncNavigator::RecvParams, this, std::placeholders::_1));
    o_param_server_.GetParam();

    cmd_vel_pub_ = node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    str_param_pub_ = node_handle_.advertise<std_msgs::String>("navifra/param_update", 10, true);
    str_navi_status_pub_ = node_handle_.advertise<std_msgs::String>("navifra/status", 10, true);
    navistatus_pub_ = node_handle_.advertise<core_msgs::NavicoreStatus>("navifra/info", 10, true);
    navialarm_pub_ = node_handle_.advertise<core_msgs::NaviAlarm>("navifra/alarm", 10, true);
    response_pub_ = node_handle_.advertise<std_msgs::String>("/navifra/response", 10, true);
    motion_info_pub_ = node_handle_.advertise<core_msgs::MotionInfo>("/motion_info", 10, true);

    // camera roi
    camera_roi_pub_ = node_handle_.advertise<core_msgs::CameraRoiInfoWia>("/path_camera_roi", 10, true);

    plc_cmd_pub_ = node_handle_.advertise<std_msgs::String>("output_command", 10, true);
    emergency_pub_ = node_handle_.advertise<std_msgs::Bool>("emergency", 10, true);
    charge_state_pub_ = node_handle_.advertise<std_msgs::Bool>("navifra/charge_on", 10, true);
    pallet_id_pub_ = node_handle_.advertise<std_msgs::String>("pallet_id", 5);

    // move
    livepath_sub_ = node_handle_.subscribe("navifra/live_path", 10, &ncNavigator::RecvLivePath, this);

    // action
    cmd_sub_ = node_handle_.subscribe("navifra/cmd", 10, &ncNavigator::RecvCmd, this);
    max_speed_sub_ = node_handle_.subscribe("navifra/speed_limit", 10, &ncNavigator::RecvMaxSpeed, this);
    speed_percent_sub_ = node_handle_.subscribe("navifra/speed_percent", 10, &ncNavigator::RecvSpeedPercent, this);
    turn_percent_sub_ = node_handle_.subscribe("navifra/turn_percent", 10, &ncNavigator::RecvTurnPercent, this);
    arrive_boundary_sub_ = node_handle_.subscribe("navifra/arrive_boundary", 10, &ncNavigator::RecvArriveBoundary, this);
    pallet_read_cmd_sub_ = node_handle_.subscribe("pallet_id_read_cmd", 10, &ncNavigator::RecvPalletReadCmd, this);

    // 한번에 정리코드
    log_sub_ = node_handle_.subscribe("navifra/log", 10, &ncNavigator::RecvLog, this);

    ros_robot_pos_sub_ =
        node_handle_.subscribe("localization/robot_pos", 10, &ncNavigator::RecvRobotoPos, this, ros::TransportHints().tcpNoDelay(true));

    motor_info_sub_ = node_handle_.subscribe("motor_info", 10, &ncNavigator::RecvMotorInfo, this);
    lidar_info_sub_ = node_handle_.subscribe("lidar_info", 10, &ncNavigator::RecvLidarInfo, this);
    upper_info_sub_ = node_handle_.subscribe("upper_info", 10, &ncNavigator::RecvUpperInfo, this);
    localize_info_sub_ = node_handle_.subscribe("localize_info", 10, &ncNavigator::RecvLocalizeInfo, this);
    battery_info_sub_ = node_handle_.subscribe("bms_info", 10, &ncNavigator::RecvBatteryInfo, this);
    cheonil_info_sub_ = node_handle_.subscribe("cheonil/read_register", 10, &ncNavigator::RecvCheonilInfo, this);

    etc_info_sub_ = node_handle_.subscribe("etc_info", 10, &ncNavigator::RecvEtcInfo, this);

    error_sub_ = node_handle_.subscribe("navifra/error", 10, &ncNavigator::RecvError, this);
    warning_sub_ = node_handle_.subscribe("navifra/warning", 10, &ncNavigator::RecvWarning, this);
    nownode_sub_ = node_handle_.subscribe("/nc_task_manager/now_node", 10, &ncNavigator::RecvNowNode, this);
    load_sub_ = node_handle_.subscribe("navifra/loaded", 1, &ncNavigator::RecvLoad, this);
    lccs_sub_ = node_handle_.subscribe("navifra/lccs_use", 1, &ncNavigator::RecvLCCS, this);
    sub_fork_position_ = node_handle_.subscribe("fork_position_state", 10, &ncNavigator::ForkPositionCallback, this);
    // task_info_sub_ = node_handle_.subscribe("/nc_task_manager/task_name", 10, &ncNavigator::onTaskName, this);

    tp_start_up_time_ = std::chrono::steady_clock::now();
    tp_goal_time_ = std::chrono::steady_clock::now();
    st_status_.tp_resent_motor_error_timestamp_ = std::chrono::steady_clock::now();
    st_status_.tp_resent_scan_timestamp_ = std::chrono::steady_clock::now();
    st_status_.tp_resent_pos_timestamp_ = std::chrono::steady_clock::now();
    st_status_.tp_last_motion_time_ = std::chrono::steady_clock::now();
    core_status_.f_robot_battery = 80;
    LoadFromFile();

    th0_ = boost::thread(boost::bind(&ncNavigator::NavigationStatusThread, this));

    core_status_.f_speed_limit_ms = 1.8;
    core_status_.f_speed_percent = 100.0;
    AbortMission();
    RegisteAllCallback();

    o_data_listener_.RegisteCallbackFunc(
        "all_sensor_data_callback", std::bind(&ncNavigator::RecvAllSensorData, this, std::placeholders::_1));
}

ncNavigator::~ncNavigator()
{
    if (th0_.joinable()) {
        th0_.join();
    }
    LOG_INFO("Desconstructor");
}

void ncNavigator::RegisteAllCallback()
{
    constexpr char NAVI_STATUS_CALLBACK[] = "navi_status_callback";
    constexpr char NAVI_ALARM_CALLBACK[] = "navi_alarm_callback";
    constexpr char GOAL_CALLBACK[] = "goal_callback";
    constexpr char NAVI_INFO_CALLBACK[] = "navi_info_callback";
    constexpr char MOTION_INFO_CALLBACK[] = "motion_info_callback";
    constexpr char REQUEST_AVOIDANCE[] = "request_avoidance";
    constexpr char NAVI_MISSION_ROI_CALLBACK[] = "navi_roi_callback";

    // aiden//
    o_mission_manager_.RegistCbFunc(NAVI_STATUS_CALLBACK, std::bind(&ncNavigator::RecvNaviStatus, this, std::placeholders::_1));
    o_mission_manager_.RegistCbFunc(NAVI_ALARM_CALLBACK, std::bind(&ncNavigator::RecvNaviAlarm, this, std::placeholders::_1));
    o_mission_manager_.RegistCbFunc(GOAL_CALLBACK, std::bind(&ncNavigator::RecvMoveToGoal, this, std::placeholders::_1));
    o_mission_manager_.RegistCbFunc(NAVI_INFO_CALLBACK, std::bind(&ncNavigator::RecvNaviInfo, this, std::placeholders::_1));
    o_mission_manager_.RegistCbFunc(NAVI_MISSION_ROI_CALLBACK, std::bind(&ncNavigator::RecvCurMissionROI, this, std::placeholders::_1));
    o_mission_manager_.RegistCbFunc(MOTION_INFO_CALLBACK, std::bind(&ncNavigator::RecvMotionInfo, this, std::placeholders::_1));
    o_mission_manager_.RegistCbFunc(REQUEST_AVOIDANCE, std::bind(&ncNavigator::RecvRequestAvoidance, this, std::placeholders::_1));
}

bool ncNavigator::LoadFromFile()
{
    // YAML 파일 경로
    std::string yaml_path = ros::package::getPath("core_msgs") + "/msg/navi_alarm.yaml";
    try {
        YAML::Node config = YAML::LoadFile(yaml_path);
        YAML::Node alarms = config["navi_alarm"];
        if (!alarms.IsMap()) {
            NLOG(error) << "Invalid format: navi_alarm is not a map";
            return false;
        }

        value_to_name_.clear();
        for (auto it = alarms.begin(); it != alarms.end(); ++it) {
            std::string name = it->first.as<std::string>();
            int value = it->second.as<int>();
            value_to_name_[value] = name;
            NLOG(info) << "Navi Alarm : " << name << ", " << value;
        }
        return true;
    }
    catch (const YAML::Exception& e) {
        NLOG(error) << "YAML load error: " << e.what();
        return false;
    }
}

std::string ncNavigator::GetAlarmText(int value) const
{
    auto it = value_to_name_.find(value);
    if (it != value_to_name_.end()) {
        return it->second;
    }
    return "ERROR_UNKNOWN";
}

void ncNavigator::PublishCmdVelZero()
{
    cmd_vel_pub_.publish(geometry_msgs::Twist());
}

void ncNavigator::RecvCurMissionROI(const boost::any& any_var)
{
    core_msgs::CameraRoiInfoWia msgs_camera = boost::any_cast<core_msgs::CameraRoiInfoWia>(any_var);
    camera_roi_pub_.publish(msgs_camera);
}

void ncNavigator::RecvAllSensorData(const boost::any& any_var)
{
    NaviFra::SensorMsg_t st_sensor_msg = boost::any_cast<NaviFra::SensorMsg_t>(any_var);
    core_status_.f_robot_current_linear_vel_x = st_sensor_msg.o_robot_speed.GetXm();
    core_status_.f_robot_current_linear_vel_y = st_sensor_msg.o_robot_speed.GetYm();
    core_status_.f_robot_current_angular_vel_w = st_sensor_msg.o_robot_speed.GetDeg();
    o_mission_manager_.SetSensorMsg(st_sensor_msg);
}

void ncNavigator::RecvBatteryInfo(const core_msgs::BmsInfo::ConstPtr& msg)
{
    core_status_.f_robot_battery = msg->f32_soc;
}

void ncNavigator::RecvPalletReadCmd(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == "start") {
        b_pallet_detect_check_ = true;
    }
}

void ncNavigator::RecvCheonilInfo(const core_msgs::CheonilReadRegister::ConstPtr& msg)
{
    core_status_.f_robot_battery = msg->battery;
    std_msgs::Bool b_msg;
    b_msg.data = static_cast<bool>(msg->charge_state);
    charge_state_pub_.publish(b_msg);
    static int n_cnt = 0;

    if (st_status_.b_running_ && b_pallet_detect_check_) {
        if (msg->load_state != 0) {
            n_cnt++;
            if(n_cnt > 3) {
                o_mission_manager_.SetGoalArrivedFlag(true);
                b_pallet_detect_check_ = false;
                n_cnt = 0;
            }
        }
        else {
            n_cnt = 0;
        }
    }
    if(msg->pallet_id_remove) {
        std_msgs::String s_msg;
        s_msg.data = "";
        pallet_id_pub_.publish(s_msg);
    }
}

void ncNavigator::RecvParams(const boost::any& any_var)
{
    LOG_DEBUG("SetNaviParamNavi");
    st_param_ = boost::any_cast<Parameters_t>(any_var);
    core_status_.f_linear_speed_max_ms = st_param_.st_motion_param.f_linear_speed_max_ms;
    core_status_.f_max_linear_velocity = st_param_.st_motion_param.f_linear_speed_max_ms;
    core_status_.f_min_linear_velocity = st_param_.st_motion_param.f_linear_speed_min_ms;
    core_status_.f_linear_acceleration = st_param_.f_linear_max_accel_mss;
    core_status_.f_linear_dcceleration = st_param_.f_linear_max_decel_mss;
    core_status_.f_max_angular_velocity = st_param_.f_align_motion_max_rot_degs;
    core_status_.f_min_angular_velocity = st_param_.f_align_motion_min_rot_degs;
    core_status_.f_angular_acceleration =
        st_param_.f_align_motion_max_rot_degs * st_param_.f_align_motion_max_rot_degs / st_param_.f_align_motion_range_accel_deg;
}

void ncNavigator::RecvMoveToGoal(const boost::any& any_var)
{
    if (any_var.type() == typeid(NaviFra::CommandVelocity_t) && st_status_.b_running_)  // running일때만 cmd_vel 추가
    {
        NaviFra::CommandVelocity_t st_vel = boost::any_cast<NaviFra::CommandVelocity_t>(any_var);

        geometry_msgs::Twist st_twist;

        st_twist.linear.x = st_vel.f_linear_speed_x_ms;
        st_twist.linear.y = st_vel.f_linear_speed_y_ms;
        st_twist.angular.z = st_vel.f_angular_speed_degs * DEGtoRAD;
        {
            core_status_.f_robot_target_linear_vel_x = st_vel.f_linear_speed_x_ms;
            core_status_.f_robot_target_linear_vel_y = st_vel.f_linear_speed_y_ms;
            core_status_.f_robot_target_angular_vel_w = st_vel.f_angular_speed_degs;
        }
        if (st_vel.b_reverse_motion) {
            st_twist.angular.x = 2;
        }
        else {
            st_twist.angular.x = 1;
        }

        if (b_addgoal_ && st_twist.linear.x == 0 && st_twist.linear.y == 0 && st_twist.angular.z == 0) {
            b_addgoal_ = false;
            return;
        }
        o_camera_controller_.SelectCamera(st_twist);
        cmd_vel_pub_.publish(st_twist);
    }
}

void ncNavigator::RecvNaviStatus(const boost::any& any_var)
{
    int n_now_status = boost::any_cast<int>(any_var);
    switch (n_now_status) {
        case static_cast<int>(NaviFra::AutoMoveMission::NAVIGATION_STATUS::IDLE):
            st_status_.b_paused_path_ = false;
            st_status_.n_cur_navistatus_ = static_cast<int>(NaviFra::AutoMoveMission::NAVIGATION_STATUS::IDLE);
            break;
        case static_cast<int>(NaviFra::AutoMoveMission::NAVIGATION_STATUS::RUNNING):
            st_status_.b_paused_by_obs_ = false;
            st_status_.b_paused_path_ = false;
            core_status_.b_obstacle_stop = false;
            st_status_.n_cur_navistatus_ = static_cast<int>(NaviFra::AutoMoveMission::NAVIGATION_STATUS::RUNNING);
            break;
        case static_cast<int>(NaviFra::AutoMoveMission::NAVIGATION_STATUS::PAUSE):
            st_status_.n_cur_navistatus_ = static_cast<int>(NaviFra::AutoMoveMission::NAVIGATION_STATUS::PAUSE);
            break;
        case static_cast<int>(NaviFra::AutoMoveMission::NAVIGATION_STATUS::OBS_STOP):
            st_status_.b_paused_by_obs_ = true;
            core_status_.b_obstacle_stop = true;
            st_status_.n_cur_navistatus_ = static_cast<int>(NaviFra::AutoMoveMission::NAVIGATION_STATUS::OBS_STOP);
            break;
        case static_cast<int>(NaviFra::AutoMoveMission::NAVIGATION_STATUS::ERROR):
            st_status_.b_paused_path_ = true;
            st_status_.n_cur_navistatus_ = static_cast<int>(NaviFra::AutoMoveMission::NAVIGATION_STATUS::ERROR);
            break;
        default:
            break;
    }
}

void ncNavigator::RecvNaviAlarm(const boost::any& any_var)
{
    core_msgs::NaviAlarm o_navi_alarm_msg;
    int n_alarm = boost::any_cast<int>(any_var);
    switch (n_alarm) {
        case core_msgs::NaviAlarm::GOAL_ARRIVED:
            LOG_INFO("RecvNaviAlarm GOAL_ARRIVED");
            tp_goal_time_ = std::chrono::steady_clock::now();
            o_navi_alarm_msg.alarm = core_msgs::NaviAlarm::GOAL_ARRIVED;
            o_navi_alarm_msg.alarm_text = GetAlarmText(o_navi_alarm_msg.alarm);
            CompleteMission();

            core_status_.s_current_node = core_status_.s_goal_node;
            core_status_.s_current_node_id = core_status_.s_goal_node_id;
            if (core_status_.s_goal_node_id == "") {
                core_status_.s_current_node_id = core_status_.s_goal_node;
            }
            break;
        case core_msgs::NaviAlarm::ERROR_PATH_NOT_CREATED:
            LOG_ERROR("RecvNaviAlarm ERROR_PATH_NOT_CREATED");
            tp_goal_time_ = std::chrono::steady_clock::now();
            o_navi_alarm_msg.alarm = core_msgs::NaviAlarm::ERROR_PATH_NOT_CREATED;
            o_navi_alarm_msg.alarm_text = GetAlarmText(o_navi_alarm_msg.alarm);
            SetNaviError(o_navi_alarm_msg.alarm);
            break;
    }
    navialarm_pub_.publish(o_navi_alarm_msg);
}

void ncNavigator::RecvLivePath(const move_msgs::CoreCommand::ConstPtr& msg)
{
    if (st_status_.b_running_) {
        b_addgoal_ = true;
    }
    else {
        b_addgoal_ = false;
    }
    tp_alarm_clear_time_ = std::chrono::steady_clock::now();
    st_status_.s_last_error_ = "";
    SetNaviError(0);
    LOG_INFO("RecvLivePath ");

    std::chrono::duration<double> sec = std::chrono::steady_clock::now() - tp_goal_time_;
    tp_goal_time_ = std::chrono::steady_clock::now();

    if (sec.count() < st_param_.f_goal_receive_time_gap_sec) {
        LOG_ERROR("RecvLivePath in 0.3 second! %.2f", sec.count());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    if (msg->list_waypoints.size() < 1) {
        LOG_ERROR("msg->list_waypoints.size() < 1");
        return;
    }
    else if (false == b_localizer_ready_) {
        LOG_ERROR("Localizer Map is not loaded ! ");
        return;
    }
    vector<NaviNode> vec_path;
    bool b_start_pause = msg->b_start_pause;

    if (msg->list_waypoints.size() == 1) {
        LOG_INFO("msg->list_waypoints.size() == 1 spin turn");
        NaviNode o_node;
        o_node.SetXm(msg->list_waypoints.at(0).f_x_m);
        o_node.SetYm(msg->list_waypoints.at(0).f_y_m);
        o_node.SetDeg(msg->list_waypoints.at(0).f_angle_deg);
        o_node.SetName(msg->list_waypoints.at(0).s_name);
        o_node.SetID(msg->list_waypoints.at(0).s_id);
        o_node.SetType(Pos::NODE_TYPE::SPIN_TURN);

        float f_robot_dist = CoreCalculator::CalcPosDistance_(o_robot_current_tf_pos_, o_node);
        if (f_robot_dist > 0.3 && false == msg->b_arrive_align) {
            NLOG(error) << "f_robot_dist > 0.3 && false == msg->b_arrive_align " << f_robot_dist << " / " << msg->b_arrive_align;
            return;
        }
        else
            vec_path.push_back(o_node);
        NLOG(info) << "spin turn start" << o_node.GetXm() << " " << o_node.GetYm() << " " << o_node.GetDeg();
    }
    else {
        for (int i = 0; i < msg->list_waypoints.size(); i++) {
            NaviNode o_node;
            o_node.SetXm(msg->list_waypoints.at(i).f_x_m);
            o_node.SetYm(msg->list_waypoints.at(i).f_y_m);
            o_node.SetDeg(msg->list_waypoints.at(i).f_angle_deg);
            o_node.SetName(msg->list_waypoints.at(i).s_name);
            o_node.SetID(msg->list_waypoints.at(i).s_id);

            NLOG(info) << i << " wp name : " << msg->list_waypoints.at(i).s_name << " id " << msg->list_waypoints.at(i).s_id
                       << " x : " << msg->list_waypoints.at(i).f_x_m << " y : " << msg->list_waypoints.at(i).f_y_m
                       << " angle : " << msg->list_waypoints.at(i).f_angle_deg << " speed : " << msg->list_waypoints.at(i).f_speed_ms
                       << " drive type : " << msg->list_waypoints.at(i).n_drive_type;

            Pos::DriveInfo_t o_drive_info;
            if (msg->list_waypoints.at(i).n_drive_type) {
                o_drive_info.e_drive_type = static_cast<Pos::DRIVE_TYPE>(msg->list_waypoints.at(i).n_drive_type);
                // QD Type이 아닐 경우 사행 주행은  일반 주행으로 수정
                if (st_param_.n_kinematics_type != 1 && st_param_.n_kinematics_type != 3 ||
                    false == st_param_.st_motion_param.b_diagonal_control) {
                    if (o_drive_info.e_drive_type == Pos::DRIVE_TYPE::FRONT_DIAGONAL)
                        o_drive_info.e_drive_type = Pos::DRIVE_TYPE::FRONT;
                    if (o_drive_info.e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL)
                        o_drive_info.e_drive_type = Pos::DRIVE_TYPE::REAR;
                }
            }

            o_drive_info.f_linear = msg->list_waypoints.at(i).f_speed_ms;
            o_drive_info.n_avoid_type = msg->list_waypoints.at(i).n_avoid_type;
            o_drive_info.f_avoid_speed = msg->list_waypoints.at(i).f_avoid_speed_ms;
            o_drive_info.f_curve_radius = msg->list_waypoints.at(i).f_curve_radius;
            o_drive_info.f_curvature = msg->list_waypoints.at(i).f_curvature;
            o_drive_info.f_heading_bias = msg->list_waypoints.at(i).f_diagonal_heading_bias;
            o_drive_info.b_start_smooth = !msg->list_waypoints.at(i).b_start_quick;
            o_drive_info.b_stop_smooth = !msg->list_waypoints.at(i).b_stop_quick;
            o_drive_info.b_diagonal_align_skip = msg->list_waypoints.at(i).b_diagonal_align_skip;
            o_drive_info.b_lccs = msg->list_waypoints.at(i).b_lccs;
            if (i == msg->list_waypoints.size() - 1 && msg->b_arrive_align) {
                o_node.SetType(Pos::NODE_TYPE::SPIN_TURN);
                if (o_drive_info.e_drive_type == Pos::DRIVE_TYPE::REAR) {
                    o_node.SetDeg(o_node.GetDeg() + 180);
                }
            }

            // // 사행 못하는 구간 체크
            // int n_diagonal = 0;  // 0 none, 1 front, 2 rear
            // if (o_drive_info.e_drive_type == Pos::DRIVE_TYPE::FRONT_DIAGONAL)  // 전진 사행
            //     n_diagonal = 1;

            // if (o_drive_info.e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL)  // 후진 사행
            //     n_diagonal = 2;

            // if (i != msg->list_waypoints.size() - 1 && n_diagonal > 0) {
            //     Pos o_next_node;
            //     o_next_node.SetXm(msg->list_waypoints.at(i + 1).f_x_m);
            //     o_next_node.SetYm(msg->list_waypoints.at(i + 1).f_y_m);
            //     o_next_node.SetDeg(msg->list_waypoints.at(i + 1).f_angle_deg);

            //     std::lock_guard<std::mutex> lock(mtx_lock_);
            //     Pos o_robot_pos = o_robot_current_tf_pos_;

            //     float f_to_deg = atan2(o_next_node.GetYm() - o_robot_pos.GetYm(), o_next_node.GetXm() - o_robot_pos.GetXm()) * RADtoDEG;
            //     LOG_WARNING("Diagonal expect  %.3f", f_to_deg);

            //     if (n_diagonal == 2)
            //         f_to_deg += 180;
            //     LOG_WARNING("Diagonal expect  %.3f", f_to_deg);

            //     float f_gap_deg = fabs(CoreCalculator::CalcAngleDomainDeg_(f_to_deg - o_next_node.GetDeg()));
            //     LOG_WARNING("Diagonal expect  %.3f", o_next_node.GetDeg());
            //     LOG_WARNING("Diagonal expect  %.3f", f_gap_deg);

            //     if (n_diagonal == 1 && f_gap_deg > 95) {
            //         LOG_WARNING("Diagonal expect o_drive_info.e_drive_type = front->rear");
            //         o_drive_info.e_drive_type = Pos::DRIVE_TYPE::REAR_DIAGONAL;
            //     }
            //     if (n_diagonal == 2 && f_gap_deg > 95) {
            //         LOG_WARNING("Diagonal expect o_drive_info.e_drive_type = rear->front");
            //         o_drive_info.e_drive_type = Pos::DRIVE_TYPE::FRONT_DIAGONAL;
            //     }
            // }

            if (msg->list_f_target_obstacle_margin.size() == 4) {
                o_drive_info.f_target_obstacle_margin_front = msg->list_f_target_obstacle_margin.at(0);
                o_drive_info.f_target_obstacle_margin_rear = msg->list_f_target_obstacle_margin.at(1);
                o_drive_info.f_target_obstacle_margin_left = msg->list_f_target_obstacle_margin.at(2);
                o_drive_info.f_target_obstacle_margin_right = msg->list_f_target_obstacle_margin.at(3);
            }
            else {
                NLOG(info) << "target obstacle not exist";
            }

            if (msg->list_waypoints.at(i).list_f_move_obstacle_margin.size() == 4) {
                o_drive_info.f_move_obstacle_margin_front = msg->list_waypoints.at(i).list_f_move_obstacle_margin.at(0);
                o_drive_info.f_move_obstacle_margin_rear = msg->list_waypoints.at(i).list_f_move_obstacle_margin.at(1);
                o_drive_info.f_move_obstacle_margin_left = msg->list_waypoints.at(i).list_f_move_obstacle_margin.at(2);
                o_drive_info.f_move_obstacle_margin_right = msg->list_waypoints.at(i).list_f_move_obstacle_margin.at(3);
            }
            else {
                NLOG(info) << "move obstacle not exist";
            }

            if (msg->list_waypoints.size() == msg->list_lidar_obs.size()) {
                if (msg->list_lidar_obs.at(i).list_f_obstacle_margin.size() == 4) {
                    o_drive_info.f_obstacle_outline_margin_front = msg->list_lidar_obs.at(i).list_f_obstacle_margin.at(0);
                    o_drive_info.f_obstacle_outline_margin_rear = msg->list_lidar_obs.at(i).list_f_obstacle_margin.at(1);
                    o_drive_info.f_obstacle_outline_margin_left = msg->list_lidar_obs.at(i).list_f_obstacle_margin.at(2);
                    o_drive_info.f_obstacle_outline_margin_right = msg->list_lidar_obs.at(i).list_f_obstacle_margin.at(3);
                }
                else {
                    NLOG(info) << "list_f_obstacle_margin not exist";
                }
            }
            else {
                NLOG(info) << "list_lidar_obs not exist";
            }

            if (msg->list_waypoints.size() == msg->list_camera_obs.size()) {
                //     if (msg->list_camera_obs.at(i).list_f_obstacle_margin.size() == 4) {
                //         o_drive_info.f_obstacle_outline_margin_front = msg->list_camera_obs.at(i).list_f_obstacle_margin.at(0);
                //         o_drive_info.f_obstacle_outline_margin_rear = msg->list_camera_obs.at(i).list_f_obstacle_margin.at(1);
                //         o_drive_info.f_obstacle_outline_margin_left = msg->list_camera_obs.at(i).list_f_obstacle_margin.at(2);
                //         o_drive_info.f_obstacle_outline_margin_right = msg->list_camera_obs.at(i).list_f_obstacle_margin.at(3);
                //     }
                //     else {
                //         NLOG(warnning) << "list_f_obstacle_margin not exist";
                //     }

                o_drive_info.d_camera_roi_x_m = msg->list_camera_obs.at(i).f_camera_roi_x_m;
                o_drive_info.d_camera_roi_y_m = msg->list_camera_obs.at(i).f_camera_roi_y_m;
                o_drive_info.d_camera_roi_z_m = msg->list_camera_obs.at(i).f_camera_roi_z_m;
                o_drive_info.d_camera_roi2_x_m = msg->list_camera_obs.at(i).f_camera_roi2_x_m;
                o_drive_info.d_camera_roi2_y_m = msg->list_camera_obs.at(i).f_camera_roi2_y_m;
                o_drive_info.d_camera_roi2_z_m = msg->list_camera_obs.at(i).f_camera_roi2_z_m;
                NLOG(info) << "camera roi " << o_drive_info.d_camera_roi_x_m << " " << o_drive_info.d_camera_roi_y_m << " "
                           << o_drive_info.d_camera_roi_z_m;
                NLOG(info) << "camera roi2 " << o_drive_info.d_camera_roi2_x_m << " " << o_drive_info.d_camera_roi2_y_m << " "
                           << o_drive_info.d_camera_roi2_z_m;
            }
            else {
                NLOG(info) << "list_lidar_obs not exist";
            }

            if (msg->list_waypoints.at(i).f_avoid_lanewidth > 0 && msg->list_waypoints.at(i).n_avoid_type == 1) {
                NLOG(info) << "msg->list_waypoints.at(i).f_avoid_lanewidth " << msg->list_waypoints.at(i).f_avoid_lanewidth;
                o_drive_info.n_avoidance_step = int(msg->list_waypoints.at(i).f_avoid_lanewidth / 0.2);
                o_drive_info.b_avoidance_right = true;
                o_drive_info.b_avoidance_left = true;
            }
            else if (msg->list_waypoints.at(i).f_avoid_lanewidth > 0 && msg->list_waypoints.at(i).n_avoid_type == 2) {
                NLOG(info) << "msg->list_waypoints.at(i).f_avoid_lanewidth " << msg->list_waypoints.at(i).f_avoid_lanewidth;
                o_drive_info.n_avoidance_step = int(msg->list_waypoints.at(i).f_avoid_lanewidth / 0.2);
                o_drive_info.b_avoidance_right = false;
                o_drive_info.b_avoidance_left = true;
            }
            else if (msg->list_waypoints.at(i).f_avoid_lanewidth > 0 && msg->list_waypoints.at(i).n_avoid_type == 3) {
                NLOG(info) << "msg->list_waypoints.at(i).f_avoid_lanewidth " << msg->list_waypoints.at(i).f_avoid_lanewidth;
                o_drive_info.n_avoidance_step = int(msg->list_waypoints.at(i).f_avoid_lanewidth / 0.2);
                o_drive_info.b_avoidance_right = true;
                o_drive_info.b_avoidance_left = false;
            }

            o_node.SetDriveInfo(o_drive_info);

            LOG_INFO(
                "RecvLivePath i : %d name : %s, type : %d, speed : %.3f, / curvespeed %.3f x : %.3f, y : %.3f, a : %.3f, lanewidth : %.3f, n_avoid %d, curve_radius %.3f, curvature %.3f margin frle %.1f %.1f %.1f %.1f, movemargin frle %.1f %.1f %.1f %.1f, targetmargin frle %.1f %.1f %.1f %.1f, offset x %.3f y %.3f deg %.3f align_skip %s, lccs : %d",
                i, o_node.GetName().c_str(), msg->list_waypoints.at(i).n_drive_type, msg->list_waypoints.at(i).f_speed_ms,
                msg->list_waypoints.at(i).f_speed_ms, o_node.GetXm(), o_node.GetYm(), o_node.GetDeg(),
                msg->list_waypoints.at(i).f_avoid_lanewidth, msg->list_waypoints.at(i).n_avoid_type, o_drive_info.f_curve_radius,
                o_drive_info.f_curvature, o_drive_info.f_obstacle_outline_margin_front, o_drive_info.f_obstacle_outline_margin_rear,
                o_drive_info.f_obstacle_outline_margin_left, o_drive_info.f_obstacle_outline_margin_right,
                o_drive_info.f_move_obstacle_margin_front, o_drive_info.f_move_obstacle_margin_rear,
                o_drive_info.f_move_obstacle_margin_left, o_drive_info.f_move_obstacle_margin_right,
                o_drive_info.f_target_obstacle_margin_front, o_drive_info.f_target_obstacle_margin_rear,
                o_drive_info.f_target_obstacle_margin_left, o_drive_info.f_target_obstacle_margin_right, msg->o_dock.f_global_offset_x,
                msg->o_dock.f_global_offset_y, msg->o_dock.f_global_offset_deg, o_drive_info.b_diagonal_align_skip ? "TRUE" : "FALSE",
                o_drive_info.b_lccs);

            if (i == msg->list_waypoints.size() - 1 && msg->o_dock.n_docking_mode) {
                // 마지막 노드에 offset 넣기
                o_node.SetXm(msg->list_waypoints.at(i).f_x_m + msg->o_dock.f_global_offset_x);
                o_node.SetYm(msg->list_waypoints.at(i).f_y_m + msg->o_dock.f_global_offset_y);
                o_node.SetDeg(msg->list_waypoints.at(i).f_angle_deg + msg->o_dock.f_global_offset_deg);
                LOG_INFO(
                    "RecvHacs Goal x %.3f y %.3f deg %.3f // boundary dist %.3f, deg %.3f", o_node.GetXm(), o_node.GetYm(), o_node.GetDeg(),
                    msg->f_arrive_boundary_dist, msg->f_arrive_boundary_deg);
            }

            vec_path.emplace_back(o_node);

            if (o_drive_info.f_curvature != 0 && i != msg->list_waypoints.size() - 1) {
                NaviNode o_virtual_node;
                o_virtual_node = o_node;
                if (fabs(o_drive_info.f_curvature) > fabs(hypot(
                                                         msg->list_waypoints.at(i + 1).f_x_m - msg->list_waypoints.at(i).f_x_m,
                                                         msg->list_waypoints.at(i + 1).f_y_m - msg->list_waypoints.at(i).f_y_m)) /
                        2.0 * 1.09) {
                    o_drive_info.f_curvature = fabs(
                                                   hypot(
                                                       msg->list_waypoints.at(i + 1).f_x_m - msg->list_waypoints.at(i).f_x_m,
                                                       msg->list_waypoints.at(i + 1).f_y_m - msg->list_waypoints.at(i).f_y_m) /
                                                   2.0 * 1.09) *
                        o_drive_info.f_curvature / fabs(o_drive_info.f_curvature);
                }
                float f_virtual_x = (msg->list_waypoints.at(i).f_x_m + msg->list_waypoints.at(i + 1).f_x_m) / 2 -
                    o_drive_info.f_curvature *
                        (cos(
                            M_PI_2 -
                            atan2(
                                msg->list_waypoints.at(i + 1).f_y_m - msg->list_waypoints.at(i).f_y_m,
                                msg->list_waypoints.at(i + 1).f_x_m - msg->list_waypoints.at(i).f_x_m)));
                float f_virtual_y = (msg->list_waypoints.at(i).f_y_m + msg->list_waypoints.at(i + 1).f_y_m) / 2 +
                    o_drive_info.f_curvature *
                        (sin(
                            M_PI_2 -
                            atan2(
                                msg->list_waypoints.at(i + 1).f_y_m - msg->list_waypoints.at(i).f_y_m,
                                msg->list_waypoints.at(i + 1).f_x_m - msg->list_waypoints.at(i).f_x_m)));

                o_virtual_node.SetXm(f_virtual_x);
                o_virtual_node.SetYm(f_virtual_y);
                o_virtual_node.SetDeg(
                    (atan2(
                        msg->list_waypoints.at(i + 1).f_y_m - msg->list_waypoints.at(i).f_y_m,
                        msg->list_waypoints.at(i + 1).f_x_m - msg->list_waypoints.at(i).f_x_m)) *
                    RADtoDEG);
                if (msg->list_waypoints.at(i).n_drive_type == 1001 || msg->list_waypoints.at(i).n_drive_type == 2001)  // 사행일경우
                    o_virtual_node.SetDeg(msg->list_waypoints.at(i).f_angle_deg);
                float f_dist = hypot(msg->list_waypoints.at(i + 1).f_x_m - msg->list_waypoints.at(i).f_x_m, msg->list_waypoints.at(i + 1).f_y_m - msg->list_waypoints.at(i).f_y_m);
                // o_drive_info.f_curve_radius = fabs(((f_dist / 2.0) * (f_dist / 2.0) + (o_drive_info.f_curvature * o_drive_info.f_curvature)) / (2.0 * o_drive_info.f_curvature));
                o_drive_info.f_curve_radius = hypot((f_dist*f_dist / (4*o_drive_info.f_curvature)), f_dist / 2);

                // o_drive_info.f_curve_radius = fabs(o_drive_info.f_curvature / 0.707f);
                o_drive_info.f_curvature = 0;
                o_virtual_node.SetDriveInfo(o_drive_info);
                NLOG(info) << "virtual radius : "<<o_drive_info.f_curve_radius;

                vec_path.emplace_back(o_virtual_node);
            }
        }
    }
    o_move_to_goal_msg_.SetGoalPos(vec_path.back());

    int n_state = -1;

    if (!pause_flags_.any()) {
        n_state = o_mission_manager_.StartMission(vec_path);

        if (n_state >= 0) {
            st_status_.b_running_ = true;
            core_msgs::NaviAlarm o_navi_alarm_msg;
            o_navi_alarm_msg.alarm = core_msgs::NaviAlarm::MOVE_START;
            o_navi_alarm_msg.alarm_text = GetAlarmText(o_navi_alarm_msg.alarm);
            navialarm_pub_.publish(o_navi_alarm_msg);
            ResumeMissionByState();
            if (b_start_pause)
                PauseMission(PauseReason::STATE);
        }
    }

    if (n_state >= 0) {
        std_msgs::String s_respone;
        s_respone.data = "StartTask/success";
        response_pub_.publish(s_respone);
    }
    else {
        std_msgs::String s_respone;
        s_respone.data = "StartTask/fail";
        response_pub_.publish(s_respone);
    }

    tp_goal_time_ = std::chrono::steady_clock::now();
}

void ncNavigator::RecvLog(const std_msgs::String::ConstPtr& msg)
{
    LOG_INFO("%s", msg->data.c_str());
}

void ncNavigator::RecvSpeedPercent(const std_msgs::Float64::ConstPtr& msg)
{
    // 0~100 까지 값
    float f_speed_percent = msg->data;
    core_status_.f_speed_percent = f_speed_percent;
    LOG_INFO("RecvSpeedPercent %.3f", f_speed_percent);
    if (f_speed_percent < 10) {
        LOG_WARNING("RecvSpeedPercent too low then 0.1m/s");
        f_speed_percent = 10;
    }
    o_mission_manager_.SetMsg(MissionManager::MSG_SPEED_PERCENT, f_speed_percent);
}

void ncNavigator::RecvTurnPercent(const std_msgs::Float64::ConstPtr& msg)
{
    float f_turn_percent = msg->data;
    LOG_INFO("RecvTurnPercent %.3f", f_turn_percent);
    if (f_turn_percent < 0) {
        LOG_ERROR("RecvTurnPercent negative, setting as zero!");
        f_turn_percent = 0.0;
    }
    o_mission_manager_.SetMsg(MissionManager::MSG_TURN_PERCENT, f_turn_percent);
}

void ncNavigator::RecvArriveBoundary(const std_msgs::Float64::ConstPtr& msg)
{
    float f_boundary = -msg->data;
    LOG_INFO("RecvArriveBoundary %.3f", f_boundary);
    if (fabs(f_boundary) > 0.3) {
        LOG_ERROR("RecvArriveBoundary negative, setting as zero!");
        f_boundary = 0.0;
    }
    else {
        o_mission_manager_.SetMsg(MissionManager::MSG_ARRIVE_BOUNDARY, f_boundary);
    }
}

void ncNavigator::RecvMaxSpeed(const std_msgs::Float64::ConstPtr& msg)
{
    float f_speed = msg->data;
    LOG_INFO("RecvMaxSpeed %.3f", f_speed);
    if (f_speed < 0.1) {
        LOG_WARNING("RecvMaxSpeed too low then 0.1m/s");
        f_speed = 0.1;
    }
    core_status_.f_speed_limit_ms = f_speed;
    o_mission_manager_.SetMsg(MissionManager::MSG_TWIST, f_speed);
}

void ncNavigator::RecvMotorInfo(const core_msgs::MotorInfo::ConstPtr& msg)
{
    if (msg->n_FL_error_code > 0) {
        st_status_.n_motor_error_code_ = msg->n_FL_error_code;
        st_status_.s_motor_error_text_ = msg->s_FL_error_text;
    }
    else if (msg->n_FR_error_code > 0) {
        st_status_.n_motor_error_code_ = msg->n_FR_error_code;
        st_status_.s_motor_error_text_ = msg->s_FR_error_text;
    }
    else if (msg->n_RL_error_code > 0) {
        st_status_.n_motor_error_code_ = msg->n_RL_error_code;
        st_status_.s_motor_error_text_ = msg->s_RL_error_text;
    }
    else if (msg->n_RR_error_code > 0) {
        st_status_.n_motor_error_code_ = msg->n_RR_error_code;
        st_status_.s_motor_error_text_ = msg->s_RR_error_text;
    }
    else if (msg->n_etc_error_code > 0) {
        st_status_.n_motor_error_code_ = msg->n_etc_error_code;
        st_status_.s_motor_error_text_ = msg->s_etc_error_text;
    }
    else {
        st_status_.n_motor_error_code_ = 0;
        st_status_.s_motor_error_text_ = "";
    }
    st_status_.f_steer_angle_deg_[0] = msg->f_FL_steer_motor_feedback_deg;
    st_status_.f_steer_angle_deg_[1] = msg->f_FR_steer_motor_feedback_deg;
    st_status_.f_steer_angle_deg_[2] = msg->f_RL_steer_motor_feedback_deg;
    st_status_.f_steer_angle_deg_[3] = msg->f_RR_steer_motor_feedback_deg;
    st_status_.tp_resent_motor_error_timestamp_ = std::chrono::steady_clock::now();
}

void ncNavigator::RecvLidarInfo(const core_msgs::LidarInfoMsg::ConstPtr& msg)
{
    if (msg->front_lidar_error_code > 0) {
        st_status_.n_lidar_error_code_ = msg->front_lidar_error_code;
        st_status_.s_lidar_error_text_ = msg->front_lidar_error_text;
    }
    else if (msg->rear_lidar_error_code > 0) {
        st_status_.n_lidar_error_code_ = msg->rear_lidar_error_code;
        st_status_.s_lidar_error_text_ = msg->rear_lidar_error_text;
    }
    else if (msg->left_lidar_error_code > 0) {
        st_status_.n_lidar_error_code_ = msg->left_lidar_error_code;
        st_status_.s_lidar_error_text_ = msg->left_lidar_error_text;
    }
    else if (msg->right_lidar_error_code > 0) {
        st_status_.n_lidar_error_code_ = msg->right_lidar_error_code;
        st_status_.s_lidar_error_text_ = msg->right_lidar_error_text;
    }
    else {
        st_status_.n_lidar_error_code_ = 0;
        st_status_.s_lidar_error_text_ = "";
    }
    st_status_.tp_resent_scan_timestamp_ = std::chrono::steady_clock::now();
}

void ncNavigator::RecvUpperInfo(const core_msgs::UpperInfo::ConstPtr& msg)
{
    if (msg->n_fork_lft_error_code > 0) {
        st_status_.n_upper_error_code_ = msg->n_fork_lft_error_code;
        st_status_.s_upper_error_text_ = msg->s_fork_lft_error_text;
    }
    else if (msg->n_fork_hrz_error_code > 0) {
        st_status_.n_upper_error_code_ = msg->n_fork_hrz_error_code;
        st_status_.s_upper_error_text_ = msg->s_fork_hrz_error_text;
    }
    else if (msg->n_turn_table_error_code > 0) {
        st_status_.n_upper_error_code_ = msg->n_turn_table_error_code;
        st_status_.s_upper_error_text_ = msg->s_turn_table_error_text;
    }
    else if (msg->n_lift_error_code > 0) {
        st_status_.n_upper_error_code_ = msg->n_lift_error_code;
        st_status_.s_upper_error_text_ = msg->s_lift_error_text;
    }
    else if (msg->n_left_aux_motor_error_code) {
        st_status_.n_upper_error_code_ = msg->n_left_aux_motor_error_code;
        st_status_.s_upper_error_text_ = msg->s_left_aux_motor_error_text;
    }
    else if (msg->n_right_aux_motor_error_code) {
        st_status_.n_upper_error_code_ = msg->n_right_aux_motor_error_code;
        st_status_.s_upper_error_text_ = msg->s_right_aux_motor_error_text;
    }
    else if (msg->n_etc_error_code) {
        st_status_.n_upper_error_code_ = msg->n_etc_error_code;
        st_status_.s_upper_error_text_ = msg->s_etc_error_text;
    }
    else {
        st_status_.n_upper_error_code_ = 0;
        st_status_.s_upper_error_text_ = "";
    }
    st_status_.tp_resent_upper_error_timestamp_ = std::chrono::steady_clock::now();
}

void ncNavigator::RecvResponseAvoidance(const std_msgs::Bool& msg)
{
    if (msg.data == true) {
        o_mission_manager_.SetAvoidPermission(true);
    }
    // else {
    //     o_mission_manager_.SetAvoidPermission(false);
    // }
}

void ncNavigator::RecvError(const std_msgs::Int64::ConstPtr& msg)
{
    try {
        if (msg->data >= 2000) {
            NLOG(error) << "RecvError : " << GetAlarmText(msg->data);
            SetNaviError(msg->data);
        }

        PauseMission(PauseReason::STATE);
    }
    catch (const std::exception& e) {
        // std::cerr << e.what() << '\n';
        NLOG(error) << "error " << e.what();
    }
}

void ncNavigator::RecvNowNode(const std_msgs::String::ConstPtr& msg)
{
    // core_status_.s_current_node = core_status_.s_goal_node;
    // core_status_.s_current_node = msg->data.substr(0, msg->data.find("|"));
    // core_status_.s_current_node_id = msg->data.substr(msg->data.find("|") + 1);
}

void ncNavigator::RecvWarning(const std_msgs::String::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(warningMutex);
    auto now = std::chrono::steady_clock::now();
    for (auto& w : vec_warnning_) {
        if (w.first == msg->data) {
            w.second = now;  // 기존 경고 시간 업데이트
            return;
        }
    }
    vec_warnning_.emplace_back(msg->data, now);
}

void ncNavigator::RecvLocalizeInfo(const core_msgs::LocalizeInfo::ConstPtr& msg)
{
    b_localizer_ready_ = msg->b_localizer_ready;

    static int n_confidence_error_cnt = 0;
    static int n_prev_confidence = 0;

    core_status_.n_map_rate = msg->n_confidence;
    core_status_.n_confidence = msg->n_confidence;

    // confidence가 이전이랑 다르면 추가 static 변
    // 강제로 낮은 confidence줘서 테스트 해보기

    if (msg->n_confidence < st_param_.n_icp_error_ratio_threshold && st_status_.b_running_ && n_prev_confidence != msg->n_confidence) {
        n_confidence_error_cnt++;
        n_prev_confidence = msg->n_confidence;

        LOG_ERROR("ICP ERROR %d, cnt %d", msg->n_confidence, n_confidence_error_cnt);

        if (n_confidence_error_cnt >= st_param_.n_icp_error_count_threshold) {
            if (st_status_.b_running_) {
                st_status_.s_last_error_ = "ERROR_CONFIDENCE_LOW";
                LOG_ERROR("ICP ERROR Detected. PAUSE Mission ... ");
                PauseMission(PauseReason::STATE);
            }
            core_msgs::NaviAlarm o_navi_alarm_msg;
            o_navi_alarm_msg.alarm = core_msgs::NaviAlarm::ERROR_CONFIDENCE_LOW;
            o_navi_alarm_msg.alarm_text = GetAlarmText(o_navi_alarm_msg.alarm);
            navialarm_pub_.publish(o_navi_alarm_msg);
            n_confidence_error_cnt = 0;
        }
    }
    else {
        if (st_status_.s_last_error_ == "ERROR_CONFIDENCE_LOW" && msg->n_confidence >= st_param_.n_icp_error_ratio_threshold) {
            LOG_ERROR("ICP RESET Detected. ERROR_CONFIDENCE_LOW ALARM CLEAR ... ");
            st_status_.s_last_error_ = "";
            n_confidence_error_cnt = 0;
        }
    }

    static int n_error_cnt = 0;
    auto result_error_code = msg->localize_result_error_code;

    static std::chrono::steady_clock::time_point tp_not_load = std::chrono::steady_clock::now();
    std::chrono::duration<double> sec_not_load = std::chrono::steady_clock::now() - tp_not_load;
    if (result_error_code == 0) {
        tp_not_load = std::chrono::steady_clock::now();
    }
    else {
        if (result_error_code == core_msgs::NaviAlarm::ERROR_MAP_NOT_LOADED && sec_not_load.count() > 5) {
            SetNaviError(result_error_code);
        }
    }

    if (st_status_.b_running_) {
        if (result_error_code != core_msgs::NaviAlarm::LOCALIZATION_LARGE_CORRECTION &&
            result_error_code != core_msgs::NaviAlarm::ERROR_ABNORMAL_IMU) {
            return;
        }

        core_msgs::NaviAlarm o_navi_alarm_msg;
        o_navi_alarm_msg.alarm = result_error_code;
        o_navi_alarm_msg.alarm_text = GetAlarmText(o_navi_alarm_msg.alarm);
        st_status_.s_last_error_ = o_navi_alarm_msg.alarm_text;
        LOG_ERROR("LOCALIZER ERROR DETECTED");
        PauseMission(PauseReason::STATE);
        navialarm_pub_.publish(o_navi_alarm_msg);
        n_error_cnt = 0;
    }
    else {
        n_error_cnt = 0;
    }
}

void ncNavigator::RecvEtcInfo(const core_msgs::EtcInfo::ConstPtr& msg)
{
    if (msg->b_mileage_total_update)
        core_status_.f_mileage_local = msg->f_mileage_total;
    if (msg->b_mileage_local_update)
        core_status_.f_mileage_total = msg->f_mileage_local;
}

void ncNavigator::RecvCmd(const std_msgs::String::ConstPtr& msg)
{
    constexpr char CANCEL[] = "cancel";
    constexpr char PAUSE[] = "pause";
    constexpr char PAUSE_ACS[] = "apause";
    constexpr char PAUSE_V2V[] = "vpause";
    constexpr char PAUSE_USER[] = "pause_user";
    constexpr char RESUME[] = "resume";
    constexpr char RESUME_STATE[] = "resume_state";
    constexpr char ALARM_CLEAR[] = "alarm_clear";

    LOG_WARNING("RecvCmd %s", msg->data.c_str());
    if (msg->data == CANCEL) {
        // st_status_.s_last_error_ = "";
        SetNaviError(0);
        AbortMission();
        ResumeMissionByUser();
        if(st_param_.b_live_topology_mode) {
            core_status_.s_current_node = "0000";
            core_status_.s_current_node_id = "0000";
            core_status_.s_next_node = "0000";
            core_status_.s_next_node_id = "0000";
            core_status_.s_goal_node = "0000";
            core_status_.s_goal_node_id = "0000";
        }

        if (b_obs_check_) {
            std_msgs::Bool msg;
            msg.data = false;
            emergency_pub_.publish(msg);
        }

        std_msgs::String s_respone;
        s_respone.data = "CancelTask/success";
        response_pub_.publish(s_respone);
    }
    else if (msg->data == PAUSE) {
        PauseMission(PauseReason::STATE);
        std_msgs::String s_respone;
        s_respone.data = "PauseTask/success";
        response_pub_.publish(s_respone);
    }
    else if (msg->data == PAUSE_ACS) {
        PauseMission(PauseReason::ACS);
        std_msgs::String s_respone;
        s_respone.data = "PauseTask/success";
        response_pub_.publish(s_respone);
    }
    else if (msg->data == PAUSE_V2V) {
        PauseMission(PauseReason::V2V);
        std_msgs::String s_respone;
        s_respone.data = "PauseTask/success";
        response_pub_.publish(s_respone);
    }
    else if (msg->data == PAUSE_USER) {
        PauseMission(PauseReason::USER);
        std_msgs::String s_respone;
        s_respone.data = "PauseTask/success";
        response_pub_.publish(s_respone);
    }
    else if (msg->data == RESUME) {
        ResumeMissionByUser();
        std_msgs::String s_respone;
        s_respone.data = "ResumeTask/success";
        response_pub_.publish(s_respone);
    }
    else if (msg->data == RESUME_STATE) {
        ResumeMissionByState();

        if (b_obs_check_) {
            std_msgs::Bool msg;
            msg.data = false;
            emergency_pub_.publish(msg);
        }

        std_msgs::String s_respone;
        s_respone.data = "ResumeTask/success";
        response_pub_.publish(s_respone);
    }
    else if (msg->data == ALARM_CLEAR) {
        tp_alarm_clear_time_ = std::chrono::steady_clock::now();
        // st_status_.s_last_error_ = "";
        SetNaviError(0);
        std_msgs::String s_respone;
        s_respone.data = "AlarmClear/success";
        response_pub_.publish(s_respone);
    }
}

void ncNavigator::RecvRobotoPos(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    NaviFra::Pos o_robot_tf_pos = PosConvert(msg->pose.pose);
    {
        const std::lock_guard<std::mutex> lock(mtx_lock_);
        o_robot_current_tf_pos_ = o_robot_tf_pos;
    }
    o_mission_manager_.SetRobotPos(o_robot_tf_pos);
    {
        std::lock_guard<std::mutex> lock(mtx_lock_status_);
        core_status_.f_robot_pos_x_m = o_robot_tf_pos.GetXm();
        core_status_.f_robot_pos_y_m = o_robot_tf_pos.GetYm();
        core_status_.f_robot_pos_deg = o_robot_tf_pos.GetDeg();
    }

    if (!st_status_.b_pos_is_in_valid_)
        LOG_INFO("now pos timing is available");

    st_status_.tp_resent_pos_timestamp_ = std::chrono::steady_clock::now();
    st_status_.b_pos_is_in_valid_ = true;
}

void ncNavigator::AbortMission()
{
    std::chrono::duration<double> sec = std::chrono::steady_clock::now() - tp_goal_time_;
    tp_goal_time_ = std::chrono::steady_clock::now();
    if (sec.count() < st_param_.f_goal_receive_time_gap_sec) {
        LOG_ERROR("Abort in 0.3 second! ");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    LOG_INFO("Abort Behavior() is called");
    PublishCmdVelZero();
    o_mission_manager_.TerminateMission();
    st_status_.completed_type_ = Status_t::LAST_COMPELETED_TYPE::ABORTED;
    PublishCmdVelZero();
    st_status_.b_running_ = false;
    b_pallet_detect_check_ = false;
    pause_flags_.reset();

    core_msgs::NaviAlarm o_navi_alarm_msg;
    o_navi_alarm_msg.alarm = core_msgs::NaviAlarm::CANCELED;
    o_navi_alarm_msg.alarm_text = GetAlarmText(o_navi_alarm_msg.alarm);
    navialarm_pub_.publish(o_navi_alarm_msg);
}

void ncNavigator::CompleteMission()
{
    LOG_INFO("CompleteMission called");

    st_status_.completed_type_ = Status_t::LAST_COMPELETED_TYPE::COMPLETED;
    PublishCmdVelZero();
    st_status_.b_running_ = false;
    b_pallet_detect_check_ = false;
    pause_flags_.reset();
}

void ncNavigator::PauseMission(int data)
{
    if (st_status_.b_running_) {
        pause_flags_.set(data, true);
        o_mission_manager_.SuspendMission();
        PublishCmdVelZero();

        core_msgs::NaviAlarm o_navi_alarm_msg;
        o_navi_alarm_msg.alarm = core_msgs::NaviAlarm::MOVE_PAUSED;
        o_navi_alarm_msg.alarm_text = GetAlarmText(o_navi_alarm_msg.alarm);
        navialarm_pub_.publish(o_navi_alarm_msg);
    }
}

void ncNavigator::ResumeMissionByState()
{
    LOG_INFO("ResumeMissionByState called");
    tp_alarm_clear_time_ = std::chrono::steady_clock::now();
    st_status_.s_last_error_ = "";
    pause_flags_.set(PauseReason::STATE, false);
    if (!pause_flags_.any()) {
        LOG_INFO("ResumeMissionByState ResumeMission called");
        o_mission_manager_.ResumeMission();

        core_msgs::NaviAlarm o_navi_alarm_msg;
        o_navi_alarm_msg.alarm = core_msgs::NaviAlarm::MOVE_RESUME;
        o_navi_alarm_msg.alarm_text = GetAlarmText(o_navi_alarm_msg.alarm);
        navialarm_pub_.publish(o_navi_alarm_msg);
    }
}

void ncNavigator::ResumeMissionByUser()
{
    pause_flags_.reset();
    o_mission_manager_.ResumeMission();

    core_msgs::NaviAlarm o_navi_alarm_msg;
    o_navi_alarm_msg.alarm = core_msgs::NaviAlarm::MOVE_RESUME;
    o_navi_alarm_msg.alarm_text = GetAlarmText(o_navi_alarm_msg.alarm);
    navialarm_pub_.publish(o_navi_alarm_msg);
}

bool ncNavigator::IsLidarValidation()
{
    std::chrono::duration<double> last_elapsed_time_sec = std::chrono::steady_clock::now() - st_status_.tp_resent_scan_timestamp_;
    float f_last_elapsed_time_sec = last_elapsed_time_sec.count();

    // st_status_.b_scan_is_in_valid_ = (f_last_elapsed_time_sec < st_status_.f_max_timeout_sec_);
    // bool result = st_status_.b_scan_is_in_valid_;
    bool result = true;

    // if (!result) {
    //     LOG_ERROR("lidar info not recived !");
    //     st_status_.n_lidar_error_code_ = core_msgs::NaviAlarm::ERROR_LIDAR_NOTCONNECTED;
    //     st_status_.s_lidar_error_text_ = "ERROR_LIDAR_NOTCONNECTED";
    // }
    if (st_status_.s_lidar_error_text_.size() > 0)
        result = false;
    if (!result) {
        LOG_WARNING("lidar error ! : %s", st_status_.s_lidar_error_text_.c_str());
    }

    return result;
}

bool ncNavigator::IsMotorValidation()
{
    if (st_param_.b_control_motor_directly == false)
        return true;

    std::chrono::duration<double> last_elapsed_time_sec = std::chrono::steady_clock::now() - st_status_.tp_resent_motor_error_timestamp_;

    float f_last_elapsed_time_sec = last_elapsed_time_sec.count();
    bool result = (f_last_elapsed_time_sec < st_status_.f_max_timeout_sec_);  // 타임아웃 넘어가면

    if (!result) {
        LOG_ERROR("motor info not recived !");
        st_status_.n_motor_error_code_ = core_msgs::NaviAlarm::ERROR_MOTOR_NOTCONNECTED;
        st_status_.s_motor_error_text_ = "ERROR_MOTOR_NOTCONNECTED";
    }
    if (st_status_.s_motor_error_text_.size() > 0) {
        result = false;
    }

    if (!result) {
        LOG_WARNING("motor error ! : %s", st_status_.s_motor_error_text_.c_str());
    }
    return result;
}

bool ncNavigator::IsUpperValidation()
{
    bool result = true;

    if (st_param_.b_upper_use) {
        std::chrono::duration<double> last_elapsed_time_sec =
            std::chrono::steady_clock::now() - st_status_.tp_resent_upper_error_timestamp_;
        float f_last_elapsed_time_sec = last_elapsed_time_sec.count();

        result = (f_last_elapsed_time_sec < st_status_.f_max_timeout_sec_);

        if (!result) {
            LOG_ERROR("upper info not recived !");
            st_status_.n_motor_error_code_ = core_msgs::NaviAlarm::ERROR_UPPER_NOTCONNECTED;
        }
        if (st_status_.s_upper_error_text_.size() > 0)
            result = false;
        if (!result) {
            LOG_ERROR("upper error ! : %s", st_status_.s_upper_error_text_.c_str());
        }
    }
    return result;
}

bool ncNavigator::IsPosValidation()
{
    std::chrono::duration<double> last_elapsed_time_sec = std::chrono::steady_clock::now() - st_status_.tp_resent_pos_timestamp_;
    float f_last_elapsed_time_sec = last_elapsed_time_sec.count();
    bool result = f_last_elapsed_time_sec < 10;

    if (!result) {
        LOG_ERROR("IsPosValidation::pos too late ! : %.3f sec", f_last_elapsed_time_sec);
    }
    return result;
}

void ncNavigator::NavigationStatusThread()
{
    while (ros::ok()) {
        UpdateNavigation();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void ncNavigator::UpdateNavigation()
{
    // new 2022.09.16 status 정리 [sally]
    // /navifra/info의 n_status 정의
    // error 상황, 로봇이 멈춤
    constexpr char NOTONPATH[] = "NOT_ON_PATH";  // 로봇이 UI상 노드, 링크 위에 있지 않은 상태
    constexpr char ERROR_ICP[] = "ERROR_LOCALIZATION_MATCHING";  // nc_localization 매칭률이 일정이하로 떨어진 상태
    constexpr char EMERGENCY[] = "emergency";  // nc_localization 매칭률이 일정이하로 떨어진 상태
    constexpr char ERROR_NODE_OUT[] = "nodeout";

    // 기본 status
    constexpr char LOADING[] = "loading";  // 네비게이션 초기 로딩 상태
    constexpr char MAPNOTLOAD[] = "mapnotload";  // 네비게이션 초기 로딩 상태
    constexpr char IDLE[] = "idle";  // 유휴상태
    constexpr char RUNNING[] = "running";  // 정상 주행 상태
    constexpr char ACTING[] = "action_";  // action tasking 상태
    // constexpr char AVOIDANCE_RUNNING[] = "avoidance_running"; // 회피 경로 만들고 주행 상태

    // pause status 상세화
    constexpr char PAUSED_BY_USER[] = "paused_by_user";  // 사용자에 의해 멈춘 상태
    constexpr char PAUSED_BY_V2V[] = "paused_by_v2v";  // 사용자에 의해 멈춘 상태
    constexpr char PAUSED_BY_ACS[] = "paused_by_acs";  // 사용자에 의해 멈춘 상태
    constexpr char PAUSED[] = "paused";  // ??
    constexpr char PAUSED_BY_OBS_CAMERA[] = "paused_by_obs_camera";  // 카메라 장애물에 의해 멈춘 상태
    constexpr char PAUSED_BY_OBS_V2V[] = "paused_by_obs_v2v";  // V2V 장애물에 의해 멈춘 상태
    constexpr char PAUSED_BY_OBS_LIDAR[] = "paused_by_obs_lidar";  // 라이다 장애물에 의해 멈춘 상태
    constexpr char PAUSED_BY_OBS[] = "paused_by_obs";  // 장애물에 의해 멈춘 상태
    constexpr char PAUSED_BY_PATH[] = "paused_by_path";  // path 에서 멀어져서 멈춘 상태 (관련 파라미터 : planning_dist_robot_to_node_m)

    std_msgs::String tmpmsg;
    std::chrono::duration<double> sec_start_up_ = std::chrono::steady_clock::now() - tp_start_up_time_;
    static core_msgs::NavicoreStatus ex_core_status;

    static bool b_update_ = false;
    if (false == b_update_ && sec_start_up_.count() > 1) {
        b_update_ = true;
        std_msgs::String s_msg_p;
        str_param_pub_.publish(s_msg_p);
    }

    if (sec_start_up_.count() < 12) {
        tmpmsg.data = LOADING;
        core_status_.n_status = core_msgs::NavicoreStatus::LOADING;
    }
    else if (false == IsMotorValidation()) {
        tmpmsg.data = st_status_.s_motor_error_text_;
        core_status_.n_status = core_msgs::NavicoreStatus::LOADING;

        std::chrono::duration<double> sec_alarm = std::chrono::steady_clock::now() - tp_alarm_clear_time_;
        if (st_status_.b_running_ && sec_alarm.count() > 2) {
            st_status_.s_last_error_ = tmpmsg.data;
            LOG_ERROR("Motor Error Detected. PAUSE Mission ... ");
            PauseMission(PauseReason::STATE);
        }
        core_msgs::NaviAlarm o_navi_alarm_msg;
        o_navi_alarm_msg.alarm = st_status_.n_motor_error_code_;
        o_navi_alarm_msg.alarm_text = GetAlarmText(o_navi_alarm_msg.alarm);
        navialarm_pub_.publish(o_navi_alarm_msg);

        if (st_status_.b_running_ &&
            (st_status_.n_motor_error_code_ != 3516 &&
             st_status_.n_motor_error_code_ != 3816))  // 일시정지 상태 확인 user/state machine/obstacle/path
            SetNaviError(st_status_.n_motor_error_code_);
    }
    // else if(true == IsMotorValidation() && st_status_.b_paused_by_status_ == true) {
    //     int n_navi_error = GetNaviError();
    //     if (n_navi_error == 0) {
    //         NLOG(info)<<"Release Motor Safety Stop!!";
    //         ResumeMissionByState();
    //     }
    // }
    else if (false == IsLidarValidation())  // scan 값 확인
    {
        tmpmsg.data = st_status_.s_lidar_error_text_;
        core_status_.n_status = core_msgs::NavicoreStatus::LOADING;

        std::chrono::duration<double> sec_alarm = std::chrono::steady_clock::now() - tp_alarm_clear_time_;
        if (st_status_.b_running_ && sec_alarm.count() > 2) {
            st_status_.s_last_error_ = tmpmsg.data;
            LOG_ERROR("Scan InValidation Detected. PAUSE Mission ... ");
            PauseMission(PauseReason::STATE);
        }
        core_msgs::NaviAlarm o_navi_alarm_msg;
        o_navi_alarm_msg.alarm = st_status_.n_lidar_error_code_;
        o_navi_alarm_msg.alarm_text = GetAlarmText(o_navi_alarm_msg.alarm);
        navialarm_pub_.publish(o_navi_alarm_msg);
    }
    else if (false == IsUpperValidation()) {
        tmpmsg.data = st_status_.s_upper_error_text_;
        core_status_.n_status = core_msgs::NavicoreStatus::LOADING;

        std::chrono::duration<double> sec_alarm = std::chrono::steady_clock::now() - tp_alarm_clear_time_;
        if (st_status_.b_running_ && sec_alarm.count() > 2) {
            st_status_.s_last_error_ = tmpmsg.data;
            LOG_ERROR("UpperValidation Detected. PAUSE Mission ... ");
            PauseMission(PauseReason::STATE);
        }
        core_msgs::NaviAlarm o_navi_alarm_msg;
        o_navi_alarm_msg.alarm = st_status_.n_upper_error_code_;
        o_navi_alarm_msg.alarm_text = GetAlarmText(o_navi_alarm_msg.alarm);
        navialarm_pub_.publish(o_navi_alarm_msg);
    }
    else if (false == IsPosValidation())  // pos 값 확인
    {
        tmpmsg.data = "ERROR_POS";
        core_status_.n_status = core_msgs::NavicoreStatus::LOADING;

        std::chrono::duration<double> sec_alarm = std::chrono::steady_clock::now() - tp_alarm_clear_time_;
        if (st_status_.b_running_ && sec_alarm.count() > 2) {
            st_status_.s_last_error_ = tmpmsg.data;
            LOG_ERROR("Pos InValidation Detected. PAUSE Mission ... ");
            PauseMission(PauseReason::STATE);
        }
        core_msgs::NaviAlarm o_navi_alarm_msg;
        o_navi_alarm_msg.alarm = core_msgs::NaviAlarm::ERROR_LOCALIZATION_NOT_START;
        o_navi_alarm_msg.alarm_text = GetAlarmText(o_navi_alarm_msg.alarm);
        navialarm_pub_.publish(o_navi_alarm_msg);
    }
    else if (b_localizer_ready_ == false) {
        tmpmsg.data = MAPNOTLOAD;
        core_status_.n_status = core_msgs::NavicoreStatus::LOADING;
    }
    else if (st_status_.b_running_)  // 일시정지 상태 확인 user/state machine/obstacle/path
    {
        if (pause_flags_.test(PauseReason::USER))  // pause 1 : user
        {
            tmpmsg.data = PAUSED_BY_USER;
            core_status_.n_status = core_msgs::NavicoreStatus::PAUSED_BY_USER;
        }
        else if (pause_flags_.test(PauseReason::STATE))  // pause 2 : state machine
        {
            tmpmsg.data = PAUSED;
            core_status_.n_status = core_msgs::NavicoreStatus::PAUSED_BY_STATUS;
            int n_navi_error = GetNaviError();
            if (n_navi_error != 0) {
                st_status_.s_last_error_ = GetAlarmText(n_navi_error);
                tmpmsg.data = st_status_.s_last_error_;
            }
        }
        else if (pause_flags_.test(PauseReason::ACS))  // pause 2 : state machine
        {
            tmpmsg.data = PAUSED_BY_USER;
            core_status_.n_status = core_msgs::NavicoreStatus::PAUSED_BY_USER;
            if (st_status_.s_last_error_.size() > 0) {
                tmpmsg.data = st_status_.s_last_error_;
            }
        }
        else if (pause_flags_.test(PauseReason::V2V))  // pause 3 : v2v
        {
            tmpmsg.data = PAUSED_BY_V2V;
            core_status_.n_status = core_msgs::NavicoreStatus::PAUSED_BY_USER;
            if (st_status_.s_last_error_.size() > 0) {
                tmpmsg.data = st_status_.s_last_error_;
            }
        }
        else if (st_status_.st_mission_info_.n_stop_because_of_obstacle != 0) {
            if (st_status_.st_mission_info_.n_stop_because_of_obstacle == 1 && st_param_.b_use_obs_alarm_separate)  // pause 3 : obstacle
            {
                tmpmsg.data = PAUSED_BY_OBS_LIDAR;
                core_status_.n_status = core_msgs::NavicoreStatus::PAUSED_BY_OBS;
                // std_msgs::String str_msg;
                // msg.data = "quick_stop_on";
                // plc_cmd_pub_.publish(msg);
            }
            else if (st_status_.st_mission_info_.n_stop_because_of_obstacle == 2 && st_param_.b_use_obs_alarm_separate)  // pause 3 :
            {
                tmpmsg.data = PAUSED_BY_OBS_CAMERA;
                core_status_.n_status = core_msgs::NavicoreStatus::PAUSED_BY_OBS;
            }
            else if (st_status_.st_mission_info_.n_stop_because_of_obstacle == 3 && st_param_.b_use_obs_alarm_separate)  // pause 3 :
            {
                tmpmsg.data = PAUSED_BY_OBS_V2V;
                core_status_.n_status = core_msgs::NavicoreStatus::PAUSED_BY_OBS;
            }
            else {
                tmpmsg.data = PAUSED_BY_OBS;
                core_status_.n_status = core_msgs::NavicoreStatus::PAUSED_BY_OBS;
            }
        }
        else if (st_status_.b_paused_path_)  // pause 4 : path far from robot
        {
            tmpmsg.data = PAUSED_BY_PATH;
            if (st_status_.b_running_) {
                st_status_.s_last_error_ = tmpmsg.data;
                LOG_ERROR("Paused_by_path Detected. PAUSE Mission ... ");
                PauseMission(PauseReason::STATE);
            }
        }
        else  // pause가 아니라면 running
        {
            tmpmsg.data = RUNNING;
            core_status_.n_status = core_msgs::NavicoreStatus::RUNNING;
        }
    }
    // n_ex_status로 aborted/completed 판단 후 Ready
    else {
        tmpmsg.data = IDLE;
        int n_navi_error = GetNaviError();
        if (n_navi_error != 0) {
            core_msgs::NaviAlarm o_navi_alarm_msg;
            o_navi_alarm_msg.alarm = n_navi_error;
            o_navi_alarm_msg.alarm_text = GetAlarmText(o_navi_alarm_msg.alarm);
            st_status_.s_last_error_ = o_navi_alarm_msg.alarm_text;
            tmpmsg.data = st_status_.s_last_error_;
            navialarm_pub_.publish(o_navi_alarm_msg);
        }
        core_status_.n_status = core_msgs::NavicoreStatus::READY;
        if (st_status_.completed_type_ == Status_t::LAST_COMPELETED_TYPE::COMPLETED) {
            core_status_.n_ex_status = core_msgs::NavicoreStatus::COMPLETED;
        }
        else if (st_status_.completed_type_ == Status_t::LAST_COMPELETED_TYPE::ABORTED) {
            core_status_.n_ex_status = core_msgs::NavicoreStatus::ABORTED;
        }
        else {
            core_status_.n_ex_status = core_msgs::NavicoreStatus::READY;
        }
    }

    if (tmpmsg.data == IDLE && s_current_task_name_.size() > 0) {
        // 로봇이 idle 인데 uuid 존재한다면
        tmpmsg.data = ACTING + s_current_task_name_;
        core_status_.n_status = core_msgs::NavicoreStatus::TASKING;
    }

    core_status_.f_goal_pos_x_m = o_move_to_goal_msg_.GetGoalPos().GetXm();
    core_status_.f_goal_pos_y_m = o_move_to_goal_msg_.GetGoalPos().GetYm();
    core_status_.f_goal_pos_deg = o_move_to_goal_msg_.GetGoalPos().GetDeg();
    if (o_move_to_goal_msg_.GetGoalPos().GetType() == Pos::NODE_TYPE::NONE && core_status_.n_diagonal_direction == 0) {
        core_status_.f_goal_pos_deg = st_status_.st_mission_info_.f_path_angle_deg;
    }
    if (o_move_to_goal_msg_.GetGoalPos().GetDriveInfo().e_drive_type == 2000 && core_status_.n_diagonal_direction == 0) {
        Pos o_goal_pos;
        o_goal_pos.SetDeg(core_status_.f_goal_pos_deg + 180);
        core_status_.f_goal_pos_deg = o_goal_pos.GetDeg();
    }

    if (tmpmsg.data == IDLE && st_param_.b_live_topology_mode && fabs(core_status_.f_robot_pos_x_m - core_status_.f_goal_pos_x_m) < 0.1 &&
        fabs(core_status_.f_robot_pos_y_m - core_status_.f_goal_pos_y_m) < 0.1) {
        core_status_.s_current_node = core_status_.s_goal_node;
    }

    // find core_status_.f_path_progress

    if (core_status_.s_current_node == core_status_.s_next_node) {
        core_status_.f_path_progress = 0;
    }

    // LOG_INFO("core status %s , tmpmsg %s",core_status_.s_status.c_str(),tmpmsg.data.c_str());
    if (core_status_.s_status != tmpmsg.data) {
        core_msgs::NaviAlarm o_navi_alarm_msg;
        if (tmpmsg.data == ERROR_NODE_OUT)
            o_navi_alarm_msg.alarm = core_msgs::NaviAlarm::ERROR_ARRIVED_POS_OUT;
        else if (tmpmsg.data == MAPNOTLOAD)
            o_navi_alarm_msg.alarm = core_msgs::NaviAlarm::ERROR_LOCALIZATION_NOT_START;
        if (o_navi_alarm_msg.alarm != 0) {
            o_navi_alarm_msg.alarm_text = GetAlarmText(o_navi_alarm_msg.alarm);
            navialarm_pub_.publish(o_navi_alarm_msg);
        }
    }
    bool b_loaded = false;
    {
        std::lock_guard<std::mutex> lock(mtx_load_);
        b_loaded = b_loaded_;
    }
    if (tmpmsg.data == IDLE) {
        NaviFra::Polygon o_polygon_outline;
        if (st_param_.map_polygon_robot_collision_.find("outline") != st_param_.map_polygon_robot_collision_.end()) {
            o_polygon_outline = st_param_.map_polygon_robot_collision_.at("outline");
        }
        NaviFra::Polygon o_polygon_collision;
        if (!b_loaded && n_fork_position_ == 1) {
            if (st_param_.map_polygon_robot_collision_.find("collision") != st_param_.map_polygon_robot_collision_.end()) {
                o_polygon_collision = st_param_.map_polygon_robot_collision_["collision"];
            }
        }
        else if (!b_loaded && n_fork_position_ == 0) {
            if (st_param_.map_polygon_robot_collision_.find("collision2") != st_param_.map_polygon_robot_collision_.end()) {
                o_polygon_collision = st_param_.map_polygon_robot_collision_["collision2"];
            }
        }
        else {
            if (st_param_.map_polygon_robot_collision_.find("collision3") != st_param_.map_polygon_robot_collision_.end()) {
                o_polygon_collision = st_param_.map_polygon_robot_collision_["collision3"];
            }
        }
        static ros::Publisher poly_outline_pub_ =
            node_handle_.advertise<geometry_msgs::PolygonStamped>("/NaviFra/visualize/robot_outline", 5, true);
        geometry_msgs::PolygonStamped msg_outline = DrawPolygon(o_polygon_outline.o_polygon_vertexs_);
        poly_outline_pub_.publish(msg_outline);

        if (st_param_.b_use_detection_mode_flag) {
            static ros::Publisher poly_collision_pub_ =
                node_handle_.advertise<geometry_msgs::PolygonStamped>("/NaviFra/visualize/robot_collision", 5, true);
            geometry_msgs::PolygonStamped msg_collision = DrawPolygon(o_polygon_collision.o_polygon_vertexs_);
            poly_collision_pub_.publish(msg_collision);
        }
    }
    static std::chrono::steady_clock::time_point tp_running_time_ = std::chrono::steady_clock::now();
    if ((tmpmsg.data == RUNNING && core_status_.s_status != RUNNING) || fabs(core_status_.f_robot_target_linear_vel_x) > 0.001 ||
        fabs(core_status_.f_robot_target_linear_vel_y) > 0.001 || fabs(core_status_.f_robot_target_angular_vel_w) > 0.1) {
        // 다른 상태에서 러닝으로 바뀌는 순간 시간체크
        tp_running_time_ = std::chrono::steady_clock::now();
        st_status_.tp_last_motion_time_ = std::chrono::steady_clock::now();
    }

    core_status_.s_status = tmpmsg.data;  // navi status
    if (core_status_.s_current_node.size() == 0) {
        core_status_.s_current_node = "0";
        core_status_.s_current_node_id = "0";
    }
    if (core_status_.s_next_node.size() == 0) {
        core_status_.s_next_node = "0";
        core_status_.s_next_node_id = "0";
    }
    if (core_status_.s_goal_node.size() == 0) {
        core_status_.s_goal_node = "0";
        core_status_.s_goal_node_id = "0";
    }
    str_navi_status_pub_.publish(tmpmsg);

    core_status_.header.stamp = ros::Time::now();

    {
        // core_status_ , error
        core_status_.f_error_pos_x_m = core_status_.f_robot_pos_x_m - core_status_.f_goal_pos_x_m;
        core_status_.f_error_pos_y_m = core_status_.f_robot_pos_y_m - core_status_.f_goal_pos_y_m;
        core_status_.f_error_pos_deg = CoreCalculator::CalcAngleDomainDeg_(core_status_.f_robot_pos_deg - core_status_.f_goal_pos_deg);
    }

    {  // warning 관리
        std::lock_guard<std::mutex> lock(warningMutex);
        auto now = std::chrono::steady_clock::now();

        // 5초 초과된 경고 삭제
        vec_warnning_.erase(
            std::remove_if(
                vec_warnning_.begin(), vec_warnning_.end(),
                [now, this](const auto& w) {
                    return std::chrono::duration_cast<std::chrono::seconds>(now - w.second).count() > n_warningDuration_;
                }),
            vec_warnning_.end());
        core_status_.s_warning.clear();
        for (const auto& w : vec_warnning_) {
            core_status_.s_warning.emplace_back(w.first);
        }
    }

    core_msgs::NavicoreStatus core_status = core_status_;
    navistatus_pub_.publish(core_status);

    // 현재 노드 바뀌었을때 로그 추가
    if (ex_core_status.s_current_node != core_status_.s_current_node) {
        LOG_INFO(
            "status : %s, cur_node : %s, next_node : %s, goal_node : %s", core_status_.s_status.c_str(),
            core_status_.s_current_node.c_str(), core_status_.s_next_node.c_str(), core_status_.s_goal_node.c_str());
    }

    ex_core_status = core_status_;

    std::chrono::duration<double> sec_running = std::chrono::steady_clock::now() - tp_running_time_;

    if (core_status_.s_status == RUNNING) {
        if (sec_running.count() > 5) {
            LOG_ERROR("ERROR_PATH_SPEED_ZERO");
            // 경로가 남았는데 target speed가 5초이상 0인경우 에러처리
            SetNaviError(core_msgs::NaviAlarm::ERROR_PATH_SPEED_ZERO);
            AbortMission();
        }
        else {
            CheckMotionTimeout();
        }
    }
}

void ncNavigator::CheckMotionTimeout()
{
    if (fabs(core_status_.f_robot_target_linear_vel_x) > 0.001 || fabs(core_status_.f_robot_target_linear_vel_y) > 0.001 ||
        fabs(core_status_.f_robot_target_angular_vel_w) > 0.1) {
        bool pos_changed = (fabs(o_robot_current_tf_pos_.GetXm() - st_status_.last_motion_pos_.GetXm()) > 0.02 ||
                            fabs(o_robot_current_tf_pos_.GetYm() - st_status_.last_motion_pos_.GetYm()) > 0.02 ||
                            fabs(o_robot_current_tf_pos_.GetDeg() - st_status_.last_motion_pos_.GetDeg()) > 0.3);

        bool steer_changed = (fabs(st_status_.f_steer_angle_deg_[0] - st_status_.last_motion_steer_deg_[0]) > 5 ||
                              fabs(st_status_.f_steer_angle_deg_[1] - st_status_.last_motion_steer_deg_[1]) > 5 ||
                              fabs(st_status_.f_steer_angle_deg_[2] - st_status_.last_motion_steer_deg_[2]) > 5 ||
                              fabs(st_status_.f_steer_angle_deg_[3] - st_status_.last_motion_steer_deg_[3]) > 5);

        if (pos_changed || steer_changed) {
            st_status_.last_motion_pos_ = o_robot_current_tf_pos_;
            for (int i = 0; i < 4; i++) {
                st_status_.last_motion_steer_deg_[i] = st_status_.f_steer_angle_deg_[i];
            }
            st_status_.tp_last_motion_time_ = std::chrono::steady_clock::now();
        }

        std::chrono::duration<double> sec_no_motion = std::chrono::steady_clock::now() - st_status_.tp_last_motion_time_;
        if (sec_no_motion.count() > 3.0) {
            LOG_ERROR("ERROR_ROBOT_STUCK_OR_SLIP");
            // target speed가 존재하는데 3초동안 robot_pos 변화가없거나 motor steering 변화가 없는경우 에러처리
            SetNaviError(core_msgs::NaviAlarm::ERROR_SLIP_PREDETECTED);
            AbortMission();
            st_status_.tp_last_motion_time_ = std::chrono::steady_clock::now();
        }
    }
    else {
        st_status_.tp_last_motion_time_ = std::chrono::steady_clock::now();
        st_status_.last_motion_pos_ = o_robot_current_tf_pos_;
        for (int i = 0; i < 4; i++) {
            st_status_.last_motion_steer_deg_[i] = st_status_.f_steer_angle_deg_[i];
        }
    }
}

void ncNavigator::SetNaviError(int n_error)
{
    std::lock_guard<std::mutex> lock(mtx_error_);
    n_navi_error_ = n_error;
}

int ncNavigator::GetNaviError()
{
    std::lock_guard<std::mutex> lock(mtx_error_);
    return n_navi_error_;
}

void ncNavigator::removePrefixARC(std::string& node_id)
{
    const std::string prefix = "ARCS";
    if (node_id.size() > 4) {
        if (node_id.rfind(prefix, 0) == 0) {  // prefix가 맨 앞에 있는지 확인
            node_id.erase(0, prefix.length());  // "ARC" 제거
        }
        const std::string prefix2 = "ARCE";
        if (node_id.rfind(prefix2, 0) == 0) {  // prefix2가 맨 앞에 있는지 확인
            node_id.erase(0, prefix2.length());  // "ARC" 제거
        }
    }
}

void ncNavigator::RecvLoad(const std_msgs::Bool::ConstPtr& msg)
{
    bool b_loaded = msg->data;
    if(b_loaded != b_loaded_) {
        LOG_INFO("Load state changed to %s", b_loaded ? "true" : "false");
    }
    {
        std::lock_guard<std::mutex> lock(mtx_load_);
        b_loaded_ = b_loaded;
    }
    o_mission_manager_.SetLoadState(b_loaded);
}

void ncNavigator::RecvLCCS(const std_msgs::Bool::ConstPtr& msg)
{
    bool b_lccs = msg->data;
    o_mission_manager_.SetUseLccs(b_lccs);
}

void ncNavigator::ForkPositionCallback(const std_msgs::Int16::ConstPtr& msg)
{
    int n_fork_position = msg->data;
    if(n_fork_position != n_fork_position_) {
        LOG_INFO("Fork position changed to %d", n_fork_position);
    }
    n_fork_position_ = n_fork_position;
    o_mission_manager_.SetForkPosition(n_fork_position);
}

void ncNavigator::RecvNaviInfo(const boost::any& any_var)
{
    NaviFra::MissionNavigationMsg_t st_core_info = boost::any_cast<NaviFra::MissionNavigationMsg_t>(any_var);
    st_status_.st_mission_info_ = st_core_info;
    core_status_.f_collision_remained_sec = st_core_info.f_collision_remained_sec;
    core_status_.f_max_linear_vel_of_path = st_core_info.f_max_linear_vel_of_path;
    // core_status_.f_path_progress            = st_core_info.f_node_to_node_percent;
    core_status_.s_current_node = st_core_info.s_current_node;
    core_status_.s_current_node_id = st_core_info.s_current_node_id;
    if (core_status_.s_current_node_id == "") {
        core_status_.s_current_node_id = st_core_info.s_current_node;
    }
    removePrefixARC(core_status_.s_current_node_id);
    core_status_.s_next_node = st_core_info.s_next_node;
    core_status_.s_next_node_id = st_core_info.s_next_node_id;
    if (core_status_.s_next_node_id == "") {
        core_status_.s_next_node_id = st_core_info.s_next_node;
    }
    removePrefixARC(core_status_.s_next_node_id);
    core_status_.s_goal_node = st_core_info.s_goal_node;
    core_status_.s_goal_node_id = st_core_info.s_goal_node_id;
    if (core_status_.s_goal_node_id == "") {
        core_status_.s_goal_node_id = st_core_info.s_goal_node;
    }
    core_status_.n_spin_turn = st_core_info.b_spin_turn;
    core_status_.n_avoid_state = st_core_info.n_avoid_state;
    core_status_.b_avoid_status = core_status_.n_avoid_state > 0 ? true : false;
    core_status_.f_path_error_dist_m = st_core_info.f_path_error_dist_m;

    core_status_.n_drive_type = st_core_info.n_drive_type;
    core_status_.n_diagonal_direction = 0;  // 0:none, 1:left, 2:right
    if (core_status_.n_drive_type == NaviFra::Pos::DRIVE_TYPE::FRONT_DIAGONAL ||
        core_status_.n_drive_type == NaviFra::Pos::DRIVE_TYPE::REAR_DIAGONAL || core_status_.b_avoid_status) {
        if (core_status_.f_robot_target_linear_vel_y > 0.05)
            core_status_.n_diagonal_direction = 1;
        if (core_status_.f_robot_target_linear_vel_y < -0.05)
            core_status_.n_diagonal_direction = 2;
    }
    std_msgs::Bool msg;
    if (st_status_.st_mission_info_.n_stop_because_of_obstacle == 0 && b_obs_check_) {
        b_obs_check_ = false;
        msg.data = b_obs_check_;
        emergency_pub_.publish(msg);
        // if(st_status_.b_paused_by_status_ == true) {
        //     ResumeMissionByState();
        // }
    }
    else if (st_status_.st_mission_info_.n_stop_because_of_obstacle != 0 && !b_obs_check_) {
        b_obs_check_ = true;
        msg.data = b_obs_check_;
        emergency_pub_.publish(msg);
    }
}

void ncNavigator::RecvMotionInfo(const boost::any& any_var)
{
    NaviFra::MotionInfo_t st_info = boost::any_cast<NaviFra::MotionInfo_t>(any_var);
    core_msgs::MotionInfo o_info_msg;
    std::string s_motion_mode;
    o_info_msg.n_motion_status = st_info.n_motion_status;
    o_info_msg.n_obs_control = st_info.n_obs_control;
    o_info_msg.f_obs_speed_ratio = st_info.f_obs_speed_ratio;
    o_info_msg.f_path_velocity = st_info.f_path_velocity;
    o_info_msg.f_mpc_input_velocity = st_info.f_mpc_input_velocity;
    o_info_msg.f_mpc_output_linear_velocity = st_info.f_mpc_output_linear_velocity;
    o_info_msg.f_mpc_output_angular_velocity = st_info.f_mpc_output_angular_velocity;
    o_info_msg.f_align_diff_angle_deg = st_info.f_align_diff_angle_deg;
    o_info_msg.f_execution_time_sec = st_info.f_execution_time_sec;
    o_info_msg.f_linear_speed_x_ms = st_info.f_linear_speed_x_ms;
    o_info_msg.f_linear_speed_y_ms = st_info.f_linear_speed_y_ms;
    o_info_msg.f_angular_speed_degs = st_info.f_angular_speed_degs;
    o_info_msg.b_reverse_motion = st_info.b_reverse_motion;
    o_info_msg.s_mpc_param_mode = st_info.s_mpc_param_mode;
    o_info_msg.s_avoid_status = st_info.s_avoid_status;

    for (int i = 0; i < st_info.vec_motion_mode.size(); i++) {
        s_motion_mode = s_motion_mode + st_info.vec_motion_mode.at(i) + " ||";
    }
    o_info_msg.s_motion_mode = s_motion_mode;

    motion_info_pub_.publish(o_info_msg);
}

void ncNavigator::RecvRequestAvoidance(const boost::any& any_var)
{
    bool b_flag = boost::any_cast<bool>(any_var);

    core_msgs::TaskAlarm avoid_task;

    boost::uuids::random_generator generator;

    // UUID 생성
    boost::uuids::uuid uuid = generator();
    std::string uuid_string = boost::uuids::to_string(uuid);
    avoid_task.uuid = uuid_string;
    if (b_flag)
        avoid_task.alarm = "request_avoidance";
    else
        avoid_task.alarm = "no_avoidance";
    task_alarm_.publish(avoid_task);
}

NaviFra::Pos ncNavigator::PosConvert(const geometry_msgs::Pose& input)
{
    NaviFra::Pos o_pos;
    o_pos.SetXm(input.position.x);
    o_pos.SetYm(input.position.y);
    o_pos.SetQuaternion(input.orientation.w, input.orientation.x, input.orientation.y, input.orientation.z);
    return o_pos;
}

NaviFra::Pos ncNavigator::PosConvert(const geometry_msgs::PoseStamped& input)
{
    NaviFra::Pos o_pos;
    o_pos.SetXm(input.pose.position.x);
    o_pos.SetYm(input.pose.position.y);
    o_pos.SetQuaternion(input.pose.orientation.w, input.pose.orientation.x, input.pose.orientation.y, input.pose.orientation.z);
    return o_pos;
}

void ncNavigator::onTaskName(const std_msgs::String msg)
{
    s_current_task_name_ = msg.data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nc_navigator");
    bool b_enable_log_file = true;
    int n_log_level = 40;  // off
    int n_log_save_date = 20;
    ros::param::param<bool>("b_enable_log_file", b_enable_log_file, true);
    ros::param::param<int>("LOG_FILE_SAVE_DATE", n_log_save_date, 15);
    ros::param::param<int>("LOG_LEVEL", n_log_level, 40);

    NaviFra::Logger::get().SetSeverityMin((severity_level)((n_log_level / 10) - 2));

    LOG_INFO("NaviCore Start. corestart");
    LOG_INFO("NaviFra log file is enabled!!\nAll NaviFra log content will be recorded in ~/navifra_solution/ros", 1);

    // system log time check
    try {
        FILE* stream = popen("uptime", "r");
        ostringstream output;
        while (!feof(stream) && !ferror(stream)) {
            char buf[128];
            int bytesRead = fread(buf, 1, 128, stream);
            output.write(buf, bytesRead);
        }
        string result = output.str();
        LOG_INFO("NaviGation ON TIME : %s", result.c_str());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }

#ifdef NAVIFRA_LICENSE_ON

    License o_license_checker;
    if (o_license_checker.CheckLicence() == 0) {
        LOG_ERROR("************************************************");
        LOG_ERROR("NAVIGATOR Your License Not Resiter. Terminate the process.");
        LOG_ERROR("************************************************");
        ros::NodeHandle node_handle_temp;
        ros::Publisher navistatus_pub = node_handle_temp.advertise<std_msgs::String>("/navifra/status", 1, true);
        std_msgs::String navi_status;
        navi_status.data = "License Not register";
        while (ros::ok()) {
            navistatus_pub.publish(navi_status);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return 0;
    }
    else {
        LOG_INFO("NAVIGATOR License is Resitered.");
    }

#else  // license 옵션 안건 경우
    LOG_WARNING("************************************************");
    LOG_WARNING("NAVIGATOR License Not Checked!  Please Check your License.");
    LOG_WARNING("************************************************");
#endif

    LOG_INFO("NaviFra navi node is launched!!", 1);

    ncNavigator o_navigation_node;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    // ros::spin();
    return 0;
}
