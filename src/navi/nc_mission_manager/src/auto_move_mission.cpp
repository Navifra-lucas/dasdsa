#include "auto_move_mission.hpp"

#include "core/util/logger.hpp"

namespace NaviFra {
constexpr char NAVI_STATUS_CALLBACK[] = "navi_status_callback";
constexpr char NAVI_INFO_CALLBACK[] = "navi_info_callback";
constexpr char GOAL_CALLBACK[] = "goal_callback";
constexpr char MOTION_INFO_CALLBACK[] = "motion_info_callback";
constexpr char REQUEST_AVOIDANCE[] = "request_avoidance";
constexpr char NAVI_MISSION_ROI_CALLBACK[] = "navi_roi_callback";

// info.robot_linear_velocity
AutoMoveMission::AutoMoveMission()
{
    pub_warning_ = node_handle_.advertise<std_msgs::String>("/navifra/warning", 5, false);
    pub_obs_pos_ = node_handle_.advertise<geometry_msgs::PolygonStamped>("/NaviFra/visualize/ui_obstacle_pos", 5, false);

    Initialize();
    NaviFra::ParamRepository::GetInstance()->RegisterCbFunc(
        "MoveBehavior_Param", std::bind(&AutoMoveMission::SetNaviParam, this, std::placeholders::_1));

    e_status_ = MISSION_STATUS::IDLE;

    th_path_planner_ = boost::thread(boost::bind(&AutoMoveMission::PathPlannerBehavior, this));
    th_motion_control_ = boost::thread(boost::bind(&AutoMoveMission::MotionControl, this));
    th_info_collect_ = boost::thread(boost::bind(&AutoMoveMission::InfoCollect, this));
    th_obstacle_check_ = boost::thread(boost::bind(&AutoMoveMission::ObstacleCheck, this));
    th_thread_monitor_ = boost::thread(boost::bind(&AutoMoveMission::ThreadMonitor, this));
    LOG_INFO("Constructor", 1);
}

AutoMoveMission::~AutoMoveMission()
{
    StopMotion();
    if (th_thread_monitor_.joinable()) {
        th_thread_monitor_.join();
    }
    if (th_path_planner_.joinable()) {
        th_path_planner_.join();
    }
    if (th_motion_control_.joinable()) {
        th_motion_control_.join();
    }
    if (th_info_collect_.joinable()) {
        th_info_collect_.join();
    }
    if (th_obstacle_check_.joinable()) {
        th_obstacle_check_.join();
    }
    LOG_INFO("Destructor", 1);
}

void AutoMoveMission::ThreadMonitor()
{
    while (ros::ok()) {
        if (!th_path_planner_.joinable()) {
            NLOG(error) << "Starting or restarting th_path_planner_ thread...\n";
            th_path_planner_ = boost::thread(&AutoMoveMission::PathPlannerBehavior, this);
        }
        if (!th_motion_control_.joinable()) {
            NLOG(error) << "Starting or restarting th_motion_control_ thread...\n";
            th_motion_control_ = boost::thread(&AutoMoveMission::MotionControl, this);
        }
        if (!th_info_collect_.joinable()) {
            NLOG(error) << "Starting or restarting th_info_collect_ thread...\n";
            th_info_collect_ = boost::thread(&AutoMoveMission::InfoCollect, this);
        }
        if (!th_obstacle_check_.joinable()) {
            NLOG(error) << "Starting or restarting th_obstacle_check_ thread...\n";
            th_obstacle_check_ = boost::thread(&AutoMoveMission::ObstacleCheck, this);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        static int n_count = 0;
        if (n_count++ > 10) {
            n_count = 0;
            NLOG(info) << "ThreadMonitor is running...";

            std::chrono::duration<double> sec_path_planner_ = std::chrono::steady_clock::now() - tp_path_planner_;
            std::chrono::duration<double> sec_motion_control_ = std::chrono::steady_clock::now() - tp_motion_control_;
            std::chrono::duration<double> sec_info_collect_ = std::chrono::steady_clock::now() - tp_info_collect_;
            std::chrono::duration<double> sec_obstacle_check_ = std::chrono::steady_clock::now() - tp_obstacle_check_;

            if (sec_path_planner_.count() > 3) {
                NLOG(error) << "Thread Die sec_path_planner_..";
            }
            if (sec_motion_control_.count() > 3) {
                NLOG(error) << "Thread Die sec_motion_control_..";
            }
            if (sec_info_collect_.count() > 3) {
                NLOG(error) << "Thread Die sec_info_collect_..";
            }
            if (sec_obstacle_check_.count() > 3) {
                NLOG(error) << "Thread Die sec_obstacle_check_..";
            }
        }
    }
}

bool AutoMoveMission::GetUserPauseFlag()
{
    return b_is_paused_by_user;
}

void AutoMoveMission::PauseUser()
{
    LOG_INFO("");
    b_is_paused_by_user = true;
}

void AutoMoveMission::ResumeUser()
{
    LOG_INFO("");
    b_is_paused_by_user = false;
}

void AutoMoveMission::SetMotionInfo(const MotionInfo_t st_info)
{
    {
        std::lock_guard<std::mutex> lock(mtx_motion_info_);
        st_info_ = st_info;
    }
}

void AutoMoveMission::AvoidMissionClear()
{
}

int AutoMoveMission::SetBehaviorMission(const NaviFra::PathDescription& mission, bool b_is_add_goal)
{
    int n_debug_line = __LINE__;
    try {
        Parameters_t st_param;
        {
            std::lock_guard<std::mutex> lock(mtx_param_);
            st_param = st_param_;
        }
        // Initialize();
        LOG_INFO("Given mission Started!!!");

        vec_current_mission_ = mission;
        {
            std::lock_guard<std::mutex> lock(mtx_mission_path_);
            vec_mission_path_ = vec_current_mission_.vec_path;
        }

        n_debug_line = __LINE__;

        // 지역 경로가 후진 경로인 경우 후진 플래그 설정
        if (vec_current_mission_.vec_path.at(0).GetDriveInfo().e_drive_type == NaviFra::Pos::DRIVE_TYPE::REAR ||
            vec_current_mission_.vec_path.at(0).GetDriveInfo().e_drive_type == NaviFra::Pos::DRIVE_TYPE::REAR_DIAGONAL) {
            // 사행 주행 구간의 여부랑 상관 없이 n_diagonal_drive_type_의 값은 결정해준다
            b_move_backward_flag_ = true;
        }
        else {
            b_move_backward_flag_ = false;
        }

        n_debug_line = __LINE__;

        // calculate heading error on given path
        NaviFra::Pos o_robot_pos_instance;
        {
            std::lock_guard<std::mutex> lock(mtx_robot_pos_);
            o_robot_pos_instance = o_robot_pos_;
        }
        if (b_move_backward_flag_) {
            o_robot_pos_instance.SetRad(o_robot_pos_instance.GetRad() + vec_current_mission_.f_yaw_bias);
        }

        n_debug_line = __LINE__;

        n_mission_path_idx_ = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_robot_pos_instance, vec_mission_path_);

        vec_mission_path_.back().GetDriveInfo().f_linear = 0;
        if (vec_mission_path_.size() > 2) {
            vec_mission_path_.back().SetDriveInfo(vec_mission_path_.at(vec_mission_path_.size() - 2).GetConstDriveInfo());
            SetFullSm(vec_mission_path_);
            SetCurvature(vec_mission_path_);
        }

        n_debug_line = __LINE__;

        n_force_comeback_target_idx_ = CoreCalculator::FindIndexAtDistance_(
            vec_mission_path_, (int)vec_mission_path_.size() - 1, st_param.f_force_comeback_target_dist_m, true);
        n_force_comeback_start_idx_ = CoreCalculator::FindIndexAtDistance_(
            vec_mission_path_, n_force_comeback_target_idx_, st_param.f_force_comeback_start_dist_m, true);

        // 미션 경로에서 n_force_comeback_start_idx_ ~ back() 까지 고선이 존재하면 n_force_comeback_start_idx_ 늘리기
        bool b_is_curve = false;
        for (std::vector<Pos>::const_iterator it = vec_mission_path_.begin() + n_force_comeback_start_idx_; it != vec_mission_path_.end();
             ++it) {
            if (std::fabs(it->GetConstDriveInfo().f_curvature) > 0.01f) {
                LOG_INFO(
                    "[AutoMoveMission::SetBehaviorMission] mission path size: %d, curve idx: %d, pos: %.3f, %.3f, %.3f, curvature: %.3f",
                    (int)vec_mission_path_.size(), int(it - vec_mission_path_.begin()), it->GetXm(), it->GetYm(), it->GetDeg(),
                    it->GetConstDriveInfo().f_curvature);
                b_is_curve = true;
                break;
            }
        }
        if (b_is_curve) {
            n_force_comeback_start_idx_ = CoreCalculator::FindIndexAtDistance_(
                vec_mission_path_, n_force_comeback_target_idx_,
                st_param.f_force_comeback_start_dist_m + st_param.f_force_comeback_in_curve_added_start_dist_m, true);
            LOG_INFO(
                "[AutoMoveMission::SetBehaviorMission] force_comeback size up in curve (%.3f)",
                st_param.f_force_comeback_start_dist_m + st_param.f_force_comeback_in_curve_added_start_dist_m);
        }

        if (!b_is_add_goal) {
            b_avoid_status_ = false;
            ClearCurrentPath();
            LOG_INFO("b_is_add_goal : %s", b_is_add_goal ? "TRUE" : "FALSE");
        }
        o_lane_avoid_path_ptr_->SetForceComebackTargetIdx(n_force_comeback_target_idx_);

        n_debug_line = __LINE__;

        // 회피 중일때는 계속 회피하기위해서
        if (!b_avoid_status_) {
            LOG_INFO("b_avoid_status_ : %s ", b_avoid_status_ ? "TRUE" : "FALSE");
            n_current_path_collision_idx_ = -1;
            n_current_target_lane_num_ = 0;
            vec_avoid_path_.clear();
            SetCurrentPathFromMissionPath(0.f);
            tp_start_time_ = std::chrono::steady_clock::now();  // start time save
            tp_detection_period_time_ = std::chrono::steady_clock::now();
            tp_start_obs_detect_time_ = std::chrono::steady_clock::now();
            SetAvoidPermission(st_param.b_avoid_permission);
            n_pp_bt_state_ = 0;
            o_last_collision_pos_.SetXm(1E9);
            o_last_collision_pos_.SetYm(1E9);
            o_static_obs_tmr_->Start();

            o_lane_avoid_path_ptr_->SetForceComebackTargetIdx(n_force_comeback_target_idx_);

            n_debug_line = __LINE__;

            // 처음에 미션을 받을 때 AWAY_FROM_PATH 일 경우지만 lane 안에 들어오면 회피경로를 새로 작성해준다
            float f_dist_tmp = std::hypot(
                o_robot_pos_instance.GetXm() - vec_mission_path_.at(n_mission_path_idx_).GetXm(),
                o_robot_pos_instance.GetYm() - vec_mission_path_.at(n_mission_path_idx_).GetYm());
            float f_del_x_m = (o_robot_pos_instance.GetXm() - vec_mission_path_.at(n_mission_path_idx_).GetXm()) *
                    std::cos(vec_mission_path_.at(n_mission_path_idx_).GetRad()) +
                (o_robot_pos_instance.GetYm() - vec_mission_path_.at(n_mission_path_idx_).GetYm()) *
                    std::sin(vec_mission_path_.at(n_mission_path_idx_).GetRad());
            float f_del_y_m = (o_robot_pos_instance.GetYm() - vec_mission_path_.at(n_mission_path_idx_).GetYm()) *
                    std::cos(vec_mission_path_.at(n_mission_path_idx_).GetRad()) -
                (o_robot_pos_instance.GetXm() - vec_mission_path_.at(n_mission_path_idx_).GetXm()) *
                    std::sin(vec_mission_path_.at(n_mission_path_idx_).GetRad());
            bool b_away = f_dist_tmp > 0.2f;
            NLOG(info) << "[AutoMoveMission::SetBehaviorMission] Distance from path: " << f_dist_tmp;
            bool b_in_lane = std::fabs(f_del_y_m) <
                st_lane_info_.f_lane_dist * std::max(st_lane_info_.n_left_max_num, st_lane_info_.n_right_max_num) + 0.2f;
            if (b_away && b_in_lane) {
                LOG_INFO("[AutoMoveMission::SetBehaviorMission] Away and in_lane -> try ForceComebackPath");
                Pos o_start_pos = o_robot_pos_instance;
                float f_del_rad = o_start_pos.GetRad() - vec_mission_path_.at(n_mission_path_idx_).GetRad();
                if (f_del_y_m * f_del_rad > 0 || std::fabs(f_del_rad) > M_PI_2)
                    o_start_pos.SetRad(vec_mission_path_.at(n_mission_path_idx_).GetRad());
                int n_force_comback_target_idx_tmp = CoreCalculator::FindIndexAtDistance_(vec_mission_path_, n_mission_path_idx_, 4.0f);
                vec_avoid_path_ = o_lane_avoid_path_ptr_->ForceComebackPath(
                    vec_mission_path_, o_start_pos, vec_mission_path_.at(n_force_comback_target_idx_tmp), st_lane_info_,
                    n_current_target_lane_num_, 0.f);
                if (!vec_avoid_path_.empty()) {
                    SetCurrentPathFromAvoidPath(0);
                    LOG_INFO("[AutoMoveMission::SetBehaviorMission] Away and in_lane -> success");
                }
                else {
                    ClearCurrentPath();
                    LOG_INFO("[AutoMoveMission::SetBehaviorMission] Away and in_lane -> fail");
                }
            }
        }

        LOG_WARNING(
            "start smooth(%s), stop smooth(%s)", (vec_mission_path_.front().GetConstDriveInfo().b_start_smooth) ? "TRUE" : "FALSE",
            (vec_mission_path_.front().GetConstDriveInfo().b_stop_smooth) ? "TRUE" : "FALSE");

        int n_local_robot_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_robot_pos_instance, vec_current_path_);
        float f_angle_deg = vec_current_path_.front().GetDeg();
        if (vec_current_path_.size() > 5) {
            f_angle_deg = atan2(
                              vec_current_path_.at(4).GetYm() - vec_current_path_.at(0).GetYm(),
                              vec_current_path_.at(4).GetXm() - vec_current_path_.at(0).GetXm()) *
                RADtoDEG;
        }
        n_debug_line = __LINE__;

        if (n_local_robot_idx >= 0)
            o_robot_pos_instance.SetDeg(
                o_robot_pos_instance.GetDeg() + vec_current_path_.at(n_local_robot_idx).GetConstDriveInfo().f_heading_bias);

        float f_head_err = abs(CoreCalculator::CalcAngleDomainDeg_(o_robot_pos_instance.GetDeg() - f_angle_deg));
        NLOG(info) << "o_robot_pos_ " << o_robot_pos_.GetDeg() << " / " << o_robot_pos_instance.GetDeg() << " // "
                   << vec_current_mission_.f_yaw_bias << " / " << f_angle_deg << endl;
        LOG_WARNING("Not an error, checking head align at initial state %.1f", f_head_err);

        const bool b_head_align_flag = std::isgreater(f_head_err, st_param.f_align_motion_threshold_for_start_deg);

        o_motion_controller_.Initialize(vec_mission_path_, n_local_robot_idx, b_is_add_goal, b_head_align_flag);

        st_vel_ex_ = CommandVelocity_t();
        SetStatus(MISSION_STATUS::RUNNING);
        LOG_INFO("Behavior staus::RUNNING", 1);
        ResumeUser();
        PubLocalPath(vec_mission_path_);

        float f_angle = 0, f_speed = 0, f_type = 0, f_curvature = 0;
        for (int i = 0; i < vec_mission_path_.size(); i++) {
            if (fabs(f_angle - vec_mission_path_.at(i).GetDeg()) > 1 ||
                fabs(f_speed - vec_mission_path_.at(i).GetConstDriveInfo().f_linear) > 0.05 ||
                f_type != vec_mission_path_.at(i).GetConstDriveInfo().e_drive_type) {
                NLOG(info) << "vec_mission_path_[" << i << "] : x " << vec_mission_path_.at(i).GetXm() << ", y "
                           << vec_mission_path_.at(i).GetYm() << ", deg " << vec_mission_path_.at(i).GetDeg() << ", curve "
                           << vec_mission_path_.at(i).GetConstDriveInfo().f_curvature << ", speed "
                           << vec_mission_path_.at(i).GetConstDriveInfo().f_linear << ", type "
                           << vec_mission_path_.at(i).GetConstDriveInfo().e_drive_type;
            }
            f_angle = vec_mission_path_.at(i).GetDeg();
            f_speed = vec_mission_path_.at(i).GetConstDriveInfo().f_linear;
            f_type = vec_mission_path_.at(i).GetConstDriveInfo().e_drive_type;
            f_curvature = vec_mission_path_.at(i).GetConstDriveInfo().f_curvature;
        }
        // notify roi info
        core_msgs::CameraRoiInfoWia cam_msgs;
        auto o_local_goal_ = vec_current_mission_.o_goal_pos;
        cam_msgs.d_x_th_m = o_local_goal_.GetDriveInfo().d_camera_roi_x_m;
        cam_msgs.d_y_th_m = o_local_goal_.GetDriveInfo().d_camera_roi_y_m;
        cam_msgs.d_z_th_m = o_local_goal_.GetDriveInfo().d_camera_roi_z_m;
        cam_msgs.d_x2_th_m = o_local_goal_.GetDriveInfo().d_camera_roi2_x_m;
        cam_msgs.d_y2_th_m = o_local_goal_.GetDriveInfo().d_camera_roi2_y_m;
        cam_msgs.d_z2_th_m = o_local_goal_.GetDriveInfo().d_camera_roi2_z_m;
        cam_msgs.n_index = o_local_goal_.GetDriveInfo().n_camera_index;
        cout << cam_msgs.n_index << cam_msgs.d_x_th_m << cam_msgs.d_y_th_m << cam_msgs.d_z_th_m << endl << endl;
        o_cb_regiser_.notify(NAVI_MISSION_ROI_CALLBACK, cam_msgs);
    }
    catch (const std::exception& e) {
        NLOG(error) << "Exception in SetBehaviorMission: " << e.what() << " / debug line : " << n_debug_line;
    }
    return 0;
}

bool AutoMoveMission::RegistCbFunc(int n_cb_name, const std::function<void(const boost::any&)>& pt_func)
{
    bool b_ret = o_cb_regiser_.RegistCbFunc(n_cb_name, pt_func);
    return b_ret;
}

bool AutoMoveMission::RegistCbFunc(const std::string& s_cb_func_name, const std::function<void(const boost::any&)>& pt_func_)
{
    bool b_ret = o_cb_regiser_.RegistCbFunc(s_cb_func_name, pt_func_);
    return b_ret;
}

void AutoMoveMission::NotifyCbFunc(const std::string& s_msg, const boost::any& any_type_var)
{
    o_cb_regiser_.notify(s_msg, any_type_var);
}

int AutoMoveMission::ExecuteJob()
{
    LOG_INFO("execute_behavior_job is called!!!");
    b_thread_run_ = true;
    o_rapid_timer_.Reset("PP_BT");
    return 0;
}

void AutoMoveMission::ConfirmAllProcTerminate(int n_obj)
{
    LOG_INFO("ConfirmAllProcTerminate");  // 끝나면 termination complete메세지 전송
    b_thread_run_ = false;
    o_cb_regiser_.notify(static_cast<int>(NOTIFY_MSG::MISSION), n_obj);
}

void AutoMoveMission::Initialize()
{
    o_collision_detector_ptr_ = std::make_shared<CollisionDetector>();
    o_lane_avoid_path_ptr_ = std::make_shared<LaneAvoidPath>();
    o_lane_avoid_path_ptr_->SetCollisionDetectorPtr(o_collision_detector_ptr_);
    o_static_obs_tmr_ = std::make_shared<Timer>("static_obs");
}

void AutoMoveMission::SetRobotPos(const NaviFra::Pos& o_robot_pos)
{
    std::lock_guard<std::mutex> lock(mtx_robot_pos_);
    o_robot_pos_ = o_robot_pos;
}

void AutoMoveMission::SetArriveBoundary(const float& f_arrive_boundary_m)
{
    Parameters_t st_param;
    {
        std::lock_guard<std::mutex> lock(mtx_param_);
        st_param = st_param_;
    }
    o_motion_controller_.SetArriveBoundary(f_arrive_boundary_m);
}

void AutoMoveMission::SetTwist(const float& f_speed)
{
    Parameters_t st_param;
    {
        std::lock_guard<std::mutex> lock(mtx_param_);
        st_param = st_param_;
    }
    o_motion_controller_.SetMaxLinearVelocity(f_speed);
}

void AutoMoveMission::SetSpeedPercent(const float& f_speed_percent)
{
    Parameters_t st_param;
    {
        std::lock_guard<std::mutex> lock(mtx_param_);
        st_param = st_param_;
    }
    o_motion_controller_.SetDeceleration(f_speed_percent);
}

void AutoMoveMission::SetTurnPercent(const float& f_turn_percent)
{
    Parameters_t st_param;
    {
        std::lock_guard<std::mutex> lock(mtx_param_);
        st_param = st_param_;
    }
    o_motion_controller_.SetTurnDeceleration(f_turn_percent);
}

void AutoMoveMission::SetNaviParam(const boost::any& any_type_var)
{
    LOG_INFO("SetNaviParam");
    Parameters_t st_param;
    {
        std::lock_guard<std::mutex> lock(mtx_param_);
        st_param_ = boost::any_cast<Parameters_t>(any_type_var);
        st_param = st_param_;
    }

    if (false == st_param.b_use_predict_mode_flag)
        f_obs_target_speed_vel_ = st_param.st_motion_param.f_linear_speed_max_ms;

    LOG_INFO("Track-ORIGIN-Motion is acticvated", 1);
    o_motion_controller_.SetParameter(st_param);
    o_camera_obstacle_check_.SetNaviParam(st_param);

    NaviFra::Polygon o_outline_polygon;
    if (st_param.map_polygon_robot_collision_.find("outline") != st_param.map_polygon_robot_collision_.end()) {
        o_outline_polygon = st_param.map_polygon_robot_collision_.at("outline");
        f_robot_radius_ = st_param.map_polygon_robot_collision_.at("outline").GetRobotWidth() * 2;
        NLOG(info) << "Key found f_robot_radius_ " << f_robot_radius_;
        if (f_robot_radius_ > 10)
            NLOG(info) << "f_robot_radius_ error ! : " << f_robot_radius_ << endl;
    }
    else {
        f_robot_radius_ = 1;
        NLOG(info) << "Key not found";
    }
    LOG_INFO("SetNaviParam Done");

    // Initialize();
    // exception: collision polygon이 없으면 안 됨
    std::vector<NaviFra::Pos> o_collision_polygon_vertexs;
    NaviFra::Polygon o_collision_polygon;
    if (st_param.map_polygon_robot_collision_.find("collision") != st_param.map_polygon_robot_collision_.end()) {
        o_collision_polygon = st_param.map_polygon_robot_collision_.at("collision");
        o_collision_polygon_vertexs = o_collision_polygon.o_polygon_vertexs_;
        // set collision box
        f_collision_box_max_xm = o_collision_polygon_vertexs.at(0).GetXm();
        f_collision_box_min_xm = o_collision_polygon_vertexs.at(0).GetXm();
        f_collision_box_max_ym = o_collision_polygon_vertexs.at(0).GetYm();
        f_collision_box_min_ym = o_collision_polygon_vertexs.at(0).GetYm();
        for (int i = 1; i < (int)o_collision_polygon_vertexs.size(); ++i) {
            if (f_collision_box_max_xm < o_collision_polygon_vertexs.at(i).GetXm())
                f_collision_box_max_xm = o_collision_polygon_vertexs.at(i).GetXm();
            if (f_collision_box_min_xm > o_collision_polygon_vertexs.at(i).GetXm())
                f_collision_box_min_xm = o_collision_polygon_vertexs.at(i).GetXm();
            if (f_collision_box_max_ym < o_collision_polygon_vertexs.at(i).GetYm())
                f_collision_box_max_ym = o_collision_polygon_vertexs.at(i).GetYm();
            if (f_collision_box_min_ym > o_collision_polygon_vertexs.at(i).GetYm())
                f_collision_box_min_ym = o_collision_polygon_vertexs.at(i).GetYm();
        }
        float f_collision_max_abs_xm = f_collision_box_max_xm * f_collision_box_max_xm > f_collision_box_min_xm * f_collision_box_min_xm
            ? f_collision_box_max_xm
            : f_collision_box_min_xm;
        float f_collision_max_abs_ym = f_collision_box_max_ym * f_collision_box_max_ym > f_collision_box_min_ym * f_collision_box_min_ym
            ? f_collision_box_max_ym
            : f_collision_box_min_ym;
        f_collision_radius_m = std::hypot(f_collision_max_abs_xm, f_collision_max_abs_ym);
    }
    else {
        NLOG(error) << "Collision Key not found";
    }

    std::vector<NaviFra::Pos> o_collision2_polygon_vertexs;
    NaviFra::Polygon o_collision2_polygon;
    if (st_param.map_polygon_robot_collision_.find("collision2") != st_param.map_polygon_robot_collision_.end()) {
        o_collision2_polygon = st_param.map_polygon_robot_collision_.at("collision2");
        o_collision2_polygon_vertexs = o_collision2_polygon.o_polygon_vertexs_;
        // set collision box
        // f_collision_box_max_xm = o_collision2_polygon_vertexs.at(0).GetXm();
        // f_collision_box_min_xm = o_collision2_polygon_vertexs.at(0).GetXm();
        // f_collision_box_max_ym = o_collision2_polygon_vertexs.at(0).GetYm();
        // f_collision_box_min_ym = o_collision2_polygon_vertexs.at(0).GetYm();
        // for (int i = 1; i < (int)o_collision2_polygon_vertexs.size(); ++i) {
        //     if (f_collision_box_max_xm < o_collision2_polygon_vertexs.at(i).GetXm())
        //         f_collision_box_max_xm = o_collision2_polygon_vertexs.at(i).GetXm();
        //     if (f_collision_box_min_xm > o_collision2_polygon_vertexs.at(i).GetXm())
        //         f_collision_box_min_xm = o_collision2_polygon_vertexs.at(i).GetXm();
        //     if (f_collision_box_max_ym < o_collision2_polygon_vertexs.at(i).GetYm())
        //         f_collision_box_max_ym = o_collision2_polygon_vertexs.at(i).GetYm();
        //     if (f_collision_box_min_ym > o_collision2_polygon_vertexs.at(i).GetYm())
        //         f_collision_box_min_ym = o_collision2_polygon_vertexs.at(i).GetYm();
        // }
        // float f_collision_max_abs_xm = f_collision_box_max_xm * f_collision_box_max_xm > f_collision_box_min_xm * f_collision_box_min_xm
        //     ? f_collision_box_max_xm
        //     : f_collision_box_min_xm;
        // float f_collision_max_abs_ym = f_collision_box_max_ym * f_collision_box_max_ym > f_collision_box_min_ym * f_collision_box_min_ym
        //     ? f_collision_box_max_ym
        //     : f_collision_box_min_ym;
        // f_collision_radius_m = std::hypot(f_collision_max_abs_xm, f_collision_max_abs_ym);
    }
    else {
        NLOG(error) << "Collision2 Key not found";
    }

    std::vector<NaviFra::Pos> o_collision3_polygon_vertexs;
    NaviFra::Polygon o_collision3_polygon;
    if (st_param.map_polygon_robot_collision_.find("collision3") != st_param.map_polygon_robot_collision_.end()) {
        o_collision3_polygon = st_param.map_polygon_robot_collision_.at("collision3");
        o_collision3_polygon_vertexs = o_collision3_polygon.o_polygon_vertexs_;
    }
    else {
        NLOG(error) << "Collision3 Key not found";
    }

    {
        std::lock_guard<std::mutex> lock(mtx_polygon_);
        o_collision_polygon_ = o_collision_polygon;
        o_collision2_polygon_ = o_collision2_polygon;
        o_collision3_polygon_ = o_collision3_polygon;
        o_outline_polygon_ = o_outline_polygon;
    }

    st_lane_info_.n_left_max_num = st_param_.st_lane_avoidance_param.n_left_num;
    st_lane_info_.n_right_max_num = st_param_.st_lane_avoidance_param.n_right_num;
    st_lane_info_.f_lane_dist = st_param_.st_lane_avoidance_param.f_interval_m;

    o_lane_avoid_path_ptr_->SetParam(st_param);
    o_collision_detector_ptr_->SetMapInfo(st_param);
    o_collision_detector_ptr_->SetRobotInfo(
        f_collision_box_max_xm, f_collision_box_min_xm, f_collision_box_max_ym, f_collision_box_min_ym, f_collision_radius_m);

    o_static_obs_tmr_->SetTimer(st_param_.f_static_obs_check_time_sec);

    LOG_INFO("Start Thread Done");
}

void AutoMoveMission::SetSensorMsg(const SensorMsg_t& st_sensor_msgs)
{
    {
        std::lock_guard<std::mutex> lock(mtx_sensor_);
        st_sensor_msg_ = st_sensor_msgs;
    }
    o_motion_controller_.SetSensorMsg(st_sensor_msgs);

    {
        std::lock_guard<std::mutex> lock(mtx_robot_speed_);
        o_robot_vel_ = st_sensor_msg_.o_robot_speed;
    }
}

void AutoMoveMission::SetSpeed(const NaviFra::Pos& o_robot_speed)
{
    std::lock_guard<std::mutex> lock(mtx_robot_speed_);
    o_robot_vel_ = o_robot_speed;
}

void AutoMoveMission::ResumeMission()
{
    o_motion_controller_.StartMotion();
    SetStatus(MISSION_STATUS::RUNNING);
}

void AutoMoveMission::SuspendMission()
{
    o_motion_controller_.StopMotion();

    SetStatus(MISSION_STATUS::SUSPENDING);
}

void AutoMoveMission::TerminateMission()
{
    LOG_INFO("");
    SetStatus(MISSION_STATUS::IDLE);
    b_thread_run_ = false;
    StopMotion();
}

void AutoMoveMission::StopMotion()
{
    if (b_is_paused_by_user) {
        tp_start_obs_detect_time_ = std::chrono::steady_clock::now();
    }

    o_motion_controller_.StopMotion();
    o_cb_regiser_.notify(GOAL_CALLBACK, CommandVelocity_t());
}

void AutoMoveMission::SetStatus(AutoMoveMission::MISSION_STATUS e_status)
{
    e_status_ = e_status;
}

AutoMoveMission::MISSION_STATUS AutoMoveMission::GetStatus()
{
    return e_status_;
}

void AutoMoveMission::SetLoadState(const bool& b_msg)
{
    b_loaded_ = b_msg;
}

void AutoMoveMission::SetForkPosition(int n_pos)
{
    n_fork_position_ = n_pos;
}

void AutoMoveMission::SetUseLccs(const bool& b_msg)
{
    b_lccs_ = b_msg;
}

void AutoMoveMission::SetGoalArrivedFlag(const bool& b_msg)
{
    if (b_thread_run_) {
        b_force_goal_arrived_ = b_msg;
    }
}

void AutoMoveMission::SetObstacleValue(
    Parameters_t& st_param, int& n_local_path_idx, vector<Pos>& vec_pos_local_path, NaviFra::Pos& o_robot_pos,
    CommandVelocity_t& regenerated_st_vel, SensorMsg_t& st_sensor_msg, NaviFra::Polygon& o_collision_polygon,
    NaviFra::Polygon& o_outline_polygon)
{
    {
        std::lock_guard<std::mutex> lock(mtx_param_);
        st_param = st_param_;
    }
    {
        std::lock_guard<std::mutex> lock(mtx_path_idx_);
        n_local_path_idx = n_mission_path_idx_;
        vec_pos_local_path = vec_mission_path_;
    }
    {
        std::lock_guard<std::mutex> lock(mtx_robot_pos_);
        o_robot_pos = o_robot_pos_;
    }
    {
        std::lock_guard<std::mutex> lock(mtx_vel_);
        regenerated_st_vel = st_vel_ex_;
    }
    {
        std::lock_guard<std::mutex> lock(mtx_sensor_);
        st_sensor_msg = st_sensor_msg_;
    }
    {
        std::lock_guard<std::mutex> lock(mtx_polygon_);
        o_collision_polygon = o_collision_polygon_;
        o_outline_polygon = o_outline_polygon_;
    }
}

void AutoMoveMission::ObstaclePub(const Pos& o_obs_pos, const Pos& o_robot_pos)
{
    sensor_msgs::PointCloud obs_cloud;

    NaviFra::Polygon o_polygon;
    Pos o_obs_pos1(o_obs_pos.GetXm() + 0.05, o_obs_pos.GetYm() + 0.05, 0);
    Pos o_obs_pos2(o_obs_pos.GetXm() - 0.05, o_obs_pos.GetYm() + 0.05, 0);
    Pos o_obs_pos3(o_obs_pos.GetXm() - 0.05, o_obs_pos.GetYm() - 0.05, 0);
    Pos o_obs_pos4(o_obs_pos.GetXm() + 0.05, o_obs_pos.GetYm() - 0.05, 0);
    Pos o_obs_pos5(o_obs_pos.GetXm() + 0.05, o_obs_pos.GetYm() + 0.05, 0);
    Pos o_obs_pos6(o_obs_pos.GetXm() - 0.05, o_obs_pos.GetYm() - 0.05, 0);
    Pos o_obs_pos7(o_obs_pos.GetXm() - 0.05, o_obs_pos.GetYm() + 0.05, 0);
    Pos o_obs_pos8(o_obs_pos.GetXm() + 0.05, o_obs_pos.GetYm() - 0.05, 0);

    o_polygon.AddVertex(o_obs_pos1);
    o_polygon.AddVertex(o_obs_pos2);
    o_polygon.AddVertex(o_obs_pos3);
    o_polygon.AddVertex(o_obs_pos4);
    o_polygon.AddVertex(o_obs_pos5);
    o_polygon.AddVertex(o_obs_pos6);
    o_polygon.AddVertex(o_obs_pos7);
    o_polygon.AddVertex(o_obs_pos8);

    geometry_msgs::PolygonStamped msg_collision = DrawPolygon(o_polygon.o_polygon_vertexs_);
    pub_obs_pos_.publish(msg_collision);
}

void AutoMoveMission::ObstacleCheck()
{
    while (ros::ok()) {
        static int n_count = 0;
        if (n_count++ > 1000) {
            n_count = 0;
            NLOG(info) << "ObstacleCheck is running...";
        }
        tp_obstacle_check_ = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds(n_obstacle_period_ms_));
        if (!b_thread_run_)
            continue;
        static Parameters_t st_param;
        static int n_local_path_idx = 0;
        static vector<Pos> vec_pos_local_path;
        static NaviFra::Pos o_robot_pos;
        static CommandVelocity_t regenerated_st_vel;
        static SensorMsg_t st_sensor_msg;
        static NaviFra::Polygon o_polygon, o_polygon_outline, o_polygon_emo;
        SetObstacleValue(
            st_param, n_local_path_idx, vec_pos_local_path, o_robot_pos, regenerated_st_vel, st_sensor_msg, o_polygon, o_polygon_outline);

        if (b_loaded_) {
            o_polygon = o_collision3_polygon_;
        }
        else if (!b_loaded_ && n_fork_position_ == 0) {
            o_polygon = o_collision2_polygon_;
        }

        if (false == st_param.b_use_detection_mode_flag)
            continue;

        Pos::DriveInfo_t o_drive_info = vec_pos_local_path.at(n_local_path_idx).GetDriveInfo();
        Pos::DriveInfo_t o_goal_drive_info = vec_current_mission_.o_goal_pos.GetDriveInfo();
        try {
            // 최소 아웃라인 보다는 크게 하기위한 비교
            o_polygon.UpdateVertexSizeMax(o_polygon_outline);
            // outline_margin값이 존재하면 outline 기준 + margin 값만큼 collision영역 수정
            if (o_drive_info.f_obstacle_outline_margin_front > 0 && o_drive_info.f_obstacle_outline_margin_rear > 0 &&
                o_drive_info.f_obstacle_outline_margin_left > 0 && o_drive_info.f_obstacle_outline_margin_right > 0) {
                o_polygon = o_polygon_outline;
                o_polygon.UpdateVertexSizePlusFLRR(
                    o_drive_info.f_obstacle_outline_margin_front, o_drive_info.f_obstacle_outline_margin_rear,
                    o_drive_info.f_obstacle_outline_margin_left, o_drive_info.f_obstacle_outline_margin_right);
            }
            else if (
                o_drive_info.f_obstacle_outline_margin_front == -2 && o_drive_info.f_obstacle_outline_margin_rear == -2 &&
                o_drive_info.f_obstacle_outline_margin_left == -2 && o_drive_info.f_obstacle_outline_margin_right == -2) {
                o_polygon = o_polygon_outline;
            }
            else if (
                o_drive_info.f_obstacle_outline_margin_front == -10 && o_drive_info.f_obstacle_outline_margin_rear == -10 &&
                o_drive_info.f_obstacle_outline_margin_left == -10 && o_drive_info.f_obstacle_outline_margin_right == -10) {
                o_polygon = o_polygon_outline;
                o_drive_info.f_move_obstacle_margin_front = 0.0;
                o_drive_info.f_move_obstacle_margin_rear = 0.4;
                o_drive_info.f_move_obstacle_margin_left = 0.7;
                o_drive_info.f_move_obstacle_margin_right = 0.7;
            }
            Pos o_start_pos;
            if (vec_pos_local_path.size() > 5) {
                o_start_pos = vec_pos_local_path.at(vec_pos_local_path.size() - 5);
            }
            Pos o_end_pos = vec_pos_local_path.at(vec_pos_local_path.size() - 1);

            // 도착지점 장애물 체크 폴리곤
            bool b_arrive_check = false;
            if ((o_goal_drive_info.f_target_obstacle_margin_front + o_goal_drive_info.f_target_obstacle_margin_rear > 0) &&
                (o_goal_drive_info.f_target_obstacle_margin_left + o_goal_drive_info.f_target_obstacle_margin_right > 0)) {
                b_arrive_check = true;
                o_lidar_obstacle_check_.TargetObsPolyMaker(
                    o_start_pos, o_end_pos, o_robot_pos, vec_pos_local_path, o_drive_info, o_goal_drive_info);
            }

            // 도킹경로 장애물 체크 폴리곤
            bool b_docking_path_obs_check = false;
            if ((o_drive_info.f_move_obstacle_margin_front + o_drive_info.f_move_obstacle_margin_rear > 0) &&
                (o_drive_info.f_move_obstacle_margin_left + o_drive_info.f_move_obstacle_margin_right > 0)) {
                o_lidar_obstacle_check_.MoveObsPolyMaker(o_start_pos, o_end_pos, o_robot_pos, o_drive_info, st_param, o_polygon, vec_pos_local_path);
                b_docking_path_obs_check = true;
            }

            // 도킹경로 장애물 체크 폴리곤
            // bool b_docking_path_obs_check = false;
            // if ((o_drive_info.f_move_obstacle_margin_front + o_drive_info.f_move_obstacle_margin_rear > 0) &&
            //     (o_drive_info.f_move_obstacle_margin_left + o_drive_info.f_move_obstacle_margin_right > 0)) {
            //     b_docking_path_obs_check = true;
            // }

            // if ((o_drive_info.f_target_obstacle_margin_front + o_drive_info.f_target_obstacle_margin_rear > 0) &&
            //     (o_drive_info.f_target_obstacle_margin_left + o_drive_info.f_target_obstacle_margin_right > 0)) {
            //     b_docking_path_obs_check = true;
            // }

            b_docking_path_obs_check_ = b_docking_path_obs_check;

            // 주행 시작 전 obstacle 감지
            bool b_start_obs_check = false;
            std::chrono::duration<double> sec_check4 = std::chrono::steady_clock::now() - tp_start_obs_detect_time_;
            if (st_param.b_use_start_obs_check_flag == true && sec_check4.count() < st_param.n_start_obs_check_sec &&
                !b_docking_path_obs_check) {
                b_start_obs_check = true;
                o_lidar_obstacle_check_.StartObsPolyMaker(o_robot_pos, vec_pos_local_path, st_param, o_polygon);
            }

            float f_linear_speed_x = regenerated_st_vel.f_linear_speed_x_ms;
            float f_angular_speed_degs = regenerated_st_vel.f_angular_speed_degs;
            // 기존 polygon, feedback_linear_vel, feedback_angular_vel, 기본 추가, 저중속 증가분, 고속 증가분, 측면 변화분, 현재
            // 장애물감지여부
            if (st_param_.b_use_lccs && b_lccs_ && o_drive_info.b_lccs && !b_docking_path_obs_check && !b_arrive_check) {
                o_lidar_obstacle_check_.UpdatePolyBySpeed(o_polygon, st_param, f_linear_speed_x, f_angular_speed_degs, n_detect_check_);
            }

            static ros::Publisher poly_collision_pub_ =
                node_handle_.advertise<geometry_msgs::PolygonStamped>("/NaviFra/visualize/robot_collision", 5, true);
            geometry_msgs::PolygonStamped msg_collision = DrawPolygon(o_polygon.o_polygon_vertexs_);
            poly_collision_pub_.publish(msg_collision);

            static ros::Publisher poly_outline_pub_ =
                node_handle_.advertise<geometry_msgs::PolygonStamped>("/NaviFra/visualize/robot_outline", 5, true);
            geometry_msgs::PolygonStamped msg_outline = DrawPolygon(o_polygon_outline.o_polygon_vertexs_);
            poly_outline_pub_.publish(msg_outline);

            std::vector<NaviFra::SimplePos> vec_sensors_relative_robot = st_sensor_msg.vec_sensors_relative_robot;
            std::vector<NaviFra::SimplePos> vec_sensors_vision_robot = st_sensor_msg.vec_vision_obs;
            n_detect_check_ = o_lidar_obstacle_check_.CheckPointInPolygon(
                vec_sensors_relative_robot, o_polygon, b_arrive_check, b_docking_path_obs_check, b_start_obs_check);

            bool b_camera_detect = false;
            if (st_param_.b_use_camera_predict_mode_flag) {
                bool b_spin_turn = st_navi_info_.b_spin_turn;
                bool b_docking_check = b_docking_path_obs_check;
                bool b_back_flag = b_move_backward_flag_;
                bool b_pre_detect = b_pre_detect_;
                bool b_obs_camera_check = false;
                if (n_detect_check_ == 2)
                    b_obs_camera_check = true;
                int n_status = int(AutoMoveMission::GetStatus());
                bool b_is_out_docking = false;
                if (o_goal_drive_info.f_target_obstacle_margin_front != 0 || o_goal_drive_info.f_target_obstacle_margin_rear != 0 ||
                    o_goal_drive_info.f_target_obstacle_margin_left != 0 || o_goal_drive_info.f_target_obstacle_margin_right != 0) {
                    b_is_out_docking = true;
                }
                b_camera_detect = o_camera_obstacle_check_.CameraObstacleCheck(
                    b_pre_detect, b_spin_turn, b_docking_check, b_back_flag, b_obs_camera_check, b_is_out_docking, n_status,
                    n_local_path_idx, o_robot_pos, vec_pos_local_path, o_drive_info, o_goal_drive_info, o_polygon,
                    vec_sensors_vision_robot);
            }

            // checking stop conditions
            if (n_detect_check_ != 0 || b_camera_detect) {
                tp_detect_time_ = std::chrono::steady_clock::now();
                tp_start_obs_detect_time_ = std::chrono::steady_clock::now();
            }
            else {
                tp_check_time_ = std::chrono::steady_clock::now();
            }

            std::chrono::duration<double> sec_check1 = tp_detect_time_ - tp_check_time_;
            if (sec_check1.count() > st_param.f_obstacle_change_delay && AutoMoveMission::GetStatus() == MISSION_STATUS::RUNNING) {
                if (b_camera_detect) {
                    n_detect_check_ = 2;
                }
                AutoMoveMission::SuspendMission();
                LOG_WARNING("SUSPENDING MISSION, OBSTACLE DETECTED!!!! %s", st_navi_info_.s_current_node.c_str());
                NLOG(info) << " n_detect_check " << n_detect_check_ << " b_camera_detect " << b_camera_detect;
                continue;
            }
            else if (sec_check1.count() > st_param.f_obstacle_change_delay && AutoMoveMission::GetStatus() == MISSION_STATUS::SUSPENDING) {
                Pos o_obs_pos = o_lidar_obstacle_check_.GetObsPos();
                if (b_camera_detect)
                    o_obs_pos = o_camera_obstacle_check_.GetObsPos();
                ObstaclePub(o_obs_pos, o_robot_pos);
                continue;
            }

            if (st_param.b_use_predict_mode_flag) {
                static float f_obs_speed_vel_pre = 1;
                static int n_obs_cnt = 0;
                float f_obs_speed_vel = 1;
                int n_robot_path_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_robot_pos, vec_pos_local_path);
                float f_path_vel = vec_pos_local_path.at(n_robot_path_idx).GetDriveInfo().f_linear;
                bool b_diagonal_path =
                    (vec_pos_local_path.at(n_robot_path_idx).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::FRONT_DIAGONAL) ||
                    (vec_pos_local_path.at(n_robot_path_idx).GetConstDriveInfo().e_drive_type == Pos::DRIVE_TYPE::REAR_DIAGONAL);
                float f_obs_target_speed_vel = f_path_vel;

                if (true == st_param.b_use_side_check_flag && !b_docking_path_obs_check) {
                    f_obs_target_speed_vel =
                        o_lidar_obstacle_check_.SideObstacleCheck(f_path_vel, st_param, o_polygon, vec_sensors_relative_robot);

                    if (f_obs_target_speed_vel != f_path_vel) {
                        std_msgs::String msg;
                        msg.data = "[mo]obs_side";
                        pub_warning_.publish(msg);

                        Pos o_obs_pos = o_lidar_obstacle_check_.GetObsPos();
                        ObstaclePub(o_obs_pos, o_robot_pos);
                    }
                }
                f_obs_speed_vel = o_lidar_obstacle_check_.PredictObstacleCheck(
                    st_param, o_polygon, vec_sensors_relative_robot, b_arrive_check, b_docking_path_obs_check, regenerated_st_vel,
                    b_diagonal_path);
                if (f_obs_speed_vel != 1) {
                    std_msgs::String msg;
                    msg.data = "[mo]obs_predict";
                    pub_warning_.publish(msg);

                    Pos o_obs_pos = o_lidar_obstacle_check_.GetObsPos();
                    ObstaclePub(o_obs_pos, o_robot_pos);
                }
                if (f_obs_speed_vel > f_obs_speed_vel_pre) {
                    n_obs_cnt++;
                    if (n_obs_cnt >= 5) {
                        f_obs_speed_vel = f_obs_speed_vel_pre + (1.0 / float(st_param.n_predict_step_num));
                    }
                    else {
                        f_obs_speed_vel = f_obs_speed_vel_pre;
                    }
                }
                else {
                    n_obs_cnt = 0;
                }
                if (f_obs_target_speed_vel == f_path_vel && f_obs_speed_vel == 1) {
                    NaviFra::Polygon o_polygon_empty;
                    Pos o_obs_pos1(+0.02, +0.02, 0);
                    Pos o_obs_pos2(-0.02, +0.02, 0);
                    Pos o_obs_pos3(-0.02, -0.02, 0);
                    Pos o_obs_pos4(+0.02, -0.02, 0);

                    o_polygon_empty.AddVertex(o_obs_pos1);
                    o_polygon_empty.AddVertex(o_obs_pos2);
                    o_polygon_empty.AddVertex(o_obs_pos3);
                    o_polygon_empty.AddVertex(o_obs_pos4);

                    geometry_msgs::PolygonStamped msg_collision = DrawPolygon(o_polygon_empty.o_polygon_vertexs_);

                    pub_obs_pos_.publish(msg_collision);
                }

                {
                    std::lock_guard<std::mutex> lock(mtx_vel_);
                    f_obs_speed_vel_ = (f_obs_speed_vel + f_obs_speed_vel_pre) / 2;
                    f_obs_target_speed_vel_ = f_obs_target_speed_vel;
                }
                f_obs_speed_vel_pre = f_obs_speed_vel;
            }
            std::chrono::duration<double> sec_check2 = tp_check_time_ - tp_detect_time_;
            if (sec_check2.count() > st_param.f_resume_start_delay && AutoMoveMission::GetStatus() == MISSION_STATUS::SUSPENDING) {
                AutoMoveMission::ResumeMission();
            }
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << '\n';
        }
    }
}

void AutoMoveMission::SetReferenceVelocity(vector<Pos>& vec_path)
{
    // set reference velocity on two elements
    // 1. ui user path velocity
    // 2. parameter near goal reference velocity
    // 3. curvature based reference velocity

    float f_total_path_dist = 0.0f, f_traveled_dist = 0.0f;
    const int n_path_size = vec_path.size();
    const float f_min_vel = st_param_.f_near_goal_speed_thres;
    const float f_real_min_vel = (vec_path.at(n_path_size - 2).GetConstDriveInfo().b_stop_smooth == false)
        ? f_min_vel
        : st_param_.st_motion_param.f_linear_speed_min_ms;
    for (int i = 1; i < vec_path.size(); i++) {
        f_total_path_dist += CoreCalculator::CalcPosDistance_(vec_path.at(i - 1), vec_path.at(i));
    }

    Pos::DriveInfo_t st_drive_info = vec_path.at(n_path_size - 2).GetConstDriveInfo();

    for (int i = 0; i < vec_path.size() - 1; i++) {
        f_traveled_dist += CoreCalculator::CalcPosDistance_(vec_path.at(i), vec_path.at(i + 1));
        float f_left_dist = f_total_path_dist - f_traveled_dist;
        const float f_path_vel_ms = vec_path.at(i).GetConstDriveInfo().f_linear;
        if (isless(f_left_dist, st_param_.f_goal_arrive_min_vel_dist_m)) {
            vec_path.at(i).GetDriveInfo().f_linear = f_real_min_vel;
        }
        else if (isless(f_left_dist, st_param_.f_near_goal_dist_thres) && st_drive_info.b_stop_smooth) {
            vec_path.at(i).GetDriveInfo().f_linear = f_min_vel;
        }

        if (std::isgreater(vec_path.at(i).GetDriveInfo().f_linear, f_path_vel_ms)) {
            vec_path.at(i).GetDriveInfo().f_linear = f_path_vel_ms;
        }
    }
    vec_path.back().GetDriveInfo().f_linear = min(f_min_vel, vec_path.back().GetConstDriveInfo().f_linear);

    return;
}

float AutoMoveMission::CalcFrontDistanceOfCurrentPath(float f_robot_speed)
{
    Parameters_t st_param;
    {
        std::lock_guard<std::mutex> lock(mtx_param_);
        st_param = st_param_;
    }
    float f_min_front_dist = 0.5f + st_param.st_safety_motion_param.f_safety_dist_from_obs_m +
        0.5f * f_robot_speed * f_robot_speed / st_param_.st_safety_motion_param.f_goal_decel_mss +
        std::fabs(f_robot_speed) * static_cast<float>(st_param_.n_path_planner_period_ms) / 1000.f;
    if (f_min_front_dist < st_param.f_min_dist_for_forward_path_m)
        f_min_front_dist = st_param.f_min_dist_for_forward_path_m;
    return f_min_front_dist;
}

void AutoMoveMission::SetFullSm(vector<Pos>& vec_path)
{
    vec_path.at(0).SetZm(0.f);
    for (int i = 1; i < (int)vec_path.size(); ++i) {
        float dx = vec_path.at(i).GetXm() - vec_path.at(i - 1).GetXm();
        float dy = vec_path.at(i).GetYm() - vec_path.at(i - 1).GetYm();
        vec_path.at(i).SetZm(vec_path.at(i - 1).GetZm() + std::sqrt(dx * dx + dy * dy));
    }
}

void AutoMoveMission::SetCurvature(vector<Pos>& vec_path)
{
    for (int i = 0; i < (int)vec_path.size() - 1; ++i) {
        float ds = vec_path.at(i + 1).GetZm() - vec_path.at(i).GetZm();
        if (ds < 0.00001)
            continue;
        float da = CoreCalculator::CalcAngleDomainRad_(vec_path.at(i + 1).GetRad() - vec_path.at(i).GetRad());
        vec_path.at(i).GetDriveInfo().f_curvature = da / ds;
    }
    vec_path.back().GetDriveInfo().f_curvature = vec_path.at(vec_path.size() - 2).GetDriveInfo().f_curvature;
}

void AutoMoveMission::ClearCurrentPath()
{
    {
        std::lock_guard<std::mutex> lock(mtx_current_path_);
        vec_current_path_.clear();
    }
    LOG_INFO("[AutoMoveMission::ClearCurrentPath]");
}

void AutoMoveMission::SetCurrentPathFromMissionPath(float f_robot_speed)
{
    // current path 경로 길이 계산
    float f_rear_dist = 1.0f;
    float f_front_dist = CalcFrontDistanceOfCurrentPath(f_robot_speed);
    int n_start_idx = std::max(n_mission_path_idx_ - 1, 0);
    int n_end_idx = std::min(n_mission_path_idx_ + 1, (int)vec_mission_path_.size());
    // calc start index of mission path for current path
    float f_sum_dist = 0;
    for (; n_start_idx > 0; --n_start_idx) {
        float f_dx = vec_mission_path_.at(n_start_idx + 1).GetXm() - vec_mission_path_.at(n_start_idx).GetXm();
        float f_dy = vec_mission_path_.at(n_start_idx + 1).GetYm() - vec_mission_path_.at(n_start_idx).GetYm();
        f_sum_dist += std::sqrt(f_dx * f_dx + f_dy * f_dy);
        if (f_sum_dist > f_rear_dist)
            break;
    }
    // calc end index of mission path for current path
    f_sum_dist = 0;
    for (; n_end_idx < (int)vec_mission_path_.size(); ++n_end_idx) {
        float f_dx = vec_mission_path_.at(n_end_idx).GetXm() - vec_mission_path_.at(n_end_idx - 1).GetXm();
        float f_dy = vec_mission_path_.at(n_end_idx).GetYm() - vec_mission_path_.at(n_end_idx - 1).GetYm();
        f_sum_dist += std::sqrt(f_dx * f_dx + f_dy * f_dy);
        if (f_sum_dist > f_front_dist) {
            ++n_end_idx;
            break;
        }
    }

    std::vector<Pos> vec_current_path;
    vec_current_path.assign(vec_mission_path_.begin() + n_start_idx, vec_mission_path_.begin() + n_end_idx);
    float s0 = vec_current_path.at(0).GetZm();
    for (int i = 0; i < (int)vec_current_path.size(); ++i) {
        vec_current_path.at(i).SetZm(vec_current_path.at(i).GetZm() - s0);
    }

    // 전역변수에 전파
    {
        std::lock_guard<std::mutex> lock(mtx_current_path_);
        vec_current_path_ = vec_current_path;
    }

    LOG_INFO("n_mission_path_idx_ : %d, n_start_idx : %d, n_end_idx: %d", n_mission_path_idx_, n_start_idx, n_end_idx);
    LOG_INFO("vec_mission_path_ size : %d", (int)vec_mission_path_.size());
    LOG_INFO("vec_current_path_ size : %d", (int)vec_current_path_.size());
}

void AutoMoveMission::SetCurrentPathFromAvoidPath(int n_cut_idx)
{
    vector<Pos> vec_current_path;
    {
        std::lock_guard<std::mutex> lock(mtx_current_path_);
        vec_current_path = vec_current_path_;
    }

    vec_current_path.erase(vec_current_path.begin() + n_cut_idx, vec_current_path.end());
    int n_last_pos_idx = (int)vec_current_path.size() - 1;
    vec_current_path.insert(vec_current_path.end(), vec_avoid_path_.begin(), vec_avoid_path_.end());
    if (n_cut_idx != 0) {
        const Pos& o_last_pos = vec_current_path.at(n_last_pos_idx);
        float dx = vec_avoid_path_.at(0).GetXm() - o_last_pos.GetXm();
        float dy = vec_avoid_path_.at(0).GetYm() - o_last_pos.GetYm();
        float s0 = o_last_pos.GetZm() - vec_avoid_path_.at(0).GetZm() + std::sqrt(dx * dx + dy * dy);
        for (int i = n_cut_idx; i < (int)vec_current_path.size(); ++i) {
            vec_current_path.at(i).SetZm(vec_current_path.at(i).GetZm() + s0);
            if (vec_current_path.at(i).GetConstDriveInfo().b_avoid_status &&
                vec_current_path.at(i).GetConstDriveInfo().f_linear > st_param_.f_avoid_vel_ms) {
                vec_current_path.at(i).GetDriveInfo().f_linear = st_param_.f_avoid_vel_ms;
            }
        }
    }

    // 전역변수에 전파
    {
        std::lock_guard<std::mutex> lock(mtx_current_path_);
        vec_current_path_ = vec_current_path;
    }
}

void AutoMoveMission::UpdateCurrentPath(int n_current_path_idx, float f_robot_speed)
{
    // 만약 vec_mission_path_ 에서 로봇이 가장 가까운 waypoint가 마지막 점이면 무시
    if (n_mission_path_idx_ == (int)vec_mission_path_.size() - 1)
        return;

    // vec_current_path_ 복사
    std::vector<Pos> vec_current_path = vec_current_path_;

    // 현재경로의 마지막 waypoint idx를 계산
    bool b_closed = false;
    int n_final_current_in_mission_idx = n_mission_path_idx_;
    float f_last_ds_sq = 1000.f;
    for (n_final_current_in_mission_idx = n_mission_path_idx_; n_final_current_in_mission_idx < (int)vec_mission_path_.size();
         ++n_final_current_in_mission_idx) {
        float dx = vec_mission_path_.at(n_final_current_in_mission_idx).GetXm() - vec_current_path.back().GetXm();
        float dy = vec_mission_path_.at(n_final_current_in_mission_idx).GetYm() - vec_current_path.back().GetYm();
        float ds_sq = dx * dx + dy * dy;
        if (!b_closed && ds_sq < 0.000025) {
            b_closed = true;
        }
        else if (b_closed && f_last_ds_sq < ds_sq) {
            --n_final_current_in_mission_idx;
            break;
        }
        f_last_ds_sq = ds_sq;
    }

    // 만약 현재경로의 마지막 waypoint가 미션경로의 마지막 index 라면 무시
    if (n_final_current_in_mission_idx > (int)vec_mission_path_.size() - 2)
        return;

    // current path 경로 길이 계산
    float f_front_dist = CalcFrontDistanceOfCurrentPath(f_robot_speed);

    // 연장해야할 앞부분 길이 계산
    f_front_dist = f_front_dist - vec_current_path_.back().GetZm() + vec_current_path_.at(n_current_path_idx).GetZm();

    // f_front_dist 가 1cm 보다 짧으면 무시
    if (f_front_dist < 0.01)
        return;

    // vec_current_path_ 에서 vec_mission_path_ 로부터 연장할 경로의 마지막 index 계산
    int n_start_idx = n_final_current_in_mission_idx + 1;
    int n_end_idx = n_start_idx;
    for (; n_end_idx < (int)vec_mission_path_.size(); ++n_end_idx) {
        float f_dx = vec_mission_path_.at(n_end_idx).GetXm() - vec_mission_path_.at(n_end_idx - 1).GetXm();
        float f_dy = vec_mission_path_.at(n_end_idx).GetYm() - vec_mission_path_.at(n_end_idx - 1).GetYm();
        float f_tmp_dist = std::sqrt(f_dx * f_dx + f_dy * f_dy);
        f_front_dist -= f_tmp_dist;
        if (f_front_dist < 0)
            break;
    }

    // 만약 연장할 index의 시작과 끝이 같다면 무시
    if (n_start_idx == n_end_idx)
        return;

    // vec_current_path_ 연장
    vec_current_path.insert(vec_current_path.end(), vec_mission_path_.begin() + n_start_idx, vec_mission_path_.begin() + n_end_idx);

    // 각 waypoint의 거리 계산
    float dx = vec_mission_path_.at(n_start_idx).GetXm() - vec_current_path_.back().GetXm();
    float dy = vec_mission_path_.at(n_start_idx).GetYm() - vec_current_path_.back().GetYm();
    float s0 = vec_current_path_.back().GetZm() - vec_mission_path_.at(n_start_idx).GetZm() + std::sqrt(dx * dx + dy * dy);
    for (int i = (int)vec_current_path_.size(); i < (int)vec_current_path.size(); ++i) {
        vec_current_path.at(i).SetZm(vec_current_path.at(i).GetZm() + s0);
    }

    // 앞부분 제거 (50)
    int n_erase_num = 0;
    if (n_current_path_idx > 50) {
        n_erase_num = n_current_path_idx - 50;
        vec_current_path.erase(vec_current_path.begin(), vec_current_path.begin() + n_erase_num);
    }

    // 전역변수에 전파
    {
        // NLOG(info) << "udpate current path " << vec_current_path_.size();
        std::lock_guard<std::mutex> lock(mtx_current_path_);
        vec_current_path_ = vec_current_path;
    }
}

void AutoMoveMission::ExtendCurrentPathFromAvoidPath(const vector<Pos>& vec_extension_path)
{
    if (vec_extension_path.empty()) {
        LOG_DEBUG("[AutoMoveMission::ExtendCurrentPathFromAvoidPath] ExtendAvoidPath Empty");
        return;
    }
    vector<Pos> vec_current_path = vec_current_path_;
    vec_current_path.insert(vec_current_path.end(), vec_extension_path.begin(), vec_extension_path.end());
    float dx = vec_extension_path.at(0).GetXm() - vec_current_path_.back().GetXm();
    float dy = vec_extension_path.at(0).GetYm() - vec_current_path_.back().GetYm();
    float s0 = vec_current_path_.back().GetZm() - vec_extension_path.at(0).GetZm() + std::sqrt(dx * dx + dy * dy);
    for (int i = vec_current_path_.size(); i < (int)vec_current_path.size(); ++i) {
        vec_current_path.at(i).SetZm(vec_current_path.at(i).GetZm() + s0);
        if (vec_current_path.at(i).GetConstDriveInfo().b_avoid_status &&
            vec_current_path.at(i).GetConstDriveInfo().f_linear > st_param_.f_avoid_vel_ms) {
            vec_current_path.at(i).GetDriveInfo().f_linear = st_param_.f_avoid_vel_ms;
        }
    }

    // 전역변수에 전파
    {
        std::lock_guard<std::mutex> lock(mtx_current_path_);
        vec_current_path_ = vec_current_path;
    }
}

int AutoMoveMission::CalcPredRobotPos(const vector<Pos>& vec_path, const Pos& o_robot_pos_instance, float f_robot_speed, float dt)
{
    int n_path_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_robot_pos_instance, vec_path);
    float f_ds = f_robot_speed * dt;
    return CoreCalculator::FindIndexAtDistance_(vec_path, n_path_idx, f_ds);
}

int AutoMoveMission::CalcPredRobotPos(const vector<Pos>& vec_path, int n_path_idx, float f_robot_speed, float dt)
{
    float f_ds = f_robot_speed * dt;
    return CoreCalculator::FindIndexAtDistance_(vec_path, n_path_idx, f_ds);
}

void AutoMoveMission::MotionControl()
{
    while (ros::ok()) {
        int n_debug_line = __LINE__;
        static int n_count = 0;
        if (n_count++ > 1000) {
            n_count = 0;
            NLOG(info) << "MotionControl is running...";
        }
        tp_motion_control_ = std::chrono::steady_clock::now();
        int n_motion_controller_period_ms = int(st_param_.f_motion_control_period * 1000);
        std::this_thread::sleep_for(std::chrono::milliseconds(n_motion_controller_period_ms));

        if (!b_thread_run_)
            continue;

        try {
            // 현재 미션 경로 받기
            n_debug_line = __LINE__;
            Parameters_t st_param;
            {
                std::lock_guard<std::mutex> lock(mtx_param_);
                st_param = st_param_;
            }
            n_debug_line = __LINE__;
            static std::vector<float> f_avg_dist;
            MotionInfo_t st_info = o_motion_controller_.GetMotionInfo();
            st_info.s_avoid_status = GetAvoidStatus();

            std::chrono::duration<double> sec_start = std::chrono::steady_clock::now() - tp_start_time_;
            if (GetUserPauseFlag()) {  // 사용자의 일시정지
                StopMotion();
                SetMotionInfo(st_info);
                continue;
            }
            else if (GetStatus() == MISSION_STATUS::END) {  // 미션 종료
                LOG_INFO("MISSION_STATUS::END");
                SetMotionInfo(st_info);
                continue;
            }
            else if (sec_start.count() < st_param.f_move_deleay_time_sec)  // 0.3 sec_start delay start
            {
                LOG_INFO("%.3f sec delay start, %.3f", st_param.f_move_deleay_time_sec, sec_start.count());
                SetMotionInfo(st_info);
                f_avg_dist.clear();
                continue;
            }
            else {
                o_motion_controller_.StartMotion();
            }

            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            n_debug_line = __LINE__;
            CommandVelocity_t st_vel;

            // vec_current_path 받기
            std::vector<NaviFra::Pos> vec_current_path;
            {
                std::lock_guard<std::mutex> lock(mtx_current_path_);
                vec_current_path = vec_current_path_;
            }
            std::vector<NaviFra::Pos> vec_mission_path;
            {
                std::lock_guard<std::mutex> lock(mtx_mission_path_);
                vec_mission_path = vec_mission_path_;
            }
            // if ((int)vec_current_path.size() < 3) {
            //     if (o_rapid_timer_.Over("MC_short", 0.2)) {
            //         NLOG(error) << "[AutoMoveMission::MotionControllerBehavior] Too short path (" << (int)vec_current_path.size() << ")";
            //     }
            //     StopMotion();
            //     continue;
            // }

            // 만약 충돌했다면 vec_current_path 자르기
            // int n_safty_stop_idx = n_current_path_collision_idx_;
            // if (n_safty_stop_idx != -1 && o_rapid_timer_.Over("MC_Collision", 0.2))
            //     LOG_INFO("[CHECK_COLLISION(MC_BT)] Path Size: %d, Collision Idx: %d", (int)vec_current_path.size(), n_safty_stop_idx);
            // if (n_safty_stop_idx == 0) {
            //     if (o_rapid_timer_.Over("MC_CollEstop", 0.2)) {
            //         NLOG(warning) << "[AutoMoveMission::MotionControllerBehavior] Collision EStop";
            //     }
            //     StopMotion();
            //     continue;
            // }
            // if (o_motion_controller_.b_need_to_start_align_ == false)  //처음 ALIGNING일 때 무시
            // {
            //     if (n_safty_stop_idx != -1) {
            //         // 충돌 지점으로부터 안전거리만큼 떨어진 index까지 자르기
            //         float f_sum_dist = 0;
            //         for (; n_safty_stop_idx > 0; --n_safty_stop_idx) {
            //             float f_dx = vec_mission_path_.at(n_safty_stop_idx).GetXm() - vec_mission_path_.at(n_safty_stop_idx - 1).GetXm();
            //             float f_dy = vec_mission_path_.at(n_safty_stop_idx).GetYm() - vec_mission_path_.at(n_safty_stop_idx - 1).GetYm();
            //             f_sum_dist += std::sqrt(f_dx * f_dx + f_dy * f_dy);
            //             if (f_sum_dist > st_param.st_safety_motion_param.f_safety_dist_from_obs_m)
            //                 break;
            //         }

            //         vec_current_path.erase(vec_current_path.begin() + n_safty_stop_idx, vec_current_path.end());
            //     }
            // }

            // if ((int)vec_current_path.size() < 3) {
            //     if (o_rapid_timer_.Over("MC_CollEstop", 0.2)) {
            //         NLOG(warning) << "[AutoMoveMission::MotionControllerBehavior] Collision EStop";
            //     }
            //     StopMotion();
            //     continue;
            // }

            n_debug_line = __LINE__;
            // PathDivider 에서 나눠진 Path 중에서 현재 Path 를 로드한다.
            NaviFra::PathDescription& o_selected_path_desc = vec_current_mission_;
            std::vector<NaviFra::Pos>& vec_selected_local_path = o_selected_path_desc.vec_path;
            f_robot_yaw_bias_ = o_selected_path_desc.f_yaw_bias;

            NaviFra::Pos o_selected_local_goal = o_selected_path_desc.o_goal_pos;
            o_selected_local_goal.SetRad(o_selected_local_goal.GetRad() + f_robot_yaw_bias_);
            n_debug_line = __LINE__;
            // 로봇이 후진 / 전진하는 경우를 참조형 인스턴스로 선택 -> o_robot_pos_instance 생성
            NaviFra::Pos o_robot_pos_instance;
            {
                std::lock_guard<std::mutex> lock(mtx_robot_pos_);
                o_robot_pos_instance = o_robot_pos_;
            }
            o_robot_pos_instance.SetRad(o_robot_pos_instance.GetRad() + f_robot_yaw_bias_);
            n_debug_line = __LINE__;
            if (GetStatus() == MISSION_STATUS::SUSPENDING) {  // 미션 종료
                StopMotion();
                o_cb_regiser_.notify(NAVI_STATUS_CALLBACK, static_cast<int>(NAVIGATION_STATUS::OBS_STOP));
                st_info.n_obs_control = 2;
                SetMotionInfo(st_info);
                continue;
            }

            int n_local_path_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_robot_pos_instance, vec_current_path);
            n_debug_line = __LINE__;

            if (vec_current_path.size() > 0) {
                b_avoid_status_ = vec_current_path.at(n_local_path_idx).GetConstDriveInfo().b_avoid_status;
            }

            n_debug_line = __LINE__;

            // 회피 중일때만 로컬 경로 pub
            static bool b_avoided = false;
            if (b_avoid_status_ || b_avoided) {
                std::chrono::duration<float> sec_path_pub = std::chrono::steady_clock::now() - tp_local_path_pub_time_;
                f_pub_interval_ = 0.2;
                if (sec_path_pub.count() >= f_pub_interval_) {
                    PubLocalPath(vec_current_path);
                }
                b_avoided = true;
            }
            // 회피 끝날때 한번만 로컬 경로 pub
            if (!b_avoid_status_ && b_avoided) {
                int n_mission_path_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_robot_pos_instance, vec_mission_path);
                float f_dist = CoreCalculator::CalcPosDistance_(o_robot_pos_instance, vec_mission_path.at(n_mission_path_idx));
                // NLOG(info) << "f_dist " << f_dist;
                if (f_dist < 0.2) {
                    PubLocalPath(vec_mission_path_);
                    b_avoided = false;
                }
            }

            n_debug_line = __LINE__;

            SensorMsg_t st_sensor_msg;
            {
                std::lock_guard<std::mutex> lock(mtx_sensor_);
                st_sensor_msg = st_sensor_msg_;
            }

            n_debug_line = __LINE__;
            o_motion_controller_.SetLocalPath(vec_current_path);

            n_debug_line = __LINE__;
            st_vel = o_motion_controller_.GenerateCommandVelocity(
                o_robot_pos_instance, vec_current_mission_.o_goal_pos, st_sensor_msg.vec_sensors_relative_robot);
            st_vel.b_reverse_motion = b_move_backward_flag_;
            st_info.b_reverse_motion = st_vel.b_reverse_motion;
            st_info.n_motion_status = st_vel.n_status;
            n_debug_line = __LINE__;

            if (st_vel.n_status == MotionManager::MOTION_STATUS::HEADING_ALIGNING)  // spinturn인 경우
            {
                st_navi_info_.b_spin_turn = true;
            }
            else {
                st_navi_info_.b_spin_turn = false;
            }

            if (b_move_backward_flag_) {
                st_vel.f_linear_speed_x_ms *= -1;
                st_vel.f_linear_speed_y_ms *= -1;
            }

            float f_obs_speed_ratio;
            float f_obs_target_speed_vel;
            {
                std::lock_guard<std::mutex> lock(mtx_vel_);
                f_obs_speed_ratio = f_obs_speed_vel_;
                f_obs_target_speed_vel = f_obs_target_speed_vel_;
            }
            if (f_obs_speed_ratio != 1) {
                st_info.n_obs_control = 1;
            }
            st_info.f_obs_speed_ratio = f_obs_speed_ratio;
            float f_obs_linear_speed = hypot(st_vel.f_linear_speed_x_ms, st_vel.f_linear_speed_y_ms) * f_obs_speed_ratio;
            if (f_obs_linear_speed >= st_param.st_motion_param.f_linear_speed_min_ms &&
                f_obs_target_speed_vel > st_param.st_motion_param.f_linear_speed_min_ms) {
                st_vel.f_linear_speed_x_ms *= f_obs_speed_ratio;
                st_vel.f_linear_speed_y_ms *= f_obs_speed_ratio;
                if (f_obs_linear_speed > f_obs_target_speed_vel) {
                    st_vel.f_linear_speed_x_ms *= f_obs_target_speed_vel / f_obs_linear_speed;
                    st_vel.f_linear_speed_y_ms *= f_obs_target_speed_vel / f_obs_linear_speed;
                }
            }
            else {
                if (f_obs_linear_speed == 0)
                    f_obs_linear_speed = 1;
                st_vel.f_linear_speed_x_ms =
                    (st_param.st_motion_param.f_linear_speed_min_ms * st_vel.f_linear_speed_x_ms * f_obs_speed_ratio) / f_obs_linear_speed;
                st_vel.f_linear_speed_y_ms =
                    (st_param.st_motion_param.f_linear_speed_min_ms * st_vel.f_linear_speed_y_ms * f_obs_speed_ratio) / f_obs_linear_speed;
            }
            st_vel.f_angular_speed_degs *= f_obs_speed_ratio;

            if(b_force_goal_arrived_) {
                st_vel.n_status = MotionManager::MOTION_STATUS::ARRIVED_AT_GOAL;
                b_force_goal_arrived_ = false;
            }

            float f_avoid_decel_ratio;
            {
                std::lock_guard<std::mutex> lock(mtx_avoid_vel_);
                f_avoid_decel_ratio = f_avoid_decel_ratio_;
            }

            n_debug_line = __LINE__;

            if (f_avoid_decel_ratio != 1) {
                float f_avoid_target_vel = st_param.f_avoid_check_vel_ms;
                float f_avoid_linear_speed = hypot(st_vel.f_linear_speed_x_ms, st_vel.f_linear_speed_y_ms) * f_avoid_decel_ratio;

                if (f_avoid_linear_speed >= st_param.st_motion_param.f_linear_speed_min_ms &&
                    f_avoid_target_vel > st_param.st_motion_param.f_linear_speed_min_ms) {
                    st_vel.f_linear_speed_x_ms *= f_avoid_decel_ratio;
                    st_vel.f_linear_speed_y_ms *= f_avoid_decel_ratio;
                    if (f_avoid_linear_speed > f_avoid_target_vel) {
                        st_vel.f_linear_speed_x_ms *= f_avoid_target_vel / f_avoid_linear_speed;
                        st_vel.f_linear_speed_y_ms *= f_avoid_target_vel / f_avoid_linear_speed;
                    }
                }
                else {
                    if (f_avoid_linear_speed == 0)
                        f_avoid_linear_speed = 1;
                    st_vel.f_linear_speed_x_ms =
                        (st_param.st_motion_param.f_linear_speed_min_ms * st_vel.f_linear_speed_x_ms * f_avoid_decel_ratio) /
                        f_avoid_linear_speed;
                    st_vel.f_linear_speed_y_ms =
                        (st_param.st_motion_param.f_linear_speed_min_ms * st_vel.f_linear_speed_y_ms * f_avoid_decel_ratio) /
                        f_avoid_linear_speed;
                }
                st_vel.f_angular_speed_degs *= f_avoid_decel_ratio;
                st_info.n_obs_control = -1;
            }
            n_debug_line = __LINE__;

            {
                std::lock_guard<std::mutex> lock(mtx_vel_);
                st_vel_ex_ = st_vel;
            }

            n_debug_line = __LINE__;

            if (st_vel.n_status == MotionManager::MOTION_STATUS::ARRIVED_AT_GOAL ||
                (vec_mission_path.size() == 1 && vec_mission_path.front().GetType() != Pos::NODE_TYPE::SPIN_TURN)) {
                o_camera_obstacle_check_.SetCameraState(false);
                LOG_INFO("MotionManager::MOTION_STATUS::ARRIVED_AT_GOAL");
                st_vel.f_linear_speed_x_ms = 0.0f;
                st_vel.f_linear_speed_y_ms = 0.0f;
                st_vel.f_angular_speed_degs = 0.0f;
                o_cb_regiser_.notify(static_cast<int>(NOTIFY_MSG::MISSION), AutoMoveMission::CB_MSG_MISSION_COMPLETED);

                SetStatus(MISSION_STATUS::END);
            }
            n_debug_line = __LINE__;

            static int n_log_count = 0;
            n_log_count++;
            if (n_log_count > 10) {
                n_log_count = 0;
                LOG_INFO(
                    "generate vel x %.3f, y %.3f, deg %.3f", st_vel.f_linear_speed_x_ms, st_vel.f_linear_speed_y_ms,
                    st_vel.f_angular_speed_degs);
            }
            n_debug_line = __LINE__;
            st_vel.b_return = b_docking_path_obs_check_;
            o_cb_regiser_.notify(GOAL_CALLBACK, st_vel);
            st_info.f_linear_speed_x_ms = st_vel.f_linear_speed_x_ms;
            st_info.f_linear_speed_y_ms = st_vel.f_linear_speed_y_ms;
            st_info.f_angular_speed_degs = st_vel.f_angular_speed_degs;

            /////////// For Monitoring ///////////
            std::chrono::duration<double> sec = std::chrono::steady_clock::now() - start;
            float f_total_exe_time_ms = sec.count() * 1000;
            if (f_total_exe_time_ms > int(st_param.f_motion_control_period * 1000.0f)) {
                LOG_WARNING("Execution time::%.3f(ms)", f_total_exe_time_ms);
                std_msgs::String msg;
                msg.data = "[mo]motion_delay";
                pub_warning_.publish(msg);
            }
            static vector<float> f_avg_ms;
            f_avg_ms.emplace_back(f_total_exe_time_ms);
            if (f_avg_ms.size() > 20) {
                f_avg_ms.erase(f_avg_ms.begin());
                float f_avg = 0;
                for (int k = 0; k < f_avg_ms.size(); k++)
                    f_avg += f_avg_ms.at(k);
                f_avg /= f_avg_ms.size();
                // NLOG(info)<<"f_total_exe_time_ms "<<f_avg;
                st_info.f_execution_time_sec = f_avg / 1000.0f;
            }
            n_debug_line = __LINE__;

            if (n_local_path_idx > 2) {
                f_avg_dist.emplace_back(CoreCalculator::CalcDistanceFromDotToLine_(
                    vec_current_path[n_local_path_idx], vec_current_path[n_local_path_idx - 3], o_robot_pos_instance));
            }
            n_debug_line = __LINE__;

            if (f_avg_dist.size() > 10) {
                f_avg_dist.erase(f_avg_dist.begin());
                float f_avg = 0;
                for (int k = 0; k < f_avg_dist.size(); k++)
                    f_avg += f_avg_dist.at(k);
                f_error_dist_ = f_avg / f_avg_dist.size();
                st_navi_info_.f_path_error_dist_m = f_error_dist_;
                static int n_f_error_dist_count = 0;

                if (f_error_dist_ > 0.05) {
                    n_f_error_dist_count++;
                    if (n_f_error_dist_count > 10) {
                        std_msgs::String msg;
                        msg.data = "[mo]path_error";
                        pub_warning_.publish(msg);
                    }
                }
                else {
                    n_f_error_dist_count = 0;
                }
            }

            n_debug_line = __LINE__;

            SetMotionInfo(st_info);

            n_debug_line = __LINE__;
        }
        catch (const std::exception& e) {
            NLOG(info) << e.what() << " / debug line : " << n_debug_line;
        }
    }
}

void AutoMoveMission::PubLocalPath(const vector<Pos>& vec_path)
{
    if (vec_path.size() > 0) {
        nav_msgs::Path path;
        path.header.seq = 0;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();
        static float f_ui_dist = 0;
        for (size_t i = 1; i < vec_path.size(); i++) {
            f_ui_dist += hypot(vec_path[i].GetXm() - vec_path[i - 1].GetXm(), vec_path[i].GetYm() - vec_path[i - 1].GetYm());
            if (f_ui_dist > 0.20 || (vec_path[i].GetConstDriveInfo().e_curve_type == Pos::CURVE && f_ui_dist > 0.05)) {
                f_ui_dist = 0;
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = vec_path[i].GetXm();
                pose.pose.position.y = vec_path[i].GetYm();
                path.poses.emplace_back(pose);
            }
        }
        // NLOG(info) << "Publish Local Path " << path.poses.size();

        static ros::Publisher localpath_pub_ = node_handle_.advertise<nav_msgs::Path>("/NaviFra/visualize/ui_local_path", 50, true);
        localpath_pub_.publish(path);
        tp_local_path_pub_time_ = std::chrono::steady_clock::now();
    }
}

int AutoMoveMission::ToBehaviorStatus(int n_motion_status)
{
    int n_status = static_cast<int>(NAVIGATION_STATUS::RUNNING);
    switch (n_motion_status) {
        case MotionManager::MOTION_STATUS::NORMAL:
            n_status = static_cast<int>(NAVIGATION_STATUS::RUNNING);
            break;
        case MotionManager::MOTION_STATUS::OBSTACLE:
            n_status = static_cast<int>(NAVIGATION_STATUS::OBS_STOP);
            break;
        case MotionManager::MOTION_STATUS::AWAY_FROM_PATH:
            n_status = static_cast<int>(NAVIGATION_STATUS::ERROR);
            break;
    }
    return n_status;
}

void AutoMoveMission::PathPlannerBehavior()
{
    while (ros::ok()) {
        // NLOG(info) << "[AutoMoveMission::PathPlannerBehavior] Thread";
        static int n_count = 0;
        if (n_count++ > 1000) {
            n_count = 0;
            NLOG(info) << "PathPlannerBehavior is running...";
        }
        tp_path_planner_ = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds(st_param_.n_path_planner_period_ms));
        if (!b_thread_run_)
            continue;
        int n_debug_line = __LINE__;

        try {
            // NLOG(info) << "[AutoMoveMission::PathPlannerBehavior] Start";
            if (vec_current_path_.size() > 2) {
                // o_rapid_timer_.ShowTime("PP_BT", 2.0);
                Parameters_t st_param;
                {
                    std::lock_guard<std::mutex> lock(mtx_param_);
                    st_param = st_param_;
                }
                n_debug_line = __LINE__;
                std::string s_avoid_status;

                std::chrono::duration<double> sec_start = std::chrono::steady_clock::now() - tp_start_time_;
                if (GetUserPauseFlag()) {  // 사용자의 일시정지
                    StopMotion();
                    continue;
                }
                static int n_log_count = 0;
                n_log_count++;
                if (GetStatus() == MISSION_STATUS::IDLE) {
                    if (n_log_count > 100) {
                        n_log_count = 0;
                        LOG_INFO("[AutoMoveMission::PathPlannerBehavior] MISSION_STATUS::IDLE");
                    }
                    // StopMotion();
                    continue;
                }

                if (GetStatus() == MISSION_STATUS::END) {  // 미션 종료
                    if (n_log_count > 100) {
                        n_log_count = 0;
                        LOG_INFO("[AutoMoveMission::PathPlannerBehavior] MISSION_STATUS::END");
                    }
                    StopMotion();
                    continue;
                }

                if (GetStatus() == MISSION_STATUS::SUSPENDING) {
                    if (n_log_count > 100) {
                        n_log_count = 0;
                        LOG_INFO("[AutoMoveMission::PathPlannerBehavior] MISSION_STATUS::SUSPENDING");
                    }
                    StopMotion();
                    o_cb_regiser_.notify(NAVI_STATUS_CALLBACK, static_cast<int>(NAVIGATION_STATUS::OBS_STOP));
                    continue;
                }
                n_debug_line = __LINE__;
                if (sec_start.count() < st_param.f_move_deleay_time_sec)  // 0.3 sec_start delay start
                {
                    LOG_DEBUG(
                        "[AutoMoveMission::PathPlannerBehavior] %.3f sec delay start, %.3f", st_param.f_move_deleay_time_sec,
                        sec_start.count());
                    continue;
                }

                if (vec_current_path_.empty()) {
                    if (o_rapid_timer_.Over("MC_emptypath", 0.5)) {
                        NLOG(warning) << "[AutoMoveMission::PathPlannerBehavior] vec_current_path_ is empty";
                    }
                    NLOG(warning) << "[AutoMoveMission::PathPlannerBehavior] vec_current_path_ is empty";
                    continue;
                }
                n_debug_line = __LINE__;
                // 현재 로봇 포즈 받기 (만약 후진 모션이면 각도에 180도 더하기)
                NaviFra::Pos o_robot_pos_instance;
                {
                    std::lock_guard<std::mutex> lock(mtx_robot_pos_);
                    o_robot_pos_instance = o_robot_pos_;
                }
                if (b_move_backward_flag_) {
                    o_robot_pos_instance.SetRad(o_robot_pos_instance.GetRad() + M_PI);  // 확인필요
                }
                // 현재 로봇 속도 받기
                NaviFra::Pos o_robot_vel;
                {
                    std::lock_guard<std::mutex> lock(mtx_robot_speed_);
                    o_robot_vel = o_robot_vel_;
                }
                float f_robot_speed = std::sqrt(o_robot_vel.GetXm() * o_robot_vel.GetXm() + o_robot_vel.GetYm() * o_robot_vel.GetYm());
                // 현재 미션 경로 받기
                std::vector<NaviFra::Pos> vec_mission_path;
                {
                    std::lock_guard<std::mutex> lock(mtx_mission_path_);
                    vec_mission_path = vec_mission_path_;
                }
                // 현재 미션 경로에서 로봇의 위치 인덱스 찾기
                int n_mission_path_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_robot_pos_instance, vec_mission_path);
                {
                    std::lock_guard<std::mutex> lock(mtx_path_idx_);
                    n_mission_path_idx_ = n_mission_path_idx;
                }
                if (n_mission_path_idx < 0 || n_mission_path_idx >= (int)vec_mission_path.size()) {
                    LOG_ERROR(
                        "[AutoMoveMission::PathPlannerBehavior] Fail to calc Path idx (%n) | Path size (%n)", n_mission_path_idx,
                        (int)vec_mission_path.size());
                    continue;
                }
                n_debug_line = __LINE__;
                // 로봇 위치 in vec_current_path_
                int n_current_path_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_robot_pos_instance, vec_current_path_);

                // 장애물 회피 허가 상태 받기
                bool b_avoid_permission = GetAvoidPermission();
                n_debug_line = __LINE__;
                if (!b_avoid_permission) {
                    UpdateCurrentPath(n_current_path_idx, f_robot_speed);
                    LOG_DEBUG("[AutoMoveMission::PathPlannerBehavior] Avoid Permission is false");
                    s_avoid_status = "Avoid Permission is false";
                    SetAvoidStatus(s_avoid_status);
                    continue;
                }
                n_debug_line = __LINE__;
                // 충돌위치 띄우기
                // if (n_current_path_collision_idx_ == -1) {
                // }
                // if (n_current_path_collision_idx_ != -1)
                // {
                //     float f_dpos_x = (vec_current_path_.at(n_current_path_collision_idx_).GetXm() - o_robot_pos_instance.GetXm()) *
                //     std::cos(o_robot_pos_instance.GetRad()) + (vec_current_path_.at(n_current_path_collision_idx_).GetYm() -
                //     o_robot_pos_instance.GetYm()) * std::sin(o_robot_pos_instance.GetRad()); float f_dpos_y
                //     =-(vec_current_path_.at(n_current_path_collision_idx_).GetXm() - o_robot_pos_instance.GetXm()) *
                //     std::sin(o_robot_pos_instance.GetRad()) + (vec_current_path_.at(n_current_path_collision_idx_).GetYm() -
                //     o_robot_pos_instance.GetYm()) * std::cos(o_robot_pos_instance.GetRad()); float f_dpos_theta =
                //     vec_current_path_.at(n_current_path_collision_idx_).GetRad() - o_robot_pos_instance.GetRad(); NaviFra::Pos
                //     o_coll_fl(f_dpos_x
                //     + (f_collision_box_max_xm + st_param.f_detect_margin) * std::cos(f_dpos_theta) - (f_collision_box_max_ym +
                //     st_param.f_detect_margin) * std::sin(f_dpos_theta), f_dpos_y + (f_collision_box_max_xm + st_param.f_detect_margin) *
                //     std::sin(f_dpos_theta) + (f_collision_box_max_ym + st_param.f_detect_margin) * std::cos(f_dpos_theta)); NaviFra::Pos
                //     o_coll_fr(f_dpos_x + (f_collision_box_max_xm + st_param.f_detect_margin) * std::cos(f_dpos_theta) -
                //     (f_collision_box_min_ym - st_param.f_detect_margin) * std::sin(f_dpos_theta), f_dpos_y + (f_collision_box_max_xm +
                //     st_param.f_detect_margin) * std::sin(f_dpos_theta) + (f_collision_box_min_ym - st_param.f_detect_margin) *
                //     std::cos(f_dpos_theta)); NaviFra::Pos o_coll_rl(f_dpos_x + (f_collision_box_min_xm - st_param.f_detect_margin) *
                //     std::cos(f_dpos_theta) - (f_collision_box_max_ym + st_param.f_detect_margin) * std::sin(f_dpos_theta), f_dpos_y +
                //     (f_collision_box_min_xm - st_param.f_detect_margin) * std::sin(f_dpos_theta) + (f_collision_box_max_ym +
                //     st_param.f_detect_margin) * std::cos(f_dpos_theta)); NaviFra::Pos o_coll_rr(f_dpos_x + (f_collision_box_min_xm -
                //     st_param.f_detect_margin) * std::cos(f_dpos_theta) - (f_collision_box_min_ym - st_param.f_detect_margin) *
                //     std::sin(f_dpos_theta), f_dpos_y + (f_collision_box_min_xm - st_param.f_detect_margin) * std::sin(f_dpos_theta) +
                //     (f_collision_box_min_ym - st_param.f_detect_margin) * std::cos(f_dpos_theta));
                // }

                // vec_mission_path_ 상에서 가까운 wp ~ 타겟 wp (dist: f_force_comeback_start_dist_m) 중 하나라도 lane avoidance가 아닌지
                // 체크
                int n_no_lane_idx = -1;
                bool b_forward_is_no_lane = false;
                float f_forward_is_no_lane_dist = st_param.st_safety_motion_param.f_safety_dist_from_obs_m + f_collision_box_max_xm + 1.0f;
                float f_big_no_lane_dist = f_forward_is_no_lane_dist > st_param.f_force_comeback_start_dist_m
                    ? f_forward_is_no_lane_dist
                    : st_param.f_force_comeback_start_dist_m;
                n_debug_line = __LINE__;
                int n_mission_forward_idx = CoreCalculator::FindIndexAtDistance_(vec_mission_path, n_mission_path_idx_, f_big_no_lane_dist);
                n_debug_line = __LINE__;
                for (std::vector<Pos>::const_iterator it = vec_mission_path.begin() + n_mission_path_idx;
                     it != vec_mission_path.begin() + n_mission_forward_idx; ++it) {
                    if (!it->GetConstDriveInfo().b_avoidance_left && !it->GetConstDriveInfo().b_avoidance_right) {
                        if (it->GetZm() - vec_mission_path.at(n_mission_path_idx).GetZm() < st_param.f_force_comeback_start_dist_m &&
                            n_no_lane_idx == -1) {
                            n_no_lane_idx = (int)(it - vec_mission_path.begin());
                            LOG_DEBUG("[AutoMoveMission::PathPlannerBehavior] n_no_lane_idx = %d", n_no_lane_idx);
                        }
                        if (it->GetZm() - vec_mission_path.at(n_mission_path_idx).GetZm() < f_forward_is_no_lane_dist &&
                            !b_forward_is_no_lane) {
                            LOG_DEBUG("[CollisionMargin] Forward is no lane, so reduce detect margin");
                            b_forward_is_no_lane = true;
                        }
                        if (n_no_lane_idx != -1 && b_forward_is_no_lane)
                            break;
                    }
                }
                n_debug_line = __LINE__;
                // 회피 lane에서만 장애물 탐색 및 회피 로직 동작
                // NLOG(info) << "n_current_path_collision_idx_ " << n_current_path_collision_idx_ << " / n_current_target_lane_num_ "
                //            << n_current_target_lane_num_;

                if (b_forward_is_no_lane) {
                    n_debug_line = __LINE__;
                    // NLOG(info) << "[AutoMoveMission::PathPlannerBehavior] Forward is no lane, so reduce detect margin";
                    UpdateCurrentPath(n_current_path_idx, f_robot_speed);
                    s_avoid_status = "No Avoid Lane";
                    SetAvoidStatus(s_avoid_status);
                    continue;
                }
                n_debug_line = __LINE__;
                SetAvoidStatus(s_avoid_status);
                n_debug_line = __LINE__;
                // 장애물 탐지
                if (n_current_target_lane_num_ == 0 && b_forward_is_no_lane) {
                    LOG_DEBUG(
                        "[CollisionMargin] f_detect_margin: %.3f -> %.3f", st_param.f_detect_margin,
                        st_param.f_origin_path_detect_obs_margin_m);
                    n_current_path_collision_idx_ = o_collision_detector_ptr_->CheckPathCollision(
                        vec_current_path_, st_param.f_origin_path_detect_obs_margin_m, b_move_backward_flag_, n_current_path_idx);
                }
                else {
                    n_current_path_collision_idx_ = o_collision_detector_ptr_->CheckPathCollision(
                        vec_current_path_, st_param.f_detect_margin, b_move_backward_flag_, n_current_path_idx);
                }
                // o_rapid_timer_.ShowTime("ObsDetection", 2.0);
                if (n_current_path_collision_idx_ != -1 && o_rapid_timer_.Over("PP_Coll", 0.2))
                    LOG_DEBUG(
                        "[CHECK_COLLISION(PP_BT)] Path Size: %d, Collision Idx: %d", (int)vec_current_path_.size(),
                        n_current_path_collision_idx_);
                n_debug_line = __LINE__;
                // 회피x -> 정적장애물 체크
                bool b_static_obs = false;
                {
                    std::lock_guard<std::mutex> lock(mtx_avoid_vel_);
                    f_avoid_decel_ratio_ = 1.0;
                }
                if (n_current_path_collision_idx_ == -1) {
                    o_static_obs_tmr_->Start();
                }
                else if (
                    st_param_.f_static_obs_check_dist_m <
                    std::hypot(
                        vec_current_path_.at(n_current_path_collision_idx_).GetXm() - o_last_collision_pos_.GetXm(),
                        vec_current_path_.at(n_current_path_collision_idx_).GetYm() - o_last_collision_pos_.GetYm())) {
                    LOG_INFO(
                        "[StaticObstacle] No static obstacle (last pos: %.3f, %.3f | curr pos: %.3f, %.3f)", o_last_collision_pos_.GetXm(),
                        o_last_collision_pos_.GetYm(), vec_current_path_.at(n_current_path_collision_idx_).GetXm(),
                        vec_current_path_.at(n_current_path_collision_idx_).GetYm());
                    o_last_collision_pos_.SetXm(vec_current_path_.at(n_current_path_collision_idx_).GetXm());
                    o_last_collision_pos_.SetYm(vec_current_path_.at(n_current_path_collision_idx_).GetYm());
                    o_static_obs_tmr_->Start();
                }
                else if (o_static_obs_tmr_->Over()) {
                    b_static_obs = true;
                    LOG_INFO(
                        "[StaticObstacle] Static obstacle (collision pos: %.3f, %.3f)", o_last_collision_pos_.GetXm(),
                        o_last_collision_pos_.GetYm());
                }
                // else {
                //     {
                //         std::lock_guard<std::mutex> lock(mtx_avoid_vel_);
                //         float f_dist = CoreCalculator::CalcPosDistance_(
                //             vec_current_path_.at(n_current_path_idx), vec_current_path_.at(n_current_path_collision_idx_));
                //         f_avoid_decel_ratio_ = std::exp(-0.1 * f_dist);
                //     }
                // }
                n_debug_line = __LINE__;
                Avoid_path_result st_avoid_path_result;
                std::chrono::duration<double> detection_time = std::chrono::steady_clock::now() - tp_detection_period_time_;
                bool b_detection_timer = (int)(detection_time.count() * 1000) > st_param.n_detection_period_ms;

                if (n_current_target_lane_num_ == 0) {
                    b_detection_timer = (int)(detection_time.count() * 1000) > st_param.n_detection_period_ms;
                }
                else {
                    b_detection_timer = (int)(detection_time.count() * 1000) > st_param.n_avoidance_period_ms;
                }
                n_debug_line = __LINE__;
                // 회피상태 o, 강제복귀 o
                if (n_current_target_lane_num_ != 0 && n_mission_path_idx >= n_force_comeback_start_idx_) {
                    n_debug_line = __LINE__;
                    int n_pred_current_idx = CalcPredRobotPos(vec_current_path_, o_robot_pos_instance, f_robot_speed, 0.5f);
                    if (n_pred_current_idx == 0)
                        n_pred_current_idx = 1;
                    Pos o_pred_pos = vec_current_path_.at(n_pred_current_idx);
                    float f_min_dist = CalcFrontDistanceOfCurrentPath(f_robot_speed);
                    std::chrono::steady_clock::time_point tp_avoid1 = std::chrono::steady_clock::now();
                    st_avoid_path_result.vec_avoid_path = o_lane_avoid_path_ptr_->ForceComebackPath(
                        vec_mission_path, o_pred_pos, vec_mission_path.at(n_force_comeback_target_idx_), st_lane_info_,
                        n_current_target_lane_num_, f_min_dist);
                    std::chrono::duration<double> sec_result1 = std::chrono::steady_clock::now() - tp_avoid1;
                    n_debug_line = __LINE__;
                    if (!st_avoid_path_result.vec_avoid_path.empty()) {
                        vec_avoid_path_ = st_avoid_path_result.vec_avoid_path;
                        n_current_target_lane_num_ = 0;
                        SetCurrentPathFromAvoidPath(n_pred_current_idx);
                        n_current_path_collision_idx_ = o_collision_detector_ptr_->CheckPathCollision(
                            vec_current_path_, st_param.f_detect_margin, b_move_backward_flag_, n_current_path_idx);
                        n_pp_bt_state_ = 1;
                        LOG_INFO("[AutoMoveMission::PathPlannerBehavior] Force Comeback Path, planning time : %.3f", sec_result1.count());
                        SetAvoidPermission(st_param.b_avoid_permission);
                        s_avoid_status = "Avoiding -> Finding ForceComebackPath Success";
                    }
                    else {
                        s_avoid_status = "Avoiding -> Finding ForceComebackPath Failed";
                    }
                    n_debug_line = __LINE__;
                }
                // 회피상태 o, no_lane o
                else if (n_no_lane_idx != -1 && n_current_target_lane_num_ != 0) {
                    n_debug_line = __LINE__;
                    int n_pred_current_idx = CalcPredRobotPos(vec_current_path_, o_robot_pos_instance, f_robot_speed, 0.5f);
                    if (n_pred_current_idx == 0)
                        n_pred_current_idx = 1;
                    Pos o_pred_pos = vec_current_path_.at(n_pred_current_idx);
                    n_debug_line = __LINE__;
                    float f_min_dist = CalcFrontDistanceOfCurrentPath(f_robot_speed);
                    std::chrono::steady_clock::time_point tp_avoid2 = std::chrono::steady_clock::now();
                    st_avoid_path_result.vec_avoid_path = o_lane_avoid_path_ptr_->ForceComebackPath(
                        vec_mission_path, o_pred_pos, vec_mission_path.at(n_no_lane_idx), st_lane_info_, n_current_target_lane_num_,
                        f_min_dist);
                    std::chrono::duration<double> sec_result2 = std::chrono::steady_clock::now() - tp_avoid2;
                    n_debug_line = __LINE__;
                    if (!st_avoid_path_result.vec_avoid_path.empty()) {
                        vec_avoid_path_ = st_avoid_path_result.vec_avoid_path;
                        n_current_target_lane_num_ = 0;
                        SetCurrentPathFromAvoidPath(n_pred_current_idx);
                        n_current_path_collision_idx_ = o_collision_detector_ptr_->CheckPathCollision(
                            vec_current_path_, st_param.f_detect_margin, b_move_backward_flag_, n_current_path_idx);
                        n_pp_bt_state_ = 1;
                        LOG_INFO(
                            "[AutoMoveMission::PathPlannerBehavior] Force Comeback Path for no_lane, planning time : %.3f",
                            sec_result2.count());
                        SetAvoidPermission(st_param.b_avoid_permission);
                        s_avoid_status = "Avoiding -> Finding ForceComebackPath For no_lane Success";
                    }
                    else {
                        s_avoid_status = "Avoiding -> Finding ForceComebackPath For no_lane Failed";
                    }
                    n_debug_line = __LINE__;
                }
                // 충돌 x 회피상태 x => currnet path 연장
                else if (n_current_path_collision_idx_ == -1 && n_current_target_lane_num_ == 0) {
                    n_debug_line = __LINE__;
                    if (0.05f > CoreCalculator::CalcPosDistance_(
                                    vec_current_path_.at(n_current_path_idx), vec_mission_path.at(n_mission_path_idx))) {
                        if (f_robot_speed < 0.01f)
                            SetAvoidRequest(false);
                        SetAvoidPermission(st_param.b_avoid_permission);
                    }
                    n_debug_line = __LINE__;
                    // NLOG(info) << "[AutoMoveMission::PathPlannerBehavior] Update Current Path";
                    UpdateCurrentPath(n_current_path_idx, f_robot_speed);
                    if (n_pp_bt_state_ != 2) {
                        n_pp_bt_state_ = 2;
                        LOG_DEBUG("[AutoMoveMission::PathPlannerBehavior] Update Current Path");
                    }
                    n_debug_line = __LINE__;
                    s_avoid_status = "Update Current Path";
                }
                // 충돌 x 회피상태 o
                else if (n_current_path_collision_idx_ == -1 && n_current_target_lane_num_ != 0) {
                    n_debug_line = __LINE__;
                    if (n_pp_bt_state_ != 3) {
                        n_pp_bt_state_ = 3;
                        LOG_DEBUG("[AutoMoveMission::PathPlannerBehavior] No collision, avoid state");
                    }
                    // 회피시간 o => 복귀경로 시도
                    int n_pred_current_idx = CalcPredRobotPos(vec_current_path_, o_robot_pos_instance, f_robot_speed, 0.5f);
                    if (n_pred_current_idx == 0)
                        n_pred_current_idx = 1;
                    std::chrono::steady_clock::time_point tp_avoid3;
                    std::chrono::duration<double> sec_result3;
                    if (b_detection_timer) {
                        LOG_DEBUG("[AutoMoveMission::PathPlannerBehavior] Start Finding ComebackPath");
                        Pos o_pred_pos = vec_current_path_.at(n_pred_current_idx);
                        float f_min_dist = CalcFrontDistanceOfCurrentPath(f_robot_speed);
                        tp_avoid3 = std::chrono::steady_clock::now();
                        st_avoid_path_result.vec_avoid_path = o_lane_avoid_path_ptr_->FindComebackPath(
                            vec_mission_path, o_pred_pos, st_lane_info_, n_current_target_lane_num_, f_min_dist);
                        sec_result3 = std::chrono::steady_clock::now() - tp_avoid3;
                    }
                    n_debug_line = __LINE__;
                    // 복귀경로가 없으면 회피연장
                    if (st_avoid_path_result.vec_avoid_path.empty()) {
                        LOG_DEBUG("[AutoMoveMission::PathPlannerBehavior] Start Extending AvoidPath");
                        float f_min_dist = CalcFrontDistanceOfCurrentPath(f_robot_speed);
                        // 연장 추가 부분만 output
                        n_debug_line = __LINE__;
                        st_avoid_path_result = o_lane_avoid_path_ptr_->ExtendAvoidPath(
                            vec_mission_path, vec_current_path_, n_current_path_idx, st_lane_info_, n_current_target_lane_num_, f_min_dist);
                        if (!st_avoid_path_result.vec_avoid_path.empty() &&
                            n_current_target_lane_num_ != st_avoid_path_result.n_target_lane_num) {
                            n_current_target_lane_num_ = st_avoid_path_result.n_target_lane_num;
                            vec_avoid_path_ = st_avoid_path_result.vec_avoid_path;
                            SetCurrentPathFromAvoidPath(n_current_path_idx);
                        }
                        else
                            ExtendCurrentPathFromAvoidPath(st_avoid_path_result.vec_avoid_path);

                        s_avoid_status = "Avoiding -> Finding CombackPath Failed -> Extending AvoidPath";
                        n_debug_line = __LINE__;
                    }
                    // 복귀경로가 있으면 복귀경로 적용
                    else {
                        n_debug_line = __LINE__;
                        vec_avoid_path_ = st_avoid_path_result.vec_avoid_path;
                        n_current_target_lane_num_ = 0;
                        SetCurrentPathFromAvoidPath(n_pred_current_idx);
                        n_debug_line = __LINE__;
                        tp_detection_period_time_ = std::chrono::steady_clock::now();
                        LOG_INFO(
                            "[AutoMoveMission::PathPlannerBehavior] Success Finding Comeback Path, planning time : %.3f",
                            sec_result3.count());
                        s_avoid_status = "Avoiding -> Finding CombackPath Success";
                    }
                    n_debug_line = __LINE__;
                }
                // 충돌 o, 시작 노드로부터 일정 거리 내에 위치하면 회피 x
                // else if (
                //     n_current_path_collision_idx_ != -1 && vec_mission_path.at(0).GetMissionType() == Pos::MISSION_NODE_TYPE::CHARGER &&
                //     n_mission_path_idx < (int)vec_mission_path.size() - n_force_comeback_target_idx_) {
                //     if (o_rapid_timer_.Over("PP_Charger", 1.0)) {
                //         LOG_DEBUG("[AutoMoveMission::PathPlannerBehavior] Closed to start charger");
                //     }
                //     s_avoid_status = "Closed to start charger";
                // }
                // 충돌 o, no_lane o
                else if (n_no_lane_idx != -1 && n_current_path_collision_idx_ != -1) {
                    LOG_DEBUG("[AutoMoveMission::PathPlannerBehavior] No lane, No avoidance");
                    s_avoid_status = "Collision Detect -> No lane, No avoidance";
                }
                // 충돌 o, static_obs x
                else if (n_current_path_collision_idx_ != -1 && !b_static_obs) {
                    SetAvoidRequest(false);
                    LOG_DEBUG("[AutoMoveMission::PathPlannerBehavior] Static obstacle not yet");
                    s_avoid_status = "Collision Detect -> Check Staic obstacle";
                    // {
                    //     std::lock_guard<std::mutex> lock(mtx_avoid_vel_);
                    //     float f_dist = CoreCalculator::CalcPosDistance_(
                    //         vec_current_path_.at(n_current_path_idx), vec_current_path_.at(n_current_path_collision_idx_));
                    //     f_avoid_decel_ratio_ = std::exp(-0.1 * f_dist);
                    // }
                }
                // 충돌 o, 회피상태 x, 회피허가 x
                else if (
                    n_current_path_collision_idx_ != -1 && !vec_current_path_.at(n_current_path_idx).GetConstDriveInfo().b_avoid_status &&
                    !b_avoid_permission && n_mission_path_idx < n_force_comeback_start_idx_) {
                    SetAvoidRequest(true);
                    s_avoid_status = "Collision Detect -> Avoid Permission False";
                }
                // 충돌 o, 회피허가 o, 회피시간 o, 강제복귀 x
                else if (b_avoid_permission && b_detection_timer && n_mission_path_idx < n_force_comeback_start_idx_) {
                    n_debug_line = __LINE__;
                    if (n_pp_bt_state_ != 4) {
                        n_pp_bt_state_ = 4;
                        LOG_DEBUG("[AutoMoveMission::PathPlannerBehavior] Collision");
                    }
                    int n_pred_current_idx = CalcPredRobotPos(vec_current_path_, o_robot_pos_instance, f_robot_speed, 0.5f);
                    if (n_pred_current_idx == 0)
                        n_pred_current_idx = 1;
                    Pos o_pred_pos = vec_current_path_.at(n_pred_current_idx);
                    float f_min_dist = CalcFrontDistanceOfCurrentPath(f_robot_speed);
                    // o_rapid_timer_.Start("avoid");
                    {
                        std::lock_guard<std::mutex> lock(mtx_avoid_vel_);
                        float f_dist = CoreCalculator::CalcPosDistance_(
                            vec_current_path_.at(n_current_path_idx), vec_current_path_.at(n_current_path_collision_idx_));
                        f_avoid_decel_ratio_ = std::exp(-0.1 * f_dist);
                    }
                    s_avoid_status = "Collision Detect -> Finding Avoid Path";
                    SetAvoidStatus(s_avoid_status);
                    n_debug_line = __LINE__;
                    std::chrono::steady_clock::time_point tp_avoid4 = std::chrono::steady_clock::now();
                    st_avoid_path_result = o_lane_avoid_path_ptr_->CheckAvoidPath(
                        vec_mission_path, o_pred_pos, st_lane_info_, n_current_target_lane_num_, f_min_dist);
                    if (st_avoid_path_result.vec_avoid_path.empty() && f_robot_speed < st_param.st_motion_param.f_linear_speed_min_ms) {
                        st_avoid_path_result = o_lane_avoid_path_ptr_->CheckAvoidPathTwice(
                            vec_mission_path, o_pred_pos, st_lane_info_, n_current_target_lane_num_, f_min_dist);
                    }
                    std::chrono::duration<double> sec_result4 = std::chrono::steady_clock::now() - tp_avoid4;
                    n_debug_line = __LINE__;
                    // o_rapid_timer_.ShowTime("avoid", 2.0);
                    if (!st_avoid_path_result.vec_avoid_path.empty()) {
                        vec_avoid_path_ = st_avoid_path_result.vec_avoid_path;
                        n_current_target_lane_num_ = st_avoid_path_result.n_target_lane_num;
                        SetCurrentPathFromAvoidPath(n_pred_current_idx);
                        tp_detection_period_time_ = std::chrono::steady_clock::now();
                        LOG_INFO(
                            "[AutoMoveMission::PathPlannerBehavior] Success to find avoidance path, planning time : %.3f",
                            sec_result4.count());
                        s_avoid_status = "Collision Detect -> Find AvoidPath Success";
                    }
                    else {
                        LOG_INFO(
                            "[AutoMoveMission::PathPlannerBehavior] Failed to find avoidance path, planning time : %.3f",
                            sec_result4.count());
                        s_avoid_status = "Collision Detect -> Find AvoidPath Failed";
                    }
                    n_debug_line = __LINE__;
                }
                else {
                    s_avoid_status = "Something Wrong";
                    LOG_WARNING(
                        "[AutoMoveMission::PathPlannerBehavior] Wrong PP_BT >> force_comeback: %s, current_target_lane_num: %d, avoidance status: %s, collision_idx: %d, no_lane_idx: %d, static obs: %s, avoid permission: %s, search timer: %s",
                        n_mission_path_idx < n_force_comeback_start_idx_ ? "false" : "true", n_current_target_lane_num_,
                        vec_current_path_.at(n_current_path_idx).GetConstDriveInfo().b_avoid_status ? "avoidance" : "normal",
                        n_current_path_collision_idx_, n_no_lane_idx, b_static_obs ? "ok" : "no", b_avoid_permission ? "ok" : "no",
                        b_detection_timer ? "ok" : "no");
                }
                n_debug_line = __LINE__;
                SetAvoidStatus(s_avoid_status);
            }
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << " n_debug_line " << n_debug_line;
        }
    }
}

void AutoMoveMission::InfoCollect()
{
    while (ros::ok()) {
        static int n_count = 0;
        if (n_count++ > 1000) {
            n_count = 0;
            NLOG(info) << "InfoCollect is running...";
        }
        tp_info_collect_ = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds(n_naviinfo_period_ms_));

        if (!b_thread_run_)
            continue;

        try {
            vector<Pos> vec_path = vec_current_mission_.vec_path;
            Pos o_robot_pos;
            {
                std::lock_guard<std::mutex> lock(mtx_robot_pos_);
                o_robot_pos = o_robot_pos_;
            }

            o_cb_regiser_.notify(NAVI_STATUS_CALLBACK, ToBehaviorStatus(st_vel_ex_.n_status));

            // motion info
            NaviFra::MotionInfo_t st_info;
            {
                std::lock_guard<std::mutex> lock(mtx_motion_info_);
                st_info = st_info_;
            }
            o_cb_regiser_.notify(MOTION_INFO_CALLBACK, st_info);
            st_navi_info_.f_collision_remained_sec = f_collision_remained_sec_;
            Pos o_now_pos;
            if (n_mission_path_idx_ < vec_path.size()) {
                o_now_pos = vec_path.at(n_mission_path_idx_);
            }
            // return 0;
            st_navi_info_.f_max_linear_vel_of_path = o_now_pos.GetConstDriveInfo().f_linear;

            st_navi_info_.f_goal_pos_x_m = o_goal_pos_.GetXm();
            st_navi_info_.f_goal_pos_y_m = o_goal_pos_.GetYm();
            st_navi_info_.f_goal_pos_deg = o_goal_pos_.GetDeg();
            st_navi_info_.f_robot_pos_x_m = o_robot_pos.GetXm();
            st_navi_info_.f_robot_pos_y_m = o_robot_pos.GetYm();
            st_navi_info_.f_robot_pos_deg = o_robot_pos.GetDeg();
            st_navi_info_.f_robot_current_linear_vel_x = st_vel_ex_.f_linear_speed_x_ms;
            st_navi_info_.f_robot_current_angular_vel_w = st_vel_ex_.f_angular_speed_degs;

            bool check_obs = (GetStatus() == MISSION_STATUS::SUSPENDING);
            st_navi_info_.n_stop_because_of_obstacle = 0;
            if (check_obs) {
                if (st_param_.b_use_obs_alarm_separate) {
                    if (n_detect_check_ == 1)  // LIDAR
                        st_navi_info_.n_stop_because_of_obstacle = 1;
                    else if (n_detect_check_ == 2)  // CAMERA
                        st_navi_info_.n_stop_because_of_obstacle = 2;
                    else if (n_detect_check_ == 3)  // V2V
                        st_navi_info_.n_stop_because_of_obstacle = 3;
                }
                else
                    st_navi_info_.n_stop_because_of_obstacle = 4;
            }
            float f_acc_dist = 0.2;

            static float f_now_dist = f_acc_dist + 0.1;
            static float f_next_dist = f_acc_dist + 0.1;
            static float f_x_pre = o_robot_pos.GetXm();
            static float f_y_pre = o_robot_pos.GetYm();

            float f_dodom = hypot(f_x_pre - o_robot_pos.GetXm(), f_y_pre - o_robot_pos.GetYm());
            f_now_dist += f_dodom;
            f_next_dist += f_dodom;

            // current next node 조건추가
            if (vec_path.size() > 0) {
                if (CoreCalculator::CalcPosDistance_(vec_path.back(), o_robot_pos) > 0.1 && !st_navi_info_.b_spin_turn) {
                    if (o_now_pos.GetNowNodeName().length() > 0 && f_now_dist > f_acc_dist &&
                        o_now_pos.GetNowNodeName() != st_navi_info_.s_current_node) {
                        st_navi_info_.s_current_node = o_now_pos.GetNowNodeName();
                        st_navi_info_.s_current_node_id = o_now_pos.GetNowNodeID();
                        f_now_dist = 0;
                    }

                    if (o_now_pos.GetNextNodeName().length() > 0 && f_next_dist > f_acc_dist &&
                        o_now_pos.GetNextNodeName() != st_navi_info_.s_next_node) {
                        st_navi_info_.s_next_node = o_now_pos.GetNextNodeName();
                        st_navi_info_.s_next_node_id = o_now_pos.GetNextNodeID();
                        f_next_dist = 0;
                    }
                }
                else if (vec_path.size() == 1) {
                    st_navi_info_.s_current_node = vec_path.back().GetNowNodeName();
                    st_navi_info_.s_current_node_id = vec_path.back().GetNowNodeID();
                    st_navi_info_.s_next_node = vec_path.back().GetNextNodeName();
                    st_navi_info_.s_next_node_id = vec_path.back().GetNextNodeID();
                }
            }
            f_x_pre = o_robot_pos.GetXm();
            f_y_pre = o_robot_pos.GetYm();

            st_navi_info_.s_goal_node = vec_current_mission_.s_goal_name;
            st_navi_info_.s_goal_node_id = vec_current_mission_.s_goal_node_id;
            // st_navi_info_.n_avoid_state = n_avoidance_state_;
            if (vec_path.size() > 10) {
                st_navi_info_.f_path_angle_deg = atan2(
                                                     vec_path.back().GetYm() - vec_path.at(vec_path.size() - 5).GetYm(),
                                                     vec_path.back().GetXm() - vec_path.at(vec_path.size() - 5).GetXm()) *
                    RADtoDEG;
            }

            int n_current_path_idx = CoreCalculator::FindMinDistanceIdxFromPosVector_(o_robot_pos, vec_current_path_);
            if (n_current_path_idx >= 0 && n_current_path_idx < vec_current_path_.size())
                st_navi_info_.n_avoid_state = vec_current_path_.at(n_current_path_idx).GetConstDriveInfo().b_avoid_status;  // need update
            st_navi_info_.n_drive_type = o_now_pos.GetConstDriveInfo().e_drive_type;

            o_cb_regiser_.notify(NAVI_INFO_CALLBACK, st_navi_info_);
        }
        catch (const std::exception& e) {
            NLOG(error) << e.what() << '\n';
        }
    }
}

void AutoMoveMission::SetAvoidPermission(bool b_permission)
{
    if (b_permission == GetAvoidPermission())
        return;
    {
        std::lock_guard<std::mutex> lock(mtx_avoid_permission_);
        b_avoid_permission_ = b_permission;
    }
    // 마지막 회피 요청 reset
    if (!b_permission)
        b_last_avoid_request_ = false;
    LOG_INFO("[AutoMoveMission::SetAvoidPermission] permission is %s", b_permission ? "approved" : "denied");
}

void AutoMoveMission::SetAvoidRequest(bool b_request)
{
    /*
    회피해야함 !!
    0. 경로 상에 장애물과 충돌하는가? O
    1. 회피허가를 받았나? X
    2. 회피허가 요청을 이미 했나? X

    회피허가 취소 !!
    0. 회피중인가? X
    1. 회피허가를 요청했나? O
    */
    bool b_permission = GetAvoidPermission();
    if (b_request && !b_permission && !b_last_avoid_request_) {
        b_last_avoid_request_ = true;
        o_cb_regiser_.notify(REQUEST_AVOIDANCE, true);
        LOG_INFO("[AutoMoveMission::SetAvoidRequest] request is true");
    }
    else if (!b_request && b_last_avoid_request_) {
        b_last_avoid_request_ = false;
        o_cb_regiser_.notify(REQUEST_AVOIDANCE, false);
        LOG_INFO("[AutoMoveMission::SetAvoidRequest] request is false");
    }
}

void AutoMoveMission::SetAvoidStatus(const std::string& s_avoid_status)
{
    {
        std::lock_guard<std::mutex> lock(mtx_avoid_status_);
        s_avoid_status_ = s_avoid_status;
    }
}

void AutoMoveMission::SetLocalMap(NaviFra::Map& o_map)
{
    n_width_m_ = o_map.GetXm();
    n_height_m_ = o_map.GetYm();
}

void AutoMoveMission::SetMap(const NaviFra::Map& o_navi_map)
{
    std::lock_guard<std::mutex> lock(mtx_global_map_);
    o_navi_map_ = std::make_shared<NaviFra::Map>(o_navi_map);
    return;
}

void AutoMoveMission::SetLocalMapPtr(std::shared_ptr<const std::vector<int8_t>> map_ptr)
{
    // setmap
    o_collision_detector_ptr_->SetData(map_ptr, o_robot_pos_);
}

geometry_msgs::PolygonStamped AutoMoveMission::DrawPolygon(const vector<NaviFra::Pos>& vec_pos)
{
    geometry_msgs::PolygonStamped poly_msg;
    geometry_msgs::Point32 p;

    poly_msg.header.frame_id = "base_link";
    int n_vec_size = vec_pos.size();
    for (int i = 0; i < n_vec_size; i++) {
        p.x = vec_pos[i].GetXm();
        p.y = vec_pos[i].GetYm();
        poly_msg.polygon.points.push_back(p);
    }
    return poly_msg;
}

}  // namespace NaviFra
