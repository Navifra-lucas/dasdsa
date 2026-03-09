/*
 * @file	: auto_move_mission.hpp
 * @date	: Jan. 1, 2022
 * @author	: "Joongtae, Park(brain)" (founder@navifra.com)"
 * @brief	:   제조물류 현장에서 사용되는 기본 주행
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2022 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_AUTO_MOVE_MISSION_HPP_
#define NAVIFRA_AUTO_MOVE_MISSION_HPP_

#include "avoid_structure.hpp"
#include "camera_obstacle_check_ros.hpp"
#include "collision_detector.hpp"
#include "core_calculator/core_calculator.hpp"
#include "core_msgs/CameraCmd.h"
#include "core_msgs/CameraRoiInfo.h"
#include "core_msgs/CameraRoiInfoWia.h"
#include "core_msgs/TransformValue.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PolygonStamped.h"
#include "glog/logging.h"
#include "lane_avoid_path.hpp"
#include "lidar_obstacle_check.hpp"
#include "map/map.hpp"
#include "mission_manager/job/job.hpp"
#include "mission_manager/mission_repository.hpp"
#include "mission_navigation_msg.hpp"
#include "motion_manager/motion_manager.hpp"  // original mpc version
#include "mpc_motion_controller/mpc_motion_controller.hpp"  // original mpc version
#include "nav_msgs/Path.h"
#include "node_path_divider.hpp"
#include "param_repository/param_repository.hpp"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"
#include "timer/timer.hpp"
#include "util/cbfunc_register.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

namespace NaviFra {
class AutoMoveMission {
public:
    static const int CB_MSG_MISSION_COMPLETED = 0;
    static const int CB_MSG_ALL_PROC_TERMINATION_COMPLETED = 1;
    static const int CB_MSG_ALL_PROC_CANCLED = 2;

    enum class NAVIGATION_STATUS
    {
        IDLE = 100,
        RUNNING,
        PAUSE,
        OBS_STOP,
        ERROR
    };

    enum NAVIGATION_ALARM
    {
        PATH_NOT_CREATED = 2000,
        GOAL_ARRIVED = 1002,
    };

    enum class MISSION_STATUS
    {
        IDLE = 0,
        RUNNING,
        SUSPENDING,
        END
    };

    enum class NOTIFY_MSG
    {
        MISSION,
    };

    enum class AVOID_STATUS
    {
        IDLE = 0,  // 일반 주행
        AVOID,  // 회피 주행
        // COMEBACKSTART,       // 복귀 주행
        COMEBACK,  // 원본 경로 복귀 주행
        COMEBACKFORCE  // 무조건 복귀 주행

    };

    AutoMoveMission();
    virtual ~AutoMoveMission();

    void SetReferenceVelocity(vector<Pos>& vec_path);
    void SetSensorMsg(const SensorMsg_t& st_sensor_msgs);
    void SetLoadState(const bool& b_msg);
    void SetForkPosition(int n_pos);
    void SetUseLccs(const bool& b_msg);
    void SetSpeed(const NaviFra::Pos& o_robot_speed);
    void SetLocalMap(NaviFra::Map& o_map);
    void SetRobotPos(const NaviFra::Pos& o_robot_pos);
    void ResumeMission();
    void SuspendMission();
    void TerminateMission();
    void SetTwist(const float& f_speed);
    void SetArriveBoundary(const float& f_arrive_boundary_m);
    void SetGoalArrivedFlag(const bool& b_msg);

    void SetSpeedPercent(const float& f_speed_percent);
    void SetTurnPercent(const float& f_turn_percent);

    /**
     * @brief Set the Behavior Mission object
     *
     * @param mission
     * @return int
     */
    int SetBehaviorMission(const NaviFra::PathDescription& mission, bool b_is_add_goal);

    /**
     * @brief ExecuteJob
     *
     * @return int
     */
    int ExecuteJob();

    /**
     * @brief ConfirmAllProcTerminate
     *
     * @param n_obj
     */
    void ConfirmAllProcTerminate(int n_obj);

    /**
     * @brief RegistCbFunc
     *
     * @param str_cbf_name
     * @param pt_func_
     * @return true
     * @return false
     */
    bool RegistCbFunc(const std::string& str_cbf_name, const std::function<void(const boost::any&)>& pt_func);

    /**
     * @brief RegistCbFunc
     *
     * @param n_cb_name
     * @param pt_func
     * @return true
     * @return false
     */
    bool RegistCbFunc(int n_cb_name, const std::function<void(const boost::any&)>& pt_func);

    /**
     * @brief NotifyCbFunc
     *
     * @param str
     * @param any_type_var
     */
    void NotifyCbFunc(const std::string& str, const boost::any& any_type_var);

    /**
     * @brief Get the User Pause Flag object
     *
     * @return true
     * @return false
     */
    bool GetUserPauseFlag();

    /**
     * @brief PauseUser
     *
     */
    void PauseUser();

    /**
     * @brief ResumeUser
     *
     */
    void ResumeUser();

    /**
     * @brief Get the Status object
     *
     * @return int
     */
    MISSION_STATUS GetStatus();

    float GetErrorDist() { return f_error_dist_; };

    void SetMap(const NaviFra::Map& o_navi_map);
    void SetNaviParam(const boost::any& any_type_var);

    // set local map
    void SetLocalMapPtr(std::shared_ptr<const std::vector<int8_t>> map_ptr);

    // permission system for avoidance
    bool GetAvoidPermission() { return b_avoid_permission_; };
    void SetAvoidPermission(bool b_permission);
    bool GetLastAvoidRequest() { return b_last_avoid_request_; };
    void SetAvoidRequest(bool b_request);
    void SetAvoidStatus(const std::string& s_avoid_status);
    std::string GetAvoidStatus() { return s_avoid_status_; };

    // 차선회피 정보
    Lane_info_t GetLaneInfo() { return st_lane_info_; };
    void SetLaneInfo(const Lane_info_t& st_lane_info) { st_lane_info_ = st_lane_info; };
    void AvoidMissionClear();
    bool GetRunningStatus() { return b_thread_run_; };

private:
    /**
     * @brief 메인 스레드 MovetoGoal의 바인딩 함수(전체 로직)
     *
     * @return int
     */
    void MotionControl();

    /**
     * @brief Path 장애물 체크 및 회피 함수
     *
     * @return void
     */
    void PathPlannerBehavior();

    /**
     * @brief CheckNaviInfo
     *
     * @return int
     */
    void InfoCollect();

    void ObstacleCheck();
    void ObstaclePub(const Pos& o_obs_pos, const Pos& o_robot_pos);

    void Initialize();

    /**
     * @brief MoveToGoalForkbotBh의 상태를 설정
     *
     * @param n_status : MISSION_STATUS enum에 정의되어 있음
     */
    void SetStatus(MISSION_STATUS e_status);

    void SetMotionInfo(const MotionInfo_t st_info);

    void SetObstacleValue(
        Parameters_t& st_param, int& n_local_path_idx, vector<Pos>& vec_pos_local_path, NaviFra::Pos& o_robot_pos,
        CommandVelocity_t& regenerated_st_vel, SensorMsg_t& st_sensor_msg, NaviFra::Polygon& o_collision_polygon,
        NaviFra::Polygon& o_outline_polygon);

    int ToBehaviorStatus(int n_motion_status);
    void PubLocalPath(const vector<Pos>& vec_path);

    void StopMotion();

    // param
    float f_collision_box_max_xm;
    float f_collision_box_min_xm;
    float f_collision_box_max_ym;
    float f_collision_box_min_ym;
    float f_collision_radius_m;

    // current path generation
    /**
     * @brief current_path_를 생성할 때 로봇으로부터 최소 생성되어야할 앞부분 경로의 길이 계산
     * @param f_robot_speed
     * @return f_front_distance, current_path_의 앞부분의 최소 길이
     */
    float CalcFrontDistanceOfCurrentPath(float f_robot_speed);
    /**
     * @brief 경로의 모든 waypoint에 대해 거리 계산
     * @param vec_path (&)
     * @return 참조된 경로 vec_path의 모든 waypoint의 Zm에 Sm 계산
     */
    void SetFullSm(vector<Pos>& vec_path);
    /**
     * @brief 경로의 모든 waypoint에 대해 곡률계산
     * @param vec_path (&)
     * @return 참조된 경로 vec_path의 모든 waypoint의 곡률 계산
     */
    void SetCurvature(vector<Pos>& vec_path);
    /**
     * @brief 현재 경로 clear
     */
    void ClearCurrentPath();
    /**
     * @brief 회피 없이 미션경로만을 이용한 current path 생성
     * @param f_robot_speed
     * @param vec_missino_path_
     * @param n_mission_path_idx_
     * @return void, 전역 변수 vec_current_path_ 가 수정됨
     */
    void SetCurrentPathFromMissionPath(float f_robot_speed);
    /**
     * @brief 현재경로와 회피경로를 병합하여 current path 생성
     * @param n_cut_idx
     * @param vec_avoid_path_
     * @param vec_current_path_
     * @return void, 전역 변수 vec_current_path_ 가 수정됨
     */
    void SetCurrentPathFromAvoidPath(int n_cut_idx);
    /**
     * @brief 충돌하지 않았지만 로봇이 앞으로 나아가면서 vec_current_path_ 를 연장하기 위한 함수
     * @param n_current_path_idx
     * @param f_robot_speed
     * @param vec_mission_path_
     * @param vec_current_path_
     * @param n_mission_path_idx_
     * @return void, 전역 변수 vec_current_path_ 가 수정됨
     */
    void UpdateCurrentPath(int n_current_path_idx, float f_robot_speed);
    // /**
    //  * @brief 충돌하지 않았지만 로봇이 앞으로 나아가면서 vec_current_path_ 를 연장하기 위한 함수
    //  * @param n_current_path_idx
    //  * @param n_end_idx
    //  * @param vec_mission_path_
    //  * @param vec_current_path_
    //  * @param n_mission_path_idx_
    //  * @return void, 전역 변수 vec_current_path_ 가 수정됨
    //  */
    // void UpdateCurrentPath(int n_current_path_idx, int n_end_idx);
    /**
     * @brief 차선회피의 연장 (차선 유지)
     * @param vec_extension_path
     * @param vec_current_path_
     * @return void, 전역 변수 vec_current_path_ 가 수정됨
     */
    void ExtendCurrentPathFromAvoidPath(const vector<Pos>& vec_extension_path);

    /**
     * @brief 로봇 속도를 고려하여 시간 dt 후에 로봇이 추종하는 경로 중 어느 위치에 있을지 예측하는 함수
     * @param vec_path 현재 로봇이 추종하고 있는 경로
     * @param o_robot_pos_instance 현재 로봇 위치
     * @param f_robot_speed 현재 로봇 속도
     * @param dt 현재 시간으로부터 예측하고 싶은 시간
     * @return 시간 dt 후 로봇의 예측된 경로 상의 위치, int
     */
    int CalcPredRobotPos(const vector<Pos>& vec_path, const Pos& o_robot_pos_instance, float f_robot_speed, float dt);
    /**
     * @brief 로봇 속도를 고려하여 시간 dt 후에 로봇이 추종하는 경로 중 어느 위치에 있을지 예측하는 함수
     * @param vec_path 현재 로봇이 추종하고 있는 경로
     * @param n_path_idx 경로 상의 로봇 위치 index
     * @param f_robot_speed 현재 로봇 속도
     * @param dt 현재 시간으로부터 예측하고 싶은 시간
     * @return 시간 dt 후 로봇의 예측된 경로 상의 위치, int
     */
    int CalcPredRobotPos(const vector<Pos>& vec_path, int n_path_idx, float f_robot_speed, float dt);
    geometry_msgs::PolygonStamped DrawPolygon(const vector<NaviFra::Pos>& vec_pos);
    void ThreadMonitor();

    SensorMsg_t st_sensor_msg_;  //로봇의 모든 센서데이터를 취합한 메세지
    MissionNavigationMsg_t st_navi_info_;

    CBFuncRegister o_cb_regiser_;

    // original version
    NaviFra::MpcMotionController o_motion_controller_;

    // obstacle check
    NaviFra::LidarObstacle o_lidar_obstacle_check_;
    NaviFra::CameraObstacle o_camera_obstacle_check_;

    int n_local_path_idx_;  // 지역 경로에 대해, 현재 로봇의 인덱스

    std::mutex mtx_sensor_;
    std::mutex mtx_vel_;
    std::mutex mtx_robot_pos_;
    std::mutex mtx_path_idx_;
    std::mutex mtx_param_;
    std::mutex mtx_robot_speed_;
    std::mutex mtx_motion_info_;
    std::mutex mtx_current_path_;
    std::mutex mtx_avoid_permission_;
    std::mutex mtx_mission_path_;
    std::mutex mtx_global_map_;
    std::mutex mtx_polygon_;
    std::mutex mtx_avoid_vel_;
    std::mutex mtx_avoid_status_;

    NaviFra::PathDescription vec_current_mission_;
    MISSION_STATUS e_status_;

    Pos o_robot_pos_, o_goal_pos_;
    Parameters_t st_param_;
    NaviFra::Pos o_pre_robot_pos_;

    vector<Pos> vec_mission_path_;
    vector<Pos> vec_avoid_path_;
    vector<Pos> vec_current_path_;

    std::shared_ptr<LaneAvoidPath> o_lane_avoid_path_ptr_;
    std::shared_ptr<CollisionDetector> o_collision_detector_ptr_;

    // 경로 상의 index
    int n_mission_path_idx_;
    int n_current_path_collision_idx_;

    vector<SimplePos> vec_virtual_pcs_;  // 가상의 장애물 데이터
    NaviFra::SimplePos o_last_collision_pos_;
    std::shared_ptr<Timer> o_static_obs_tmr_;

    // 차선회피 정보
    int n_current_target_lane_num_ = 0;
    // lane info
    Lane_info_t st_lane_info_;
    // 미션경로 마다 마지막에 무조건 복귀하는 waypoint의 index
    int n_force_comeback_start_idx_ = 0;
    int n_force_comeback_target_idx_ = 0;

    // 로봇 속도 받아오기
    NaviFra::Pos o_robot_vel_;

    std::vector<NaviFra::SimplePos> vec_vision_obs_;
    std::mutex mtx_lock_vision_;

    NaviFra::Polygon o_collision_polygon_, o_collision2_polygon_, o_collision3_polygon_;
    NaviFra::Polygon o_outline_polygon_;
    // ros::ServiceClient camera_cmd_req_;

    // void ObstacleCallback(const sensor_msgs::PointCloud::ConstPtr& msg);

    bool b_move_backward_flag_;
    bool b_is_paused_by_user = false;

    std::string s_avoid_status_;

    float f_front_base = 0.6;
    float f_rear_base = 0.6;
    float f_side_base = 0.4;
    float f_gain = 1;
    float f_linear_speed_x = 0.0;
    float f_angular_speed_degs = 0.0;
    float f_outline_height_m = 0.0;
    float f_outline_width_m = 0.0;
    float f_robot_yaw_bias_ = 0.0f;
    float f_collision_remained_sec_ = 1.0;
    float f_robot_radius_ = 1.0f;
    float f_obs_speed_vel_ = 1.0;
    float f_obs_target_speed_vel_ = 1.0;
    CommandVelocity_t regenerated_st_vel_;
    CommandVelocity_t st_vel_ex_;
    MotionInfo_t st_info_;
    float f_error_dist_ = 0;

    float f_avoid_decel_ratio_ = 1.0;

    int n_obstacle_period_ms_ = 50;
    int n_naviinfo_period_ms_ = 50;
    int n_detect_check_ = 0;

    bool b_camera_docking_check_ = false;
    bool b_is_pre_docking_ = false;
    bool b_move_pre_backward_flag_ = false;
    bool b_robot_speed_check_ = false;
    bool b_ready_to_sto_ = false;
    bool b_set_area_ratio_ = false;

    bool b_last_avoid_request_ = false;
    bool b_avoid_permission_ = false;
    bool b_pre_detect_ = false;
    bool b_detect_check_ = false;
    bool b_camera_detect_check_ = false;
    bool b_start_obs_check_ = false;
    bool b_obs_camera_check_ = false;
    bool b_obs_lidar_check_ = false;
    bool b_obs_v2v_check_ = false;
    bool b_camera_detect_time_ = false;
    bool b_camera_off_check_ = false;
    bool b_camera_sto_check_ = false;
    bool b_pre_camera_sto_ = true;
    bool b_camera_clear_flag_ = false;

    bool b_disable_cam_ = false;
    bool b_loaded_ = false;
    bool b_lccs_ = true;
    int n_fork_position_ = 0;
    bool b_force_goal_arrived_ = false;

    bool b_avoid_status_ = false;

    bool b_docking_path_obs_check_ = false;




    std::chrono::steady_clock::time_point tp_start_time_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_start_obs_detect_time_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_detect_time_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_camera_detect_time_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_check_time_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_detection_period_time_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_camera_clear_time_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_set_area_ratio_time_ = std::chrono::steady_clock::now();

    std::chrono::steady_clock::time_point tp_local_path_pub_time_ = std::chrono::steady_clock::now();

    std::chrono::steady_clock::time_point tp_path_planner_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_motion_control_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_info_collect_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_obstacle_check_ = std::chrono::steady_clock::now();

    float f_pub_interval_ = 1.0;

    RapidTimer o_rapid_timer_;
    int n_pp_bt_state_ = 0;  // 0: idle, 1: forcecomeback, 2: update currentpath, 3: 충돌 x, 회피상태 o, 4: 회피시도

    int n_width_m_ = -1;
    int n_height_m_ = -1;

    std::shared_ptr<NaviFra::Map> o_navi_map_ = nullptr;

    ros::NodeHandle node_handle_;
    ros::Publisher pub_warning_;
    ros::Publisher pub_obs_pos_;
    boost::thread th_path_planner_;
    boost::thread th_motion_control_;
    boost::thread th_info_collect_;
    boost::thread th_obstacle_check_;
    boost::thread th_thread_monitor_;

    bool b_thread_run_ = false;
};
}  // namespace NaviFra
#endif
