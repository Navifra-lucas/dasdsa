/*
 * @file	: cnavigation.hpp
 * @date	: Mar 15, 2022
 * @author	:"Joongtae, Park(brain)" (founder@navifra.com)"
 * @brief	:  로봇의 주행 부분을 관장하는 ros node
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2022 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_NC_NAVIGATOR_H_
#define NAVIFRA_NC_NAVIGATOR_H_

#include "core_msgs/BmsInfo.h"
#include "core_msgs/CameraCmd.h"
#include "core_msgs/CameraRoiInfo.h"
#include "core_msgs/EtcInfo.h"
#include "core_msgs/Goal.h"
#include "core_msgs/GoalList.h"
#include "core_msgs/HacsNode.h"
#include "core_msgs/HacsNodeList.h"
#include "core_msgs/LidarInfoMsg.h"
#include "core_msgs/LocalizeInfo.h"
#include "core_msgs/MapJson.h"
#include "core_msgs/MotionInfo.h"
#include "core_msgs/MotorInfo.h"
#include "core_msgs/NaviAlarm.h"
#include "core_msgs/NaviStatus.h"
#include "core_msgs/NavicoreStatus.h"
#include "core_msgs/PGVPoseList.h"
#include "core_msgs/TaskAlarm.h"
#include "core_msgs/UpperInfo.h"
#include "core_msgs/CheonilReadRegister.h"
#include "camera_controller.hpp"
#include "mission_manager.hpp"
#include "move_msgs/CoreCommand.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Odometry.h"
#include "param_repository/param_repository.hpp"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

#include <boost/thread.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <nc_navigator/data_listener.hpp>
// #include <nc_navigator/nc_error_alarm.hpp>
#include <nc_navigator/param_server.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include "util/license_check.hpp"
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <unordered_map>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <bitset>

class ncNavigator {
public:
    ncNavigator();
    virtual ~ncNavigator();

    enum PauseReason {
        STATE = 0,
        USER,
        ACS,
        V2V,
        MAX_REASON  // 개수 관리용
    };
    
private:
    ros::NodeHandle node_handle_;
    ros::Publisher response_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher navialarm_pub_;
    ros::Publisher navistatus_pub_;
    ros::Publisher str_navi_status_pub_;
    ros::Publisher str_param_pub_;
    ros::Publisher motion_info_pub_;
    ros::Publisher camera_roi_pub_;
    ros::Publisher plc_cmd_pub_;
    ros::Publisher emergency_pub_;
    ros::Publisher charge_state_pub_;
    ros::Publisher pallet_id_pub_;

    ros::Publisher task_alarm_;

    ros::Subscriber cmd_sub_;
    ros::Subscriber livepath_sub_;
    ros::Subscriber ros_robot_pos_sub_;
    ros::Subscriber battery_info_sub_;
    ros::Subscriber cheonil_info_sub_;
    ros::Subscriber pallet_read_cmd_sub_;

    ros::Subscriber motor_info_sub_;
    ros::Subscriber lidar_info_sub_;
    ros::Subscriber upper_info_sub_;
    ros::Subscriber max_speed_sub_;
    ros::Subscriber speed_percent_sub_;
    ros::Subscriber turn_percent_sub_;
    ros::Subscriber arrive_boundary_sub_;
    ros::Subscriber log_sub_;

    ros::Subscriber is_slam_sub_;

    ros::Subscriber localize_info_sub_;
    ros::Subscriber etc_info_sub_;
    ros::Subscriber error_sub_;
    ros::Subscriber warning_sub_;
    ros::Subscriber nownode_sub_;
    ros::Subscriber load_sub_;
    ros::Subscriber lccs_sub_;
    ros::Subscriber task_info_sub_;
    ros::Subscriber sub_fork_position_;

    Parameters_t st_param_;  // 파라미터

    NaviFra::Pos o_robot_current_tf_pos_;

    NaviFra::MissionMsg o_move_to_goal_msg_;

    std::mutex mtx_lock_, mtx_sensor_, mtx_upper_, mtx_qr_, mtx_avoid_, mtx_robot_radius_;
    std::mutex mtx_lock_status_, mtx_error_, mtx_load_;

    std::vector<NaviFra::NaviNode> vec_topological_map_;
    core_msgs::NavicoreStatus core_status_;

    DataListener o_data_listener_;

    CameraController o_camera_controller_;

    NaviFra::MissionManager o_mission_manager_;
    ParamServer o_param_server_;
    std::chrono::steady_clock::time_point tp_goal_time_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_start_up_time_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_alarm_clear_time_ = std::chrono::steady_clock::now();

    bool b_localizer_ready_ = true;
    bool b_loaded_ = false;

    std::chrono::duration<double> sec_start_up_;
    boost::thread th0_;

    int n_icp_ratio_ = 100;
    int n_fork_position_ = 0;

    vector<NaviNode> vec_node_info_;
    string s_posout_ = "";

    std::mutex warningMutex;
    std::vector<std::pair<std::string, std::chrono::steady_clock::time_point>> vec_warnning_;
    const int n_warningDuration_ = 5;  // 경고 유지 시간 (초)
    bool b_addgoal_ = false;
    bool b_obs_check_ = false;
    bool b_pallet_detect_check_ = false;

    int n_navi_error_ = 0;
    std::unordered_map<int, std::string> value_to_name_;

    std::string s_current_task_name_ = "";
    std::bitset<MAX_REASON> pause_flags_;

private:
    void RecvMoveToGoal(const boost::any& any_var);
    void RecvAllSensorData(const boost::any& any_var);

    void RecvLivePath(const move_msgs::CoreCommand::ConstPtr& msg);

    void RecvNaviStatus(const boost::any& any_var);
    void RecvNaviAlarm(const boost::any& any_var);
    void RecvCurMissionROI(const boost::any& any_var);

    void RecvCmd(const std_msgs::String::ConstPtr& msg);
    void RecvMaxSpeed(const std_msgs::Float64::ConstPtr& msg);
    void RecvSpeedPercent(const std_msgs::Float64::ConstPtr& msg);
    void RecvTurnPercent(const std_msgs::Float64::ConstPtr& msg);
    void RecvArriveBoundary(const std_msgs::Float64::ConstPtr& msg);

    void RecvLog(const std_msgs::String::ConstPtr& msg);
    void RecvEmergency(const std_msgs::Bool::ConstPtr& msg);
    void RecvLoad(const std_msgs::Bool::ConstPtr& msg);
    void RecvLCCS(const std_msgs::Bool::ConstPtr& msg);
    void RecvPalletReadCmd(const std_msgs::String::ConstPtr& msg);
    void ForkPositionCallback(const std_msgs::Int16::ConstPtr& msg);

    void PublishCmdVelZero();

    void RegisteAllCallback();

    void RecvRobotoPos(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);  // icps Subscribe하여 로봇의 현재위치 저장
    void RecvNaviInfo(const boost::any& any_var);
    void RecvMotionInfo(const boost::any& any_var);
    void RecvRequestAvoidance(const boost::any& any_var);

    void NavigationStatusThread();
    void UpdateNavigation();
    void CheckMotionTimeout();

    void AbortMission();
    void CompleteMission();
    void ResumeMissionByState();
    void PauseMissionByV2V();
    void ResumeMissionByUser();
    void PauseMission(int data);

    bool IsPosValidation();
    bool IsLidarValidation();
    bool IsMotorValidation();
    bool IsUpperValidation();

    void RecvMotorInfo(const core_msgs::MotorInfo::ConstPtr& msg);
    void RecvLidarInfo(const core_msgs::LidarInfoMsg::ConstPtr& msg);
    void RecvUpperInfo(const core_msgs::UpperInfo::ConstPtr& msg);
    void RecvBatteryInfo(const core_msgs::BmsInfo::ConstPtr& msg);
    void RecvCheonilInfo(const core_msgs::CheonilReadRegister::ConstPtr& msg);
    void RecvResponseAvoidance(const std_msgs::Bool& msg);

    void RecvParams(const boost::any& any_var);
    void RecvLocalizeInfo(const core_msgs::LocalizeInfo::ConstPtr& msg);
    void RecvError(const std_msgs::Int64::ConstPtr& msg);
    void RecvWarning(const std_msgs::String::ConstPtr& msg);
    void RecvNowNode(const std_msgs::String::ConstPtr& msg);
    void RecvEtcInfo(const core_msgs::EtcInfo::ConstPtr& msg);
    void removePrefixARC(std::string& node_id);
    void SetNaviError(int n_error);
    int GetNaviError();
    bool LoadFromFile();
    std::string GetAlarmText(int value) const;

    NaviFra::Pos PosConvert(const geometry_msgs::PoseStamped& ros_pos);
    NaviFra::Pos PosConvert(const geometry_msgs::Pose& ros_pos);
    void onTaskName(const std_msgs::String msg);

    geometry_msgs::PolygonStamped DrawPolygon(const vector<NaviFra::Pos>& vec_pos)
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

    struct Status_t {
        NaviFra::MissionNavigationMsg_t st_mission_info_;
        int n_cur_navistatus_ = static_cast<int>(NaviFra::AutoMoveMission::NAVIGATION_STATUS::IDLE);

        // nc_navigatior 에서 판단
        bool b_emergency_ = false;
        bool b_map_loaded_ = false;
        bool b_running_ = false;

        // 외부 state machine, 유저 제어 신호에 의해 판단
        // or 논리로 동작
        bool b_paused_by_user_ = false;
        bool b_paused_by_status_ = false;
        bool b_paused_by_obs_ = false;
        bool b_paused_path_ = false;
        bool b_confidence_error_ = false;

        // 중단했을 때,
        // 목적지에 도달해서 중단했는지, 중단 신호를 받았는지 판단
        enum LAST_COMPELETED_TYPE
        {
            NONE = 0,
            COMPLETED,
            ABORTED
        };
        LAST_COMPELETED_TYPE completed_type_ = NONE;

        const float f_max_timeout_sec_ = 0.4f;
        std::chrono::steady_clock::time_point tp_resent_pos_timestamp_;
        std::chrono::steady_clock::time_point tp_resent_scan_timestamp_;
        std::chrono::steady_clock::time_point tp_resent_motor_error_timestamp_;
        std::chrono::steady_clock::time_point tp_resent_upper_error_timestamp_;

        // motor error 체크용
        std::string s_motor_error_text_;
        int n_motor_error_code_;
        std::string s_lidar_error_text_;
        int n_lidar_error_code_;

        std::string s_upper_error_text_;
        int n_upper_error_code_;
        float f_steer_angle_deg_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

        std::string s_last_error_;

        bool b_pos_is_in_valid_ = false;
        bool b_odom_is_in_valid_ = false;
        bool b_scan_is_in_valid_ = false;
        bool b_img_map_loaded_ = false;
 
        NaviFra::Pos last_motion_pos_;
        float last_motion_steer_deg_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        std::chrono::steady_clock::time_point tp_last_motion_time_;
    } st_status_;
};

#endif
