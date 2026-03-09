#ifndef NAVIFRA_STATECHECK_INITIALIZER_H
#define NAVIFRA_STATECHECK_INITIALIZER_H

#include "core_agent/manager/initializer_manager.h"
#include "core_msgs/BmsInfo.h"
#include "core_msgs/CheonilReadRegister.h"
#include "core_msgs/PLCInfo.h"
#include "core_msgs/TaskAlarm.h"
#include "nc_wia_agent/data/alarm_status.h"
#include "nc_wia_agent/data/battery_status.h"
#include "nc_wia_agent/data/robot_basic_status.h"
#include "nc_wia_agent/util/task_memory_util.h"
#include "nc_wia_agent/util/task_result_publisher.h"
#include "nc_wia_agent/util/wingbody_offset_checker.h"
#include "util/logger.hpp"

#include <Poco/DateTimeFormatter.h>
#include <Poco/JSON/Object.h>
#include <core_agent/core/navicore.h>
#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_info.h>
#include <core_agent/manager/alarm_manager.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <move_msgs/CoreCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <fstream>
#include <optional>

enum NcActionType
{
    STATE_IDLE = 0,
    STATE_MOVE = 1,
    STATE_EVENT = 2,
    STATE_PAUSE = 3,
    STATE_CANCEL = 4,
    STATE_CHARGE = 7
};

namespace NaviFra {

class StateCheckInitializer : public Initializer {
public:
    void initialize() override;
    int priority() const override { return 160; }  // MQTT 이후

    bool isAtGoal(const GoalInfo& goal);
    void checkAcsHeartBeat();
    void statusCheckCallback(const ros::TimerEvent& event);
    void RecvNaviCmd(const std_msgs::String::ConstPtr& msg);
    void RecvNaviAlarm(const core_msgs::NaviAlarm::ConstPtr& msg);
    void RecvTaskAlarm(const core_msgs::TaskAlarm::ConstPtr& msg);
    void updateAlarmFromStatus();
    void alarmCheckCallback(const ros::TimerEvent& e);
    void pubchargerelay(bool state);
    void cmdPub(std::string command);
    void RecvCheonilInfo(const core_msgs::CheonilReadRegister::ConstPtr& msg);
    void RecvPalletID(const std_msgs::String::ConstPtr& msg);

    void handleResult();
    void handlePlcState(bool b_pinup);
    void handleWingbodyOffset();

    void handleAlarm(std::string s_navi_status, std::string rid);

    void RecvPlcInfo(const core_msgs::PLCInfo::ConstPtr& msg);
    void RecvBmsInfo(const core_msgs::BmsInfo::ConstPtr& msg);
    void RecvMissionRestart(const std_msgs::String::ConstPtr& msg);
    void RecvObs(const geometry_msgs::PolygonStamped::ConstPtr& msg);
    void RecvScan(const sensor_msgs::PointCloudConstPtr& msg);
    void RecvSlamTrigger(const std_msgs::Bool::ConstPtr& msg);
    void RecvWingBodyPos(const geometry_msgs::PoseArray::ConstPtr& msg);
    void RecvWingBodyTrigger(const std_msgs::Int32::ConstPtr& msg);
    void RecvWingBodyApproachMoveTrigger(const std_msgs::Int32::ConstPtr& msg);
    void RecvHplcTest(const geometry_msgs::Pose::ConstPtr& msg);

    bool checkForklift(const std::vector<GoalInfo>& goalList, int current_index, int& out_forklift_index);
    float calculateDistance(const std::vector<GoalInfo>& goalList, int current_index, int forklift_index);
    void publishForkPreControl(int rack_type, int drive_type);

    void RecvScenarioTrigger(const std_msgs::Int32::ConstPtr& msg);
    void controlPlcScenario();
    void resetAllScenarios();

private:
    ros::Subscriber navi_cmd_sub;
    ros::Subscriber navi_info_sub;
    ros::Subscriber navi_alarm_sub;
    ros::Subscriber task_alarm_sub;
    ros::Subscriber bms_sub;
    ros::Subscriber charge_relay_sub;
    ros::Subscriber plc_info_sub;
    ros::Subscriber mission_restart_sub;
    ros::Subscriber scan_data_sub;
    ros::Subscriber obs_data_sub;
    ros::Subscriber cheonil_info_sub;
    ros::Subscriber pallet_id_sub;
    ros::Subscriber hplc_test_sub;
    ros::Subscriber start_sub_;

    ros::Subscriber slam_trigger_sub;
    ros::Subscriber wingbody_pos_sub;
    ros::Subscriber wingbody_trigger_sub;
    ros::Subscriber wingbody_approach_move_trigger_sub;

    ros::Subscriber scenario_trigger_sub;

    ros::Publisher task_res_pub;
    ros::Publisher now_task_pub;
    ros::Publisher navifra_cmd_pub;
    ros::Publisher fork_docking_pub_;
    ros::Publisher wingbody_check_pub_;
    ros::Publisher live_path_pub;
    ros::Publisher fork_precontrol_pub_;

    ros::Publisher cmd_vel_pub_;
    ros::Publisher scenario_pub_;
    ros::Timer status_check_timer;
    ros::Timer alarm_check_timer;

    // int n_task_done_cnt_ = 0;
    // std::atomic<int> n_task_done_cnt_{0};
    bool b_task_done_ = false;

    Poco::FastMutex fastMutex_;
    Poco::FastMutex plcMutex_;
    Poco::FastMutex dockStateMutex_;

    core_msgs::PLCInfo plc_info_;

    // bool b_task_done_ = false;
    bool b_now_charge_;
    int n_turntask_cnt_ = 0;
    bool b_acs_connect_ = false;
    bool b_send_result_ = false;
    bool b_change_slam_ = false;

    int n_last_task_index_ = -2;
    std::string s_prev_alarm_;
    std::string s_prev_status_;

    int n_paused_by_obstacle_timeout_ = 1200;
    std::optional<Poco::Timestamp> n_start_time_;
    Poco::Timestamp n_obstacle_time_;

    bool b_last_alarm_exist_ = false;

    geometry_msgs::PoseArray wingbody_poses_;
    Poco::FastMutex wingbodyMutex_;

    WingbodyOffsetChecker wingbody_offset_checker_;
    Poco::FastMutex wingbodyOffsetMutex_;

    float f_fork_precontrol_threshold_ = 5.0f;  // meters
    bool b_fork_precontrol_sent_ = false;

    // Scenario control state machine
    enum ScenarioState {
        SCENARIO_IDLE,            // 대기 중, 시나리오 비활성
        SCENARIO_TRIGGERED,       // 트리거 발행됨, 완료 신호 대기 중
        SCENARIO_WAITING_RESET    // PLC write 완료, PLC가 false로 바뀔 때까지 대기
    };
    
    ScenarioState scenario_1_state_ = SCENARIO_IDLE;
    ScenarioState scenario_2_state_ = SCENARIO_IDLE;
    
    // Sub-scenario completion tracking
    bool scenario_1_1_completed_ = false;  // 값 1
    bool scenario_1_2_completed_ = false;  // 값 2
    bool scenario_2_1_completed_ = false;  // 값 3
    bool scenario_2_2_completed_ = false;  // 값 4
    
    int scenario_1_retry_count_ = 0;  // WAITING_RESET 상태 카운터
    int scenario_2_retry_count_ = 0;
    const int SCENARIO_RETRY_THRESHOLD = 10;  // 10회 = 2초 (0.2초 * 10)
};

REGISTER_INITIALIZER(StateCheckInitializer)

}  // namespace NaviFra

#endif  // NAVIFRA_STATECHECK_INITIALIZER_H
