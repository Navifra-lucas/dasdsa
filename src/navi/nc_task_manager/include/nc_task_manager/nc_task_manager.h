/**
 * @brief Brain UI 용 TaskManager
 * @details Robot을 제어 하기 위해 Agent와 통신을 담당 하는 Node 기존 TaskManager를 Refactoring
 * @author Kerry kerry@navifra.com
 * @date 2023-11-29
 * @version 1.0.0
 */

#ifndef NAVIFRA_NC_TASK_MANAGER_H
#define NAVIFRA_NC_TASK_MANAGER_H

#include "core/util/logger.hpp"
#include "nc_task_manager/nc_task_manager_pch.h"
#include "nc_task_manager/waypoint_planner.hpp"
#include "node/navi_node.hpp"
#include "pos/pos.hpp"
#include "ros/ros.h"

#include <Poco/Activity.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <core_msgs/AddTask.h>
#include <core_msgs/CheonilReadRegister.h>
#include <core_msgs/CommonString.h>
#include <core_msgs/CommonStringRequest.h>
#include <core_msgs/CommonStringResponse.h>
#include <core_msgs/Goal.h>
#include <core_msgs/HacsWakeup.h>
#include <core_msgs/JsonList.h>
#include <core_msgs/NaviAlarm.h>
#include <core_msgs/NavicoreStatus.h>
#include <core_msgs/PLCInfo.h>
#include <core_msgs/TaskAlarm.h>
#include <core_msgs/WiaForkInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_msgs/CameraObstacle.h>
#include <move_msgs/CoreCommand.h>
#include <move_msgs/Dock.h>
#include <move_msgs/LidarObstacle.h>
#include <move_msgs/Waypoint.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <task_msgs/BatteryInfo.h>
#include <task_msgs/LiftInfo.h>
#include <task_msgs/PlcInfo.h>
#include <task_msgs/TbotBatteryInfo.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <thread>

using namespace std;

namespace NaviFra {
class Task;
class TaskManager {
    enum class Mode
    {
        MANUAL,
        AUTO
    };

public:
    TaskManager();
    ~TaskManager();

public:
    template <class MSG>
    void publish(std::string topic, MSG msg)
    {
        if (rosPub_.find(topic) != rosPub_.end()) {
            rosPub_[topic].publish(msg);
        }
    }

private:
    ros::NodeHandle nh_;
    int n_thread_period_millisec_;
    ros::ServiceServer commonString_server_;

private:
    bool TaskAddSrv(core_msgs::CommonStringRequest& req, core_msgs::CommonStringResponse& res);
    void TaskAddCallback(const core_msgs::AddTask::ConstPtr& msg);
    void TaskCmdCallback(const std_msgs::String::ConstPtr& msg);
    void NaviInfoCallback(const core_msgs::NavicoreStatus::ConstPtr& msg);
    void NaviAlarmCallback(const core_msgs::NaviAlarm::ConstPtr& msg);
    void DriverInfoCallback(const std_msgs::String::ConstPtr& msg);
    void DriverAlarmCallback(const std_msgs::String::ConstPtr& msg);
    void RecvSetSeverityMin(const std_msgs::Int16 msg);
    void CaclPosDenominator(
        Task::MoveData& move_data_curve, const NaviFra::Pos& o_pos1, const NaviFra::Pos& o_pos2, const float& cx, const float& cy);
    void TaskPubStatus(const string& data);
    void TaskPubAlarm(const string& uuid, const string& data);

    void CheckingCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void PLCLoadingCallback(const task_msgs::LiftInfo::ConstPtr& msg);
    void PLCInfoCallback(const core_msgs::PLCInfo::ConstPtr& msg);
    void DockingSuccessCallback(const std_msgs::Bool::ConstPtr& msg);
    // void ChargingSuccessCallback(const std_msgs::Bool::ConstPtr& msg);
    // void UnChargingSuccessCallback(const std_msgs::Bool::ConstPtr& msg);
    void CheonilRegisterCallback(const core_msgs::CheonilReadRegister::ConstPtr& msg);

    void LoadingFailCallback(const std_msgs::Bool::ConstPtr& msg);
    void RecvChargeTrigger(const std_msgs::String::ConstPtr& msg);
    void RecvChargeState(const std_msgs::Bool::ConstPtr& msg);
    void RecvWakeUpWait(const core_msgs::HacsWakeup::ConstPtr& msg);
    void ForkLiftCallback(const std_msgs::Bool::ConstPtr& msg);
    void WingbodyCheckCallback(const std_msgs::Bool::ConstPtr& msg);

    void NavifraCmdPub(const std::string& cmd);
    void DriverCmdPub(const std::string& cmd);

    void pause();
    void resume();
    void TaskCancel();
    void setMode(Mode newMode);

    /// inline
    void start();
    void stop();
    bool isStopped();
    void process();
    void processImpl();

    bool loadBehaviorTree(const std::string& bt_xml_filename = "");
    void RecvMapeditorNode(const core_msgs::JsonList& msg);
    void RecvInitialPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    bool MoveProcess(BT::SharedQueue<Task>& tasks);

private:
    Poco::Activity<TaskManager> activity_;

    std::map<std::string, ros::Publisher> rosPub_;
    std::vector<ros::Subscriber> rosSubs_;

    // Behavior Tree to be executed when goal is received
    BT::Tree tree_;
    // The blackboard shared by all of the nodes in the tree
    BT::Blackboard::Ptr blackboard_;
    BT::BehaviorTreeFactory factory_;
    std::shared_ptr<BT::Groot2Publisher> groot2_;
    WaypointPlanner o_waypoint_planner_;

    Mode currentMode_ = Mode::AUTO;
    bool hasTask;

    bool b_map_loaded_ = false;
    bool docking_charge_result_ = false;
    NaviFra::Pos o_robot_pos_;
    std::string s_current_node_id_;
    std::chrono::system_clock::time_point tp_move_ = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point tp_idle_ = std::chrono::system_clock::now();
};

inline bool TaskManager::isStopped()
{
    return activity_.isStopped();
}

inline void TaskManager::start()
{
    activity_.start();
}

inline void TaskManager::stop()
{
    activity_.stop();
    activity_.wait(1000);
}

}  // namespace NaviFra
#endif
