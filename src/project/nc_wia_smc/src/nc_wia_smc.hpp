#ifndef NAVIFRA_WIASMC_HPP_
#define NAVIFRA_WIASMC_HPP_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "core_msgs/NavicoreStatus.h"
#include "core_msgs/BmsInfo.h"
#include "core_msgs/PLCInfo.h"
#include <core_msgs/LedSoundStatus.h> 
#include "core/util/logger.hpp"
#include <boost/any.hpp>
#include <chrono>
#include <thread>
#include "nc_robot_state_utils.hpp"

using namespace std;

namespace NaviFra {
class WiaSMC {
public:
    WiaSMC(ros::NodeHandle& nh, ros::NodeHandle& nhp);
    ~WiaSMC();

    void RegistTalker();
    void RegistListener();
    void InitializeState();
    void updateStateLoop();
    bool isCritical(RobotState state);
    bool isPuased(RobotState state);
    bool isTasking(RobotState state);
    bool isCharging(RobotState state);

private:
    void stateCallback(const core_msgs::NavicoreStatus::ConstPtr& msg);
    void wiataskCallback(const std_msgs::String::ConstPtr& msg);
    void bmsCallback(const core_msgs::BmsInfo::ConstPtr& msg);
    void outputCommandCallback(const std_msgs::String::ConstPtr& msg);
    void plcInfoCallback(const core_msgs::PLCInfo::ConstPtr& msg);

    /*
        brake, action, mode
    */
   
    void setLED(RobotState state);
    void setSound(RobotState state);

    ros::NodeHandle nh_, nhp_;
    ros::Subscriber sub_naviinfo_;
    ros::Subscriber sub_now_task_;
    
    ros::Subscriber sub_bmsinfo_;
    ros::Subscriber sub_ledsound_state_;
    ros::Subscriber sub_output_cmd_;
    ros::Subscriber sub_plc_info_;

    ros::Publisher pub_led_;
    ros::Publisher pub_sound_;
    ros::Publisher pub_charge_;

    std::mutex mtx_info_;
    RobotState current_state_ = RobotState::STATE_IDLE;
    RobotState robot_state_ = RobotState::STATE_IDLE;
    RobotState wia_state_ = RobotState::STATE_IDLE;
    RobotState bms_state_ = RobotState::STATE_IDLE;

    RobotState last_received_state_ = RobotState::STATE_IDLE;
    int same_state_count_ = 0;
    const int kStableThreshold = 3; 

    std::thread state_thread_;
    std::atomic<bool> th_active_{false};
    std::mutex state_mtx_;
    RobotState sound_state_ = RobotState::STATE_IDLE;
    bool b_charge_start_ = false;
    bool b_manual_charge_ = false;

    std::chrono::steady_clock::time_point last_state_change_time_ = std::chrono::steady_clock::now();
};
};  // namespace NaviFra
#endif