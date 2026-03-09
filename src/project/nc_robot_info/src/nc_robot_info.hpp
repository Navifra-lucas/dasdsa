#ifndef ROBOT_INFO_HPP
#define ROBOT_INFO_HPP

#include "util/logger.hpp"

#include <core/util/logger.hpp>
#include <core_msgs/EtcInfo.h>
#include <core_msgs/LidarInfoMsg.h>
#include <core_msgs/LocalizeInfo.h>
#include <core_msgs/MotionInfo.h>
#include <core_msgs/MotorInfo.h>
#include <core_msgs/NavicoreStatus.h>
#include <core_msgs/NaviAlarm.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/Imu.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
class RobotInfo {
private:
    ros::NodeHandle nh;

    std::string motor_data_;
    std::string lidar_data_;
    std::string robot_status_data_;

    ros::Subscriber navifra_info_sub;
    ros::Subscriber motor_info_sub;
    ros::Subscriber lidar_info_sub;
    ros::Subscriber motion_info_sub;
    ros::Subscriber localize_info_sub;
    ros::Subscriber answer_report_sub;
    ros::Subscriber imu_use_sub;
    ros::Subscriber imu_data_sub;
    ros::Subscriber task_info_sub;

    ros::Publisher str_robot_log_pub_;
    ros::Publisher str_custom_pub_;
    ros::Publisher str_total_mileage_pub_;
    ros::Publisher str_local_mileage_pub_;
    ros::Publisher etc_info_pub_;
    ros::Publisher str_warn_pub_;
    ros::Publisher clock_pub_;
    ros::Publisher str_error_pub_;
    ros::Publisher initpose_pub_;
    ros::Publisher initpose_correct_position_pub_;
    boost::thread th0_;

    core_msgs::MotorInfo m_motor_;
    core_msgs::NavicoreStatus m_navi_;
    core_msgs::MotionInfo m_motion_;
    core_msgs::LocalizeInfo m_local_;

    std::mutex data_mutex_;

    ros::Timer robot_info_pub_timer_;

    ros::Time last_robot_msg_time_;
    ros::Time last_motor_msg_time_;
    ros::Time last_lidar_msg_time_;

    double pub_interval = 1.0;
    const double TIMEOUT_DURATION = 3.0;
    float f_mileage_ = 0;
    float f_mileage_local_ = 0;
    std::ofstream stm_file_;
    std::mutex fileMutex;
    std::string s_filename_ = "pose.txt";
    std::string s_mile_filename_ = "mileage.txt";

    bool b_use_imu_ = false;
    float f_imu_angular_vel_ = 0;

    std::vector<std::string> vec_warning_;

    std::chrono::steady_clock::time_point t_cpu_high_;
    std::chrono::steady_clock::time_point t_mem_high_;
    std::chrono::steady_clock::time_point t_disk_high_;
    bool b_cpu_high_ = false;
    bool b_mem_high_ = false;
    bool b_disk_high_ = false;

    bool b_running_ = true;


    int n_cpu_overload_percent_ = 80; // CPU overload threshold
    int n_cpu_overload_time_second_ = 20; // CPU overload threshold
    int n_cpu_temper_warning_c_ = 60;
    int n_mem_overload_percent_ = 80; // Memory overload threshold
    int n_mem_overload_time_second_ = 20; // Memory overload threshold
    int n_disk_avaliable_gbytes_ = 10; // Disk available threshold in GB
    int n_disk_usage_percent_ = 90; // Disk usage threshold in percent
    int n_disk_usage_time_second_ = 20; // Disk usage threshold time in seconds
    
    std::string s_current_task_ = "";
    std::string s_previous_task_ = "";

public:
    RobotInfo();

    void RosSubscriber();
    void NavifraStatusDetailCallback(const core_msgs::NavicoreStatus::ConstPtr& msg);
    void MotorCallback(const core_msgs::MotorInfo::ConstPtr& msg);
    void LidarInfo(const core_msgs::LidarInfoMsg::ConstPtr& msg);
    void MotionInfo(const core_msgs::MotionInfo::ConstPtr& msg);
    void LocalizeInfo(const core_msgs::LocalizeInfo::ConstPtr& msg);
    void LocalizeReport(const std_msgs::String::ConstPtr& msg);
    void ImuUseFlag(const std_msgs::Bool::ConstPtr& msg);
    void ImuData(const sensor_msgs::Imu::ConstPtr& msg);
    
    std::vector<std::string> splitString(const std::string& str, char delimiter);
    void onTaskInfo(const std_msgs::String msg);

    std::string roundTo(double value, int n);

    void PublishData(const ros::TimerEvent& e);
    bool IsTopicAlive(const ros::Time& last_msg_time);
    std::string GetUiDate();
    std::string getCurrentTime();
    void ClockThread();

    std::string GetPcDate();
    std::string exec(const char* cmd);

    void SaveMileage();
    void SavePos();
    void openFile(const std::string& s_name);
    void forceSync(const std::string& s_name);
    void closeFile();
};

#endif
