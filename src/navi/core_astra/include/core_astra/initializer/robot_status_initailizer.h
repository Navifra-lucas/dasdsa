#ifndef NAVIFRA_ROBOT_STATUS_INITIALIZER_H
#define NAVIFRA_ROBOT_STATUS_INITIALIZER_H

#pragma once
#include "core_agent/manager/initializer_manager.h"

#include <core_msgs/NavicoreStatus.h>
#include <core_msgs/BmsInfo.h>
#include <core_msgs/PLCInfo.h>
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

namespace NaviFra {

class RobotStatusInitializer : public Initializer {
public:
    RobotStatusInitializer() = default;
    virtual ~RobotStatusInitializer();
    virtual void initialize() override;
    virtual void finalize() override;
    int priority() const override { return 10; }  // RobotVerify 이후 실행되도록 우선순위 설정
private:
    void onErrorDist(const std_msgs::Float64::ConstPtr msg);
    void onNavifrainfo(const core_msgs::NavicoreStatus::ConstPtr msg);
    void onOdometry(const nav_msgs::Odometry::ConstPtr msg);
    void onParamUpdate(const std_msgs::String::ConstPtr msg);
    void onTaskInfo(const std_msgs::String::ConstPtr msg);
    void onBmsInfo(const core_msgs::BmsInfo::ConstPtr msg);
    void onPlcInfo(const core_msgs::PLCInfo::ConstPtr msg);
    void onNowTask(const std_msgs::String::ConstPtr msg);

public:
    ros::Subscriber param_update_subscriber_;
    ros::Subscriber task_info_subscriber_;
    ros::Subscriber odometry_subscriber_;
    ros::Subscriber navicore_status_subscriber_;
    ros::Subscriber error_dist_subscriber_;
    ros::Subscriber bms_subscriber_;
    ros::Subscriber plc_info_subscriber_;

    // Helper function to split a string by a delimiter
    static std::vector<std::string> splitString(const std::string& str, char delimiter);
};

REGISTER_INITIALIZER(RobotStatusInitializer)

}  // namespace NaviFra

#endif  // NAVIFRA_ROBOT_STATUS_INITIALIZER_H