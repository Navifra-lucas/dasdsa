#ifndef NAVIFRA_ROBOT_SAFETY_STATUS_INITIALIZER_H
#define NAVIFRA_ROBOT_SAFETY_STATUS_INITIALIZER_H
#pragma once
#include "core_agent/manager/initializer_manager.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "core_msgs/PLCInfo.h"

namespace NaviFra {
class RobotSafetyStatusInitializer : public Initializer {
public:
    RobotSafetyStatusInitializer();
    virtual ~RobotSafetyStatusInitializer() {}
    void initialize() override;
    virtual void finalize() override;
    int priority() const override { return 40; }  // RobotVerify 이후 실행되도록 우선순위 설정

private:
    ros::Subscriber lift_sub_;
    ros::Subscriber bumper_sub_;
    ros::Subscriber lccs_sub_;
    ros::Subscriber plc_info_sub_;

    // 상태 저장 멤버
    bool lift_active_;
    bool bumper_active_;
    bool lccs_active_;
    bool manual_active_;

    // 콜백
    void liftCallback(const std_msgs::Bool::ConstPtr& msg);
    void bumperCallback(const std_msgs::Bool::ConstPtr& msg);
    void lccsCallback(const std_msgs::Bool::ConstPtr& msg);
    void plcinfoCallback(const core_msgs::PLCInfo::ConstPtr& msg);
};

REGISTER_INITIALIZER(RobotSafetyStatusInitializer)
}  // namespace NaviFra

#endif  // NAVIFRA_ROBOT_SAFETY_STATUS_INITIALIZER_H