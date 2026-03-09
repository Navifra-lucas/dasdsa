#ifndef NAVIFRA_MOTOR_STATUS_INITIALIZER_H
#define NAVIFRA_MOTOR_STATUS_INITIALIZER_H
#pragma once

#include "core_agent/manager/initializer_manager.h"
#include "motor_msgs/MotorDataInfo.h"
#include "motor_status.pb.h"

#include <ros/ros.h>

#include <thread>

namespace NaviFra {
class MotorStatusInitializer : public Initializer {
public:
    virtual ~MotorStatusInitializer();
    void shutdown();
    void initialize() override;
    int priority() const override { return 30; }  // RobotVerify 이후 실행되도록 우선순위 설정
    virtual void finalize() override;  // 선택적으로 오버라이드

private:
    void recvMotorData(const motor_msgs::MotorDataInfo::ConstPtr& msg);
    void publishLoop();

private:
    ros::Subscriber motor_data_subscriber_;
    std::thread pub_thread_;
    std::atomic<bool> running_{false};
    double publish_hz_ = 20.0;  // 필요시 파라미터로 변경
    core_astra::MotorStatusList motor_status_list_;
    std::mutex motor_status_mutex_;
};

REGISTER_INITIALIZER(MotorStatusInitializer)
}  // namespace NaviFra

#endif  // NAVIFRA_MOTOR_STATUS_INITIALIZER_H