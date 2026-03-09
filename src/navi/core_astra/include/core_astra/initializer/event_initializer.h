#ifndef NAVIFRA_EVENT_INITIALIZER_H
#define NAVIFRA_EVENT_INITIALIZER_H

#pragma once

#include "core_agent/manager/initializer_manager.h"
#include "core_msgs/MotorInfo.h"
#include "core_msgs/NaviAlarm.h"
#include "core_msgs/TaskAlarm.h"

#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <ros/subscriber.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

namespace NaviFra {

class EventInitializer : public Initializer {
public:
    EventInitializer() = default;
    virtual ~EventInitializer() = default;

    virtual void finalize() override;  // 선택적으로 오버라이드

    void initialize() override;
    int priority() const override { return 40; }

private:
    // 콜백 함수
    void onObstacle(const geometry_msgs::PolygonStamped::ConstPtr& msg);
    void onLocalPath(const nav_msgs::Path::ConstPtr& msg);
    void onGlobalPath(const nav_msgs::Path::ConstPtr& msg);
    void onPredictCollision(const geometry_msgs::PolygonStamped::ConstPtr& msg);
    void onReflectors(const std_msgs::String::ConstPtr& msg);
    void onTaskAlarm(const core_msgs::TaskAlarm::ConstPtr msg);
    void onTaskResponse(const core_msgs::TaskAlarm::ConstPtr& msg);
    void onNaviAlarm(const core_msgs::NaviAlarm::ConstPtr& msg);
    void onMappingProgress(const std_msgs::Int16::ConstPtr& msg);
    void onMotorInfo(const core_msgs::MotorInfo::ConstPtr& msg);
    void onSLAMGraph(const std_msgs::String::ConstPtr& msg);
    void onCaliProgress(const std_msgs::Float32::ConstPtr& msg);
    void onHardwareInfo(const std_msgs::String::ConstPtr& msg);
    void onCustom(const std_msgs::String::ConstPtr& msg);

    // Subscriber 멤버 변수 (생명주기 유지)
    ros::Subscriber sub_obstacle_;
    ros::Subscriber sub_local_path_;
    ros::Subscriber sub_global_path_;
    ros::Subscriber sub_predict_collision_;
    ros::Subscriber sub_reflectors_;
    ros::Subscriber sub_task_alarm_;
    ros::Subscriber sub_task_response_;
    ros::Subscriber sub_navi_alarm_;
    ros::Subscriber sub_mapping_progress_;
    ros::Subscriber sub_motor_info_;
    ros::Subscriber sub_slam_graph_;
    ros::Subscriber sub_cali_progress_;
    ros::Subscriber sub_hardware_info_;
    ros::Subscriber sub_custom_;
};

REGISTER_INITIALIZER(EventInitializer)

}  // namespace NaviFra

#endif  // NAVIFRA_EVENT_INITIALIZER_H
