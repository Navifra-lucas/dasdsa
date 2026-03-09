#include "core_astra/initializer/robot_safety_status_initializer.h"

#include "core_agent/data/memory_repository.h"
#include "core_agent/data/robot_info.h"  // 속도 데이터 제공 객체 (예: RobotInfo)
#include "core_agent/manager/publish_manager.h"
#include "core_astra/data/publish_channel.h"
#include "core_astra/zmq_handler.h"
#include "robot_safety_status.pb.h"
#include "util/logger.hpp"

using namespace NaviFra;

RobotSafetyStatusInitializer::RobotSafetyStatusInitializer()
{
}

void RobotSafetyStatusInitializer::initialize()
{
    ros::NodeHandle nh;

    // ✅ ROS Subscriber 등록
    lift_sub_ = nh.subscribe<std_msgs::Bool>("lift_status", 1, &RobotSafetyStatusInitializer::liftCallback, this);
    bumper_sub_ = nh.subscribe<std_msgs::Bool>("bumper_status", 1, &RobotSafetyStatusInitializer::bumperCallback, this);
    lccs_sub_ = nh.subscribe<std_msgs::Bool>("lccs_status", 1, &RobotSafetyStatusInitializer::lccsCallback, this);
    plc_info_sub_ = nh.subscribe<core_msgs::PLCInfo>("plc_info", 1, &RobotSafetyStatusInitializer::plcinfoCallback, this);

    // 로봇 속도 정보 전송
    PublisherManager::instance().addChannel(
        (int)PUBLISH_CHANNEL::CHANNEL_STATUS_SAFETY,  // 새 채널 정의 권장
        [this]() mutable {
            core_astra::RobotSafetyStatus safety;
            safety.set_lift_active(lift_active_);
            safety.set_bumper_active(bumper_active_);
            safety.set_lccs_active(lccs_active_);
            safety.set_manual_active(manual_active_);

            std::string buffer;
            if (!safety.SerializeToString(&buffer)) {
                NLOG(error) << "[RobotSafetyStatus] SerializeToString failed";
                return;
            }

            ZMQHandler::instance().send("robot_safety_status", buffer);
        },
        1000, 0);

    LOG_INFO("RobotSafetyStatusInitializer completed initialization");
}

void RobotSafetyStatusInitializer::finalize()
{
    lift_sub_.shutdown();
    bumper_sub_.shutdown();
    lccs_sub_.shutdown();
}

void RobotSafetyStatusInitializer::liftCallback(const std_msgs::Bool::ConstPtr& msg)
{
    lift_active_ = msg->data;
}

void RobotSafetyStatusInitializer::bumperCallback(const std_msgs::Bool::ConstPtr& msg)
{
    bumper_active_ = msg->data;
}

void RobotSafetyStatusInitializer::lccsCallback(const std_msgs::Bool::ConstPtr& msg)
{
    lccs_active_ = msg->data;
}

void RobotSafetyStatusInitializer::plcinfoCallback(const core_msgs::PLCInfo::ConstPtr& msg)
{
    manual_active_ = msg->io_mode_select_2;
}