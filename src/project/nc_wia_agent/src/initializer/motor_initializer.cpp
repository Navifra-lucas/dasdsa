#include "nc_wia_agent/initializer/motor_initializer.h"

#include "nc_wia_agent/data/robot_motor_feed_store.h"
#include "util/logger.hpp"

#include <core_agent/data/memory_repository.h>
#include <motor_msgs/MotorDataInfo.h>
#include <ros/ros.h>

namespace NaviFra {

namespace {
ros::Subscriber motor_data_sub;

void motorDataCallback(const motor_msgs::MotorDataInfo::ConstPtr& msg)
{
    auto store = InMemoryRepository::instance().get<RobotMotorFeedStore>(RobotMotorFeedStore::KEY);
    if (store) {
        store->setDrivingMotors(*msg);
    }
}
}  // namespace

void MotorInitializer::initialize()
{
    ros::NodeHandle nh;

    // Subscriber 등록
    motor_data_sub = nh.subscribe("motor_data/info", 10, motorDataCallback);

    LOG_INFO("Subscribed to /motor_data/info");
}

}  // namespace NaviFra
