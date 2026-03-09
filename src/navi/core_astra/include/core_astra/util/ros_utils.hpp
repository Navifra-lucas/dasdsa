#ifndef NAVIFRA_ROS_UTILS_HPP
#define NAVIFRA_ROS_UTILS_HPP
#pragma once
#include <ros/ros.h>

namespace NaviFra {

// ros::init() 완료까지 최대 timeoutSec 대기. 완료되면 true
inline bool wait_ros_ready(double timeoutSec = 5.0)
{
    const auto start = ros::WallTime::now();
    while (!ros::isInitialized()) {
        if (timeoutSec >= 0.0 && (ros::WallTime::now() - start).toSec() > timeoutSec) {
            return false;
        }
        ros::WallDuration(0.01).sleep();
    }
    return true;
}

}  // namespace NaviFra

#endif  // NAVIFRA_ROS_UTILS_HPP