#ifndef SENSOR_MSG_T_HPP_
#define SENSOR_MSG_T_HPP_

#include "pos/pos.hpp"
#include "scan.hpp"

#include <vector>

namespace NaviFra {
struct SensorMsg_t {
    // sacn (sensor_msgs::LaserScan) points vector
    std::vector<NaviFra::Scan_t> vec_scan;

    // scan cloud (sensor_msgs::PointCloud) points vector
    std::vector<NaviFra::SimplePos> vec_cloud;

    // All sensor data vectors relative to the robot
    std::vector<NaviFra::SimplePos> vec_sensors_relative_robot;

    std::vector<NaviFra::SimplePos> vec_vision_obs;

    NaviFra::Pos o_robot_speed;

    bool b_robot_loaded = false;
};
}  // namespace NaviFra

#endif
