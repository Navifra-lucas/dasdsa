/*
 * @file	: safety_detector.hpp
 * @date	: Feb 28, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Collision detector
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef SAFETY_DETECTOR_HPP_
#define SAFETY_DETECTOR_HPP_

#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <utility>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "utils/param/sub/safety_area_param.hpp"
#include "utils/timer.hpp"
#include "utils/pose.hpp"
#include "utils/robot_config.hpp"
#include "utils/motion_planner_type_list.hpp"

namespace NVFR {

class SafetyDetector
{
public:
  SafetyDetector();
  ~SafetyDetector() = default;

  void SetCircleRobot(double d_radius_m);
  void SetBoxRobot(double d_max_x_m, double d_min_x_m, double d_max_y_m, double d_min_y_m, double d_padding_m);
  void SetRobotConfig(const RobotConfig& o_robot_config);

  void SetVecSafetyArea(std::vector<SafetyAreaParam_t> vec);

  /**
   * @brief check safety area using sensor points
   * @param vec_sensor_data local points of all of sensor
   */
  // int SafetyDetection(const std::vector<Pos>& vec_sensor_data) const;
  SafetyAreaParam_t SafetyDetection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;

  // int SafetyDetectionCIRCLE(const std::vector<Pos>& vec_sensor_data) const;
  // int SafetyDetectionBOX(const std::vector<Pos>& vec_sensor_data) const;
  SafetyAreaParam_t SafetyDetectionCIRCLE(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;
  SafetyAreaParam_t SafetyDetectionBOX(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;

private:
  const SafetyAreaParam_t st_collision_area_;
  RobotConfig o_robot_config_;
  std::vector<SafetyAreaParam_t> vec_safety_area_;
  mutable std::mutex mtx_;
  Timer o_log_timer_;
};

} // namespace NVFR

#endif
