/*
 * @file	: expr_data.hpp
 * @date	: Jun 12, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: edge contour detection from image
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef EXPLORER_EXPR_DATA_HPP_
#define EXPLORER_EXPR_DATA_HPP_

#include <iostream>
#include <sstream>

#include <opencv2/opencv.hpp>

#include "utils/time_util.hpp"
#include "utils/pose.hpp"

namespace NVFR {

class ExprData
{
public:
  ExprData();
  ~ExprData();
  void Terminator();

  void operator=(const ExprData& rhs);

  bool CheckDataSet(bool b_display=false) const;
  bool CheckRobot() const;
  bool CheckMap() const;

  std::string toStr() const;
  friend std::ostream& operator<<(std::ostream& os, const ExprData& o);

  // copy count
  uint64_t GetCount() const { return n_cnt; };

  // robot
  void SetRobotPose(double d_x_m, double d_y_m, double d_yaw_rad);
  const Pose& GetRobotPose() const { return o_robot_pose; };

  // map
  bool SetMap(const cv::Mat& image, float mapResM, float offsetX, float offsetY);
  const cv::Mat& GetMapImage() const { return mapImage; };
  float GetMapRes() const { return f_res; };
  float GetMapOffsetX() const { return f_offset_x; };
  float GetMapOffsetY() const { return f_offset_y; };

private:
  // copy count
  mutable uint64_t n_cnt;

  // this unique time point
  TimeUtil::_tp tp_unique;

  // this origin time point
  mutable TimeUtil::_tp tp_origin;

  // robot
  Pose o_robot_pose;
  TimeUtil::_tp tp_robot;

  // map
  cv::Mat mapImage;
  float f_res, f_offset_x, f_offset_y;
  TimeUtil::_tp tp_map;

};

} // namespace NVFR

#endif
