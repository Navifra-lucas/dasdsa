/*
 * @file	: explorer.hpp
 * @date	: Jun 12, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: edge contour detection from image
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef EXPLORER_HPP_
#define EXPLORER_HPP_

#include <vector>
#include <string>
#include <tuple>
#include <mutex>
#include <atomic>

#include <opencv2/opencv.hpp>

#include "utils/publish_callback.hpp"
#include "utils/key_list.hpp"
#include "utils/image/image_converter.hpp"

#include "explorer/utils.hpp"
#include "explorer/param/expr_param.hpp"
#include "explorer/expr_config.hpp"
#include "explorer/expr_data.hpp"
#include "explorer/edge_contour_detection.hpp"
#include "explorer/voronoi_detection.hpp"
#include "explorer/view_point_sampling.hpp"
#include "explorer/next_best_view.hpp"
#include "explorer/slam_nodes.hpp"

namespace NVFR {

class Explorer
{
public:
  Explorer();
  ~Explorer();
  void Terminator();

  Pose GetRobotPose() const { return o_expr_data_origin_.GetRobotPose(); }

  // regist callback functions
  template <typename ClassType, typename... Args>
  bool RegistCbFunc(int n_key, void (ClassType::*func)(const Args& ...), ClassType* obj)
  {
    return PublishCb<Args ...>::GetInstance()->RegistCbFunc(n_key, [obj, func](const Args& ...args) { (obj->*func)(args...); });
  }
  template <typename ClassType>
  bool RegistCbFuncVoid(int n_key, void (ClassType::*func)(void), ClassType* obj)
  {
    return PublishCb<void>::GetInstance()->RegistCbFunc(n_key, [obj, func](void) { (obj->*func)(); });
  }

  void Initialize();

  // parameters
  void SetParam(const ExprParam_t& st_param);

  // data set
  bool CheckDataSet(bool b_display=false) const;
  bool CheckRobot() const;
  bool CheckMap() const;
  void SetRobotPos(double d_x_m, double d_y_m, double d_yaw_rad);
  bool SetImage(const cv::Mat& mapImage, float mapResM, float offsetX, float offsetY);
  void SetStartPose(const Pose& o_start_pose);
  void SetSlamNodes(const Polygon& slam_nodes);

  // start
  void StartExplore();
  void EndExplore();

  // explore
  void NewExplore();
  void NextExplore();
  bool Empty();

private:
  // config
  ExprConfig_t st_config_;

  // data set
  ExprData o_expr_data_origin_;
  ExprData o_expr_data_copy_;

  // mutex
  mutable std::mutex mtx_config_;
  mutable std::mutex mtx_data_;

  // atomic variables
  std::atomic<bool> is_running_;
  std::atomic<bool> is_terminated_;

  // object
  EdgeContourDetector o_edge_contour_;
  VoronoiDetector o_voronoi_;
  ViewPointSampling o_vps_;
  NextBestView o_nbv_;
  SlamNodes o_slam_nodes_;

  // start pose
  Pose o_start_pose_;

  // graph nodes of slam
  Polygon vec_slam_nodes_;

  // analytical data
  TimeUtil::_tp tp_start_;

  // sub-fucns
  bool Execute();
  void Analysis();

};

} // namespace NVFR

#endif
