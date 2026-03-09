#ifndef NAVIFRA_LOCALIZER_H_
#define NAVIFRA_LOCALIZER_H_
#include "common/pose2d.h"
#include "common/scan2d.h"
#include "common/voxel_filter.h"
#include "globalmapper/mapper.h"
#include "icp/localizer_parameter_container.hpp"
#include "icp/point_to_plane_icp.hpp"
#include "icp2d.hpp"
#include "localmapper/localmap/grid2d.h"
#include "localmapper/scanmatcher/ceres_scan_matcher.h"
#include "simplepos/simplepos.hpp"

#include <iostream>
#include <memory>
#include <thread>
#include <vector>

using namespace std;

namespace NaviFra {

class nc_localizer {
private:
  /* data */

  NaviFra::SimplePos o_prev_robot_pos_; // 이전 로봇 pos (ICP결과)
  NaviFra::SimplePos
      o_robot_pos_; // 현재 로봇 pos (지난 ICP결과 + Odometry 쌓일때마다 갱신)
  NaviFra::SimplePos o_vel_pos_; // 현재 로봇 속도 (ICP 실행 전 각속도 검사)

  NaviFra::SimplePos o_prev_odom_; // 이전 로봇 Odom
  NaviFra::SimplePos
      o_current_odom_; // 현재 로봇 Odom (오돔 차이 구해서 o_robot_pos_ 갱신)
  NaviFra::SimplePos
      o_delta_odom_; // 전프레임과 Odom 차이 (ICP 실행 전 위치차이 검사)

  vector<SimplePos>
      vec_current_data_; // scan_cloud data from lidar_merger (laser frame)

  std::thread localization_thread_;
  LOCALIZE_RESULT st_localize_result_;

  shared_ptr<PointToPlaneICP> localization_matcher_;
  shared_ptr<PointToPlaneICP> initial_pose_matcher_;
  shared_ptr<localizer_parameter_container> param_container_;

  std::mutex mtx_lock_robot_pose_;
  std::mutex mtx_localizer_run_check_;
  std::mutex mtx_localizer_error_code_;
  // for lidar odometry
  unique_ptr<NaviFra::SLAM2D::CeresScanMatcher> scan_matcher_;
  shared_ptr<NaviFra::SLAM2D::Localmap2D> current_localmap_;
  NaviFra::SLAM2D::Pose2D map_to_robot_;
  NaviFra::SLAM2D::Pose2D last_map_to_robot_;
  bool b_set_localmap_ = false;
  bool b_initialized_ = false;
  float f_last_confidence_ = 0.f;

  std::chrono::steady_clock::time_point odom_time_; // When Odom data is entered
  std::chrono::steady_clock::time_point
      scan_time_; // When scan_cloud data is entered

  // ----------------------- Parameters  --------------------------------

  bool b_localization_run_ = false; // 현재 Localize thread가 실행중인지

  float f_acc_dist_m_ = 0.f; // 오도메트리에 의해 추정된 로봇의 이동거리
                             // (Between ICP 수행단위간) why acc?
  float f_acc_deg_ = 0.f; // 오도메트리에 의해 추정된 로봇의 회전량 Unit : Deg
                          // (Between ICP 수행단위간)
  float f_acc_steer_deg_ = 0.f;

  float f_external_acc_dist_m_ =
      0.f; // 외부에서 받은 로봇의 이동거리 (Between ICP 수행단위간)
  float f_external_acc_deg_ =
      0.f; // 외부에서 받은 로봇의 회전량 Unit : Deg (Between ICP 수행단위간)

  float f_initial_condition_dist_ = 0.f;      // For QR localization
  float f_initial_condition_deg_ = 0.f;       // For QR localization
  float f_initial_condition_steer_deg_ = 0.f; // For QR localization

  int n_error_count_ = 0; // ICP large gap error count

  // ----------------------- Not used  --------------------------------
  bool b_init_odom_ = false;                // Not used
  int n_localize_result_error_code_ = 0;    // Not used
  float f_rotate_vel_condition_deg_ = 1.0f; // Not used
  float f_del_odom_condition_dist_ = 0.01f; // Not used

  std::function<void(const string &s_json_log)> PublishJsonLog;

  // std::chrono::steady_clock::time_point navi_time_;
  // std::chrono::steady_clock::time_point icp_time_;
  // std::chrono::steady_clock::time_point init_pose_time_;
  // std::chrono::steady_clock::time_point slam_time_;

  // ----------------------- Main process --------------------------------

  void LocalizeThread();
  bool CheckLocalizeCondition();
  bool IsZero(const SimplePos &o_pos1) {
    if (fabs(o_pos1.GetXm()) < FLT_EPSILON &&
        fabs(o_pos1.GetYm()) < FLT_EPSILON &&
        fabs(o_pos1.GetDeg()) < FLT_EPSILON)
      return true;
    return false;
  }
  std::mutex mtx_lock_external_;
  std::chrono::steady_clock::time_point external_pose_time_;
  TimeChecker tc_;

  std::shared_ptr<icp2d> o_icp_;
  void OnLiDAROdometry(const std::vector<SimplePos> &vec_current_data);
  void OffLiDAROdometry();

public:
  nc_localizer(/* args */);
  ~nc_localizer();

  // ----------------------- Main methods --------------------------------

  void InitializeLocalizer();
  void FindInitPose(const std::vector<SimplePos> &vec_current_data);
  bool Localize(const std::vector<SimplePos> &vec_current_data);

  // ----------------------- Get Set -----------------------------------

  bool
  SetMap(const std::vector<NaviFra::SimplePos> &vec_icp_db); // From Map server
  void SetVel(const NaviFra::SimplePos &o_vel_pose);       // From Motor driver
  void SetOdometry(const NaviFra::SimplePos &o_odom_pose); // From Motor driver
  void SetSteerDeg(const float &f_deg);                    // From Motor driver
  void
  SetJSONLogCallback(std::function<void(const string &s_json_log)> callback) {
    PublishJsonLog = callback;
  }

  bool ReadParameter();
  shared_ptr<localizer_parameter_container> GetParameter() {
    return param_container_;
  }

  const NaviFra::SimplePos GetRobotPose() {
    NaviFra::SimplePos o_robot_pos;

    {
      std::lock_guard<std::mutex> lock(mtx_lock_robot_pose_);
      o_robot_pos = o_robot_pos_;
    }

    return o_robot_pos;
  }

  void SetRobotPose(const NaviFra::SimplePos &o_robot_pos) {
    {
      std::lock_guard<std::mutex> lock(mtx_lock_robot_pose_);
      o_robot_pos_ = o_robot_pos;
      o_prev_robot_pos_ = o_robot_pos_;
      f_acc_dist_m_ = 0.f;
      f_acc_deg_ = 0.f;
    }

    InitErrorCount();
  }

  const LOCALIZE_RESULT GetLocalizeResult() {
    LOCALIZE_RESULT st_localize_result;

    {
      std::lock_guard<std::mutex> lock(mtx_lock_robot_pose_);
      st_localize_result = st_localize_result_;
    }
    return st_localize_result;
  }
  void ResetExternalPosTime();
  float GetExternalPosTime();
  bool GetExternalPosDistDeg();

  const int GetLocalizeErrorCode() { return n_localize_result_error_code_; }

  void ErrorClear() { n_localize_result_error_code_ = 0; }

  //
  void InitErrorCount();

  // ----------------------- Not used -----------------------------------

  void AddOdometry(const NaviFra::SimplePos &prev_to_current);
  void LocalizeOnce();
};
} // namespace NaviFra
#endif