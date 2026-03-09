/*
 * @file	: align_controller.hpp
 * @date	: Jun 19, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: controller to align
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef ALIGN_CONTROLLER_HPP_
#define ALIGN_CONTROLLER_HPP_

#include <stdint.h>
#include <vector>
#include <memory>

#include "utils/motion_planner_type_list.hpp"
#include "utils/align_info.hpp"
#include "utils/pose.hpp"
#include "utils/param/navigator_param.hpp"

#include "motion_controller/data_struct/target_speed_limit.hpp"

namespace NVFR {

class AlignController
{
public:
  AlignController();
  virtual ~AlignController() {};

  bool IsIdle() const { return e_status_ == STATUS::IDLE; };
  bool IsDone() const { return e_status_ == STATUS::DONE; };
  bool IsAligned(double d_robot_rad, double d_target_rad, double d_threshold_rad) const;
  bool IsAlignedAlready(double d_robot_rad, double d_target_rad) const;
  MPTL::ALIGNDIR GetAlignDirection() const;
  double GetTargetRad() const;
  double GetRemainRad() const;

  void SetParam(const MotionParam_t& st_param);

  void Reset();
  bool ResetIfDone();

  Pose MaxDecelStop(const Pose& o_robot_vel) const;

  void Initialize(double d_robot_rad, double d_target_rad, MPTL::ALIGNDIR e_align_direction);
  Pose Execution(const Pose& o_robot_state, double d_target_rad, MPTL::ALIGNDIR e_align_direction, const TargetSpeedLimit_t& st_target_speed_limit);
  Pose Controller(double d_robot_vw, double d_remain_rad, const TargetSpeedLimit_t& st_target_speed_limit);
  static void CalcAlignState(double& d_remain_rad, MPTL::ALIGNDIR& e_align_direction);

private:
  /**
   * @note type: uint8_t
   * @param IDLE, RUNNING, NEAR_GOAL, DONE
  */
  enum class STATUS : uint8_t
  {
    IDLE = 0,
    RUNNING = 1,
    NEAR_GOAL,
    DONE
  } e_status_ = AlignController::STATUS::IDLE;

  struct ModifiedAlignParam_t
  {
    double d_motion_period_sec=0.04; // [sec]

    double d_threshold_for_new_dyaw_rad=0.00872665; // [rad]
    double d_threshold_for_end_rad=0.00872665; // [rad]
    double d_threshold_for_start_motion_rad=0.0872665; // [rad]
    double d_arround_target_min_vel_angle_rad=0.0174533; // [rad]
    double d_min_vel_rads=0.0174533; // [rad/s]
    double d_max_vel_rads=0.436332; // [rad/s]
    double d_accel_radss=0.174533; // [rad/ss]
    double d_decel_radss=0.174533; // [rad/ss]
    double d_max_decel_radss = 1.74533; // [m/ss]
  } st_param_;

  MPTL::ALIGNDIR e_align_direction_;
  double d_target_rad_;
  double d_remain_rad_;

};

} // namespace NVFR

#endif
