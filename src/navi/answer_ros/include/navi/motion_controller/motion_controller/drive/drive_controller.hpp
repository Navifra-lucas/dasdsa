/*
 * @file	: drive_controller.hpp
 * @date	: Jun 19, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: parent class of motion controllers (pp, stanley, mpc, ect)
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef DRIVE_CONTROLLER_HPP_
#define DRIVE_CONTROLLER_HPP_

#include <stdint.h>
#include <vector>
#include <memory>

#include "utils/motion_planner_calculator.hpp"
#include "utils/motion_planner_type_list.hpp"
#include "utils/motion_planner_type.hpp"
#include "utils/pose.hpp"
#include "utils/mission_info.hpp"
#include "utils/param/navigator_param.hpp"

#include "motion_controller/data_struct/traj_info.hpp"
#include "motion_controller/data_struct/control_info.hpp"
#include "motion_controller/data_struct/last_control.hpp"
#include "motion_controller/data_struct/target_speed_limit.hpp"
#include "motion_controller/drive/trajectory.hpp"

#include "motion_controller/drive/motion_controller.hpp"
#include "motion_controller/drive/purepersuit.hpp"
#include "motion_controller/drive/stanley.hpp"
#include "motion_controller/drive/hec.hpp"
#include "motion_controller/drive/mpc.hpp"
#include "motion_controller/drive/mpc_qd.hpp"
#include "motion_controller/drive/mpc_sd.hpp"

namespace NVFR {

class DriveController
{
public:
  DriveController(MotionPlannerType::Ptr);
  virtual ~DriveController() {};

  bool IsIdle() const { return e_status_ == STATUS::IDLE; };
  bool IsDone() const { return e_status_ == STATUS::DONE; };

  const Path& GetPrediction() const { return o_mc_ptr_->GetPrediction(); };

  virtual void SetParam(const MotionParam_t& st_param);

  void Reset();
  bool ResetIfDone();

  Pose MaxDecelStop(const Pose& o_robot_vel) const;

  virtual void Initialize();
  Pose Execution(const Path& raw_path, const Pose& o_robot_state, const DriveInfo& o_drive_info, int n_closest_idx, int n_stop_idx, const TargetSpeedLimit_t& st_target_speed_limit);

protected:
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
  } e_status_ = STATUS::IDLE;
  enum class CONTROL_STEP : uint8_t
  {
    STOP = 0,
    NORMAL_START = 1,
    SLOW_START,
    NORMAL,
    DECEL,
    DOCKING,
  } e_control_step_;

  MotionParam_t st_param_;
  LastControl_t st_last_control_;
  Trajectory o_trajectory_;

  MotionController::Ptr o_mc_ptr_;

private:
  bool Exception(const Path& path, int n_closest_idx) const;

public:
  static void SetTrajInfo(TrajInfo_t& st_traj_info, const MPTL::START& e_start, const MPTL::GOAL& e_goal, double d_drive_limit_m_s, const MotionParam_t& st_param);
  static void SetControlInfo(ControlInfo_t& st_control_info, const LastControl_t& st_last_control, const Path& path, const Pose& o_robot_state, int n_closest_idx, const MotionParam_t& st_param);
  static void CalcLastControl(const Pose& o_control_output, LastControl_t& st_last_control, double d_dt);

};

} // namespace NVFR

#endif
