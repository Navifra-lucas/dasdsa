/*
 * @file	: motion_handler.hpp
 * @date	: Jun 19, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: handle motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MOTION_HANDLER_HPP_
#define MOTION_HANDLER_HPP_

#include <stdint.h>
#include <vector>
#include <memory>

#include "utils/motion_planner_calculator.hpp"
#include "utils/motion_planner_type_list.hpp"
#include "utils/motion_planner_type.hpp"
#include "utils/pose.hpp"
#include "utils/mission_info.hpp"

#include "utils/data_storage.hpp"

#include "motion_controller/align/align_controller.hpp"
#include "motion_controller/drive/drive_controller.hpp"

namespace NVFR {

class MotionHandler
{
public:
  using Ptr = std::shared_ptr<MotionHandler>;
  using ConstPtr = std::shared_ptr<const MotionHandler>;

  MotionHandler(TypeHandler::Ptr o_type_handler_ptr);
  virtual ~MotionHandler();

  bool IsDone() const;
  bool IsRunning() const { return (e_running_type_ != RUNNING_TYPE::NONE); };
  bool IsAligning() const;
  bool IsAlignedAlready(double d_robot_rad, double d_target_rad) const;

  MPTL::ALIGNDIR GetAlignDirection() const;
  double GetTargetRad() const;
  AlignInfo GetAlignInfo() const;
  double GetRemainRad() const;
  const Path GetPrediction() const { return o_drive_controller_.GetPrediction(); };

  void SetParam(const MotionParam_t& st_param);

  // signal
  void Reset();

  Pose MaxDecelStop(const Pose& o_robot_vel) const;

  Pose AlignExecution(const Pose& o_robot_state, double d_target_rad, MPTL::ALIGNDIR e_align_direction, const TargetSpeedLimit_t& st_target_speed_limit);
  Pose MotionExecution(const Path& path, const Pose& o_robot_state, const DriveInfo& o_drive_info, int n_stop_idx, const TargetSpeedLimit_t& st_target_speed_limit);
  Pose MotionExecutionWithIdx(const Path& path, const Pose& o_robot_state, const DriveInfo& o_drive_info, int n_closest_idx, int n_stop_idx, const TargetSpeedLimit_t& st_target_speed_limit);

private:
  /**
   * @note type: uint8_t
   * @param NONE, ALIGN, DRIVE_ALIGN, DRIVE
  */
  enum class RUNNING_TYPE : uint8_t
  {
    NONE = 0,
    ALIGN = 1,
    DRIVE_ALIGN,
    DRIVE,
  } e_running_type_ = RUNNING_TYPE::NONE;

  TypeHandler::Ptr o_type_handler_ptr_;

  AlignController o_align_controller_;
  DriveController o_drive_controller_;

};

} // namespace NVFR

#endif
