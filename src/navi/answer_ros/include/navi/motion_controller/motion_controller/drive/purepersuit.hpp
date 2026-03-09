/*
 * @file	: purepersuit.hpp
 * @date	: Jun 19, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: purepersuit motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef PUREPERSUIT_HPP_
#define PUREPERSUIT_HPP_

#include "motion_controller/drive/motion_controller.hpp"
#include "motion_controller/utils/pid_controller.hpp"

namespace NVFR {

class Purepersuit : public MotionController
{
public:
  Purepersuit();

  virtual void SetParam(const MotionParam_t& st_param) override;

  virtual void Reset() override;

  virtual Pose Control(const Path& path, const Pose& o_robot_state, int n_closest_idx, const TrajInfo_t& st_traj_info, const ControlInfo_t& st_control_info) override;

private:
  PID<double> o_pid_a_;
  PID<double> o_pid_w_;

  virtual double CalcPPCurv(const Path& path, const Pose& o_robot_state, int n_closest_idx, const ControlInfo_t& st_control_info) const;

};

} // namespace NVFR

#endif
