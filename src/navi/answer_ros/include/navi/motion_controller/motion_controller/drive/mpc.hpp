/*
 * @file	: mpc.hpp
 * @date	: Jun 19, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: model predictive controller (mpc)
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MPC_HPP_
#define MPC_HPP_

#include "motion_controller/drive/motion_controller.hpp"
#include "motion_controller/opt_solver/mpc_solver.hpp"
#include "motion_controller/utils/pid_controller.hpp"
#include "motion_controller/utils/mekf.hpp"

namespace NVFR {

class MPC : public MotionController
{
public:
  MPC();

  virtual void SetParam(const MotionParam_t& st_param) override;

  virtual void Reset() override;

  virtual Pose Control(const Path& path, const Pose& o_robot_state, int n_closest_idx, const TrajInfo_t& st_traj_info, const ControlInfo_t& st_control_info) override;

private:
  MpcSolver o_mpc_solver_;
  PID<double> o_pid_vw_;
  Mekf o_mekf_;

  MpcSolver::Control_t u_last_;

};

} // namespace NVFR

#endif
