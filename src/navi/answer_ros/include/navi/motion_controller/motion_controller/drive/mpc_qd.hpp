/*
 * @file	: mpc_qd.hpp
 * @date	: Dec 15, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: model predictive controller (mpc)
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MPC_QD_HPP_
#define MPC_QD_HPP_

#include "motion_controller/drive/motion_controller.hpp"
#include "motion_controller/opt_solver/mpc_qd_solver.hpp"
#include "motion_controller/utils/pid_controller.hpp"
#include "motion_controller/utils/mekf.hpp"

namespace NVFR {

class MPC_QD : public MotionController
{
public:
  MPC_QD();

  virtual void SetParam(const MotionParam_t& st_param) override;

  virtual void Reset() override;

  virtual Pose Control(const Path& path, const Pose& o_robot_state, int n_closest_idx, const TrajInfo_t& st_traj_info, const ControlInfo_t& st_control_info) override;

private:
  MpcQdSolver o_mpc_qd_solver_;
  Mekf o_mekf_;

  MpcQdSolver::State_t z_last_;
  MpcQdSolver::Control_t u_last_;

};

} // namespace NVFR

#endif
