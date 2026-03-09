/*
 * @file	: mpc_solver.hpp
 * @date	: Jan 6, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: mpc solver
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MPC_SOLVER_HPP_
#define MPC_SOLVER_HPP_

#include <vector>
#include <memory>

#include "utils/param/sub/mpc_param.hpp"
#include "utils/osqp_wrapper.hpp"

namespace NVFR {

class MpcSolver
{
public:
  struct State_t {
    OSQPFloat x;      // [0] [m]
    OSQPFloat y;      // [1] [m]
    OSQPFloat vel;    // [2] [m/s]
    OSQPFloat angle;  // [3] [rad]
    OSQPFloat curv;   // [1/m]
  };
  struct Control_t {
    OSQPFloat lin_out=0.0;
    OSQPFloat ang_out=0.0;
  };
  struct AddInfo_t {
    bool b_smooth_stop=false;
    double d_remain_m=1000.0;
    Control_t u_last;
  };

  explicit MpcSolver(int n_step_size, int n_state_size=4);
  virtual ~MpcSolver() = default;

  void SetStepSize(int n_step_size);
  size_t GetStepSize() const { return static_cast<size_t>(N); };

  virtual void SetParam(
    int n_step_size, double d_dt,
    const MPCParam_t& st_param);

  OSQPInt Solve(
    const State_t& z0,
    const Control_t& u0,
    const std::vector<State_t>& traj,
    const AddInfo_t& info);
  std::vector<Control_t> GetSolution() const;

  std::string toStr() const { return qp_.toStr(); };

protected:
  // config
  const OSQPInt z_dim;
  const OSQPInt u_dim;

  // param
  OSQPInt N;
  OSQPFloat dt;
  MPCParam_t st_param_;

  // osqp problem
  OsqpWrapper::QPProblem qp_;

  // index functions
  OSQPInt state_var_idx(OSQPInt t, OSQPInt id) const { return t * z_dim + id; }
  OSQPInt control_var_idx(OSQPInt t, OSQPInt id, OSQPInt N) const { return (N+1) * z_dim + t * u_dim + id; }
  OSQPInt control_derivative_row_idx(OSQPInt t, OSQPInt id, OSQPInt n) const { return n + t * u_dim + id; }

  // model
  virtual void SetMpcModelMatrix(
    const State_t& z0,
    const Control_t& u0,
    const std::vector<State_t>& traj,
    const AddInfo_t& info);

private:
  //

};

} // namespace NVFR

#endif
