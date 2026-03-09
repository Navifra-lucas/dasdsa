/*
 * @file	: motion_controller.hpp
 * @date	: Jan 17, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: parent class of motion controllers (pp, stanley, mpc, ect)
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MOTION_CONTROLLER_HPP_
#define MOTION_CONTROLLER_HPP_

#include <stdint.h>
#include <vector>
#include <memory>
#include <mutex>

#include "utils/pose.hpp"
#include "utils/param/sub/motion_param.hpp"

#include "motion_controller/data_struct/traj_info.hpp"
#include "motion_controller/data_struct/control_info.hpp"
#include "motion_controller/utils/motion_dynamics.hpp"
#include "motion_controller/opt_solver/speed_optimizer.hpp"

namespace NVFR {

class MotionController
{
public:
  using Ptr = std::shared_ptr<MotionController>;

  struct Result_t : public SpeedOptimizer::Result_t {
    bool b_success=false;
    int n_idx_interval=1;
    double d_ds=0.01;
    void operator=(const SpeedOptimizer::Result_t& spd_opt_result) {
      this->vecVptr = spd_opt_result.vecVptr;
      this->vecAptr = spd_opt_result.vecAptr;
    }
  };
  MotionController();
  virtual ~MotionController() = default;

  virtual void SetParam(const MotionParam_t& st_param);

  virtual void Reset() = 0;

  const Path& GetPrediction() const { return vec_predicted_states_; };

  virtual Pose Control(const Path& path, const Pose& o_robot_state, int n_closest_idx, const TrajInfo_t& st_traj_info, const ControlInfo_t& st_control_info) = 0;

protected:
  MotionParam_t st_param_;
  Path vec_predicted_states_;

public:
  int CalcPathIdxInterval(const Path& path, int n_closest_idx, size_t n_pred_step, double v0, double d_dt) const;

  MotionController::Result_t CalcSpdOpt(const Path& path, const Pose& o_robot_state, int n_closest_idx, const TrajInfo_t& st_traj_info, const ControlInfo_t& st_control_info);

private:
  SpeedOptimizer o_spd_opt_;

  MotionController::Result_t SpeedSmoother(const Path& path, const Pose& o_robot_state, int n_closest_idx, const TrajInfo_t& st_traj_info, const ControlInfo_t& st_control_info);

};

} // namespace NVFR

#endif
