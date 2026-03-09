/*
 * @file	: motion_prediction.hpp
 * @date	: Jan 17, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: parent class of motion controllers (pp, stanley, mpc, ect)
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MOTION_PREDICTION_HPP_
#define MOTION_PREDICTION_HPP_

#include <stdint.h>
#include <vector>
#include <memory>
#include <mutex>

#include "utils/pose.hpp"
#include "utils/param/sub/motion_param.hpp"

#include "motion_controller/util/traj_info.hpp"
#include "motion_controller/util/control_info.hpp"
#include "motion_controller/opt_solver/speed_optimizer.hpp"

namespace NVFR {

class MotionPrediction
{
public:
  struct Result_t : public SpeedOptimizer::Result_t {
    bool b_success=false;
    int n_idx_interval=1;
    double d_ds=0.01;
    void operator=(const SpeedOptimizer::Result_t& spd_opt_result) {
      this->vecVptr = spd_opt_result.vecVptr;
      this->vecAptr = spd_opt_result.vecAptr;
    }
  };
  MotionPrediction();
  virtual ~MotionPrediction() = default;

  void SetParam(const MotionParam_t& st_param);

  void TrajSmoother(Path& path, const TrajInfo_t& st_traj_info, int n_drive_speed_percent) const;
  int CalcPathIdxInterval(const Path& path, int n_closest_idx, size_t n_pred_step, double v0, double d_dt) const;

  MotionPrediction::Result_t CalcPredSpd(const Path& path, const Pose& o_robot_state, int n_closest_idx, const TrajInfo_t& st_traj_info, const ControlInfo_t& st_control_info);
private:
  double d_motion_period_sec = 0.04;
  int n_motion_pred_step = 30;

  SpeedOptimizer o_spd_opt_;

  MotionPrediction::Result_t SpeedSmoother(const Path& path, const Pose& o_robot_state, int n_closest_idx, const TrajInfo_t& st_traj_info, const ControlInfo_t& st_control_info);

};

} // namespace NVFR

#endif
