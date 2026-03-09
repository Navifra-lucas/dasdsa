/*
 * @file	: trajectory.hpp
 * @date	: Jan 17, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: parent class of motion controllers (pp, stanley, mpc, ect)
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

#include <stdint.h>
#include <vector>
#include <memory>
#include <mutex>

#include "utils/pose.hpp"

#include "motion_controller/data_struct/traj_info.hpp"

namespace NVFR {

class Trajectory
{
public:
  Trajectory() = default;
  virtual ~Trajectory() = default;

  void SmoothTrajectory(
    Path& path,
    const TrajInfo_t& st_traj_info,
    int n_drive_speed_percent,
    double d_motion_period_sec) const;

private:

};

} // namespace NVFR

#endif
