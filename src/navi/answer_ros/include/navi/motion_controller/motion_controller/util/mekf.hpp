/*
 * @file	: mekf.hpp
 * @date	: Aug 26, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Extended Kalman Filter for motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MEKF_HPP_
#define MEKF_HPP_

#include <array>
#include <deque>
#include <blaze/Math.h>

#include "utils/param/sub/mekf_param.hpp"
#include "utils/pose.hpp"

namespace NVFR {

class Mekf
{
using Vec3  = blaze::StaticVector<double, 3UL>;
using Mat33 = blaze::StaticMatrix<double, 3UL, 3UL>;
public:
  Mekf();
  virtual ~Mekf() = default;

  void SetParam(const MekfParam_t& st_mekf_param, double d_dt);
  void SetState(const Pose& o_robot_state);

  Pose Update(const Pose& o_robot_state);

  double GetUncertainty() const;

private:
  // parameters
  MekfParam_t st_param_;
  double d_dt_;

  // robot state
  bool b_init_;
  Pose o_last_robot_state_;

  // blaze vector & matrix
  Mat33 P;        // covariant for update

  // parameters
  Vec3 cvg_thr;   // convergence threshold
  Mat33 Qu;       // sigma matrix for noise of process
  Mat33 R;        // covariant for noise of observation

  // preset memory
  Vec3 x;         // state (movement)
  Vec3 z;         // observation (movement)
  Vec3 y;         // innovation
  Vec3 q;         // max error of process
  Mat33 G;        // gradient of x by u for Q
  Mat33 Q;        // covariant for noise of process
  Mat33 S;        // covariant for innovation
  Mat33 S_inv;    // inverse matrix of S
  Mat33 K;        // kalman gain matrix

  Mat33 Diag(const Vec3& vec3) const;
  Mat33 DiagSq(const Vec3& vec3) const;
  Vec3 Abs(const Vec3& vec3) const;
  double RSS(const Vec3& vec3) const;
  bool InThr(const Vec3& vec3, const Vec3& thr) const;
  bool InThr(const Mat33& mat33, const Vec3& thr) const;
  std::array<double, 3UL> Transform(const Vec3& local, const Vec3& origin) const;

  std::string toStr(const Vec3& vec3) const;
  std::string toStr(const Mat33& mat33) const;

};

} // namespace NVFR

#endif
