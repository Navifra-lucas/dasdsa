/*
 * @file	: dekf.hpp
 * @date	: Aug 26, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Extended Kalman Filter for docking
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef DEKF_HPP_
#define DEKF_HPP_

#include <array>
#include <deque>
#include <mutex>
#include <blaze/Math.h>

#include "utils/param/sub/dekf_param.hpp"
#include "utils/timer.hpp"

namespace NVFR {

class Dekf
{
using Vec3  = blaze::StaticVector<double, 3UL>;
using Vec4  = blaze::StaticVector<double, 4UL>;
using Mat33 = blaze::StaticMatrix<double, 3UL, 3UL>;
public:
  /**
   * @note type: unsigned char
   * @param INIT, UPDATE, CONVERGENCE
  */
  enum class STATUS : unsigned char {
    INIT = 0,
    UPDATE = 1,
    CONVERGENCE,
  };
  Dekf(const DekfParam_t& st_dekf_param);
  virtual ~Dekf() = default;

  STATUS GetStatus() const;
  std::array<double, 3UL> GetEstimation() const;

  void SetRobotPose(const std::array<double, 3UL>& robot_pose);
  void SetMeasurement(const std::array<double, 3UL>& measurement);
  void Update();

private:
  // status
  STATUS e_status_;
  mutable std::mutex mtx_status_;

  // parameters
  DekfParam_t st_param_;

  // log timer
  RapidTimer o_rapid_timer_;

  // robot pose & movement
  bool b_init_pose_;
  Vec3 movement_;
  Vec3 last_robot_pose_;
  mutable std::mutex mtx_robot_;

  // measurement
  std::deque< std::array<double, 3UL> > dq_buffer_;
  mutable std::mutex mtx_msm_;

  // output
  Vec3 x_out;     // output data of extimation
  mutable std::mutex mtx_out_;

  // blaze vector & matrix
  Vec3 x;         // estimation
  Mat33 P;        // covariant for update

  // parameters
  Vec3 msm_thr;   // measurement threshold
  Vec3 cvg_thr;   // convergence threshold
  Mat33 Q0;       // default covariant for noise of process
  Mat33 Qm;       // model of additinal covariant for noise of process
  Mat33 R0;       // default covariant for noise of measurement
  Mat33 Rm;       // model of additinal covariant for noise of measurement

  // preset memory
  Vec4 z_avg;     // average of measurement by buffer
  Vec3 z;         // measurement (with buffer)
  Vec3 y;         // innovation
  Mat33 H;        // model matrix (state -> measurement)
  Mat33 H_trans;  // transpose matrix of H
  Mat33 Q;        // covariant for noise of process
  Mat33 R;        // covariant for noise of measurement
  Mat33 S;        // covariant for innovation
  Mat33 S_inv;    // inverse matrix of S
  Mat33 K;        // kalman gain matrix

  void SetStatus(const STATUS& e_status);

  Mat33 ConvertVec2Mat(const Vec3& vec3) const;
  bool InThr(const Vec3& vec3, const Vec3& thr) const;
  bool InThr(const Mat33& mat33, const Vec3& thr) const;

  std::string toStr(const Vec3& vec3) const;
  std::string toStr(const Mat33& mat33) const;

};

} // namespace NVFR

#endif
