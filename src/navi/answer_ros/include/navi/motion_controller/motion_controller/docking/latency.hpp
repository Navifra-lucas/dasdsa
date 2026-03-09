/*
 * @file	: latency.hpp
 * @date	: Aug 25, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: pose + time
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef LATENCY_HPP_
#define LATENCY_HPP_

#include <queue>
#include <mutex>

#include "utils/pose.hpp"

#include "motion_controller/docking/pose_stamp.hpp"

namespace NVFR {

class Latency
{
public:
  Latency(int64_t n_delay_msec = 0);
  virtual ~Latency() = default;

  void Initialize(int64_t n_delay_msec);
  void Push(const Pose& o_pose);
  void Pop();
  bool Empty() const;
  Pose GetPose() const;

  virtual std::string toStr() const;
  friend std::ostream& operator<<(std::ostream& os, const Latency& o);

private:
  int64_t n_delay_msec_;
  std::queue<PoseStamp> qu_;
  mutable std::mutex mtx_;

};

} // namespace NVFR

#endif
