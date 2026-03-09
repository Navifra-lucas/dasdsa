/*
 * @file	: pose_stamp.hpp
 * @date	: Aug 25, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: pose + time
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef POSE_STAMP_HPP_
#define POSE_STAMP_HPP_

#include "utils/pose.hpp"
#include "utils/time_util.hpp"

namespace NVFR {

class PoseStamp
{
public:
  PoseStamp(const Pose& o_pose);
  virtual ~PoseStamp() = default;

  const Pose& GetPose() const;
  bool IsOld(int64_t n_msec) const;
  bool IsOld(const TimeUtil::_tp& tp_cmp, const int64_t& n_msec) const;

  virtual std::string toStr(const TimeUtil::_tp& tp_cmp = TimeUtil::GetNow()) const;
  friend std::ostream& operator<<(std::ostream& os, const PoseStamp& o);

private:
  Pose o_pose_;
  TimeUtil::_tp tp_;

};

} // namespace NVFR

#endif
