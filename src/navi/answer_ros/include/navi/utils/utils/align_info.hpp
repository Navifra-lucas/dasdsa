/*
 * @file	: align_info.hpp
 * @date	: Dec 12, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: class for align information
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef ALIGN_INFO_HPP_
#define ALIGN_INFO_HPP_

#include <memory>

#include "utils/motion_planner_type_list.hpp"

namespace NVFR {

class AlignInfo
{
public:
  using Ptr = std::shared_ptr<AlignInfo>;
  using ConstPtr = std::shared_ptr<const AlignInfo>;

  AlignInfo();
  AlignInfo(double d_target_rad, MPTL::ALIGNDIR e_align_direction=MPTL::ALIGNDIR::AUTO);
  virtual ~AlignInfo() {};

  void Initialize();

  MPTL::ALIGNDIR GetAlignDirection() const { return e_align_direction_; };
  double GetTargetRad() const { return d_target_rad_; };

  void SetAlignDirection(MPTL::ALIGNDIR e_align_direction);
  void SetAlignTargetRad(double d_target_rad);
  void SetAlignInfo(double d_target_rad, MPTL::ALIGNDIR e_align_direction=MPTL::ALIGNDIR::AUTO);

private:
  MPTL::ALIGNDIR e_align_direction_;
  double d_target_rad_; // Align 함수를 위한 변수
};

} // namespace NVFR

#endif
