/*
 * @file	: avoid_info.hpp
 * @date	: Dec 12, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: class for mission types
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef AVOID_INFO_HPP_
#define AVOID_INFO_HPP_

#include <iostream>

#include "utils/motion_planner_type_list.hpp"
#include "utils/motion_planner_type.hpp"
#include "utils/lane_info.hpp"

namespace NVFR {

class AvoidInfo
{
public:
  AvoidInfo();
  explicit AvoidInfo(MPTL::AVOID e_avoid_type, const LaneInfo& o_lane_info);
  virtual ~AvoidInfo() {};

  MPTL::AVOID GetAvoidType() const { return e_avoid_type_; };
  LaneInfo GetLaneInfo() const { return o_lane_info_; };

  void SetAvoidType(MPTL::AVOID e_avoid_type);
  void SetLaneInfo(LaneInfo o_lane_info);
  void SetAvoidInfo(MPTL::AVOID e_avoid_type, const LaneInfo& o_lane_info);

  AvoidInfo Reverse() const;

  /**
   * @brief check this mission info is valid or not
   * @return true (valid) / false (invalid)
  */
  bool IsValid(TypeHandler::Ptr o_type_handler_ptr) const;

  virtual std::string toStr() const;
  friend std::ostream& operator<<(std::ostream& os, const AvoidInfo& o);

private:
  MPTL::AVOID e_avoid_type_;

  LaneInfo o_lane_info_;
};

} // namespace NVFR

#endif
