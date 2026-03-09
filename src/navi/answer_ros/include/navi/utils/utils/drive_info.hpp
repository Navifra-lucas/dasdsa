/*
 * @file	: drive_info.hpp
 * @date	: Dec 12, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: class for drive information
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef DRIVE_INFO_HPP_
#define DRIVE_INFO_HPP_

#include <iostream>
#include <sstream>
#include <string>
#include <memory>

#include "utils/motion_planner_type_list.hpp"

namespace NVFR {

class DriveInfo
{
public:
  using Ptr = std::shared_ptr<DriveInfo>;
  using ConstPtr = std::shared_ptr<const DriveInfo>;

  DriveInfo();
  DriveInfo(MPTL::MOVEDIR _e_move_dir, MPTL::DRIVE _e_drive_type, MPTL::START _e_start_type, MPTL::GOAL _e_goal_type);
  virtual ~DriveInfo() {};

  MPTL::MOVEDIR GetMoveDir() const { return e_move_dir_; };
  MPTL::DRIVE GetDriveType() const { return e_drive_type_; };
  MPTL::START GetStartType() const { return e_start_type_; };
  MPTL::GOAL GetGoalType() const { return e_goal_type_; };

  void SetMoveDir(const MPTL::MOVEDIR& e_move_dir);
  void SetDriveType(const MPTL::DRIVE& e_drive_type);
  void SetStartType(const MPTL::START& e_start_type);
  void SetGoalType(const MPTL::GOAL& e_goal_type);

  bool IsSimilar(const DriveInfo& rhs) const;

  std::string toStr() const;
  friend std::ostream& operator<<(std::ostream& os, const DriveInfo& o);

private:
  MPTL::MOVEDIR e_move_dir_;
  MPTL::DRIVE e_drive_type_;
  MPTL::START e_start_type_;
  MPTL::GOAL e_goal_type_;

};

} // namespace NVFR

#endif
