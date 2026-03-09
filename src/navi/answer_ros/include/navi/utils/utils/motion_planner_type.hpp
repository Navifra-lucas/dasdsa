/*
 * @file	: motion_planner_type.hpp
 * @date	: Mar 5, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Type Manager of Kinematics, Motion and Planner
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MOTION_PLANNER_TYPE_HPP_
#define MOTION_PLANNER_TYPE_HPP_

#include <stdint.h>
#include <vector>
#include <memory>

#include "utils/motion_planner_type_list.hpp"

namespace NVFR {

class MotionPlannerType
{
public:
  using Ptr = std::shared_ptr<MotionPlannerType>;

  MotionPlannerType(const MPTL::KINEMATICS e_kinematics_type, const MPTL::CONTROLLER e_controller_type, const uint16_t n_avoid_type_list, const bool b_able_to_align=true);
  virtual ~MotionPlannerType() {};

  MPTL::KINEMATICS GetKinematics() const { return e_kinematics_type_; };
  MPTL::CONTROLLER GetController() const { return e_controller_type_; };
  uint16_t GetAvoidList() const { return n_avoid_type_list_; };
  bool GetAbletoAlign() const { return b_able_to_align_; };

private:
  const MPTL::KINEMATICS e_kinematics_type_;
  const MPTL::CONTROLLER e_controller_type_;
  const uint16_t n_avoid_type_list_;
  const bool b_able_to_align_;
};

class TypeHandler
{
public:
  using Ptr = std::shared_ptr<TypeHandler>;

  TypeHandler(const MPTL::KINEMATICS e_kinematics_type, const MPTL::CONTROLLER e_controller_type, const MPTL::AVOID e_avoid_type=MPTL::AVOID::NONE, const bool b_able_to_align=true);
  TypeHandler(const MPTL::KINEMATICS e_kinematics_type, const MPTL::CONTROLLER e_controller_type, const std::vector<MPTL::AVOID> vec_avoid_type_list, const bool b_able_to_align=true);
  virtual ~TypeHandler();

  MotionPlannerType::Ptr GetTypePtr() const { return o_type_ptr_; };

  void ShowType() const;

  /**
   * @brief check this robot is able to align or not
   * @return true (able) / false (disable)
  */
  bool IsAbletoAlign() const { return o_type_ptr_->GetAbletoAlign(); };
  /**
   * @brief verify to reverse type
   * @param e_move_dir [uint8_t | MOVEDIR]
   * @param e_drive_type [uint8_t | DRIVE]
   * @return true (reverse type) / false (non-reverse type)
  */
  bool IsReverseType(MPTL::MOVEDIR e_move_dir, MPTL::DRIVE e_drive_type) const;
  /**
   * @brief verify to be able quad control
   * @param e_drive_type [uint8_t | DRIVE]
   * @return true (quad control) / false (bike control)
  */
  bool IsQuadType(MPTL::DRIVE e_drive_type) const;
  /**
   * @brief verify to be able to use a drive type
   * @param e_drive_type [uint8_t | DRIVE] drive type that is checked available type
   * @return true (disable) / false (able)
  */
  bool IsDisableDriveType(MPTL::DRIVE e_drive_type) const;
  /**
   * @brief verify to be able to use a planner type
   * @param e_avoid_type [uint8_t | AVOID] avoid type that is checked available type
   * @return true (disable) / false (able)
  */
  bool IsDisableAvoidType(MPTL::AVOID e_avoid_type) const;

private:
  MotionPlannerType::Ptr o_type_ptr_;
};

} // namespace NVFR

#endif
