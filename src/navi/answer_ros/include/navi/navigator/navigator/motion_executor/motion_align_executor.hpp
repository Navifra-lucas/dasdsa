/*
 * @file	: motion_align_executor.hpp
 * @date	: Feb 28, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Decision Maker class for path planner and motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MOTION_ALIGN_EXECUTOR_HPP_
#define MOTION_ALIGN_EXECUTOR_HPP_

#include "navigator/motion_executor/motion_virtual_executor.hpp"

namespace NVFR {

class MotionAlignExecutor : public MotionExecutor
{
public:
  MotionAlignExecutor(TypeHandler::Ptr o_type_handler_ptr, MissionBundle_t& st_mission_bundle, const NavigatorParam_t& st_param);
  virtual ~MotionAlignExecutor();

  virtual bool ModifyMission(MissionBundle_t& st_mission_bundle) override;

protected:
  // init
  virtual void Initialize() override;

  // done condition
  virtual bool IsDone() const override;
  virtual void Motion() override;

private:

};

} // namespace NVFR

#endif
