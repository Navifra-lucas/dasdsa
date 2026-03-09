/*
 * @file	: motion_explore_executor.hpp
 * @date	: Apr 21, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Decision Maker class for path planner and motion controller
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MOTION_EXPLOER_EXECUTOR_HPP_
#define MOTION_EXPLOER_EXECUTOR_HPP_

#include "navigator/motion_executor/motion_virtual_executor.hpp"

namespace NVFR {

class MotionExploreExecutor : public MotionExecutor
{
public:
  MotionExploreExecutor(TypeHandler::Ptr o_type_handler_ptr, MissionBundle_t& st_mission_bundle, const NavigatorParam_t& st_param);
  virtual ~MotionExploreExecutor();

  virtual bool ModifyMission(MissionBundle_t& st_mission_bundle) override;

protected:
  // init
  virtual void Initialize() override;

  // done condition
  virtual bool IsDone() const override;
  virtual void Motion() override;

private:
  bool ExploreMission(MissionBundle_t& st_mission_bundle, const Pose& o_robot_pose, const GlobalPlannerParam_t& st_param) const;

  bool IsStoped() const;

  bool IsAligning(const MotionHandler& o_motion_handler, const Pose& o_closest_wp, const Pose& o_robot_state) const;

  void Avoid();

  bool IsAbletoAvoid(const PathManager& o_local_path_handler);

  std::condition_variable cv_avoid_;
  mutable std::mutex mtx_avoid_;

  mutable RapidTimer o_rapid_time_;

};

} // namespace NVFR

#endif
