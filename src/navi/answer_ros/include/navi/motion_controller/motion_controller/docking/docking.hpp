/*
 * @file	: docking.hpp
 * @date	: Aug 25, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: docking system
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef DOCKING_HPP_
#define DOCKING_HPP_

#include <mutex>

#include "utils/param/sub/docking_param.hpp"
#include "utils/data_manager.hpp"
#include "utils/pose.hpp"

#include "motion_controller/docking/latency.hpp"
#include "motion_controller/docking/dekf.hpp"

namespace NVFR {

class Docking
{
public:
  Docking(
    const Pose& o_robot2sensor,
    const Pose& o_map2object,
    const Pose& o_object2goal,
    const DockingParam_t& st_param);
  virtual ~Docking() = default;

  // switch (on/off)
  bool GetSwitch() const;
  void TurnOn();
  void TurnOff();

  // get estimated goal pose
  Pose GetEstGoalPose() const;

  // transfer robot pose from estimated goal frame to origin goal frame
  Pose GetInvTfPose(const Pose& o_pose) const;

  // transfer pose from origin goal frame to estimated goal frame
  Pose GetTfPose(const Pose& o_pose) const;
  // transfer pose from origin goal frame to estimated goal frame
  void EditTfPose(Pose& o_pose) const;
  // transfer path from origin goal frame to estimated goal frame
  Path GetTfPath(const Path& vec_path) const;
  // transfer path from origin goal frame to estimated goal frame
  void EditTfPath(Path& vec_path) const;

  void SetRobotPose(const Pose& o_robot_pose);

  /**
   * @brief update ekf : if ekf is updated, return true
   * @param o_Sensor2Object measurement (z) based on sensor
   * @return true (updated) / false (not initialied or convergenced)
   */
  bool SetSensor2Object(const Pose& o_Sensor2Object);

  /**
   * @brief update ekf : if ekf is updated, return true
   * @param o_Map2Object measurement (z) based on map
   * @return true (updated) / false (not initialied or convergenced)
   */
  bool SetMap2Object(const Pose& o_Map2Object);

private:
  // switch (on/off)
  bool b_switch_ = false;
  mutable std::mutex mtx_switch_;

  // static tf
  Pose o_Robot2Sensor_;
  Pose o_Object2Goal_;
  Pose o_Map2Goal_;

  // tf by estimation of goal
  DataManager<Pose> o_Map2EstGoal_;

  // object
  Latency o_latency_based_robot_;
  Dekf o_ekf_;

  bool ExecuteEkf(const Pose& o_robot_pose, const Pose& o_object_pose);

};

} // namespace NVFR

#endif
