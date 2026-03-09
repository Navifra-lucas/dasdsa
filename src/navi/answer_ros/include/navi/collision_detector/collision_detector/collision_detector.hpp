/*
 * @file	: collision_detector.hpp
 * @date	: Feb 28, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Collision detector
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef COLLISION_DETECTOR_HPP_
#define COLLISION_DETECTOR_HPP_

#include <memory>
#include <stdint.h>

#include "utils/pose.hpp"
#include "utils/robot_config.hpp"
#include "utils/motion_planner_type_list.hpp"
#include "utils/grid_map/grid_map_calculator.hpp"

namespace NVFR {

namespace CollisionDetector {

/**
 * @brief check collision on the path based on map coordinate
 * @param grid_map grid map
 * @param path it is global path to need to check collision
 * @param d_margin_m collision margin
 * @param b_backward direction of the movement of the robot
 * @param n_start_idx start index in the path
 * @return the collision index on the path | -1 : collision free
 */
int CheckPathCollision(const std::vector<Pose>& path, const std::vector<int8_t>& grid_map, const Pose& o_center_pose, const MapInfo_t& st_map_info, const Padding_t& st_padding, const RobotConfig& o_robot_config, double d_margin_m, bool b_backward, int n_start_idx=0);

/**
 * @brief check collision at the position based on map coordinate
 * @param grid_map grid map
 * @param o_pose it is global pose to need to check collision
 * @param d_margin_m collision margin
 * @param b_backward direction of the movement of the robot
 * @return true : collision | false : nothing
 */
bool CheckPoseCollision(const Pose& o_pose, const std::vector<int8_t>& grid_map, const Pose& o_center_pose, const MapInfo_t& st_map_info, const Padding_t& st_padding, const RobotConfig& o_robot_config, double d_margin_m, bool b_backward);

/**
 * @brief check collision (one-rotation) at the position based on map coordinate
 * @param grid_map grid map
 * @param o_pose it is global pose to need to check collision
 * @param d_margin_m collision margin
 * @return true : collision | false : nothing
 */
bool CheckSpinCollision(const Pose& o_pose, const std::vector<int8_t>& grid_map, const Pose& o_center_pose, const MapInfo_t& st_map_info, const Padding_t& st_padding, const RobotConfig& o_robot_config, double d_margin_m);

int CheckPathCollisionCIRCLE(const std::vector<Pose>& path, const std::vector<int8_t>& grid_map, const Pose& o_center_pose, const MapInfo_t& st_map_info, const Padding_t& st_padding, double d_margin_m, int n_start_idx=0);
int CheckPathCollisionBOX(const std::vector<Pose>& path, const std::vector<int8_t>& grid_map, const Pose& o_center_pose, const MapInfo_t& st_map_info, const Padding_t& st_padding, const RobotConfig& o_robot_config, double d_margin_m, bool b_backward, int n_start_idx=0);

bool CheckPoseCollisionCIRCLE(const Pose& o_pose, const std::vector<int8_t>& grid_map, const Pose& o_center_pose, const MapInfo_t& st_map_info, const Padding_t& st_padding, double d_margin_m);
bool CheckPoseCollisionBOX(const Pose& o_pose, const std::vector<int8_t>& grid_map, const Pose& o_center_pose, const MapInfo_t& st_map_info, const Padding_t& st_padding, const RobotConfig& o_robot_config, double d_margin_m, bool b_backward);

bool CheckSpinCollisionCIRCLE(const Pose& o_pose, const std::vector<int8_t>& grid_map, const Pose& o_center_pose, const MapInfo_t& st_map_info, const Padding_t& st_padding, double d_margin_m);
bool CheckSpinCollisionBOX(const Pose& o_pose, const std::vector<int8_t>& grid_map, const Pose& o_center_pose, const MapInfo_t& st_map_info, const Padding_t& st_padding, const RobotConfig& o_robot_config, double d_margin_m);

} // namespace CollisionDetector

} // namespace NVFR

#endif
