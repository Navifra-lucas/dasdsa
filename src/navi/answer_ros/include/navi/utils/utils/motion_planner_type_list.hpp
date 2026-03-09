/*
 * @file	: motion_planner_type_list.hpp
 * @date	: Mar 5, 2024
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: Name List of Type Param of Kinematics, Motion, Planner, ect
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MOTION_PLANNER_TYPE_LIST_HPP_
#define MOTION_PLANNER_TYPE_LIST_HPP_

#include <iostream>
#include <stdint.h>
#include <typeinfo>

namespace NVFR {
namespace MOTION_PLANNER_TYPE_LIST {

template <class C> bool IsValid(int n_type) { std::cout << "Not registed type : " << typeid(C).name() << "\n"; return false; };

/**
 * @note type: uint8_t
 * @param BOX, CIRCLE
*/
enum class ROBOT_SHAPE : uint8_t
{
  BOX = 0,
  CIRCLE = 1,
};
static constexpr char* ROBOT_SHAPE_NAME[] = {
  (char*)"BOX   ",
  (char*)"CIRCLE",
};
template <> bool IsValid<ROBOT_SHAPE>(int);

/**
 * @note type: uint8_t
 * @param DD, SD, QD
*/
enum class KINEMATICS : uint8_t
{
  DD = 0,
  SD = 1,
  QD,
};
static constexpr char* KINEMATICS_NAME[] = {
  (char*)"DD",
  (char*)"SD",
  (char*)"QD",
};
template <> bool IsValid<KINEMATICS>(int);

/**
 * @note type: uint8_t
 * @param PUREPERSUIT, HEC, STANLEY, MPC
*/
enum class CONTROLLER : uint8_t
{
  PUREPERSUIT = 0,
  HEC = 1,
  STANLEY,
  MPC,
};
static constexpr char* CONTROLLER_NAME[] = {
  (char*)"PUREPERSUIT",
  (char*)"HEC        ",
  (char*)"STANLEY    ",
  (char*)"MPC        ",
};
template <> bool IsValid<CONTROLLER>(int);

/**
 * @note type: uint16_t
 * @param NONE, A_STAR, HYBRID_A_STAR, LANE, TEB
*/
enum AVOID : uint16_t
{
  NONE = 0,
  A_STAR = 1,
  HYBRID_A_STAR,
  LANE,
  TEB,
};
static constexpr char* AVOID_NAME[] = {
  (char*)"NONE         ",
  (char*)"A_STAR       ",
  (char*)"HYBRID_A_STAR",
  (char*)"LANE         ",
  (char*)"TEB          ",
};
template <> bool IsValid<AVOID>(int);

/**
 * @note type: uint8_t
 * @param FORWARD, BACKWARD
*/
enum class MOVEDIR : uint8_t
{
  FORWARD = 0,
  BACKWARD = 1,
};
static constexpr char* MOVEDIR_NAME[] = {
  (char*)"FORWARD ",
  (char*)"BACKWARD",
};
template <> bool IsValid<MOVEDIR>(int);

/**
 * @note type: uint8_t
 * @param BIKE, QUAD
*/
enum class DRIVE : uint8_t
{
  BIKE = 0,
  QUAD = 1,
};
static constexpr char* DRIVE_NAME[] = {
  (char*)"BIKE",
  (char*)"QUAD",
};
template <> bool IsValid<DRIVE>(int);

/**
 * @note type: uint8_t
 * @param IDLE, ALIGN, NODE_PATH, START_PATH, SEARCH_PATH, EXPLORE, 
*/
enum class MISSION : uint8_t
{
  IDLE = 0,
  ALIGN = 1,
  NODE_PATH,
  START_PATH,
  SEARCH_PATH,
  EXPLORE,
};
static constexpr char* MISSION_NAME[] = {
  (char*)"IDLE       ",
  (char*)"ALIGN      ",
  (char*)"NODE_PATH  ",
  (char*)"START_PATH ",
  (char*)"SEARCH_PATH",
  (char*)"EXPLORE    ",
};
template <> bool IsValid<MISSION>(int);

/**
 * @note type: uint8_t
 * @param NORMAL_START, SLOW_START, DOCKING_START
*/
enum class START : uint8_t
{
  NORMAL_START = 0,
  SLOW_START = 1,
  DOCKING_START,
};
static constexpr char* START_NAME[] = {
  (char*)"NORMAL_START ",
  (char*)"SLOW_START   ",
  (char*)"DOCKING_START",
};
template <> bool IsValid<START>(int);

/**
 * @note type: uint8_t
 * @param NORMAL_GOAL, SLOW_GOAL, DOCKING_GOAL
*/
enum class GOAL : uint8_t
{
  NORMAL_GOAL = 0,
  SLOW_GOAL = 1,
  DOCKING_GOAL,
};
static constexpr char* GOAL_NAME[] = {
  (char*)"NORMAL_GOAL ",
  (char*)"SLOW_GOAL   ",
  (char*)"DOCKING_GOAL",
};
template <> bool IsValid<GOAL>(int);

/**
 * @note type: uint8_t
 * @param AUTO, CW, CCW
*/
enum class ALIGNDIR : uint8_t
{
  AUTO = 0,
  CW = 1,
  CCW,
};
static constexpr char* ALIGNDIR_NAME[] = {
  (char*)"AUTO",
  (char*)"CW  ",
  (char*)"CCW ",
};
template <> bool IsValid<ALIGNDIR>(int);

/**
 * @note type: uint8_t
 * @param LINE, ARC, BEZIER, POLY
*/
enum class EDGE_PATH : uint8_t
{
  LINE = 0,
  ARC = 1,
  BEZIER,
  POLY
};
static constexpr char* EDGE_PATH_NAME[] = {
  (char*)"LINE  ",
  (char*)"ARC   ",
  (char*)"BEZIER",
  (char*)"POLY  "
};
template <> bool IsValid<EDGE_PATH>(int);

/**
 * @note type: uint8_t
 * @param ABORT, CANCEL, OCS
*/
enum class INTERRUPTION : uint8_t {
  NONE = 0,
  ABORT = 1,
  CANCEL,
  OCS,
};
static constexpr char* INTERRUPTION_NAME[] = {
  (char*)"NONE  ",
  (char*)"ABORT ",
  (char*)"CANCEL",
  (char*)"OCS   ",
};
template <> bool IsValid<INTERRUPTION>(int);


/**
 * @note type: uint8_t
 * @param START, DONE, PAUSED, RESUME, ABORTED, CANCELED, END_OCS
*/
enum class NAVI_STATUS : uint8_t
{
  START = 0,
  DONE = 1,
  PAUSED,
  RESUME,
  ABORTED,
  CANCELED,
  END_OCS,
};
static constexpr char* NAVI_STATUS_NAME[] = {
  (char*)"START   ",
  (char*)"DONE    ",
  (char*)"PAUSED  ",
  (char*)"RESUME  ",
  (char*)"ABORTED ",
  (char*)"CANCELED",
  (char*)"END_OCS ",
};
template <> bool IsValid<NAVI_STATUS>(int);

/**
 * @note type: uint8_t
 * @param SENSOR, OTHER_NODE, PATH_GENERATION, FAR_FROM_PATH
*/
enum class NAVI_ERROR : uint8_t
{
  SENSOR = 0,
  OTHER_SYSTEM = 1,
  PATH_GENERATION,
  FAR_FROM_PATH,
};
static constexpr char* NAVI_ERROR_NAME[] = {
  (char*)"SENSOR         ",
  (char*)"OTHER_SYSTEM   ",
  (char*)"PATH_GENERATION",
  (char*)"FAR_FROM_PATH  ",
};
template <> bool IsValid<NAVI_ERROR>(int);

} // namespace TYPE
namespace MPTL = MOTION_PLANNER_TYPE_LIST;
} // namespace NVFR

#endif
