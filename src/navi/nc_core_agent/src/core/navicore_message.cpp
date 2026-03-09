#include "core_agent/core_agent.h"

#include <core_agent/core/navicore_message.h>

using namespace NaviFra;

const std::string CoreMessage::CORE_MESSAGE_GLOBAL_PATH = "NaviFra/visualize/ui_global_path";
const std::string CoreMessage::CORE_MESSAGE_LOCAL_PATH = "NaviFra/visualize/ui_local_path";
const std::string CoreMessage::CORE_MESSAGE_OBSTACLE = "NaviFra/visualize/ui_obstacle_pos";
const std::string CoreMessage::CORE_MESSAGE_TASK_ALAM = "nc_task_manager/task_alarm";
const std::string CoreMessage::CORE_MESSAGE_TASK_RESPONSE = "nc_task_manager/task_response";
const std::string CoreMessage::CORE_MESSAGE_PREDICT_COLLISION = "NaviFra/visualize/robot_collision_predict";
const std::string CoreMessage::CORE_MESSAGE_MAPING_PROGRESS = "answer/mapping_progress";
const std::string CoreMessage::CORE_MESSAGE_CUSTOM = "etc_custom_message";
const std::string CoreMessage::CORE_MESASGE_MOTOR_INFO = "motor_info";
const std::string CoreMessage::CORE_MESSAGE_SLAM_GRAPH = "answer/pose_graph";
const std::string CoreMessage::CORE_MESSAGE_CALI_PROGRESS = "cali/error_progress";
const std::string CoreMessage::CORE_MESSAGE_HARDWARE_INFO = "navifra/fab/hw_info";
const std::string CoreMessage::CORE_MESSAGE_NAVI_ALARM = "navifra/alarm";
const std::string CoreMessage::CORE_MESSAGE_SET_MARKER = "navifra/set_marker";
const std::string CoreMessage::CORE_MESSAGE_PLC_ALARM_CLEAR = "plc/alarm_clear";
const std::string CoreMessage::CORE_MESSAGE_SET_REFLECTORS = "answer/set_reflectors";
const std::string CoreMessage::CORE_MESSAGE_FORK_CMD = "forkinfo";
const std::string CoreMessage::CORE_MESSAGE_CHARGING_CMD = "nc_task_manager/charging";
const std::string CoreMessage::CORE_MESSAGE_LIGHTING_CMD = "navifra/lighting";
const std::string CoreMessage::CORE_MESSAGE_OSSD_FIELD = "navifra/ossd_field";