#include "core_agent/core_agent.h"

#include <core_agent/core/navicore_command.h>

using namespace NaviFra;

const std::string CoreCommand::ROBOT_CONTROL = "control";
const std::string CoreCommand::ROBOT_INIT_POSE = "init_pose";
const std::string CoreCommand::ROBOT_CORRECTION_INIT_POSE = "correction_init_pose";
const std::string CoreCommand::ROBOT_MOVE = "move";
const std::string CoreCommand::ROBOT_DOCKING = "docking";
const std::string CoreCommand::ROBOT_ACTION = "action";
const std::string CoreCommand::ROBOT_STOP = "stop";
const std::string CoreCommand::ROBOT_CANCEL = "cancel";
const std::string CoreCommand::ROBOT_DEVICE_COMMAND = "device_command";
const std::string CoreCommand::ROBOT_DEVICE_GOAL = "device_goal";
const std::string CoreCommand::ROBOT_DEVICE_MAP_UPDATE = "device_map_update";
const std::string CoreCommand::ROBOT_DEVICE_PATH = "device_path";
const std::string CoreCommand::ROBOT_START_SLAM = "start_slam";
const std::string CoreCommand::ROBOT_STOP_SLAM = "stop_slam";
const std::string CoreCommand::ROBOT_SAVE_SLAM = "save_slam";
const std::string CoreCommand::ROBOT_GET_SLAM_MAP = "get_slam_map";
const std::string CoreCommand::ROBOT_UPDATE_MAP = "update_map";
const std::string CoreCommand::ROBOT_TEACHING = "teaching";
const std::string CoreCommand::ROBOT_TEACHED = "teached";
const std::string CoreCommand::ROBOT_GET_PARAMETERS = "get_parameters";
const std::string CoreCommand::ROBOT_UPDATE_PARAMETERS = "update_parameters";
const std::string CoreCommand::ROBOT_START_SCAN = "start_scan";
const std::string CoreCommand::ROBOT_STOP_SCAN = "stop_scan";
const std::string CoreCommand::ROBOT_START_MOTOR_CHECK = "start_motorcheck";
const std::string CoreCommand::ROS_GET_NODES = "get_ros_nodes";
const std::string CoreCommand::ROS_START_NODE = "start_ros_node";
const std::string CoreCommand::ROS_STOP_NODE = "stop_ros_node";
const std::string CoreCommand::TASK_ADD = "add_task";
const std::string CoreCommand::TASK_COMMAND = "task_cmd";
const std::string CoreCommand::ROBOT_TASKALARM = "task_alarm";
const std::string CoreCommand::SYSTEM_REBOOT = "reboot";
const std::string CoreCommand::SYSTEM_SHUTDOWN = "shutdown";
const std::string CoreCommand::ANSWER_CONTROL = "answer/control";

const std::string CoreCommand::ROBOT_CALIBRATION = "calibration";
const std::string CoreCommand::ROBOT_CALIBRATION_STOP = "calibration_stop";
const std::string CoreCommand::ROBOT_CALIBRATION_CALCULATE = "calibration_calculate";
const std::string CoreCommand::ROBOT_CALIBRATION_RESET = "calibration_reset";
const std::string CoreCommand::ROBOT_CALIBRATION_SAVE = "calibration_save";
const std::string CoreCommand::ROBOT_CALIBRATION_DOCKING_READ = "calibration_docking_read";
const std::string CoreCommand::ROBOT_CALIBRATION_DOCKING_SAVE = "calibration_docking_save";
const std::string CoreCommand::ROBOT_CALIBRATION_STEER_WRITE = "calibration_steer_write";
const std::string CoreCommand::ROBOT_CALIBRATION_STEER_ZEROSET = "calibration_steer_zeroset";

const std::string CoreCommand::ROBOT_EXD_UPDATEROBOT_INFO = "plc";
const std::string CoreCommand::ROBOT_START_EXD_STATUS = "start_exd_status";
const std::string CoreCommand::ROBOT_EXD_JIG = "exd_jig";
const std::string CoreCommand::ROBOT_UPDATE_PRESETS = "update_presets";

const std::string CoreCommand::ROBOT_MOTORDRIVER_INIT = "motor_driver_init";
const std::string CoreCommand::ROBOT_MOTORDRIVER_RESET = "motor_driver_reset";
const std::string CoreCommand::MAP_UPDATE = "map_request";
const std::string CoreCommand::PARAM_UPDATE = "param_update";

const std::string CoreCommand::ROBOT_PLAYBACK_LIST = "playback_list";
const std::string CoreCommand::ROBOT_PLAYBACK_PLAY = "playback_play";
const std::string CoreCommand::ROBOT_PLAYBACK_STOP = "playback_stop";
const std::string CoreCommand::ROBOT_PLAYBACK_PAUSE = "playback_pause";
const std::string CoreCommand::ROBOT_PLAYBACK_SPEED = "playback_speed";
const std::string CoreCommand::ROBOT_PLAYBACK_SEEK = "playback_seek";
const std::string CoreCommand::ROBOT_PLAYBACK_DOWNLOAD = "playback_download";
const std::string CoreCommand::ROBOT_PLAYBACK = "playback";
const std::string CoreCommand::ROBOT_LOG_DOWNLOAD = "log_download";

const std::string CoreCommand::ROBOT_REPEAT_DRIVE = "repeat_drive";
const std::string CoreCommand::ROBOT_REPEAT_DRIVE_CMD = "repeat_drive/cmd";
const std::string CoreCommand::ROBOT_BARCODE_READER = "barcode_reader";
const std::string CoreCommand::ROBOT_FAB_COLOR = "fab_color";
const std::string CoreCommand::ROBOT_FAB_SOUND = "fab_sound";

const std::string CoreCommand::ROBOT_MOVE_POSE = "move_pose";
const std::string CoreCommand::ROBOT_RESPONSE_AVOIDANCE = "response_avoidance";
const std::string CoreCommand::ROBOT_RSR_INFO = "rsr_info";
const std::string CoreCommand::NAVIFRA_COMMAND = "navifra/cmd";

const std::string CoreCommand::ROBOT_CHEONIL_READ_REGISTER = "cheonil/read_register";
const std::string CoreCommand::ROBOT_CHEONIL_READ_COIL = "cheonil/read_coil";
const std::string CoreCommand::ROBOT_SPEED_LIMIT = "navifra/speed_limit";
const std::string CoreCommand::ROBOT_LOADED = "navifra/loaded";
const std::string CoreCommand::ROBOT_CHARGING_SUCCESS = "charging_success";
const std::string CoreCommand::ROBOT_FORK_POSITION_STATE = "fork_position_state";

CoreCommand::CoreCommand()
{
}

CoreCommand::~CoreCommand()
{
}