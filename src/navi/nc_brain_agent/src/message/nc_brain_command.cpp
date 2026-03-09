#include "nc_brain_agent/nc_brain_agent.h"

#include <nc_brain_agent/message/nc_brain_command.h>

using namespace NaviFra;

const std::string NcBrainCommand::BRAIN_ROBOT_CONTROL = "control";
const std::string NcBrainCommand::BRAIN_ROBOT_INIT_POSE = "init_pose";
const std::string NcBrainCommand::BRAIN_ROBOT_CORRECTION_INIT_POSE = "correction_init_pose";
const std::string NcBrainCommand::BRAIN_ROBOT_MOVE = "move";
const std::string NcBrainCommand::BRAIN_ROBOT_DOCKING = "docking";
const std::string NcBrainCommand::BRAIN_ROBOT_ACTION = "action";
const std::string NcBrainCommand::BRAIN_ROBOT_STOP = "stop";
const std::string NcBrainCommand::BRAIN_ROBOT_CANCEL = "cancel";
const std::string NcBrainCommand::BRAIN_ROBOT_DEVICE_COMMAND = "device_command";
const std::string NcBrainCommand::BRAIN_ROBOT_DEVICE_GOAL = "device_goal";
const std::string NcBrainCommand::BRAIN_ROBOT_DEVICE_MAP_UPDATE = "device_map_update";
const std::string NcBrainCommand::BRAIN_ROBOT_DEVICE_PATH = "device_path";
const std::string NcBrainCommand::BRAIN_ROBOT_START_SLAM = "start_slam";
const std::string NcBrainCommand::BRAIN_ROBOT_STOP_SLAM = "stop_slam";
const std::string NcBrainCommand::BRAIN_ROBOT_SAVE_SLAM = "save_slam";
const std::string NcBrainCommand::BRAIN_ROBOT_GET_SLAM_MAP = "get_slam_map";
const std::string NcBrainCommand::BRAIN_ROBOT_UPDATE_MAP = "update_map";
const std::string NcBrainCommand::BRAIN_ROBOT_SYNC_MAP = "sync_map";
const std::string NcBrainCommand::BRAIN_ROBOT_TEACHING = "teaching";
const std::string NcBrainCommand::BRAIN_ROBOT_TEACHED = "teached";
const std::string NcBrainCommand::BRAIN_ROBOT_GET_PARAMETERS = "get_parameters";
const std::string NcBrainCommand::BRAIN_ROBOT_UPDATE_PARAMETERS = "update_parameters";
const std::string NcBrainCommand::BRAIN_ROBOT_START_SCAN = "start_scan";
const std::string NcBrainCommand::BRAIN_ROBOT_STOP_SCAN = "stop_scan";
const std::string NcBrainCommand::BRAIN_ROBOT_START_MOTOR_CHECK = "start_motorcheck";
const std::string NcBrainCommand::BRAIN_ROS_GET_NODES = "get_ros_nodes";
const std::string NcBrainCommand::BRAIN_ROS_START_NODE = "start_ros_node";
const std::string NcBrainCommand::BRAIN_ROS_STOP_NODE = "stop_ros_node";
const std::string NcBrainCommand::BRAIN_TASK_ADD = "add_task";
const std::string NcBrainCommand::BRAIN_TASK_COMMAND = "task_cmd";
const std::string NcBrainCommand::BRAIN_ROBOT_TASKALARM = "task_alarm";
const std::string NcBrainCommand::BRAIN_SYSTEM_REBOOT = "reboot";
const std::string NcBrainCommand::BRAIN_SYSTEM_SHUTDOWN = "shutdown";

const std::string NcBrainCommand::BRAIN_ROBOT_CALIBRATION = "calibration";
const std::string NcBrainCommand::BRAIN_ROBOT_CALIBRATION_STOP = "calibration_stop";
const std::string NcBrainCommand::BRAIN_ROBOT_CALIBRATION_CALCULATE = "calibration_calculate";
const std::string NcBrainCommand::BRAIN_ROBOT_CALIBRATION_RESET = "calibration_reset";
const std::string NcBrainCommand::BRAIN_ROBOT_CALIBRATION_SAVE = "calibration_save";
const std::string NcBrainCommand::BRAIN_ROBOT_CALIBRATION_DOCKING_READ = "calibration_docking_read";
const std::string NcBrainCommand::BRAIN_ROBOT_CALIBRATION_DOCKING_SAVE = "calibration_docking_save";
const std::string NcBrainCommand::BRAIN_ROBOT_CALIBRATION_STEER_WRITE = "calibration_steer_write";
const std::string NcBrainCommand::BRAIN_ROBOT_CALIBRATION_STEER_ZEROSET = "calibration_steer_zeroset";

const std::string NcBrainCommand::BRAIN_ROBOT_EXD_UPDATEROBOT_INFO = "plc";
const std::string NcBrainCommand::BRAIN_ROBOT_START_EXD_STATUS = "start_exd_status";
const std::string NcBrainCommand::BRAIN_ROBOT_EXD_JIG = "exd_jig";
const std::string NcBrainCommand::BRAIN_ROBOT_UPDATE_PRESETS = "update_presets";

const std::string NcBrainCommand::BRAIN_ROBOT_MOTORDRIVER_INIT = "motor_driver_init";
const std::string NcBrainCommand::BRAIN_ROBOT_MOTORDRIVER_RESET = "motor_driver_reset";
const std::string NcBrainCommand::BRAIN_MAP_UPDATE = "map_request";
const std::string NcBrainCommand::BRAIN_PARAM_UPDATE = "param_update";

const std::string NcBrainCommand::BRAIN_ROBOT_PLAYBACK_LIST = "playback_list";
const std::string NcBrainCommand::BRAIN_ROBOT_PLAYBACK_PLAY = "playback_play";
const std::string NcBrainCommand::BRAIN_ROBOT_PLAYBACK_STOP = "playback_stop";
const std::string NcBrainCommand::BRAIN_ROBOT_PLAYBACK_PAUSE = "playback_pause";
const std::string NcBrainCommand::BRAIN_ROBOT_PLAYBACK_SPEED = "playback_speed";
const std::string NcBrainCommand::BRAIN_ROBOT_PLAYBACK_SEEK = "playback_seek";
const std::string NcBrainCommand::BRAIN_ROBOT_PLAYBACK_DOWNLOAD = "playback_download";
const std::string NcBrainCommand::BRAIN_ROBOT_PLAYBACK = "playback";
const std::string NcBrainCommand::BRAIN_ROBOT_LOG_DOWNLOAD = "log_download";

const std::string NcBrainCommand::BRAIN_ROBOT_REPEAT_DRIVE = "repeat_drive";
const std::string NcBrainCommand::BRAIN_ROBOT_REPEAT_DRIVE_CMD = "repeat_drive/cmd";
const std::string NcBrainCommand::BRAIN_ROBOT_BARCODE_READER = "barcode_reader";

NcBrainCommand::NcBrainCommand()
{
}

NcBrainCommand::~NcBrainCommand()
{
}