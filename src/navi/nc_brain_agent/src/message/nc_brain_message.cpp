#include "nc_brain_agent/nc_brain_agent.h"

#include <nc_brain_agent/message/nc_brain_message.h>

using namespace NaviFra;

const std::string NcBrainMessage::MESSAGE_ROBOT_REQUEST = "robot.request:";
const std::string NcBrainMessage::MESSAGE_TASK_REQUEST = "task_manager.request:";
const std::string NcBrainMessage::MESSAGE_BACKEND_REQUEST = "backend.status.robot";
const std::string NcBrainMessage::MESSAGE_BACKEND_AREA = "backend.status.map.areas";
const std::string NcBrainMessage::MESSAGE_BACKEND_PORTAL = "backend.status.map.portal";
const std::string NcBrainMessage::MESSAGE_ROBOT_STATUS_INFO = "robot.status.info:";
const std::string NcBrainMessage::MESSAGE_ROBOT_STATUS_PATH = "robot.status.path:";
const std::string NcBrainMessage::MESSAGE_ROBOT_STATUS_HARDWARE_INFO = "robot.status.hw_info:";
const std::string NcBrainMessage::MESSAGE_ROBOT_STATUS_STARTUP = "robot.status.startup:";
const std::string NcBrainMessage::MESSAGE_ACS_REQUEST = "acs.request";
const std::string NcBrainMessage::MESSAGE_ACS_RESPONSE = "acs.response";
const std::string NcBrainMessage::MESSAGE_ROBOT_RESPONSE = "robot.response:";
const std::string NcBrainMessage::MESSAGE_ROBOT_HEARTBEAT_STATUS = "robot.status.heartbeat:";
const std::string NcBrainMessage::MESSAGE_ROBOT_STATUS_SLAM = "robot.status.slam:";
const std::string NcBrainMessage::MESSAGE_ROBOT_STATUS_SLAM_SAVE = "robot.status.slam-save:";
const std::string NcBrainMessage::MESSAGE_TASK_RESPONSE = "task_manager.response:";
const std::string NcBrainMessage::MESSAGE_ROBOT_STATUS_SCAN = "robot.status.scan:";
const std::string NcBrainMessage::MESSAGE_ROBOT_STATUS_CALIBRATION = "robot.status.calibration:";
const std::string NcBrainMessage::MESSAGE_ROBOT_STATUS_EXD = "robot.status.exd:";
const std::string NcBrainMessage::MESSAGE_ROBOT_STATUS_POLYGON = "robot.status.polygon:";
const std::string NcBrainMessage::MESSAGE_ROBOT_STATUS_MOTOR = "robot.status.motors:";
const std::string NcBrainMessage::MESSAGE_ROBOT_STATUS_PLAYBACK = "robot.status.playback:";
const std::string NcBrainMessage::MESSAGE_ROBOT_STATUS_REPEAT_DRIVE = "robot.status.repeat_drive:";
const std::string NcBrainMessage::MESSAGE_ROBOT_STATUS_MESSAGE = "robot.status.message:";
const std::string NcBrainMessage::MESSAGE_ROBOT_STATUS_MARKER = "robot.status.marker:";

NcBrainMessage::NcBrainMessage()
{
}

NcBrainMessage::~NcBrainMessage()
{
}