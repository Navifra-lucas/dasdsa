#ifndef NC_BRAIN_COMMAND_H
#define NC_BRAIN_COMMAND_H

namespace NaviFra {
class NcBrainCommand {
public:
    NcBrainCommand();
    ~NcBrainCommand();

    static const std::string BRAIN_ROBOT_CONTROL;
    static const std::string BRAIN_ROBOT_INIT_POSE;
    static const std::string BRAIN_ROBOT_CORRECTION_INIT_POSE;
    static const std::string BRAIN_ROBOT_MOVE;
    static const std::string BRAIN_ROBOT_DOCKING;
    static const std::string BRAIN_ROBOT_ACTION;
    static const std::string BRAIN_ROBOT_STOP;
    static const std::string BRAIN_ROBOT_CANCEL;
    static const std::string BRAIN_ROBOT_DEVICE_COMMAND;
    static const std::string BRAIN_ROBOT_DEVICE_GOAL;
    static const std::string BRAIN_ROBOT_DEVICE_MAP_UPDATE;
    static const std::string BRAIN_ROBOT_DEVICE_PATH;
    static const std::string BRAIN_ROBOT_START_SLAM;
    static const std::string BRAIN_ROBOT_STOP_SLAM;
    static const std::string BRAIN_ROBOT_SAVE_SLAM;
    static const std::string BRAIN_ROBOT_GET_SLAM_MAP;
    static const std::string BRAIN_ROBOT_UPDATE_MAP;
    static const std::string BRAIN_ROBOT_SYNC_MAP;
    static const std::string BRAIN_ROBOT_TEACHING;
    static const std::string BRAIN_ROBOT_TEACHED;
    static const std::string BRAIN_ROBOT_GET_PARAMETERS;
    static const std::string BRAIN_ROBOT_UPDATE_PARAMETERS;
    static const std::string BRAIN_ROBOT_START_SCAN;
    static const std::string BRAIN_ROBOT_STOP_SCAN;
    static const std::string BRAIN_ROBOT_START_MOTOR_CHECK;
    static const std::string BRAIN_ROS_GET_NODES;
    static const std::string BRAIN_ROS_START_NODE;
    static const std::string BRAIN_ROS_STOP_NODE;
    static const std::string BRAIN_TASK_ADD;
    static const std::string BRAIN_TASK_COMMAND;
    static const std::string BRAIN_ROBOT_TASKALARM;
    static const std::string BRAIN_SYSTEM_REBOOT;
    static const std::string BRAIN_SYSTEM_SHUTDOWN;

    static const std::string BRAIN_ROBOT_CALIBRATION;
    static const std::string BRAIN_ROBOT_CALIBRATION_STOP;
    static const std::string BRAIN_ROBOT_CALIBRATION_CALCULATE;
    static const std::string BRAIN_ROBOT_CALIBRATION_RESET;
    static const std::string BRAIN_ROBOT_CALIBRATION_SAVE;
    static const std::string BRAIN_ROBOT_CALIBRATION_DOCKING_READ;
    static const std::string BRAIN_ROBOT_CALIBRATION_DOCKING_SAVE;
    static const std::string BRAIN_ROBOT_CALIBRATION_STEER_WRITE;
    static const std::string BRAIN_ROBOT_CALIBRATION_STEER_ZEROSET;

    static const std::string BRAIN_ROBOT_EXD_UPDATEROBOT_INFO;
    static const std::string BRAIN_ROBOT_START_EXD_STATUS;
    static const std::string BRAIN_ROBOT_EXD_JIG;
    static const std::string BRAIN_ROBOT_UPDATE_PRESETS;

    static const std::string BRAIN_ROBOT_MOTORDRIVER_INIT;
    static const std::string BRAIN_ROBOT_MOTORDRIVER_RESET;

    static const std::string BRAIN_MAP_UPDATE;
    static const std::string BRAIN_PARAM_UPDATE;
    static const std::string BRAIN_LAUNCH_MANAGER;

    static const std::string BRAIN_ROBOT_PLAYBACK_LIST;
    static const std::string BRAIN_ROBOT_PLAYBACK_PLAY;
    static const std::string BRAIN_ROBOT_PLAYBACK_STOP;
    static const std::string BRAIN_ROBOT_PLAYBACK_PAUSE;
    static const std::string BRAIN_ROBOT_PLAYBACK_SPEED;
    static const std::string BRAIN_ROBOT_PLAYBACK_SEEK;
    static const std::string BRAIN_ROBOT_PLAYBACK_DOWNLOAD;
    static const std::string BRAIN_ROBOT_PLAYBACK;
    static const std::string BRAIN_ROBOT_LOG_DOWNLOAD;

    static const std::string BRAIN_ROBOT_REPEAT_DRIVE;
    static const std::string BRAIN_ROBOT_REPEAT_DRIVE_CMD;

    static const std::string BRAIN_ROBOT_BARCODE_READER;
};
}  // namespace NaviFra
#endif
