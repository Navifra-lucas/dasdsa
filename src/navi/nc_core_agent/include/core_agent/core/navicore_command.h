#ifndef NAVIFRA_CORE_COMMAND_H
#define NAVIFRA_CORE_COMMAND_H

#include <string>

namespace NaviFra {
class CoreCommand {
public:
    CoreCommand();
    ~CoreCommand();

    static const std::string ROBOT_CONTROL;
    static const std::string ROBOT_INIT_POSE;
    static const std::string ROBOT_CORRECTION_INIT_POSE;
    static const std::string ROBOT_MOVE;
    static const std::string ROBOT_DOCKING;
    static const std::string ROBOT_ACTION;
    static const std::string ROBOT_STOP;
    static const std::string ROBOT_CANCEL;
    static const std::string ROBOT_DEVICE_COMMAND;
    static const std::string ROBOT_DEVICE_GOAL;
    static const std::string ROBOT_DEVICE_MAP_UPDATE;
    static const std::string ROBOT_DEVICE_PATH;
    static const std::string ROBOT_START_SLAM;
    static const std::string ROBOT_STOP_SLAM;
    static const std::string ROBOT_SAVE_SLAM;
    static const std::string ROBOT_GET_SLAM_MAP;
    static const std::string ROBOT_UPDATE_MAP;
    static const std::string ROBOT_TEACHING;
    static const std::string ROBOT_TEACHED;
    static const std::string ROBOT_GET_PARAMETERS;
    static const std::string ROBOT_UPDATE_PARAMETERS;
    static const std::string ROBOT_START_SCAN;
    static const std::string ROBOT_STOP_SCAN;
    static const std::string ROBOT_START_MOTOR_CHECK;
    static const std::string ROS_GET_NODES;
    static const std::string ROS_START_NODE;
    static const std::string ROS_STOP_NODE;
    static const std::string TASK_ADD;
    static const std::string TASK_COMMAND;
    static const std::string ROBOT_TASKALARM;
    static const std::string SYSTEM_REBOOT;
    static const std::string SYSTEM_SHUTDOWN;
    static const std::string ANSWER_CONTROL;

    static const std::string ROBOT_CALIBRATION;
    static const std::string ROBOT_CALIBRATION_STOP;
    static const std::string ROBOT_CALIBRATION_CALCULATE;
    static const std::string ROBOT_CALIBRATION_RESET;
    static const std::string ROBOT_CALIBRATION_SAVE;
    static const std::string ROBOT_CALIBRATION_DOCKING_READ;
    static const std::string ROBOT_CALIBRATION_DOCKING_SAVE;
    static const std::string ROBOT_CALIBRATION_STEER_WRITE;
    static const std::string ROBOT_CALIBRATION_STEER_ZEROSET;

    static const std::string ROBOT_EXD_UPDATEROBOT_INFO;
    static const std::string ROBOT_START_EXD_STATUS;
    static const std::string ROBOT_EXD_JIG;
    static const std::string ROBOT_UPDATE_PRESETS;

    static const std::string ROBOT_MOTORDRIVER_INIT;
    static const std::string ROBOT_MOTORDRIVER_RESET;

    static const std::string MAP_UPDATE;
    static const std::string PARAM_UPDATE;

    static const std::string ROBOT_PLAYBACK_LIST;
    static const std::string ROBOT_PLAYBACK_PLAY;
    static const std::string ROBOT_PLAYBACK_STOP;
    static const std::string ROBOT_PLAYBACK_PAUSE;
    static const std::string ROBOT_PLAYBACK_SPEED;
    static const std::string ROBOT_PLAYBACK_SEEK;
    static const std::string ROBOT_PLAYBACK_DOWNLOAD;
    static const std::string ROBOT_PLAYBACK;
    static const std::string ROBOT_LOG_DOWNLOAD;

    static const std::string ROBOT_REPEAT_DRIVE;
    static const std::string ROBOT_REPEAT_DRIVE_CMD;

    static const std::string ROBOT_BARCODE_READER;

    static const std::string ROBOT_FAB_COLOR;
    static const std::string ROBOT_FAB_SOUND;

    static const std::string ROBOT_MOVE_POSE;
    static const std::string ROBOT_RESPONSE_AVOIDANCE;
    static const std::string ROBOT_RSR_INFO;
    static const std::string NAVIFRA_COMMAND;
    static const std::string ROBOT_CHEONIL_READ_REGISTER;
    static const std::string ROBOT_CHEONIL_READ_COIL;
    static const std::string ROBOT_SPEED_LIMIT;
    static const std::string ROBOT_LOADED;
    static const std::string ROBOT_CHARGING_SUCCESS;
    static const std::string ROBOT_FORK_POSITION_STATE;
};
}  // namespace NaviFra
#endif
