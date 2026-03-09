#ifndef NC_BRAIN_MESSAGE_H
#define NC_BRAIN_MESSAGE_H

namespace NaviFra {
class NcBrainMessage {
public:
    NcBrainMessage();
    ~NcBrainMessage();

    static const std::string MESSAGE_ROBOT_REQUEST;
    static const std::string MESSAGE_TASK_REQUEST;
    static const std::string MESSAGE_BACKEND_REQUEST;
    static const std::string MESSAGE_BACKEND_AREA;
    static const std::string MESSAGE_BACKEND_PORTAL;
    static const std::string MESSAGE_ROBOT_STATUS_INFO;
    static const std::string MESSAGE_ROBOT_STATUS_PATH;
    static const std::string MESSAGE_ROBOT_STATUS_SCAN;
    static const std::string MESSAGE_ACS_REQUEST;
    static const std::string MESSAGE_ACS_RESPONSE;
    static const std::string MESSAGE_ROBOT_RESPONSE;
    static const std::string MESSAGE_ROBOT_HEARTBEAT_STATUS;
    static const std::string MESSAGE_ROBOT_STATUS_SLAM;
    static const std::string MESSAGE_ROBOT_STATUS_SLAM_SAVE;
    static const std::string MESSAGE_TASK_RESPONSE;
    static const std::string MESSAGE_ROBOT_STATUS_CALIBRATION;
    static const std::string MESSAGE_ROBOT_STATUS_EXD;
    static const std::string MESSAGE_ROBOT_STATUS_POLYGON;
    static const std::string MESSAGE_ROBOT_STATUS_MOTOR;
    static const std::string MESSAGE_ROBOT_STATUS_PLAYBACK;
    static const std::string MESSAGE_ROBOT_STATUS_REPEAT_DRIVE;
    static const std::string MESSAGE_ROBOT_STATUS_MESSAGE;
    static const std::string MESSAGE_ROBOT_STATUS_HARDWARE_INFO;
    static const std::string MESSAGE_ROBOT_STATUS_STARTUP;
    static const std::string MESSAGE_ROBOT_STATUS_MARKER;
};
}  // namespace NaviFra
#endif
