#ifndef NAVIFRA_CORE_MESSAGE_H
#define NAVIFRA_CORE_MESSAGE_H

#include <string>

namespace NaviFra {
class CoreMessage {
public:
    CoreMessage();
    ~CoreMessage() = default;

    static const std::string CORE_MESSAGE_GLOBAL_PATH;
    static const std::string CORE_MESSAGE_LOCAL_PATH;
    static const std::string CORE_MESSAGE_OBSTACLE;
    static const std::string CORE_MESSAGE_TASK_ALAM;
    static const std::string CORE_MESSAGE_TASK_RESPONSE;
    static const std::string CORE_MESSAGE_PREDICT_COLLISION;
    static const std::string CORE_MESSAGE_MAPING_PROGRESS;
    static const std::string CORE_MESSAGE_CUSTOM;
    static const std::string CORE_MESASGE_MOTOR_INFO;
    static const std::string CORE_MESSAGE_SLAM_GRAPH;
    static const std::string CORE_MESSAGE_CALI_PROGRESS;
    static const std::string CORE_MESSAGE_HARDWARE_INFO;
    static const std::string CORE_MESSAGE_NAVI_ALARM;
    static const std::string CORE_MESSAGE_SET_MARKER;
    static const std::string CORE_MESSAGE_CHARGING_CMD;
    static const std::string CORE_MESSAGE_LIGHTING_CMD;
    static const std::string CORE_MESSAGE_OSSD_FIELD;

    //plc
    static const std::string CORE_MESSAGE_PLC_ALARM_CLEAR;
    static const std::string CORE_MESSAGE_SET_REFLECTORS;
    static const std::string CORE_MESSAGE_FORK_CMD;
};
}  // namespace NaviFra
#endif
