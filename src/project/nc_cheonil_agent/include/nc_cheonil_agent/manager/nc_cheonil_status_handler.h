#ifndef NC_CHEONIL_STATUS_HANDLER_H
#define NC_CHEONIL_STATUS_HANDLER_H

#include "nc_cheonil_agent/nc_cheonil_agent_pch.h"
#include "nc_cheonil_agent/manager/nc_cheonil_pdu_manager.h"
#include "nc_cheonil_agent/data/PDU.h"
#include "nc_cheonil_agent/data/PDUdefinition.h"
#include "core_agent/data/memory_repository.h"
#include "core_agent/data/robot_info.h"

#include <Poco/Mutex.h>
#include <string>

namespace NaviFra {
class NcCheonilStatusHandler {
public:
    NcCheonilStatusHandler();
    ~NcCheonilStatusHandler();

    void handleStatusPLC();
    void Status();
    void ObstacleCheck();
    void JogMove();
    // void JogLeftRight();
    void Charger();
    void ManualCharger();
    // void InPolygon();

    int16_t n_robot_status_ = 1;

private:
    Poco::FastMutex mutex_;

    enum class RobotStatus
    {
        IDLE,
        RUNNING,
        PAUSED,
        PausedByUser,
        PausedByObs,
        PausedByPath
    };

    enum JogStatus
    {
        JOG_NONE,      
        JOG_FRONT_LEFT,      
        JOG_REAR_RIGHT,       
        PRE_JOG = 99
    };

    enum ChargerStatus
    {
        UNCHARGING,
        CHARGING
    };

    enum RobotMode
    {
        OFF,
        ON
    };


    RobotStatus stringToRobotStatus(const std::string& status)
    {
        if (status == "idle")
            return RobotStatus::IDLE;
        else if (status == "running")
            return RobotStatus::RUNNING;
        else if (status == "paused")
            return RobotStatus::PAUSED;
        else if (status == "paused_by_user")
            return RobotStatus::PausedByUser;
        else if (status == "paused_by_obs")
            return RobotStatus::PausedByObs;
        else if (status == "paused_by_path")
            return RobotStatus::PausedByPath;
        else
            return RobotStatus::IDLE;
    }

    RobotStatus s_pre_robot_status = RobotStatus::IDLE;
};
}

#endif // NC_CHEONIL_STATUS_HANDLER_H