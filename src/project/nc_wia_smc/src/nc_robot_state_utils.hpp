#pragma once
#include <string>

enum class RobotState {
    STATE_OFF          = 0,
    STATE_ON           = 1,   
    STATE_BOOTING      = 2, 

    STATE_IDLE         = 10,  
    STATE_CHARGING     = 11,
    STATE_PAUSE        = 12,
    STATE_BRAKE_RELEASE= 13,
    STATE_RESUME       = 14,
    STATE_PIN          = 15,  // Working on board
    STATE_CHARGED      = 16,
    STATE_STANDBY      = 17,

    STATE_DRIVING      = 20,
    STATE_ACTION       = 21,
    STATE_DOCKING      = 22,
    STATE_DOCKING_OUT  = 23,

    STATE_AUTO         = 30,
    STATE_MANUAL       = 31,
    //
    STATE_STOP         = 40,

    STATE_OBSTACLE     = 50,
    STATE_BRAKE        = 60,
    STATE_ALARM        = 70,
    STATE_ERROR        = 80,
    STATE_EMERGENCY    = 90
};

RobotState convertState(const std::string& status_str) {
    if (status_str == "emergency") return RobotState::STATE_EMERGENCY;
    else if (status_str.find("ERROR") != std::string::npos) return RobotState::STATE_ERROR;
    else if (status_str == "mapnotload") return RobotState::STATE_ERROR;
    else if (status_str == "nodeout") return RobotState::STATE_ERROR;
    else if (status_str.find("paused_by_obs") != std::string::npos) return RobotState::STATE_OBSTACLE;
    else if (status_str == "manual") return RobotState::STATE_MANUAL;
    else if (status_str == "brake_release") return RobotState::STATE_BRAKE_RELEASE;
    else if (status_str == "stop") return RobotState::STATE_STOP;
    else if (status_str == "paused") return RobotState::STATE_PAUSE;
    else if (status_str == "charging") return RobotState::STATE_CHARGING; //
    else if (status_str == "idle") return RobotState::STATE_IDLE;
    else if (status_str == "running") return RobotState::STATE_DRIVING;
    else if (status_str == "resume") return RobotState::STATE_RESUME;
    else if (status_str == "loading") return RobotState::STATE_BOOTING;
    else if (status_str == "charged") return RobotState::STATE_CHARGED;
    else if (status_str == "docking") return RobotState::STATE_DOCKING;
    else if (status_str == "undocking") return RobotState::STATE_DOCKING_OUT;
    else if (status_str == "slam_docking") return RobotState::STATE_DOCKING;
    else if (status_str == "charge_docking") return RobotState::STATE_DOCKING;
    else if (status_str == "standby") return RobotState::STATE_STANDBY;
    else if (status_str == "lift") return RobotState::STATE_PIN;  // Working on board
    else return RobotState::STATE_IDLE;
}

std::string getStateColor(RobotState state) {
    switch(state) {
        case RobotState::STATE_EMERGENCY:
            return "emergency";
        case RobotState::STATE_ERROR:
        case RobotState::STATE_ALARM:
            return "error";
        case RobotState::STATE_DOCKING:
        case RobotState::STATE_DOCKING_OUT:
        case RobotState::STATE_PIN:
        case RobotState::STATE_DRIVING:
        case RobotState::STATE_RESUME:
            return "working";
        case RobotState::STATE_IDLE:
        case RobotState::STATE_STANDBY:
            return "standby";
        case RobotState::STATE_CHARGING:
            return "charging";
        case RobotState::STATE_CHARGED:
            return "charged";
        case RobotState::STATE_PAUSE:
        case RobotState::STATE_OBSTACLE:
            return "pause";
        case RobotState::STATE_BOOTING:
            return "booting";
        // case RobotState::STATE_BRAKE_RELEASE:
        // case RobotState::STATE_MANUAL:
        // case RobotState::STATE_OBSTACLE:
            // return "yellow";
        default:
            return "white";
    }
}

std::string getStateSound(RobotState state) {
    switch (state) {
        case RobotState::STATE_DRIVING:
            return "moving";
        case RobotState::STATE_DOCKING:
            return "docking";
        case RobotState::STATE_DOCKING_OUT:
            return "docking_out";
        case RobotState::STATE_PIN:
            return "workingonboard";
        case RobotState::STATE_IDLE:
            return "idle";
        case RobotState::STATE_CHARGING:
            return "charging";
        case RobotState::STATE_BOOTING:
            return "init";
        case RobotState::STATE_STANDBY:
            return "standby";
        case RobotState::STATE_OBSTACLE:
        case RobotState::STATE_PAUSE:
            return "pause";
        case RobotState::STATE_ERROR:
        case RobotState::STATE_EMERGENCY:
            return "error";
        default: return "";
    }
}

std::string toString(RobotState state) {
    switch(state) {
        case RobotState::STATE_ON: return "ON";
        case RobotState::STATE_IDLE: return "IDLE";
        case RobotState::STATE_MANUAL: return "MANUAL";
        case RobotState::STATE_AUTO: return "AUTO";
        case RobotState::STATE_BRAKE: return "BRAKE";
        case RobotState::STATE_BRAKE_RELEASE: return "BRAKE_RELEASE";
        case RobotState::STATE_EMERGENCY: return "EMERGENCY";
        case RobotState::STATE_ERROR: return "ERROR";
        case RobotState::STATE_ALARM: return "ALARM";
        case RobotState::STATE_PAUSE: return "PAUSE";
        case RobotState::STATE_OBSTACLE: return "OBSTACLE";
        case RobotState::STATE_CHARGING: return "CHARGING";
        case RobotState::STATE_DRIVING: return "DRIVING";
        case RobotState::STATE_RESUME: return "RESUME";
        case RobotState::STATE_STOP: return "STOP";
        case RobotState::STATE_ACTION: return "ACTION";
        case RobotState::STATE_PIN: return "PIN";  // Working on board
        case RobotState::STATE_DOCKING: return "DOCKING";
        case RobotState::STATE_DOCKING_OUT: return "UNDOCKING";
        case RobotState::STATE_CHARGED: return "CHARGED";
        case RobotState::STATE_BOOTING: return "BOOTING";
        case RobotState::STATE_STANDBY: return "STANDBY";
        default: return "UNKNOWN";
    }
}