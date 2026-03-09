#ifndef NC_TASK_MANAGER_TYPES_H
#define NC_TASK_MANAGER_TYPES_H
#include "ros/ros.h"

#include <string>

namespace NaviFra {
struct ROSPublisher {
    ros::Publisher& pub_task_response;
};

static const struct TYPE {
    const std::string MOVE = "move";
    const std::string TURN = "turn";
    const std::string ELSE = "else";
    const std::string MANUAL = "manual";
    const std::string CHARGING = "charging";
    const std::string UNCHARGING = "uncharging";
    const std::string DOCKING = "docking";
    const std::string UNDOCKING = "undocking";
    const std::string PICKUP_CHECKING = "pickup_checking";
    const std::string RETURN_CHECKING = "return_checking";
    const std::string TO_CHECKING = "to_checking";
    const std::string LOADING = "loading";
    const std::string UNLOADING = "unloading";
    const std::string WAIT = "wait";
    const std::string PERCEPTION = "perception";
    const std::string ACTION = "action";
    const std::string FORKLIFT = "forklift";
} TYPE;

static const struct STATUS {
    const std::string EMPTY = "empty";
    const std::string IDLE = "idle";
    const std::string RUNNING = "running";
    const std::string PAUSED = "paused";
} STATUS;

static const struct ALARM {
    const std::string START = "start";
    const std::string DONE = "done";
    const std::string ERROR = "error";
    const std::string PAUSED = "paused";
    const std::string RESUME = "resume";
} ALARM;

static const struct RESPONSE {
    const std::string ACCEPT = "accept";
    const std::string REJECT = "reject";
} RESPONSE;

static const struct CMD {
    const std::string CANCEL = "cancel";
    const std::string PAUSE = "pause";
    const std::string RESUME = "resume";
} CMD;

}  // namespace NaviFra
#endif  // NC_TASK_MANAGER_TYPES_H