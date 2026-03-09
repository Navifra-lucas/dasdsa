#ifndef NC_DCS_STATE_HANDLER_H
#define NC_DCS_STATE_HANDLER_H

#include "core_msgs/CheonilReadRegister.h"
#include "nc_dcs/mock_msgs.h"
#include "nc_dcs/sequence_loader.h"
#include "util/logger.hpp"

#include <core_msgs/WiaForkInfo.h>
#include <dcs_msgs/DockingInfo.h>
#include <nlohmann/json.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <mutex>

namespace nc_dcs {

class ActionExecutor;
class RosInterface;

class StateHandler {
public:
    StateHandler(ActionExecutor& action_executor, RosInterface* ros_interface);

    bool executeStep(const SequenceStep& step, const Mission& mission, const nlohmann::json& target_cmd_json, std::mutex& pose_mutex);

    bool checkStepCompletion(
        const SequenceStep& step, const Mission& mission, const core_msgs::CheonilReadRegister& lift_status, std::mutex& lift_status_mutex,
        bool perception_received, bool intensity_received, bool docking_done, bool forward_done, bool backward_done, bool goback_done, bool align_done, bool lift_done,
        int& delay_counter, const nlohmann::json& target_cmd_json, std::mutex& pose_mutex, ros::Time& step_start_time, bool b_lift_adjusted_once = false);

    void publishCompletion(bool success);

private:
    ActionExecutor& action_executor_;
    RosInterface* ros_interface_;
};

}  // namespace nc_dcs

#endif  // NC_DCS_STATE_HANDLER_H
