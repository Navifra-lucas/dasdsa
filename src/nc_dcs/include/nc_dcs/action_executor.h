#ifndef NC_DCS_ACTION_EXECUTOR_H
#define NC_DCS_ACTION_EXECUTOR_H

#include "core_msgs/WiaForkInfo.h"
#include "nc_dcs/fork_parameters_loader.h"
#include "nc_dcs/mock_msgs.h"
#include "nc_dcs/sequence_loader.h"
#include "nc_dcs/ros_interface.h"

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <mutex>
// #include "dcs_msgs/DockingInfo.h"
#include "util/logger.hpp"

#include <nlohmann/json.hpp>

namespace nc_dcs {

class ActionExecutor {
public:
    ActionExecutor(RosInterface* ros_interface);

    // Set fork parameters loader
    void setForkParametersLoader(const ForkParametersLoader& fork_params);

    void executeForkUpDown(const ActionParams& params, const Mission& mission);
    void executeForkSide(const ActionParams& params);
    void executeForkTilt(const ActionParams& params);
    void executeIntensityHeight(const ActionParams& params, const Mission& mission);
    void executeMove(const ActionParams& params, const nlohmann::json& target_cmd_json, std::mutex& pose_mutex, const Mission& mission);
    void executePerception(const ActionParams& params, const Mission& mission);

private:
    RosInterface* ros_interface_;

    // Fork parameters loader
    const ForkParametersLoader* fork_params_loader_;

    void publishPerceptionCmd(const std::string& cmd, const std::string& target_type, int n_pallet_type, float f_docking_dist);
    void publishWingbodyPerceptionCmd(const std::string& cmd, const std::string& target_type, const Mission& mission, float f_docking_dist);
    void publishPathPlanCmd(const nlohmann::json& info);
    void publishPalletDetectTrigger(const std::string& cmd);
    void publishForkObsTrigger(bool mode);

public:
    void publishIntensityCmd(const std::string& cmd);
};

}  // namespace nc_dcs

#endif  // NC_DCS_ACTION_EXECUTOR_H
