#include "nc_dcs/state_handler.h"

#include "nc_dcs/action_executor.h"
#include "nc_dcs/ros_interface.h"

namespace nc_dcs {

StateHandler::StateHandler(ActionExecutor& action_executor, RosInterface* ros_interface)
    : action_executor_(action_executor)
    , ros_interface_(ros_interface)
{
}

bool StateHandler::executeStep(
    const SequenceStep& step, const Mission& mission, const nlohmann::json& target_cmd_json, std::mutex& pose_mutex)
{
    const ActionParams& params = step.params;

    // Execute based on action_type
    if (step.action_type == "fork_updown") {
        action_executor_.executeForkUpDown(params, mission);
    }
    else if (step.action_type == "fork_side") {
        action_executor_.executeForkSide(params);
    }
    else if (step.action_type == "fork_tilt") {
        action_executor_.executeForkTilt(params);
    }
    else if (step.action_type == "intensity") {
        action_executor_.executeIntensityHeight(params, mission);
    }
    else if (step.action_type == "move") {
        action_executor_.executeMove(params, target_cmd_json, pose_mutex, mission);
    }
    else if (step.action_type == "perception") {
        action_executor_.executePerception(params, mission);
    }
    else {
        LOG_INFO("[DCS] Unknown action_type: %s", step.action_type.c_str());
        return false;
    }

    return true;
}

bool StateHandler::checkStepCompletion(
    const SequenceStep& step, const Mission& mission, const core_msgs::CheonilReadRegister& lift_status, std::mutex& lift_status_mutex,
    bool perception_received, bool intensity_received, bool docking_done, bool forward_done, bool backward_done, bool goback_done, bool align_done, bool lift_done,
    int& delay_counter, const nlohmann::json& target_cmd_json, std::mutex& pose_mutex, ros::Time& time, bool b_lift_adjusted_once)
{
    // Handle continuous publishing for docking
    // if (step.action_type == "move") {
    //     bool continuous_publish = step.params.getBool("continuous_publish", false);
    //     if (continuous_publish) {
    //         std::lock_guard<std::mutex> lock(pose_mutex);
    //         action_executor_.executeMove(step.params, target_cmd_json, pose_mutex);
    //     }
    // }

    // Handle delay-based completion
    if (step.delay_sec > 0.0) {
        int required_ticks = static_cast<int>(step.delay_sec * 20.0);  // 20Hz
        delay_counter++;
        if (delay_counter >= required_ticks) {
            return true;
        }
        return false;
    }

    // Handle wait_for conditions
    if (step.wait_for == "completed") {
        // Check based on action type
        // NLOG(info) << "[DCS] Checking completion for action_type: " << step.action_type;
        if (step.action_type == "fork_updown") {
            std::lock_guard<std::mutex> lock(lift_status_mutex);
            NLOG(info) << "[DCS] Lift Status: Complete=" << lift_status.fork_up_down_complete
                       << ", Position=" << lift_status.fork_up_down_position << ", Target=" << mission.n_real_target_height;
            bool b_lift_done = abs(lift_status.fork_up_down_position - mission.n_real_target_height) <= 5;
            std::string operation = step.params.getString("operation", "");
            bool use_fork_obs_check = step.params.getBool("fork_obs_check", false);
            if(step.action_type == "fork_updown" && operation == "place" && mission.type == "UNLOADING" && mission.n_rack_type == 2) {
                b_lift_done = (lift_done && b_lift_done);
                if (ros_interface_ && b_lift_done) {
                    std_msgs::String msg;
                    msg.data = "stop";
                    ros_interface_->getIntensityCmdPub().publish(msg);
                }
            }
            if(use_fork_obs_check && b_lift_done) {
                if (ros_interface_) {
                    std_msgs::Bool msg;
                    msg.data = false;
                    ros_interface_->getSentForkObsTrigger().publish(msg);
                }
            }
            // return lift_status.fork_up_down_complete && abs(lift_status.fork_up_down_position - mission.n_real_target_height) < 5;
            return b_lift_done;
        }
        else if (step.action_type == "fork_tilt") {
            // Check if tilt position matches the target
            std::string direction = step.params.getString("direction", "down");
            int target_tilt = (direction == "up") ? 2 : 1;
            std::lock_guard<std::mutex> lock(lift_status_mutex);
            NLOG(info) << "[DCS] Lift Status: Tilting=" << lift_status.tilting_up_down << ", Target Tilt=" << target_tilt;
            return (lift_status.tilting_up_down == target_tilt);
        }
        else if (step.action_type == "fork_side") {
            // Check if fork width matches the target
            std::string position = step.params.getString("position", "wide");
            int target_width = (position == "wide") ? 2 : 1;
            std::lock_guard<std::mutex> lock(lift_status_mutex);
            NLOG(info) << "[DCS] Lift Status: Fork Width=" << lift_status.fork_width << ", Target Width=" << target_width;
            return (lift_status.fork_width == target_width);
        }
        else if (step.action_type == "perception") {
            return perception_received;
        }
        else if (step.action_type == "intensity") {
            return intensity_received;
        }
        else if (step.action_type == "move") {
            std::string move_type = step.params.getString("type", "docking");
            NLOG(info) << "[DCS] Checking move completion: type=" << move_type;
            //            << ", docking_done=" << docking_done
            //            << ", goback_done=" << goback_done;
            if (move_type == "docking") {
                if (docking_done) {
                    NLOG(info) << "[DCS] Docking done! Calling path_plan/done service";
                    // Call /path_plan/done service when docking is complete
                    nlohmann::json target_cmd_json;
                    target_cmd_json["n_mission_type"] = 0;  // NONE = 0, PALLET = 1, WINGBODY = 2, GOBACK = 3
                    target_cmd_json["o_fNode"] = {{"x", 0.0}, {"y", 0.0}, {"deg", 0.0}};
                    target_cmd_json["o_object2goal"] = {{"x", -0.4}, {"y", 0.0}, {"deg", 0.0}};
                    target_cmd_json["o_pose"] = {{"x", 0.0}, {"y", 0.0}, {"deg", 0.0}};
                    target_cmd_json["b_local"] = false;

                    // path_plan_cmd_pub_.publish(msg);
                    if (ros_interface_) {
                        std_msgs::String msg;
                        msg.data = target_cmd_json.dump();
                        ros_interface_->getPathPlanCmdPub().publish(msg);

                        msg.data = "stop";
                        ros_interface_->getSendPalletDetectTrigger().publish(msg);
                    }
                    else {
                        NLOG(warning) << "[DCS] ros_interface_ is nullptr!";
                    }
                    return true;
                }
                return false;
            }
            else if (move_type == "goback") {
                return goback_done;
            }
            else if (move_type == "backward") {
                if (ros_interface_ && backward_done) {
                    std_msgs::String msg;
                    msg.data = "stop";
                    ros_interface_->getIntensityCmdPub().publish(msg);
                }
                return backward_done;
            }
            else if (move_type == "forward") {
                return forward_done;
            }
            else if (move_type == "align") {
                return align_done;
            }
        }
        // Default: wait for general completion flag
        NLOG(info) << "[DCS] Using default completion check: lift_done=" << lift_done << ", docking_done=" << docking_done
                   << ", goback_done=" << goback_done << ", perception_received=" << perception_received;
        return lift_done || docking_done || goback_done || backward_done || perception_received;
    }
    else if (step.wait_for.empty() || step.wait_for == "none") {
        // No wait condition, complete immediately
        return true;
    }
    else if (step.wait_for == "time") {
        if(ros::Time::now() - time > ros::Duration(1.0)) {
            return true;
        }
    }

    return false;
}

void StateHandler::publishCompletion(bool success)
{
    std_msgs::Bool completion_msg;
    completion_msg.data = success;
    if (ros_interface_) {
        ros_interface_->getActionReachedPub().publish(completion_msg);
    }
    LOG_INFO("[DCS] Published fork_lift_reached: %s", success ? "true" : "false");
}
}  // namespace nc_dcs
