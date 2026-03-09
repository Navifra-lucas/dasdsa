#include "nc_dcs/action_executor.h"

#include <move_msgs/CoreCommand.h>
#include <move_msgs/Waypoint.h>

namespace nc_dcs {

ActionExecutor::ActionExecutor(RosInterface* ros_interface)
    : ros_interface_(ros_interface)
    , fork_params_loader_(nullptr)
{
}

void ActionExecutor::setForkParametersLoader(const ForkParametersLoader& fork_params)
{
    fork_params_loader_ = &fork_params;
}

void ActionExecutor::executeForkUpDown(const ActionParams& params, const Mission& mission)
{
    double height = params.getDouble("height", 0.0);
    bool use_mission_height = params.getBool("use_mission_height", false);
    bool use_intensity_height = params.getBool("use_intensity_height", false);
    bool use_rack_heights = params.getBool("use_rack_heights", false);
    bool use_fork_obs_check = params.getBool("fork_obs_check", false);
    std::string operation = params.getString("operation", "");

    publishForkObsTrigger(use_fork_obs_check);
    if (use_fork_obs_check) {
        LOG_INFO("[DCS] Fork obstacle detection triggered!!!!!!!!!!!!!!!!");
    }

    // Priority order: 1) use_mission_height, 2) use_intensity_height, 3) use_rack_heights with operation, 4) explicit height
    // if (use_intensity_height) {
    //     auto current_fork_height = fork_params_loader_->getHeight(mission.n_rack_type, operation);
    //     double d_m_pose = 0.55;
    //     double d_diff_pose = mission.intensity_height + d_m_pose;
    //     height = current_fork_height + d_diff_pose * 1000;
    //     // height = 1265;  // 임시....... ㅜㅜ
    //     LOG_INFO("[DCS] Using intensity height: %.2f / %.2f", height, d_m_pose);
    // }
    if (use_rack_heights && !operation.empty() && fork_params_loader_) {
        int rack_type = mission.n_rack_type;

        if (fork_params_loader_->hasRackType(rack_type)) {
            height = fork_params_loader_->getHeight(rack_type, operation);
            LOG_INFO("[DCS] Using rack_heights: rack_type=%d, operation=%s, height=%.0f", rack_type, operation.c_str(), height);
        }
        else {
            LOG_WARNING("[DCS] Rack type %d not found in fork parameters, using explicit height: %.2f", rack_type, height);
        }
    }
    else if (use_rack_heights && fork_params_loader_) {
        // If no operation specified but use_rack_heights is true, try to infer operation
        int rack_type = mission.n_rack_type;

        if (fork_params_loader_->hasRackType(rack_type)) {
            // Default to pickup height if no operation specified
            height = fork_params_loader_->getPickupHeight(rack_type);
            LOG_INFO("[DCS] Using default pickup height for rack_type=%d: %.0f", rack_type, height);
        }
        else {
            LOG_WARNING("[DCS] Rack type %d not found in fork parameters, using explicit height: %.2f", rack_type, height);
        }
    }

    if (mission.type == "LOADING" && mission.n_rack_type == 2 && mission.b_perception_valid && operation == "pickup") {
        double d_diff_lift_height = mission.f_perception_z;
        height -= d_diff_lift_height;

        LOG_INFO("[DCS] Applying perception height correction: target height=%.3f, offset=%.2f", height, d_diff_lift_height);
    }

    // Store the actual height being commanded for completion condition checking
    const_cast<Mission&>(mission).n_real_target_height = static_cast<int>(height);

    core_msgs::WiaForkInfo msg;
    msg.n_fork_height = height;
    msg.n_fork_wide = -1;
    msg.f_fork_tilt = -1;
    if (ros_interface_) {
        ros_interface_->getLiftCmdPub().publish(msg);
    }
    
    if(use_intensity_height && mission.n_rack_type == 2){
        LOG_INFO("[DCS] Intensity Height - Sending START command");
        // std::string s_target_deg = std::to_string(mission.f_target_deg);
        std::string s_cmd = "start";
        publishIntensityCmd(s_cmd);
    }

    LOG_INFO("[DCS] Fork UpDown: target_height=%.2f, stored_real_target_height=%d", height, mission.n_real_target_height);
}

void ActionExecutor::executeForkSide(const ActionParams& params)
{
    std::string position = params.getString("position", "narrow");

    core_msgs::WiaForkInfo msg;
    msg.n_fork_height = -1;
    msg.f_fork_tilt = -1;

    if (position == "wide") {
        msg.n_fork_wide = 1;
    }
    else {
        msg.n_fork_wide = 0;
    }
    if (ros_interface_) {
        ros_interface_->getLiftCmdPub().publish(msg);
    }
    LOG_INFO("[DCS] Fork Side: position=%s", position.c_str());
}

void ActionExecutor::executeForkTilt(const ActionParams& params)
{
    std::string direction = params.getString("direction", "down");

    core_msgs::WiaForkInfo msg;
    msg.n_fork_height = -1;
    msg.n_fork_wide = -1;
    if (direction == "up") {
        msg.f_fork_tilt = 1;
    }
    else {
        msg.f_fork_tilt = 0;
    }
    if (ros_interface_) {
        ros_interface_->getLiftCmdPub().publish(msg);
    }
    LOG_INFO("[DCS] Fork Tilt: direction=%s", direction.c_str());
}

static bool parseXYZ(const std::string& s, float& x, float& y, float& deg)
{
    try {
        std::stringstream ss(s);
        std::string token;

        std::getline(ss, token, ',');
        x = std::stof(token);

        std::getline(ss, token, ',');
        y = std::stof(token);

        std::getline(ss, token, ',');
        deg = std::stof(token);

        return true;
    }
    catch (...) {
        return false;
    }
}

void ActionExecutor::executeMove(
    const ActionParams& params, const nlohmann::json& target_cmd_json, std::mutex& pose_mutex, const Mission& mission)
{
    std::string type = params.getString("type", "docking");

    Mission m = mission;

    if (params.has("start_pos") && params.has("end_pos")) {
        std::string start_str = params.getString("start_pos", "");
        std::string end_str = params.getString("end_pos", "");

        float sx, sy, sdeg;
        float ex, ey, edeg;

        if (!parseXYZ(start_str, sx, sy, sdeg) || !parseXYZ(end_str, ex, ey, edeg)) {
            LOG_ERROR("[DCS] Invalid start_pos/end_pos format. Expected \"x,y,deg\"");
            return;
        }

        // Mission Override
        m.f_target_x = sx;
        m.f_target_y = sy;
        m.f_target_deg = sdeg;

        m.f_current_x = ex;
        m.f_current_y = ey;
        m.f_current_deg = edeg;

        LOG_INFO("[DCS] Mission overridden via params: start(%.3f, %.3f, %.1f) → end(%.3f, %.3f, %.1f)", sx, sy, sdeg, ex, ey, edeg);
    }
    double distance = params.getDouble("distance", 0.0);
    double angle = params.getDouble("angle", 0.0);
    bool use_perception_pose = params.getBool("use_perception_pose", false);
    bool continuous_publish = params.getBool("continuous_publish", false);
    auto use_sitting_sensor = params.getBool("use_sitting_sensor", false);
    bool use_intensity_height = params.getBool("use_intensity_height", false);

    if (type == "docking") {
        NLOG(info) << "[DCS] Executing Docking Command";
        if (use_perception_pose && continuous_publish) {
            std::lock_guard<std::mutex> lock(pose_mutex);
            publishPathPlanCmd(target_cmd_json);
            if(use_intensity_height){
                // std::string s_target_deg = std::to_string(mission.f_target_deg);
                std::string s_cmd = "start";
                publishIntensityCmd(s_cmd);
            }
            if (use_sitting_sensor) {
                std::string s_cmd = "start";
                publishPalletDetectTrigger(s_cmd);
            }
        }
    }
    else if (type == "goback") {
        NLOG(info) << "[DCS] Executing GoBack Command";
        nlohmann::json empty_cmd = target_cmd_json;
        empty_cmd["n_mission_type"] = 3;  // GOBACK
        publishPathPlanCmd(empty_cmd);
    }
    else if (type == "forward") {
        LOG_INFO("[DCS] Move forward: angle=%.2f", angle);

        move_msgs::CoreCommand cmd_msg;
        cmd_msg.b_start_createpath = true;
        cmd_msg.b_start_pause = false;

        // Current Waypoint
        move_msgs::Waypoint current_wp;
        current_wp.f_x_m = m.f_target_x;
        current_wp.f_y_m = m.f_target_y;
        current_wp.f_angle_deg = m.f_target_deg;
        current_wp.n_drive_type = 1000;  // Rear
        current_wp.f_speed_ms = 1.0;
        current_wp.b_stop_quick = true;
        if(mission.type == "LOADING"){
            current_wp.list_f_move_obstacle_margin.push_back(1.35);
            current_wp.list_f_move_obstacle_margin.push_back(0.2);
            current_wp.list_f_move_obstacle_margin.push_back(0.7);
            current_wp.list_f_move_obstacle_margin.push_back(0.7);
        }
        else {
            current_wp.list_f_move_obstacle_margin.push_back(1.35);
            current_wp.list_f_move_obstacle_margin.push_back(0.2);
            current_wp.list_f_move_obstacle_margin.push_back(1.1);
            current_wp.list_f_move_obstacle_margin.push_back(1.1);
        }
        cmd_msg.list_waypoints.push_back(current_wp);

        // Target Waypoint
        move_msgs::Waypoint target_wp;
        target_wp.f_x_m = m.f_current_x;
        target_wp.f_y_m = m.f_current_y;
        target_wp.f_angle_deg = m.f_current_deg;
        target_wp.n_drive_type = 1000;  // Rear
        target_wp.f_speed_ms = 1.0;
        target_wp.b_stop_quick = true;
        if(mission.type == "LOADING"){
            target_wp.list_f_move_obstacle_margin.push_back(1.35);
            target_wp.list_f_move_obstacle_margin.push_back(0.2);
            target_wp.list_f_move_obstacle_margin.push_back(0.7);
            target_wp.list_f_move_obstacle_margin.push_back(0.7);
        }
        else {
            target_wp.list_f_move_obstacle_margin.push_back(1.35);
            target_wp.list_f_move_obstacle_margin.push_back(0.2);
            target_wp.list_f_move_obstacle_margin.push_back(1.1);
            target_wp.list_f_move_obstacle_margin.push_back(1.1);
        }

        move_msgs::LidarObstacle lidar_obs;

        for (int i = 0; i < 4; i++) {
            lidar_obs.list_f_obstacle_margin.push_back(-2);
        }

        cmd_msg.list_waypoints.push_back(target_wp);
        cmd_msg.list_lidar_obs.push_back(lidar_obs);
        cmd_msg.list_lidar_obs.push_back(lidar_obs);

        if (ros_interface_) {
            ros_interface_->getSendLivePathPub().publish(cmd_msg);
        }
        LOG_INFO(
            "[DCS] Published forward path to navifra/live_path: current(%.2f, %.2f) -> target(%.2f, %.2f)", m.f_target_x, m.f_target_y,
            m.f_current_x, m.f_current_y);
    }
    else if (type == "backward") {
        LOG_INFO("[DCS] Move backward: angle=%.2f", angle);

        move_msgs::CoreCommand cmd_msg;
        cmd_msg.b_start_createpath = true;
        cmd_msg.b_start_pause = false;

        // Current Waypoint
        move_msgs::Waypoint current_wp;
        current_wp.f_x_m = m.f_current_x;
        current_wp.f_y_m = m.f_current_y;
        current_wp.f_angle_deg = m.f_target_deg;
        current_wp.n_drive_type = 2000;  // Rear
        current_wp.f_speed_ms = 0.5;
        if(mission.type == "UNLOADING"){
            current_wp.list_f_move_obstacle_margin.push_back(0.0);
            current_wp.list_f_move_obstacle_margin.push_back(0.05);
            current_wp.list_f_move_obstacle_margin.push_back(0.7);
            current_wp.list_f_move_obstacle_margin.push_back(0.7);
        }
        else {
            current_wp.list_f_move_obstacle_margin.push_back(0.0);
            current_wp.list_f_move_obstacle_margin.push_back(0.05);
            current_wp.list_f_move_obstacle_margin.push_back(1.1);
            current_wp.list_f_move_obstacle_margin.push_back(1.1);
        }
        cmd_msg.list_waypoints.push_back(current_wp);

        // Target Waypoint
        move_msgs::Waypoint target_wp;
        target_wp.f_x_m = m.f_target_x;
        target_wp.f_y_m = m.f_target_y;
        target_wp.f_angle_deg = m.f_current_deg;
        target_wp.n_drive_type = 2000;  // Rear
        target_wp.f_speed_ms = 0.5;
        if(mission.type == "UNLOADING"){
            target_wp.list_f_move_obstacle_margin.push_back(0.0);
            target_wp.list_f_move_obstacle_margin.push_back(0.05);
            target_wp.list_f_move_obstacle_margin.push_back(0.7);
            target_wp.list_f_move_obstacle_margin.push_back(0.7);
        }
        else {
            target_wp.list_f_move_obstacle_margin.push_back(0.0);
            target_wp.list_f_move_obstacle_margin.push_back(0.05);
            target_wp.list_f_move_obstacle_margin.push_back(1.1);
            target_wp.list_f_move_obstacle_margin.push_back(1.1);
        }

        cmd_msg.list_waypoints.push_back(target_wp);

        move_msgs::LidarObstacle lidar_obs;

        for (int i = 0; i < 4; i++) {
            lidar_obs.list_f_obstacle_margin.push_back(-2);
        }
        cmd_msg.list_lidar_obs.push_back(lidar_obs);
        cmd_msg.list_lidar_obs.push_back(lidar_obs);

        if (ros_interface_) {
            ros_interface_->getSendLivePathPub().publish(cmd_msg);
        }
        LOG_INFO(
            "[DCS] Published backward path to navifra/live_path: current(%.2f, %.2f) -> target(%.2f, %.2f)", m.f_current_x, m.f_current_y,
            m.f_target_x, m.f_target_y);
            
        if(use_intensity_height && mission.n_rack_type == 2){
            LOG_INFO("[DCS] Intensity Height - Sending START command");
            // std::string s_target_deg = std::to_string(mission.f_target_deg);
            std::string s_cmd = "start";
            publishIntensityCmd(s_cmd);
        }

        if (use_sitting_sensor) {
            std::string s_cmd = "start";
            publishPalletDetectTrigger(s_cmd);
        }
    }
    else if (type == "align") {
        LOG_INFO("[DCS] Move backward: angle=%.2f", angle);

        move_msgs::CoreCommand cmd_msg;

        // Target Waypoint
        move_msgs::Waypoint target_wp;
        target_wp.f_x_m = mission.f_current_x;
        target_wp.f_y_m = mission.f_current_y;
        target_wp.f_angle_deg = mission.f_current_deg;
        target_wp.n_drive_type = 1000;
        target_wp.f_speed_ms = 0.5;
        cmd_msg.list_waypoints.push_back(target_wp);
        cmd_msg.b_arrive_align = true;

        if (ros_interface_) {
            ros_interface_->getSendLivePathPub().publish(cmd_msg);
        }
        LOG_INFO(
            "[DCS] Published backward path to navifra/live_path: current(%.2f, %.2f) -> target(%.2f, %.2f)", mission.f_current_x,
            mission.f_current_y, mission.f_target_x, mission.f_target_y);
    }

    LOG_INFO("[DCS] Move: type=%s, distance=%.2f, angle=%.2f", type.c_str(), distance, angle);
}

void ActionExecutor::executePerception(const ActionParams& params, const Mission& mission)
{
    std::string command = params.getString("command", "");
    std::string target_type = params.getString("target_type", "pallet");
    float f_docking_dist = std::hypot(mission.f_current_x - mission.f_target_x, mission.f_current_y - mission.f_target_y);
    bool enabled = params.getBool("enabled", true);

    if (enabled && !command.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if(target_type == "pallet"){
            publishPerceptionCmd(command, target_type, mission.n_pallet_type, f_docking_dist);
        }
        else if (target_type == "wingbody"){
            publishWingbodyPerceptionCmd(command, target_type, mission, f_docking_dist);
        }
        LOG_INFO(
            "[DCS] Perception: command=%s, target_type=%s, n_pallet_type=%d", command.c_str(), target_type.c_str(), mission.n_pallet_type);
    }
}

void ActionExecutor::executeIntensityHeight(const ActionParams& params, const Mission& mission)
{
    LOG_INFO("[DCS] Executing Intensity Perception - Sending START command");
    // std::string s_target_deg = std::to_string(mission.f_target_deg);
    std::string s_cmd = "start";
    publishIntensityCmd(s_cmd);
}

void ActionExecutor::publishPerceptionCmd(const std::string& cmd, const std::string& target_type, int n_pallet_type, float f_docking_dist)
{
    // Create JSON object
    nlohmann::json json_data;
    json_data["cmd"] = cmd;
    json_data["dist"] = f_docking_dist;
    json_data["x"] = f_docking_dist;
    json_data["y"] = 0;
    json_data["z"] = 0;
    json_data["deg"] = 0;
    json_data["width"] = 0;
    json_data["height"] = 0;
    json_data["type"] = n_pallet_type;
    json_data["local"] = false;

    // Convert to string and publish
    std_msgs::String msg;
    msg.data = json_data.dump();
    if (ros_interface_) {
        ros_interface_->getPerceptionPub().publish(msg);
    }
    LOG_DEBUG("[DCS] Published Perception Cmd as JSON: %s", msg.data.c_str());
}

void ActionExecutor::publishWingbodyPerceptionCmd(const std::string& cmd, const std::string& target_type, const Mission& mission, float f_docking_dist)
{
    // Create JSON object
    nlohmann::json json_data;
    json_data["cmd"] = cmd;
    json_data["dist"] = f_docking_dist;
    json_data["x"] = mission.f_target_x;
    json_data["y"] = mission.f_target_y;
    json_data["z"] = 1;
    json_data["deg"] = mission.f_target_deg;
    json_data["width"] = 0;
    json_data["height"] = 0;
    json_data["type"] = 0;
    json_data["local"] = false;

    // Convert to string and publish
    std_msgs::String msg;
    msg.data = json_data.dump();
    if (ros_interface_) {
        ros_interface_->getWingbodyPerceptionPub().publish(msg);
    }
    LOG_DEBUG("[DCS] Published Wingbody Perception Cmd as JSON: %s", msg.data.c_str());
}

// lccs case
//안씀// 씀 (로드) // 씀 ( 언로드)스 // 개별케이스

void ActionExecutor::publishPathPlanCmd(const nlohmann::json& info)
{
    std_msgs::String msg;
    msg.data = info.dump();
    if (ros_interface_) {
        ros_interface_->getPathPlanCmdPub().publish(msg);
    }
}

void ActionExecutor::publishPalletDetectTrigger(const std::string& cmd)
{
    std_msgs::String msg;
    msg.data = cmd;
    if (ros_interface_) {
        ros_interface_->getSendPalletDetectTrigger().publish(msg);
    }
}

void ActionExecutor::publishForkObsTrigger(bool mode)
{
    std_msgs::Bool msg;
    msg.data = mode;
    if (ros_interface_) {
        ros_interface_->getSentForkObsTrigger().publish(msg);
    }
}

void ActionExecutor::publishIntensityCmd(const std::string& cmd)
{
    std_msgs::String msg;
    msg.data = cmd;
    if (ros_interface_) {
        ros_interface_->getIntensityCmdPub().publish(msg);
    }
    LOG_INFO("[DCS] Published Intensity Cmd: %s", cmd.c_str());
}

}  // namespace nc_dcs
