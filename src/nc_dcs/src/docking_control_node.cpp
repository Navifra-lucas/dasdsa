#include "nc_dcs/docking_control_node.h"

#include "pos/pos.hpp"

#include <move_msgs/CoreCommand.h>
#include <move_msgs/Waypoint.h>
#include <ros/package.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace nc_dcs {

double Quat2Yaw(const geometry_msgs::Quaternion& quat)
{
    // Convert quaternion to yaw angle in degrees
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    double yaw_rad = std::atan2(siny_cosp, cosy_cosp);
    return yaw_rad * (180.0 / M_PI);  // Convert to degrees
}

double Quat2Pitch(const geometry_msgs::Quaternion& quat)
{
    // Convert quaternion to pitch angle in degrees
    double sinp = 2.0 * (quat.w * quat.y - quat.z * quat.x);
    double pitch_rad;
    if (std::abs(sinp) >= 1.0) {
        pitch_rad = std::copysign(M_PI / 2.0, sinp);  // Gimbal lock
    } else {
        pitch_rad = std::asin(sinp);
    }
    return pitch_rad * (180.0 / M_PI);  // Convert to degrees
}

DockingControlNode::DockingControlNode()
    : pnh_("~")
    , loop_rate_(20) // 20Hz
    , current_state_(State::IDLE)
    , current_step_index_(0)
    , pause_step_index_(-1)
    , perception_received_(false)
    , intensity_received_(false)
    , b_docking_path_received_(false)
    , docking_done_(false)
    , goback_done_(false)
    , lift_done_(false)
    , mission_received_(false)
    , path_plan_cmd_sent_(false)
    , step_initialized_(false)
    , delay_counter_(0)
    , perception_update_count_(0)
    , intensity_update_count_(0)
    , retry_available_(false)
    , retry_step_index_(0)
    , is_retry_execution_(false)
{
    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Load sequence configuration
    std::string config_file;
    if (pnh_.getParam("sequence_config", config_file)) {
        LOG_INFO("[DCS] Loading sequences from parameter: %s", config_file.c_str());
        if (!sequence_loader_.loadFromFile(config_file)) {
            LOG_INFO("[DCS] Failed to load from parameter, trying default location");
        }
    }

    // Try default location if not loaded
    if (sequence_loader_.getAvailableDriveTypes().empty()) {
        std::string default_config = ros::package::getPath("nc_dcs") + "/config/sequences.yaml";
        LOG_INFO("[DCS] Loading sequences from: %s", default_config.c_str());
        if (!sequence_loader_.loadFromFile(default_config)) {
            LOG_ERROR("[DCS] Failed to load sequence configuration!");
        }
    }

    // Load fork parameters configuration
    std::string fork_params_file;
    if (pnh_.getParam("fork_parameters_config", fork_params_file)) {
        LOG_INFO("[DCS] Loading fork parameters from parameter: %s", fork_params_file.c_str());
        if (!fork_params_loader_.loadFromFile(fork_params_file)) {
            LOG_INFO("[DCS] Failed to load from parameter, trying default location");
        }
    }

    // Try default location if not loaded
    if (!fork_params_loader_.hasRackType(0) && !fork_params_loader_.hasRackType(1) && !fork_params_loader_.hasRackType(2)) {
        std::string default_fork_config = ros::package::getPath("nc_dcs") + "/config/fork_parameters.yaml";
        LOG_INFO("[DCS] Loading fork parameters from: %s", default_fork_config.c_str());
        if (!fork_params_loader_.loadFromFile(default_fork_config)) {
            LOG_ERROR("[DCS] Failed to load fork parameters configuration!");
        }
    }

    // Initialize ROS Interface
    ros_interface_ = std::make_unique<RosInterface>(nh_, pnh_);
    ros_interface_->setupPublishers();

    lift_cmd_pub_ = ros_interface_->getLiftCmdPub();

    // Initialize Action Executor
    action_executor_ = std::make_unique<ActionExecutor>(ros_interface_.get());

    // Set fork parameters loader to action executor
    action_executor_->setForkParametersLoader(fork_params_loader_);

    // Initialize State Handler (use RosInterface directly)
    state_handler_ = std::make_unique<StateHandler>(*action_executor_, ros_interface_.get());

    // Setup subscribers with callbacks
    ros_interface_->setPerceptionCallback(boost::bind(&DockingControlNode::perceptionCallback, this, _1));
    ros_interface_->setWingbodyPerceptionCallback(boost::bind(&DockingControlNode::wingbodyPerceptionCallback, this, _1));
    ros_interface_->setPathPlanStatusCallback(boost::bind(&DockingControlNode::pathPlanStatusCallback, this, _1));
    ros_interface_->setLiftStatusCallback(boost::bind(&DockingControlNode::liftStatusCallback, this, _1));
    ros_interface_->setForkLiftCallback(boost::bind(&DockingControlNode::forkLiftCallback, this, _1));
    ros_interface_->setTaskCmdCallback(boost::bind(&DockingControlNode::TaskCmdCallback, this, _1));
    ros_interface_->setDockingPathPlanCallback(boost::bind(&DockingControlNode::dockingPathPlanCallback, this, _1));
    ros_interface_->setReturnPathPlanCallback(boost::bind(&DockingControlNode::returnPathPlanCallback, this, _1));
    ros_interface_->setNaviAlarmCallback(boost::bind(&DockingControlNode::naviAlarmCallback, this, _1));
    ros_interface_->setForkPauseCallback(boost::bind(&DockingControlNode::forkPauseCallback, this, _1));
    ros_interface_->setIntensityPoseCallback(boost::bind(&DockingControlNode::intensityPoseCallback, this, _1));
    ros_interface_->setRobotPosCallback(boost::bind(&DockingControlNode::robotPosCallback, this, _1));
    ros_interface_->setOdomCallback(boost::bind(&DockingControlNode::odomCallback, this, _1));

    ros_interface_->setupSubscribers();

    LOG_INFO("[DCS] Docking Control Node initialized");

    // Print available sequences
    auto types = sequence_loader_.getAvailableDriveTypes();
    LOG_INFO("[DCS] Available drive types: %zu", types.size());
    for (int type : types) {
        auto seq = sequence_loader_.getSequence(type);
        LOG_INFO("[DCS]   - Drive Type %d: %s (%zu steps)", type, seq.name.c_str(), seq.steps.size());
    }
}

void DockingControlNode::TaskCmdCallback(const std_msgs::String::ConstPtr& msg)
{
    LOG_INFO("[DCS] TaskCmdCallback received: %s", msg->data.c_str());

    if (msg->data == "cancel") {
        if (current_state_ == State::EXECUTING_SEQUENCE || current_state_ == State::PAUSED) {
            saveRetryState();  // Save retry state before cancellation
            current_state_ = State::STOPPED;
            LOG_INFO("[DCS] Sequence cancelled, state -> IDLE");
        }
        else {
            LOG_INFO("[DCS] CANCEL ignored - no active sequence");
        }
    }
    else if (msg->data == "pause") {
        if (current_state_ == State::EXECUTING_SEQUENCE) {
            current_state_ = State::PAUSED;
            LOG_INFO("[DCS] PAUSE command - Sequence paused at step %zu/%zu", current_step_index_ + 1, current_sequence_.steps.size());
        }
        else {
            LOG_INFO("[DCS] PAUSE ignored - not executing (current state: %d)", static_cast<int>(current_state_));
        }
    }
    else if (msg->data == "resume") {
        if (current_state_ == State::PAUSED) {
            current_state_ = State::EXECUTING_SEQUENCE;
            LOG_INFO("[DCS] RESUME command - Continuing from step %zu/%zu", current_step_index_ + 1, current_sequence_.steps.size());
        }
        else {
            LOG_INFO("[DCS] RESUME ignored - not paused (current state: %d)", static_cast<int>(current_state_));
        }
    }
    else if (msg->data == "retry") {
        if (current_state_ == State::IDLE) {
            RetryCommand();
        }
    }
    else {
        LOG_ERROR("[DCS] Unknown task command: %s", msg->data.c_str());
    }
}

void DockingControlNode::perceptionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    perception_recv_time_ = ros::Time::now();
    float f_diff_dist = hypot(goal_pose_.pose.position.x - msg->pose.position.x, goal_pose_.pose.position.y - msg->pose.position.y);
    // if(b_docking_path_received_ && f_diff_dist > 1) {
    //     NLOG(info)<<"[DCS] perceptionCallback: goal pose is too far from current pose. diff dist : "<<f_diff_dist;
    //     b_updated_perception_ = false;
    //     return;
    // }
    {
        std::lock_guard<std::mutex> lock(target_pose_mutex_);
        target_pose_ = *msg;
        perception_received_ = true;

        // Automatically forward perception pose to /path_plan/pose
        ros_interface_->getPathPlanPosePub().publish(*msg);
    }

    if (msg->pose.position.z == -999.0) {
        n_perception_999_cnt_++;
    }

    if (perception_update_count_ >= MAX_PERCEPTION_UPDATES) {
        return;
    }
    bool should_update = false;

    // First update: immediate
    if (perception_update_count_ == 0) {
        should_update = true;
    }
    // Subsequent updates: check time window
    else if (ros::Time::now() - perception_window_start_ >= ros::Duration(PERCEPTION_WINDOW_SEC)) {
        should_update = true;
    }

    if (should_update) {
        {
            std::lock_guard<std::mutex> lock(target_pose_mutex_);
            perception_received_ = true;
            b_updated_perception_ = true;
        }

        perception_window_start_ = ros::Time::now();
        perception_update_count_++;

        double perception_yaw = Quat2Yaw(msg->pose.orientation);
        double perception_pitch = Quat2Pitch(msg->pose.orientation);
        LOG_INFO(
            "[DCS] Perception Updated. Count: %d/%d | x=%.3f, y=%.3f, z=%.3f, yaw=%.2f, pitch=%.2f", perception_update_count_,
            MAX_PERCEPTION_UPDATES, msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, perception_yaw, perception_pitch);
    }
}

void DockingControlNode::wingbodyPerceptionCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    perception_recv_time_ = ros::Time::now();
    static std::vector<double> vec_center_x_sum(5, 0);
    static std::vector<double> vec_center_y_sum(5, 0);
    static std::vector<double> vec_center_deg_sum(5, 0);
    static std::vector<std::vector<double>> vec_center_x(10, std::vector<double>(5, 0));
    static std::vector<std::vector<double>> vec_center_y(10, std::vector<double>(5, 0));
    static std::vector<std::vector<double>> vec_center_deg(10, std::vector<double>(5, 0));

    static int wingbody_update_count = 0;
    if (!perception_received_) {
        if (wingbody_update_count < 9) {
            wingbody_update_count++;
            for (int i = 0; i < msg->poses.size(); i++) {
                double center_x = msg->poses[i].position.x;
                double center_y = msg->poses[i].position.y;
                double center_deg = Quat2Yaw(msg->poses[i].orientation);
                vec_center_x_sum.at(i) += center_x;
                vec_center_y_sum.at(i) += center_y;
                vec_center_deg_sum.at(i) += center_deg;
                vec_center_x.at(wingbody_update_count).at(i) = center_x;
                vec_center_y.at(wingbody_update_count).at(i) = center_y;
                vec_center_deg.at(wingbody_update_count).at(i) = center_deg;
                LOG_INFO("[DCS] Wingbody Perception Received %d : %f, %f, %f", i, center_x, center_y, center_deg);
            }
        }
        else if (wingbody_update_count >= 9) {
            // Calculate point 1.5m away
            std::vector<NaviFra::Pos> o_wingbody_poses;
            for (int i = 0; i < msg->poses.size(); i++) {
#if 1
                double center_x = vec_center_x_sum.at(i) / static_cast<double>(wingbody_update_count);
                double center_y = vec_center_y_sum.at(i) / static_cast<double>(wingbody_update_count);
                double center_deg = vec_center_deg_sum.at(i) / static_cast<double>(wingbody_update_count);
#else
                std::sort(vec_center_x.at(wingbody_update_count).begin(), vec_center_x.at(wingbody_update_count).end());
                std::sort(vec_center_y.at(wingbody_update_count).begin(), vec_center_y.at(wingbody_update_count).end());
                std::sort(vec_center_deg.at(wingbody_update_count).begin(), vec_center_deg.at(wingbody_update_count).end());
                double center_x = vec_center_x.at(4).at(i);
                double center_y = vec_center_y.at(4).at(i);
                double center_deg = vec_center_deg.at(4).at(i);
#endif
                double dist = 1.59;
                double rad = center_deg * M_PI / 180.0;
                double target_x = center_x + dist * cos(rad);
                double target_y = center_y + dist * sin(rad);

                LOG_INFO("[DCS] Target 1.5m offset calculated: x=%f, y=%f (deg=%f)", target_x, target_y, center_deg);
                NaviFra::Pos o_pos;
                o_pos.SetXm(static_cast<float>(target_x));
                o_pos.SetYm(static_cast<float>(target_y));
                o_pos.SetDeg(static_cast<float>(center_deg));
                o_wingbody_poses.push_back(o_pos);
            }

            // Publish wingbody poses
            geometry_msgs::PoseArray wingbody_poses_msg;
            wingbody_poses_msg.header = msg->header;
            for (const auto& pos : o_wingbody_poses) {
                geometry_msgs::Pose pose;
                pose.position.x = pos.GetXm();
                pose.position.y = pos.GetYm();
                pose.position.z = 0.0;
                tf2::Quaternion q;
                q.setRPY(0, 0, pos.GetDeg() * M_PI / 180.0);
                pose.orientation = tf2::toMsg(q);
                wingbody_deg_ = pos.GetDeg();
                wingbody_poses_msg.poses.push_back(pose);
            }

            geometry_msgs::Pose wingbody_pose_offset;

            wingbody_pose_offset.position.x = wingbody_poses_msg.poses[2].position.x - current_mission_.f_target_x;
            wingbody_pose_offset.position.y = wingbody_poses_msg.poses[2].position.y - current_mission_.f_target_y;
            wingbody_pose_offset.position.z = 0.0;

            tf2::Quaternion q_offset;
            q_offset.setRPY(0, 0, (wingbody_deg_ - current_mission_.f_target_deg) * M_PI / 180.0);
            wingbody_pose_offset.orientation = tf2::toMsg(q_offset);

            NLOG(info) << "[DCS] Publishing wingbody pose offset: x=" << wingbody_pose_offset.position.x
                       << ", y=" << wingbody_pose_offset.position.y << ", theta=" << Quat2Yaw(wingbody_pose_offset.orientation);

            ros_interface_->getWingbodyOffsetPub().publish(wingbody_pose_offset);

            ros_interface_->getWingbodyPoseArrayPub().publish(wingbody_poses_msg);
            vec_center_x_sum.clear();
            vec_center_x_sum.resize(5, 0);
            vec_center_y_sum.clear();
            vec_center_y_sum.resize(5, 0);
            vec_center_deg_sum.clear();
            vec_center_deg_sum.resize(5, 0);
            vec_center_x.clear();
            vec_center_x.resize(10, std::vector<double>(5, 0));
            vec_center_y.clear();
            vec_center_y.resize(10, std::vector<double>(5, 0));
            vec_center_deg.clear();
            vec_center_deg.resize(10, std::vector<double>(5, 0));

            wingbody_update_count = 0;
            perception_received_ = true;
        }
    }
}

void DockingControlNode::intensityPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{	
    perception_recv_time_ = ros::Time::now();
    static bool should_update = false;
    static int n_pre_height = 0;

    // Wait 1 second after intensity step started before processing pose
    if (ros::Time::now() - intensity_start_time_ < ros::Duration(1.0)) {
        return;
    }

    // Step 1: Apply sensor→robot transformation
    geometry_msgs::PoseStamped robot_frame_pose = transformSensorToRobot(msg);

    // Step 2: Transform from robot's base_link to map frame using TF2
    geometry_msgs::PoseStamped map_frame_pose;
    if (!transformRobotToMap(robot_frame_pose, map_frame_pose)) {
        return;
    }
    geometry_msgs::PoseStamped robot_pos;
    {
        std::lock_guard<std::mutex> lock(robot_pose_mutex_);
        robot_pos = robot_pose_;
    }
    float f_remain_dist =
        hypot(map_frame_pose.pose.position.x - robot_pos.pose.position.x, map_frame_pose.pose.position.y - robot_pos.pose.position.y);
    {
        std::lock_guard<std::mutex> lock(target_pose_mutex_);
        // target_pose_ = *msg;
        target_pose_ = map_frame_pose;
        // Set yaw to 0
        tf2::Quaternion q;
        // q.setRPY(0, 0, 3.1415926535 + current_mission_.f_target_deg * M_PI / 180.0);  // roll=0, pitch=0, yaw=0
        // q.setRPY(0, 0, 3.1415926535 + wingbody_deg_ * M_PI/180.0);  // roll=0, pitch=0, yaw=0
        // target_pose_.pose.orientation = tf2::toMsg(q);
		static int n_intensity_999_cnt = 0;

		if (!intensity_received_){
			++n_intensity_999_cnt;
			if (n_intensity_999_cnt > 5) {
				intensity_received_ = true; // Prevent further processing of -999 poses after 5 consecutive messages
				//LOG_("[DCS] Intensity pose has invalid z value (-999) for %d consecutive messages", n_intensity_999_cnt);
				n_intensity_999_cnt = 0; // Reset counter after flagging
			}
		}

        if (f_remain_dist > 3.0) {
            ros_interface_->getPathPlanPosePub().publish(target_pose_);
        }
    }

    if (intensity_update_count_ >= MAX_INTENSITY_UPDATES || b_updated_move_height_) {
        return;
    }

    // First update: immediate
    if (intensity_update_count_ == 0) {
        should_update = true;
    }
    // Subsequent updates: check time window
    else if ((ros::Time::now() - intensity_window_start_ >= ros::Duration(INTENSITY_WINDOW_SEC))) {
        NLOG(info) << "[DCS] Intensity Update Triggered";
        should_update = true;
        b_fork_lift_completed_ = false;
        if (abs(n_pre_height - lift_status_.fork_up_down_position) <= 5) {
            NLOG(info) << "[DCS] Intensity Last Update Triggered";
            b_last_intensity_update_ = true;
        }
    }

    if (should_update) {
        // double height = msg->pose.position.z + 0.150; // use flug height
        double height = msg->pose.position.z;  // flat height
        // intensity_height_sum_ += height;
        // intensity_samples_count_++;
        // NLOG(info)<<"[DCS] Intensity Updated. Count: "<<intensity_samples_count_<<"/"<<TARGET_INTENSITY_SAMPLES;
        // if (intensity_samples_count_ >= TARGET_INTENSITY_SAMPLES) {
        // double avg_height = intensity_height_sum_ / intensity_samples_count_;
        // Store measured height in mission
        {
            std::lock_guard<std::mutex> lock(mission_mutex_);
            current_mission_.intensity_height = height;
        }
        intensity_window_start_ = ros::Time::now();
        intensity_update_count_++;
        // intensity_samples_count_ = 0;
        // intensity_height_sum_ = 0;
        b_updated_intensity_ = true;
        should_update = false;
        n_pre_height = lift_status_.fork_up_down_position;
        LOG_INFO("[DCS] Intensity Averaging Complete. Height: %.3f, Count: %d/%d", height, intensity_update_count_, MAX_INTENSITY_UPDATES);
        // }
    }
}

void DockingControlNode::robotPosCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(robot_pose_mutex_);
    sub_robot_pos_.header = msg->header;
    sub_robot_pos_.pose = msg->pose.pose;  // PoseWithCovariance에서 Pose 추출

    if (!b_odom_use_) {
        robot_pose_ = sub_robot_pos_;
    }
}

void DockingControlNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (b_odom_use_) {
        NaviFra::SimplePos o_odom_pos = PosConvert(msg);

        if (!b_odom_start_) {
            o_prev_odom_ = o_odom_pos;
            robot_pose_ = sub_robot_pos_;
            b_odom_start_ = true;
        }

        mtx_lock_odom_.lock();
        NaviFra::SimplePos o_robot(robot_pose_.pose.position.x, robot_pose_.pose.position.y, Quat2Yaw(robot_pose_.pose.orientation));

        auto prev_to_current = o_prev_odom_.inv() * o_odom_pos;
        auto new_robot_pos = o_robot * prev_to_current;

        robot_pose_.pose.position.x = new_robot_pos.GetXm();
        robot_pose_.pose.position.y = new_robot_pos.GetYm();
        tf2::Quaternion q;
        q.setRPY(0, 0, new_robot_pos.GetRad());
        robot_pose_.pose.orientation = tf2::toMsg(q);

        mtx_lock_odom_.unlock();
        o_prev_odom_ = o_odom_pos;

        PublishExternalPos(robot_pose_);
    }
}

void DockingControlNode::pathPlanStatusCallback(const std_msgs::String::ConstPtr& msg)
{
    LOG_INFO("[DCS] Path Plan Status received: '%s'", msg->data.c_str());
}

void DockingControlNode::dockingPathPlanCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    // Convert PoseArray to CoreCommand and forward to /navifra/live_path
    move_msgs::CoreCommand tmp_path;
    float f_speed = 0.4;
    float f_rad = 0;
    NaviFra::Pos pos1, pos2;
    NaviFra::SimplePos pos_m;

    if (msg->poses.size() == 3) {
        // f_speed = 0.2;

        pos1.SetXm(msg->poses[0].position.x);
        pos1.SetYm(msg->poses[0].position.y);
        {
            tf2::Quaternion q(
                msg->poses[0].orientation.x, msg->poses[0].orientation.y, msg->poses[0].orientation.z, msg->poses[0].orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            pos1.SetRad(yaw);
        }

        pos2.SetXm(msg->poses[1].position.x);
        pos2.SetYm(msg->poses[1].position.y);
        {
            tf2::Quaternion q(
                msg->poses[1].orientation.x, msg->poses[1].orientation.y, msg->poses[1].orientation.z, msg->poses[1].orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            pos2.SetRad(yaw);
        }

        auto local_pos1 = pos2.inv() * pos1;
        float f_L = fabs(local_pos1.GetXm());
        float f_D = fabs(local_pos1.GetYm());
        f_rad = 0.5 * (pow(local_pos1.GetXm(), 2) + pow(local_pos1.GetYm(), 2)) / f_D;
        float f_N = -f_D / sin(atan2(f_L, f_rad - f_D));
        NLOG(info) << " f_L : " << f_L << " D : " << f_D << " f_n : " << f_N << " f_rad : " << f_rad;
        pos_m.SetXm(pos2.GetXm() + f_N * cos(pos2.GetRad()));
        pos_m.SetYm(pos2.GetYm() + f_N * sin(pos2.GetRad()));
    }

    // Convert each Pose to Waypoint
    for (size_t i = 0; i < msg->poses.size(); i++) {
        const auto& pose = msg->poses[i];
        if (i == msg->poses.size() - 1) {
            goal_pose_.pose = pose;
        }
        move_msgs::Waypoint wp;

        // Extract position
        wp.f_x_m = pose.position.x;
        wp.f_y_m = pose.position.y;

        // Convert quaternion to yaw angle in degrees
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        wp.f_angle_deg = yaw * (180.0 / M_PI) + 180;
        if (wp.f_angle_deg > 180.0)
            wp.f_angle_deg -= 360.0;
        else if (wp.f_angle_deg <= -180.0)
            wp.f_angle_deg += 360.0;

        // Set default values
        if (msg->poses.size() == 3 && i == 0) {
            // if (fabs(f_rad) > 10) {
            //     f_speed = 0.5;
            // }
            if (fabs(f_rad) < 1.5) {
                f_speed = 0.3;
            }
            // else if (fabs(f_rad) < 6) {
            //     f_speed = 0.4;
            // }
            // else {
            //     f_speed = 0.2;
            // }
        }
        wp.f_speed_ms = f_speed;

        wp.n_drive_type = 2000;
        wp.s_id = "";
        wp.s_name = "";
        wp.list_f_move_obstacle_margin.clear();
        wp.b_stop_quick = false;
        wp.b_start_quick = true;

        if (current_mission_.type == "UNLOADING" && current_mission_.n_rack_type == 2) {
            wp.list_f_move_obstacle_margin.push_back(0);
            wp.list_f_move_obstacle_margin.push_back(0.2);
            wp.list_f_move_obstacle_margin.push_back(1.1);
            wp.list_f_move_obstacle_margin.push_back(1.1);
        }
        else {
            wp.list_f_move_obstacle_margin.push_back(0);
            wp.list_f_move_obstacle_margin.push_back(0.2);
            wp.list_f_move_obstacle_margin.push_back(0.7);
            wp.list_f_move_obstacle_margin.push_back(0.7);
        }

        move_msgs::LidarObstacle lidar_obs;
        lidar_obs.list_f_obstacle_margin.resize(4);
        lidar_obs.list_f_obstacle_margin.at(0) = -2;
        lidar_obs.list_f_obstacle_margin.at(1) = -2;
        lidar_obs.list_f_obstacle_margin.at(2) = -2;
        lidar_obs.list_f_obstacle_margin.at(3) = -2;

        tmp_path.list_waypoints.push_back(wp);
        tmp_path.list_lidar_obs.push_back(lidar_obs);

        if (msg->poses.size() == 3 && i == 0) {
            wp.f_x_m = pos_m.GetXm();
            wp.f_y_m = pos_m.GetYm();

            float f_deg_diff = std::fmod(pos2.GetDeg() - pos1.GetDeg(), 360.0f);
            if (f_deg_diff > 180.0f) {
                f_deg_diff -= 360.0f;
            }
            else if (f_deg_diff < -180.0f) {
                f_deg_diff += 360.0f;
            }

            wp.f_angle_deg = pos1.GetDeg() + f_deg_diff * 0.5f;
            wp.f_curve_radius = f_rad;

            tmp_path.list_waypoints.push_back(wp);
            tmp_path.list_lidar_obs.push_back(lidar_obs);
        }
    }
    // if (current_mission_.type == "UNLOADING") {
    //     tmp_path.b_arrive_align = true;
    // }
    ros_interface_->getSendLivePathPub().publish(tmp_path);
    b_docking_path_received_ = true;
}

void DockingControlNode::returnPathPlanCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    // Convert PoseArray to CoreCommand and forward to /navifra/live_path
    move_msgs::CoreCommand tmp_path;
    float f_rad;
    NaviFra::Pos pos1, pos2;
    NaviFra::SimplePos pos_m;

    if (msg->poses.size() > 2) {
        pos1.SetXm(msg->poses[2].position.x);
        pos1.SetYm(msg->poses[2].position.y);
        {
            tf2::Quaternion q(
                msg->poses[2].orientation.x, msg->poses[2].orientation.y, msg->poses[2].orientation.z, msg->poses[2].orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            pos1.SetRad(yaw);
        }

        pos2.SetXm(msg->poses[1].position.x);
        pos2.SetYm(msg->poses[1].position.y);
        {
            tf2::Quaternion q(
                msg->poses[1].orientation.x, msg->poses[1].orientation.y, msg->poses[1].orientation.z, msg->poses[1].orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            pos2.SetRad(yaw);
        }

        auto local_pos1 = pos2.inv() * pos1;
        float f_L = fabs(local_pos1.GetXm());
        float f_D = fabs(local_pos1.GetYm());
        f_rad = 0.5 * (pow(local_pos1.GetXm(), 2) + pow(local_pos1.GetYm(), 2)) / f_D;
        float f_N = f_D / sin(atan2(f_L, f_rad - f_D));
        NLOG(info) << " f_L : " << f_L << " D : " << f_D << " f_n : " << f_N << " f_rad : " << f_rad;
        pos_m.SetXm(pos2.GetXm() + f_N * cos(pos2.GetRad()));
        pos_m.SetYm(pos2.GetYm() + f_N * sin(pos2.GetRad()));
    }

    // Convert each Pose to Waypoint
    for (size_t i = 0; i < msg->poses.size(); i++) {
        const auto& pose = msg->poses[i];
        move_msgs::Waypoint wp;

        // Extract position
        wp.f_x_m = pose.position.x;
        wp.f_y_m = pose.position.y;

        // Convert quaternion to yaw angle in degrees
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        wp.f_angle_deg = yaw * (180.0 / M_PI);

        // Set default values
        wp.b_stop_quick = true;
        wp.b_start_quick = true;
        wp.f_speed_ms = 1.0;
        wp.n_drive_type = 1000;
        wp.s_id = "";
        wp.s_name = "";
        wp.list_f_move_obstacle_margin.clear();
        if (current_mission_.type == "UNLOADING" && current_mission_.n_rack_type == 2) {
            wp.list_f_move_obstacle_margin.push_back(1.35);
            wp.list_f_move_obstacle_margin.push_back(0.2);
            wp.list_f_move_obstacle_margin.push_back(0.7);
            wp.list_f_move_obstacle_margin.push_back(0.7);
        }
        else {
            wp.list_f_move_obstacle_margin.push_back(1.35);
            wp.list_f_move_obstacle_margin.push_back(0.2);
            wp.list_f_move_obstacle_margin.push_back(1.1);
            wp.list_f_move_obstacle_margin.push_back(1.1);
        }

        move_msgs::LidarObstacle lidar_obs;
        lidar_obs.list_f_obstacle_margin.resize(4);
        lidar_obs.list_f_obstacle_margin.at(0) = -2;
        lidar_obs.list_f_obstacle_margin.at(1) = -2;
        lidar_obs.list_f_obstacle_margin.at(2) = -2;
        lidar_obs.list_f_obstacle_margin.at(3) = -2;

        tmp_path.list_waypoints.push_back(wp);
        tmp_path.list_lidar_obs.push_back(lidar_obs);

        if (msg->poses.size() > 2 && i == 1) {
            wp.f_x_m = pos_m.GetXm();
            wp.f_y_m = pos_m.GetYm();
            if (fabs(f_rad) > 10) {
                wp.f_speed_ms = 0.8;
            }
            else if (fabs(f_rad) > 5) {
                wp.f_speed_ms = 0.5;
            }
            else {
                wp.f_speed_ms = 0.5;
            }

            float f_deg_diff = std::fmod(pos2.GetDeg() - pos1.GetDeg(), 360.0f);
            if (f_deg_diff > 180.0f) {
                f_deg_diff -= 360.0f;
            }
            else if (f_deg_diff < -180.0f) {
                f_deg_diff += 360.0f;
            }

            wp.f_angle_deg = pos1.GetDeg() + f_deg_diff * 0.5f;
            wp.f_curve_radius = f_rad;

            tmp_path.list_waypoints.push_back(wp);
            tmp_path.list_lidar_obs.push_back(lidar_obs);
        }
    }

    ros_interface_->getSendLivePathPub().publish(tmp_path);
}

void DockingControlNode::naviAlarmCallback(const core_msgs::NaviAlarm::ConstPtr& msg)
{
    // Handle GOAL_ARRIVED alarm to complete move/goback operations
    if (msg->alarm == core_msgs::NaviAlarm::GOAL_ARRIVED) {
        LOG_INFO("[DCS] GOAL_ARRIVED alarm received - completing move/goback");

        // Check which type of move is currently being executed
        if (current_step_index_ < current_sequence_.steps.size()) {
            const SequenceStep& step = current_sequence_.steps[current_step_index_];
            if (step.action_type == "move") {
                std::string move_type = step.params.getString("type", "docking");
                if (move_type == "docking") {
                    docking_done_ = true;
                    LOG_INFO("[DCS] Docking move completed via GOAL_ARRIVED");
                }
                else if (move_type == "goback") {
                    goback_done_ = true;
                    LOG_INFO("[DCS] Goback move completed via GOAL_ARRIVED");
                }
                else if (move_type == "backward") {
                    backward_done_ = true;
                    LOG_INFO("[DCS] backward move completed via GOAL_ARRIVED");
                }
                else if (move_type == "forward") {
                    forward_done_ = true;
                    LOG_INFO("[DCS] forward move completed via GOAL_ARRIVED");
                }
                else if (move_type == "align") {
                    align_done_ = true;
                    LOG_INFO(" [DCS] align move completed via GOAL_ARRIVED");
                }
            }
        }
    }
    else if (msg->alarm >= 2000) {
        if (current_state_ == State::EXECUTING_SEQUENCE || current_state_ == State::PAUSED) {
            LOG_INFO("[DCS] Alarm received - %d", msg->alarm);
            saveRetryState();  // Save retry state before termination
            current_state_ = State::STOPPED;
        }
    }
}

void DockingControlNode::forkPauseCallback(const std_msgs::Bool::ConstPtr& msg)
{
    bool paused = msg->data;
    if (paused) {
        if (current_state_ == State::EXECUTING_SEQUENCE) {
            LOG_INFO("[DCS] Fork obstacle detected → PAUSE forklift movement");
            current_state_ = State::PAUSED;
        }
        else {
            LOG_INFO("[DCS] PAUSE ignored - not executing (current state: %d)", static_cast<int>(current_state_));
        }
    }
    else {
        if (current_state_ == State::PAUSED) {
            LOG_INFO("[DCS] Fork obstacle cleared → RESUME forklift movement");
            current_state_ = State::EXECUTING_SEQUENCE;
            step_initialized_ = false;
        }
        else {
            LOG_INFO("[DCS] RESUME ignored - not paused (current state: %d)", static_cast<int>(current_state_));
        }

        // resendLastCommand();
        // step_initialized_ 여기서 false로 바꿔주기...? 더 필요한거 있을라나
    }
}

void DockingControlNode::liftStatusCallback(const core_msgs::CheonilReadRegister::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(lift_status_mutex_);
    lift_status_ = *msg;

    if (msg->fork_up_down_complete) {
        LOG_INFO("[DCS] Fork lift completed: %d", msg->fork_up_down_complete);
        b_fork_lift_completed_ = true;
    }
    
    // UNLOADING place (once mode only): check fork height change without intensity callback
    // When fork stops moving (height change <= 5mm), lift 30mm then mark done
    // if (current_mission_.type == "UNLOADING" && current_mission_.n_rack_type == 2 &&
    //     current_state_ == State::EXECUTING_SEQUENCE &&
    //     current_step_index_ < current_sequence_.steps.size()) {
        
    //     const SequenceStep& current_step = current_sequence_.steps[current_step_index_];
    //     if (current_step.action_type == "fork_updown" && 
    //         current_step.params.getString("operation", "") == "place") {
        
    //         if (ros::Time::now() - once_mode_height_check_time_ >= ros::Duration(INTENSITY_WINDOW_SEC)) {
    //             int current_height = msg->fork_up_down_position;
                
    //             if (!b_once_mode_lift_30mm_commanded_) {
    //                 // Phase 1: Detect fork stopped (height change <= 5mm)
    //                 if (n_once_mode_prev_height_ != 0 && abs(n_once_mode_prev_height_ - current_height) <= 5) {
    //                     // Fork stopped - command 30mm lift before pulling out
    //                     n_once_mode_lift_30mm_target_ = current_height + 30;
    //                     b_once_mode_lift_30mm_commanded_ = true;
                        
    //                     core_msgs::WiaForkInfo fork_msg;
    //                     fork_msg.n_fork_height = n_once_mode_lift_30mm_target_;
    //                     fork_msg.f_fork_tilt = -1;
    //                     fork_msg.n_fork_wide = -1;
    //                     fork_msg.b_cancel = false;
    //                     lift_cmd_pub_.publish(fork_msg);
                        
    //                     // Update real target height for checkStepCompletion position check
    //                     current_mission_.n_real_target_height = n_once_mode_lift_30mm_target_;
                        
    //                     LOG_INFO("[DCS] UNLOADING place: Fork stopped (prev=%d, curr=%d) - commanding 30mm lift to %d",
    //                              n_once_mode_prev_height_, current_height, n_once_mode_lift_30mm_target_);
    //                 }
    //             } else {
    //                 // Phase 2: Check if 30mm lift is complete
    //                 if (abs(current_height - n_once_mode_lift_30mm_target_) <= 5) {
    //                     lift_done_ = true;
    //                     LOG_INFO("[DCS] UNLOADING place: 30mm lift complete (target=%d, curr=%d) - lift_done=true",
    //                              n_once_mode_lift_30mm_target_, current_height);
    //                 }
    //             }
                
    //             n_once_mode_prev_height_ = current_height;
    //             once_mode_height_check_time_ = ros::Time::now();
    //         }
    //     }
    // }
}

void DockingControlNode::forkLiftCallback(const core_msgs::ForkLift::ConstPtr& msg)
{
    int drive_type = msg->n_drive_type;

    LOG_INFO("[DCS] Received ForkLift message: drive_type=%d, target_height=%d", drive_type, msg->n_target_height);

    if (!sequence_loader_.hasSequence(drive_type)) {
        LOG_ERROR("[DCS] No sequence configured for drive_type=%d", drive_type);
        return;
    }

    current_sequence_ = sequence_loader_.getSequence(drive_type);

    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        current_mission_.type = current_sequence_.name;
        current_mission_.f_current_x = msg->f_current_x;
        current_mission_.f_current_y = msg->f_current_y;
        current_mission_.f_current_deg = msg->f_current_deg;
        current_mission_.f_target_x = msg->f_target_x;
        current_mission_.f_target_y = msg->f_target_y;
        current_mission_.f_target_deg = msg->f_target_deg;
        current_mission_.n_rack_level = msg->n_rack_level;
        current_mission_.n_target_level = msg->n_target_level;
        current_mission_.n_target_height = msg->n_target_height;
        current_mission_.n_rack_type = msg->n_rack_type;
        current_mission_.n_pallet_type = msg->n_pallet_type;

        // Reset perception data for new mission
        current_mission_.b_perception_valid = false;
        current_mission_.f_perception_z = 0.0;
    }

    mission_received_ = true;
    is_retry_execution_ = false;  // Reset retry execution flag for new normal mission
    LOG_INFO(
        "[DCS] Starting sequence: %s (%s) with %zu steps", current_sequence_.name.c_str(), current_sequence_.description.c_str(),
        current_sequence_.steps.size());
}

void DockingControlNode::resetFlags()
{
    perception_received_ = false;
    intensity_received_ = false;
    b_updated_perception_ = false;
    b_updated_intensity_ = false;
    b_updated_move_height_ = false;
    b_last_intensity_update_ = false;
    b_docking_path_received_ = false;
    b_lift_adjusted_once_ = false;  // Reset lift adjustment flag for "once" mode
    perception_update_count_ = 0;
    n_perception_999_cnt_ = 0;
    intensity_update_count_ = 0;
    n_lift_up_down_retry_count_ = 0;
    docking_done_ = false;
    forward_done_ = false;
    align_done_ = false;
    backward_done_ = false;
    goback_done_ = false;
    lift_done_ = false;
    path_plan_cmd_sent_ = false;
    step_initialized_ = false;
    delay_counter_ = 0;
    intensity_samples_count_ = 0;
    intensity_height_sum_ = 0.0;
    pause_step_index_ = -1;
    b_odom_use_ = false;
    b_odom_start_ = false;
    n_once_mode_prev_height_ = 0;  // Reset for "once" mode height check
    once_mode_height_check_time_ = ros::Time::now();
    b_once_mode_lift_30mm_commanded_ = false;  // Reset for LOADING once mode 30mm lift
    n_once_mode_lift_30mm_target_ = 0;
}

void DockingControlNode::run()
{
    while (ros::ok()) {
        ros::spinOnce();

        // NLOG(info) << "[DCS] Current State: " << static_cast<int>(current_state_);

        switch (current_state_) {
            case State::IDLE:
                handleIdle();
                break;

            case State::EXECUTING_SEQUENCE:
                handleExecutingSequence();
                break;

            case State::PAUSED:
                handlePaused();
                // Do nothing while paused, just wait for RESUME or CANCEL
                break;

            case State::COMPLETE:
                handleComplete();
                break;

            case State::STOPPED:
                handleStopped();
                break;
        }

        loop_rate_.sleep();
    }
}

void DockingControlNode::handleIdle()
{
    if (mission_received_) {
        mission_received_ = false;
        resetFlags();
        current_step_index_ = 0;
        current_state_ = State::EXECUTING_SEQUENCE;
        LOG_INFO("[DCS] State: IDLE -> EXECUTING_SEQUENCE");
    }
}

void DockingControlNode::handlePaused()
{
    if (pause_step_index_ == -1) {
        pause_step_index_ = current_step_index_;
        core_msgs::WiaForkInfo msg;
        msg.n_fork_height = -1;
        msg.f_fork_tilt = -1;
        msg.n_fork_wide = -1;
        msg.b_cancel = true;
        lift_cmd_pub_.publish(msg);

        LOG_INFO("[DCS] Sequence paused at step %zu/%zu", pause_step_index_ + 1, current_sequence_.steps.size());
        return;
    }
}

void DockingControlNode::handleExecutingSequence()
{
    if (current_step_index_ >= current_sequence_.steps.size()) {
        current_state_ = State::COMPLETE;
        LOG_INFO("[DCS] All steps completed -> COMPLETE");
        return;
    }
    pause_step_index_ = -1;
    SequenceStep& step = current_sequence_.steps[current_step_index_];

    LOG_INFO(
        "current_mission type : %s , current seq type : %d, current step : %d/%d", current_mission_.type.c_str(),
        current_sequence_.drive_type, current_step_index_ + 1, current_sequence_.steps.size());

    // Process perception parameters
    if (!processPerceptionParams(step)) {
        return;
    }

    // Calculate target command
    nlohmann::json target_cmd_json = calculateTargetCmd(step);

    // Initialize step on first execution
    if (!step_initialized_) {
        LOG_INFO("[DCS] Step %zu/%zu: %s", current_step_index_ + 1, current_sequence_.steps.size(), step.action_type.c_str());
        if (step.action_type == "move" && step.params.getString("type", "") == "align") {
            float current_yaw_deg;
            {
                std::lock_guard<std::mutex> lock(robot_pose_mutex_);
                current_yaw_deg = Quat2Yaw(robot_pose_.pose.orientation);
            }

            float remain_deg = fabs(current_yaw_deg - current_mission_.f_current_deg);

            if (remain_deg < 1.0) {
                LOG_INFO("[DCS] Skipping 'align' step. remain_deg : %.3f < 1.0", remain_deg);
                resetFlags();  // Reset flags for the current step
                current_step_index_++;  // Move to the next step
                LOG_INFO("[DCS] Step passed, moving to next step");
                mission_start_ = ros::Time::now();
                return;
            }
        }

        if (!state_handler_->executeStep(step, current_mission_, target_cmd_json, target_pose_mutex_)) {
            LOG_ERROR("[DCS] Failed to execute step: %s", step.action_type.c_str());
            current_state_ = State::STOPPED;
            return;
        }

        if (step.action_type == "fork_updown") {
            lift_move_time_ = ros::Time::now();
            lift_action_start_ = ros::Time::now();
        }
        else if (step.action_type == "perception") {
            perception_recv_time_ = ros::Time::now();
        }
        else if (step.action_type == "intensity") {
            perception_recv_time_ = ros::Time::now();
            intensity_start_time_ = ros::Time::now();
        }
        // else if (step.action_type == "move" && step.params.getString("type", "") == "docking") {
        //     b_odom_use_ = true;
        //     b_odom_start_ = true;
        // }

        step_start_time_ = ros::Time::now();
        step_initialized_ = true;
    }
    else {
        updateLiftControl(step);
        if (errorCheck(step)) {
            current_state_ = State::STOPPED;
            saveRetryState();
            return;
        }
    }

    // Check if step is complete
    bool step_complete = state_handler_->checkStepCompletion(
        step, current_mission_, lift_status_, lift_status_mutex_, perception_received_, intensity_received_, docking_done_, forward_done_,
        backward_done_, goback_done_, align_done_, lift_done_, delay_counter_, target_cmd_json, target_pose_mutex_, step_start_time_, b_lift_adjusted_once_);

    if (step_complete) {
        finalizeStep(step);
    }
}

void DockingControlNode::handleComplete()
{
    LOG_INFO("[DCS] Sequence '%s' completed successfully", current_sequence_.name.c_str());

    // Publish completion status
    state_handler_->publishCompletion(true);

    // mission 완료시 current_mission_ 초기화
    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        current_mission_ = Mission();
    }

    resetFlags();
    current_step_index_ = 0;
    current_state_ = State::IDLE;
    is_retry_execution_ = false;  // Reset retry execution flag
    LOG_INFO("[DCS] State: COMPLETE -> IDLE");
}

void DockingControlNode::handleStopped()
{
    LOG_INFO("[DCS] Sequence '%s' stopped", current_sequence_.name.c_str());

    // Publish completion status
    state_handler_->publishCompletion(false);

    LOG_INFO("[DCS] CANCEL command - Terminating current sequence");
    resetFlags();
    current_step_index_ = 0;
    current_state_ = State::IDLE;
    // is_retry_execution_ = false;  // Reset retry execution flag

    // Publish cancellation status
    std_msgs::Bool cancel_msg;
    cancel_msg.data = false;  // false indicates cancelled
    ros_interface_->getActionReachedPub().publish(cancel_msg);
    ros_interface_->getSentForkObsTrigger().publish(cancel_msg);

    NLOG(info) << "[DCS] Docking done! Calling path_plan/done service";
    // Call /path_plan/done service when docking is complete
    nlohmann::json target_cmd_json;
    target_cmd_json["n_mission_type"] = 0;  // NONE = 0, PALLET = 1, WINGBODY = 2, GOBACK = 3
    target_cmd_json["o_fNode"] = {{"x", 0.0}, {"y", 0.0}, {"deg", 0.0}};
    target_cmd_json["o_object2goal"] = {{"x", -0.363}, {"y", 0.0}, {"deg", 0.0}};
    target_cmd_json["o_pose"] = {{"x", 0.0}, {"y", 0.0}, {"deg", 0.0}};
    target_cmd_json["b_local"] = false;

    // path_plan_cmd_pub_.publish(msg);
    if (ros_interface_) {
        std_msgs::String msg;
        msg.data = target_cmd_json.dump();
        ros_interface_->getPathPlanCmdPub().publish(msg);

        msg.data = "stop";
        ros_interface_->getIntensityCmdPub().publish(msg);
        msg.data = "protective_stop";
        ros_interface_->getSendPalletDetectTrigger().publish(msg);

        nlohmann::json json_data;
        json_data["cmd"] = "stop";
        json_data["dist"] = 0;
        json_data["x"] = 0;
        json_data["y"] = 0;
        json_data["z"] = 0;
        json_data["deg"] = 0;
        json_data["width"] = 0;
        json_data["height"] = 0;
        json_data["type"] = 0;
        json_data["local"] = false;

        // Convert to string and publish
        msg.data = json_data.dump();
        ros_interface_->getPerceptionPub().publish(msg);
    }
}

geometry_msgs::PoseStamped DockingControlNode::transformSensorToRobot(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::PoseStamped robot_frame_pose;
    robot_frame_pose.header = msg->header;
    robot_frame_pose.header.frame_id = "base_link";

    double sensor_x = msg->pose.position.x;
    double sensor_y = msg->pose.position.y;
    double sensor_z = msg->pose.position.z;

    double deg_to_rad = M_PI / 180.0;
    double cos_theta = cos(robot2sensor_deg_ * deg_to_rad);
    double sin_theta = sin(robot2sensor_deg_ * deg_to_rad);

    robot_frame_pose.pose.position.x = robot2sensor_x_ + (cos_theta * sensor_x - sin_theta * sensor_y);
    robot_frame_pose.pose.position.y = robot2sensor_y_ + (sin_theta * sensor_x + cos_theta * sensor_y);
    robot_frame_pose.pose.position.z = sensor_z;

    float sensor_deg = Quat2Yaw(msg->pose.orientation);
    tf2::Quaternion robot_quat;
    robot_quat.setRPY(0, 0, (robot2sensor_deg_ - sensor_deg) * deg_to_rad);
    robot_frame_pose.pose.orientation = tf2::toMsg(robot_quat);

    LOG_INFO(
        "[DCS] Sensor->Robot transform: sensor(%.3f, %.3f, %.3f, %.3f) -> robot(%.3f, %.3f, %.3f, %.3f)", sensor_x, sensor_y, sensor_z,
        Quat2Yaw(msg->pose.orientation), robot_frame_pose.pose.position.x, robot_frame_pose.pose.position.y,
        robot_frame_pose.pose.position.z, Quat2Yaw(robot_frame_pose.pose.orientation));

    return robot_frame_pose;
}

bool DockingControlNode::transformRobotToMap(const geometry_msgs::PoseStamped& robot_frame_pose, geometry_msgs::PoseStamped& map_frame_pose)
{
    try {
        tf_buffer_->transform(robot_frame_pose, map_frame_pose, "map", ros::Duration(0.1));

        LOG_INFO(
            "[DCS] Robot->Map transform: base_link(%.3f, %.3f, %.3f, %.3f) -> map(%.3f, %.3f, %.3f, %.3f)",
            robot_frame_pose.pose.position.x, robot_frame_pose.pose.position.y, robot_frame_pose.pose.position.z,
            Quat2Yaw(robot_frame_pose.pose.orientation), map_frame_pose.pose.position.x, map_frame_pose.pose.position.y,
            map_frame_pose.pose.position.z, Quat2Yaw(map_frame_pose.pose.orientation));
        return true;
    }
    catch (tf2::TransformException& ex) {
        LOG_ERROR("[DCS] TF2 transform failed (base_link->map): %s", ex.what());
        return false;
    }
}

bool DockingControlNode::processPerceptionParams(SequenceStep& step)
{
    if (current_mission_.type == "UNLOADING" && step.action_type == "perception") {
        if (current_mission_.n_rack_type == 2) {
            step.params.setString("target_type", "wingbody");
        }
        else {
            LOG_INFO("[DCS] Skipping 'perception' step.");
            resetFlags();
            current_step_index_++;
            LOG_INFO("[DCS] Step passed, moving to next step");
            return false;
        }
    }
    return true;
}

nlohmann::json DockingControlNode::calculateTargetCmd(const SequenceStep& step)
{
    nlohmann::json target_cmd_json;
    target_cmd_json["o_fNode"] = {{"x", 0.0}, {"y", 0.0}, {"deg", 0.0}};
    target_cmd_json["o_pose"] = {{"x", 0.0}, {"y", 0.0}, {"deg", 0.0}};

    if (current_mission_.type == "LOADING") {
        target_cmd_json["n_mission_type"] = 1;
        target_cmd_json["b_local"] = false;
        target_cmd_json["o_object2goal"] = {{"x", -0.39}, {"y", 0.0}, {"deg", 0.0}};

        {
            std::lock_guard<std::mutex> lock(last_path_point_mutex_);
            target_cmd_json["o_fNode"] = {
                {"x", current_mission_.f_current_x}, {"y", current_mission_.f_current_y}, {"deg", current_mission_.f_current_deg}};
        }
        target_cmd_json["o_pose"] = {
            {"x", target_pose_.pose.position.x}, {"y", target_pose_.pose.position.y}, {"deg", Quat2Yaw(target_pose_.pose.orientation)}};
    }
    else if (current_mission_.type == "UNLOADING") {
        target_cmd_json["n_mission_type"] = 2;
        target_cmd_json["b_local"] = false;
        target_cmd_json["o_object2goal"] = {{"x", -2.63}, {"y", 0.0}, {"deg", 0.0}};

        {
            std::lock_guard<std::mutex> lock(last_path_point_mutex_);
            target_cmd_json["o_fNode"] = {
                {"x", current_mission_.f_current_x}, {"y", current_mission_.f_current_y}, {"deg", current_mission_.f_current_deg}};
        }
        target_cmd_json["o_pose"] = {
            {"x", target_pose_.pose.position.x}, {"y", target_pose_.pose.position.y}, {"deg", Quat2Yaw(target_pose_.pose.orientation)}};
    }
    return target_cmd_json;
}

void DockingControlNode::updateLiftControl(const SequenceStep& step)
{
    // Calculate tilt value first (will be used in combined command)
    float tilt_value = calculateTiltValue();


    if (current_mission_.type == "LOADING" &&
        (current_mission_.n_rack_type == 2 || current_mission_.n_rack_type == 21 || current_mission_.n_rack_type == 22) &&
        b_updated_perception_ && b_fork_lift_completed_) {
        
        // Wait for path planning to complete before adjusting lift height
        // This prevents lift movement from interfering with perception during path generation
        if (!b_docking_path_received_) {
            NLOG(info) << "[DCS] Waiting for path generation to complete before lift adjustment (LOADING perception)";
            return;
        }
        
        // // Skip if already adjusted once
        // if (b_lift_adjusted_once_) {
        //     b_updated_perception_ = false;
        //     NLOG(info) << "[DCS] Lift adjustment mode is 'once' - skipping further adjustments";
        //     return;
        // }
        
        core_msgs::WiaForkInfo msg;
        double d_m_height = -0.045;
        double d_diff_lift_height = (d_m_height - target_pose_.pose.position.z) * 1000;
        float f_remain_dist =
            hypot(target_pose_.pose.position.x - robot_pose_.pose.position.x, target_pose_.pose.position.y - robot_pose_.pose.position.y);
        if (f_remain_dist < 2.3) {
            d_diff_lift_height = 0;
        }
        NLOG(info) << "setForkHeight : " << d_diff_lift_height << " / m_pose : " << d_m_height << " / remain dist : " << f_remain_dist;
        static float f_speed_percent_pre = 100;
        float f_speed_percent = fabs(d_diff_lift_height) >= fork_params_loader_.getLiftHeightDiffSpeedThreshold() ? fork_params_loader_.getLiftHeightDiffSpeedPercent() : 100;
        std_msgs::Float64 speed_msg;
        if (f_speed_percent != f_speed_percent_pre) {
            speed_msg.data = f_speed_percent;
            NLOG(info) << "Set speed percent: " << speed_msg.data;
            ros_interface_->getSpeedPercentPub().publish(speed_msg);
            f_speed_percent_pre = speed_msg.data;
        }

        auto current_fork_height = lift_status_.fork_up_down_position;
        msg.n_fork_height = current_fork_height - d_diff_lift_height;
        msg.f_fork_tilt = tilt_value;  // Apply calculated tilt value
        msg.n_fork_wide = -1;
        lift_cmd_pub_.publish(msg);
        b_updated_perception_ = false;
        b_fork_lift_completed_ = false;
		
		current_mission_.n_real_target_height = current_fork_height - d_diff_lift_height;
        
        // // Mark as adjusted (wing body LOADING always once)
        // b_lift_adjusted_once_ = true;
        // LOG_INFO("[DCS] Lift adjusted once (LOADING wing body) - height: %.0f", msg.n_fork_height);
        
        double approach_height = fork_params_loader_.getApproachHeight(current_mission_.n_rack_type);
        {
            std::lock_guard<std::mutex> mission_lock(mission_mutex_);
            current_mission_.f_perception_z = approach_height - (current_fork_height - d_diff_lift_height);
            current_mission_.b_perception_valid = true;
        }
    }
    else if (current_mission_.type == "UNLOADING" && current_mission_.n_rack_type == 2 && b_updated_intensity_) {
        
        // Wait for path planning to complete before adjusting lift height
        // This prevents lift movement from interfering with intensity detection during path generation
        std::string operation = step.params.getString("operation", "");
        if (!b_docking_path_received_ && step.action_type == "move") {
            NLOG(info) << "[DCS] Waiting for path generation to complete before lift adjustment (UNLOADING intensity)";
            return;
        }
        
        // UNLOADING wing body: once mode - skip all further intensity adjustments (including place)
        // Place will use 5mm fork stop detection + 30mm lift in liftStatusCallback instead
        if (b_lift_adjusted_once_) {
            b_updated_intensity_ = false;
            NLOG(info) << "[DCS] UNLOADING wing body once mode - skipping intensity adjustment (place uses 5mm detection)";
            return;
        }
        
        core_msgs::WiaForkInfo msg;
        int n_current_fork_height = lift_status_.fork_up_down_position;
        int n_diff_lift_height = current_mission_.intensity_height * 1000;
        std::string move_type = step.params.getString("type", "docking");
        if (step.action_type == "move" && (move_type == "backward" || move_type == "docking") && !b_updated_move_height_) {
            // n_diff_lift_height += 330;
            n_diff_lift_height = 0;
            b_updated_move_height_ = true;
        }

        if (step.action_type == "fork_updown" && operation == "place") {
            LOG_INFO("[DCS] Current Intensity Height : true, %.2f", current_mission_.intensity_height);
            if (current_mission_.intensity_height >= -0.01) {
                lift_done_ = true;
                b_last_intensity_update_ = false;
                LOG_INFO("[DCS] lift_done : true, %.2f", current_mission_.intensity_height);
            }
            else if (b_last_intensity_update_) {
                lift_done_ = true;
                n_diff_lift_height = 30;
                LOG_INFO("[DCS] lift_done : true, %d", n_current_fork_height + n_diff_lift_height);
            }
        }

        if ((step.action_type == "move" && (move_type == "backward" || move_type == "docking")) ||
            (step.action_type == "fork_updown" && operation == "place" && (!lift_done_ || b_last_intensity_update_))) {
            msg.n_fork_height = n_current_fork_height + n_diff_lift_height;
            if (n_current_fork_height + n_diff_lift_height < 1800 && step.action_type == "move" &&
                (move_type == "backward" || move_type == "docking")) {
                msg.n_fork_height = 1800;
                NLOG(info) << "TEST2 : " << n_current_fork_height + n_diff_lift_height;
            }
            msg.f_fork_tilt = tilt_value;  // Apply calculated tilt value
            msg.n_fork_wide = -1;
            lift_cmd_pub_.publish(msg);

            LOG_INFO(
                "[DCS] updated real target height : %d, current height : %d, diff height : %d/%.2f", current_mission_.n_real_target_height,
                n_current_fork_height, n_diff_lift_height, current_mission_.intensity_height);
            b_updated_intensity_ = false;
            b_last_intensity_update_ = false;
            current_mission_.n_real_target_height = n_current_fork_height + n_diff_lift_height;
            
            // Mark as adjusted once (only for move steps in once mode)
            if (step.action_type == "move") {
                b_lift_adjusted_once_ = true;
                LOG_INFO("[DCS] Lift adjusted once (UNLOADING wing body) - commanded_height: %d, current_height: %d, intensity_height: %.3f, diff: %d",
                         static_cast<int>(msg.n_fork_height), n_current_fork_height, current_mission_.intensity_height, n_diff_lift_height);
            }
        }
    }
    else if (tilt_value >= 0) {
        // Only tilt control needed (no lift height change)
        core_msgs::WiaForkInfo msg;
        msg.n_fork_height = -1;  // No height change
        msg.f_fork_tilt = tilt_value;
        msg.n_fork_wide = -1;
        msg.b_cancel = false;
        lift_cmd_pub_.publish(msg);
        LOG_INFO("[DCS] Tilt-only command: tilt=%.1f", tilt_value);
    }
}

float DockingControlNode::calculateTiltValue()
{
    // Returns: 0=tilt down, 1=tilt up, -1=no change needed

    // Check if tilt correction is enabled for current sequence
    if (!current_sequence_.tilt_correction_enabled) {
        return -1.0f;  // Tilt correction disabled in sequence config
    }

    if (!b_updated_perception_) {
        return -1.0f;  // No perception update, no tilt change
    }

    // 퍼셉션 pose의 pitch(앞뒤 기울기)를 deg로 변환
    double perception_pitch_deg = Quat2Pitch(target_pose_.pose.orientation);
    const double TILT_TOLERANCE_DEG = 0.5;

    int current_tilt = lift_status_.tilting_up_down;  // 1: down, 2: up

    if (perception_pitch_deg > TILT_TOLERANCE_DEG && current_tilt != 1) {
        // pitch가 양수(위로 기울어짐) → 틸트 내려야 함
        LOG_INFO(
            "[DCS] calculateTiltValue: pitch=%.2f > +%.1f → tilt DOWN (current_tilt=%d)", perception_pitch_deg, TILT_TOLERANCE_DEG,
            current_tilt);
        return 0.0f;  // down
    }
    else if (perception_pitch_deg < -TILT_TOLERANCE_DEG && current_tilt != 2) {
        // pitch가 음수(아래로 기울어짐) → 틸트 올려야 함
        LOG_INFO(
            "[DCS] calculateTiltValue: pitch=%.2f < -%.1f → tilt UP (current_tilt=%d)", perception_pitch_deg, TILT_TOLERANCE_DEG,
            current_tilt);
        return 1.0f;  // up
    }
    else {
        LOG_DEBUG(
            "[DCS] calculateTiltValue: pitch=%.2f within tolerance ±%.1f → no action (current_tilt=%d)", perception_pitch_deg,
            TILT_TOLERANCE_DEG, current_tilt);
        return -1.0f;  // no change
    }
}

void DockingControlNode::finalizeStep(const SequenceStep& step)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    resetFlags();
    if (current_mission_.n_rack_type == 2) {
        if (step.action_type == "move" && step.params.getString("type", "") == "align") {
            mission_start_ = ros::Time::now();
        }
        if (current_step_index_ == current_sequence_.steps.size() - 1) {
            auto mission_time = ros::Time::now() - mission_start_;
            NLOG(info) << "[DCS] Total " << current_sequence_.name << " " << current_sequence_.steps.size()
                       << " mission time : " << mission_time.toSec();
        }

        if (step.action_type == "fork_updown") {
            auto lift_mission_time = ros::Time::now() - lift_action_start_;
            NLOG(info) << "[DCS] " << current_sequence_.name << " " << current_step_index_ + 1 << "Step fork up/down "
                       << step.params.getString("operation", "") << " mission time : " << lift_mission_time.toSec();
        }
    }
    current_step_index_++;

    LOG_INFO("[DCS] Step completed, moving to next step");
}

void DockingControlNode::RetryCommand()
{
    if (!retry_available_) {
        LOG_INFO("[DCS] RETRY ignored - no retry state available");
        return;
    }

    LOG_INFO("[DCS] RETRY command - Restoring mission and restarting from beginning");

    // Restore mission and sequence
    current_mission_ = retry_mission_;
    current_sequence_ = retry_sequence_;

    // Check if we need to add compensating move command at the beginning
    if (retry_step_index_ < retry_sequence_.steps.size()) {
        const SequenceStep& interrupted_step = retry_sequence_.steps[retry_step_index_];

        if (interrupted_step.action_type == "move") {
            std::string move_type = interrupted_step.params.getString("type", "");

            if (move_type == "docking") {
                // For docking: add goback command at the beginning
                LOG_INFO("[DCS] Adding goback command at the beginning before restarting sequence");

                SequenceStep goback_step;
                goback_step.action_type = "move";
                goback_step.params.setString("type", "goback");
                goback_step.wait_for = "completed";
                goback_step.delay_sec = 0.0;

                // Insert goback step at the beginning (index 0)
                current_sequence_.steps.insert(current_sequence_.steps.begin(), goback_step);
            }
            else if (move_type == "backward") {
                // For backward: add forward command at the beginning
                LOG_INFO("[DCS] Adding forward command at the beginning before restarting sequence");

                SequenceStep forward_step;
                forward_step.action_type = "move";
                forward_step.params.setString("type", "forward");
                forward_step.wait_for = "completed";
                forward_step.delay_sec = 0.0;

                // Insert forward step at the beginning (index 0)
                current_sequence_.steps.insert(current_sequence_.steps.begin(), forward_step);
            }
            else if (move_type == "goback" || move_type == "forward") {
                // Start from the current step instead of the beginning
                LOG_INFO("[DCS] Resuming from step %zu (%s) without compensating move", retry_step_index_ + 1, move_type.c_str());
                // retry_step_index_-1 인 이유는 fork_side 작업도 확인 필요.
                current_sequence_.steps.erase(current_sequence_.steps.begin(), current_sequence_.steps.begin() + retry_step_index_ - 1);
            }
        }
    }

    // Reset flags and start execution from the beginning (step 0)
    mission_received_ = true;
    is_retry_execution_ = true;  // Mark as retry execution to prevent re-saving retry state
    retry_available_ = false;  // Clear retry state after using it

    LOG_INFO("[DCS] Retry started from step %zu/%zu (beginning of sequence)", current_step_index_ + 1, current_sequence_.steps.size());
}

void DockingControlNode::saveRetryState()
{
    // Don't save retry state if we're already executing a retry mission
    if (is_retry_execution_) {
        LOG_INFO("[DCS] Skipping retry state save - already in retry execution");
        return;
    }

    // Check if current step is a move action with type "docking" or "backward"
    if (current_step_index_ < current_sequence_.steps.size()) {
        const SequenceStep& step = current_sequence_.steps[current_step_index_];

        if (step.action_type == "move") {
            std::string move_type = step.params.getString("type", "");

            if (move_type == "docking" || move_type == "backward") {
                // Save retry state
                {
                    std::lock_guard<std::mutex> lock(mission_mutex_);
                    retry_mission_ = current_mission_;
                }
                retry_sequence_ = current_sequence_;
                retry_step_index_ = current_step_index_;
                retry_available_ = true;

                LOG_INFO(
                    "[DCS] Retry state saved: mission=%s, step=%zu/%zu, type=%s", current_mission_.type.c_str(), retry_step_index_ + 1,
                    retry_sequence_.steps.size(), move_type.c_str());
            }
            else if (move_type == "goback" || move_type == "forward") {
                // Save retry state
                {
                    std::lock_guard<std::mutex> lock(mission_mutex_);
                    retry_mission_ = current_mission_;
                }
                retry_sequence_ = current_sequence_;
                retry_step_index_ = current_step_index_;
                retry_available_ = true;

                LOG_INFO(
                    "[DCS] Retry state saved: mission=%s, step=%zu/%zu, type=%s", current_mission_.type.c_str(), retry_step_index_ + 1,
                    retry_sequence_.steps.size(), move_type.c_str());
            }
        }
    }
}

NaviFra::SimplePos DockingControlNode::PosConvert(const nav_msgs::Odometry::ConstPtr& input)
{
    NaviFra::SimplePos o_pos;
    o_pos.SetXm(input->pose.pose.position.x);
    o_pos.SetYm(input->pose.pose.position.y);
    o_pos.SetQuaternion(
        input->pose.pose.orientation.w, input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z);
    return o_pos;
}

void DockingControlNode::PublishExternalPos(const geometry_msgs::PoseStamped& pos)
{
    geometry_msgs::PoseWithCovarianceStamped p;
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();
    p.pose.pose = pos.pose;

    ros_interface_->getExternalPosPub().publish(p);
}

}  // namespace nc_dcs

int main(int argc, char** argv)
{
    ros::init(argc, argv, "docking_control_node");

    // Use AsyncSpinner for multi-threaded callback handling
    ros::AsyncSpinner spinner(2);  // 2 threads for callbacks
    spinner.start();

    nc_dcs::DockingControlNode node;
    node.run();

    spinner.stop();
    return 0;
}