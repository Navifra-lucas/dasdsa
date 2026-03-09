#ifndef NC_DCS_DOCKING_CONTROL_NODE_H
#define NC_DCS_DOCKING_CONTROL_NODE_H

#include "core_msgs/CheonilReadRegister.h"
#include "core_msgs/ForkLift.h"
#include "core_msgs/NaviAlarm.h"
#include "core_msgs/WiaForkInfo.h"
#include "nc_dcs/action_executor.h"
#include "nc_dcs/fork_parameters_loader.h"
#include "nc_dcs/mock_msgs.h"
#include "nc_dcs/ros_interface.h"
#include "nc_dcs/sequence_loader.h"
#include "nc_dcs/state_handler.h"
#include "pos/pos.hpp"
#include "util/logger.hpp"

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <mutex>

namespace nc_dcs {

enum class State
{
    IDLE,
    EXECUTING_SEQUENCE,
    PAUSED,
    COMPLETE,
    STOPPED
};

class DockingControlNode {
public:
    DockingControlNode();
    void run();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Rate loop_rate_;
    State current_state_;

    ros::Publisher lift_cmd_pub_;

    // TF2 for coordinate transformation
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Module instances
    std::unique_ptr<RosInterface> ros_interface_;
    std::unique_ptr<ActionExecutor> action_executor_;
    std::unique_ptr<StateHandler> state_handler_;

    // Configuration loaders
    SequenceLoader sequence_loader_;
    ForkParametersLoader fork_params_loader_;
    Sequence current_sequence_;
    size_t current_step_index_;
    size_t pause_step_index_;

    // Data
    geometry_msgs::PoseStamped target_pose_;
    geometry_msgs::PoseStamped goal_pose_;
    std::mutex target_pose_mutex_;  // Mutex for target_pose_
    geometry_msgs::PoseStamped robot_pose_;
    std::mutex robot_pose_mutex_;  // Mutex for robot_pose_
    NaviFra::SimplePos o_prev_odom_;
    geometry_msgs::PoseStamped sub_robot_pos_;
    geometry_msgs::Pose last_path_point_;  // Store last point from navifra/live_path
    std::mutex last_path_point_mutex_;  // Mutex for last_path_point_
    Mission current_mission_;
    std::mutex mission_mutex_;  // Mutex for current_mission_
    core_msgs::CheonilReadRegister lift_status_;  // Store latest lift status
    std::mutex lift_status_mutex_;  // Mutex for lift_status_
    std::mutex mtx_lock_odom_;
    std::atomic<bool> perception_received_;
    std::atomic<bool> wingbody_perception_received_;
    std::atomic<bool> intensity_received_;
    std::atomic<bool> docking_done_;
    std::atomic<bool> goback_done_;
    std::atomic<bool> backward_done_;
    std::atomic<bool> forward_done_;
    std::atomic<bool> align_done_;
    std::atomic<bool> lift_done_;
    std::atomic<bool> mission_received_;
    std::atomic<bool> path_plan_cmd_sent_;  // Track if /path_plan/cmd was sent
    std::atomic<bool> b_docking_path_received_ = false;
    bool b_updated_perception_ = false;
    bool b_updated_intensity_ = false;
    bool b_updated_move_height_ = false;
    bool b_last_intensity_update_ = false;
    bool step_initialized_ = false;
    bool b_fork_lift_completed_ = true;
    bool b_lift_adjusted_once_ = false;  // Flag for "once" lift adjustment mode
    int n_once_mode_prev_height_ = 0;  // Previous height for "once" mode feedback check
    ros::Time once_mode_height_check_time_;  // Time for height change check in "once" mode
    bool b_once_mode_lift_30mm_commanded_ = false;  // Flag for 30mm lift after fork stopped in LOADING once mode
    int n_once_mode_lift_30mm_target_ = 0;  // Target height for 30mm lift in LOADING once mode
    bool b_odom_use_ = false;
    bool b_odom_start_ = false;
    int delay_counter_;
    int intensity_samples_count_ = 0;
    double intensity_height_sum_ = 0.0;
    const int TARGET_INTENSITY_SAMPLES = 5;

    // Perception Averaging
    ros::Time perception_window_start_;
    ros::Time intensity_window_start_;
    ros::Time intensity_start_time_ = ros::Time::now();  // Reset intensity delay timer;  // Time when intensity step started (for 1-sec delay)
    ros::Time mission_start_;
    ros::Time lift_action_start_;
    ros::Time lift_move_time_;
    ros::Time step_start_time_;
    ros::Time perception_recv_time_;
    int perception_update_count_;
    int n_perception_999_cnt_;
    int intensity_update_count_;
    int n_lift_up_down_retry_count_;
    const int MAX_PERCEPTION_UPDATES = 100;
    const int MAX_INTENSITY_UPDATES = 10;
    const int MAX_LIFT_UP_DOWN_RETRY = 3;

    const double PERCEPTION_WINDOW_SEC = 0.5;
    const double INTENSITY_WINDOW_SEC = 2.0;

    // Retry mechanism
    Mission retry_mission_;
    Sequence retry_sequence_;
    size_t retry_step_index_;
    bool retry_available_;
    bool is_retry_execution_;  // Flag to prevent saving retry state during retry execution

    // Sensor-to-robot transformation parameters

    //     // t-mini(상)
    double robot2sensor_x_ = -0.363;
    double robot2sensor_y_ = 0.065;
    double robot2sensor_deg_ = 180.0;

    // // t-mini(하)
    // double robot2sensor_x_ = -0.343;
    // double robot2sensor_y_ = 0.19;
    // double robot2sensor_deg_ = 180;

    float wingbody_deg_ = 0.0;

    // Callbacks
    void TaskCmdCallback(const std_msgs::String::ConstPtr& msg);
    void perceptionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void wingbodyPerceptionCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void pathPlanStatusCallback(const std_msgs::String::ConstPtr& msg);
    void liftStatusCallback(const core_msgs::CheonilReadRegister::ConstPtr& msg);
    void forkLiftCallback(const core_msgs::ForkLift::ConstPtr& msg);
    void dockingPathPlanCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void returnPathPlanCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void naviAlarmCallback(const core_msgs::NaviAlarm::ConstPtr& msg);
    void forkPauseCallback(const std_msgs::Bool::ConstPtr& msg);
    void intensityPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void robotPosCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    // Helpers
    void resetFlags();

    // State Handlers
    void handleIdle();
    void handleExecutingSequence();
    void handlePaused();
    void handleComplete();
    void handleStopped();
    bool errorCheck(SequenceStep& step);

    bool perceptionErrorCheck(SequenceStep& step);
    bool forkErrorCheck(SequenceStep& step);

    // Refactoring Helpers
    bool processPerceptionParams(SequenceStep& step);
    nlohmann::json calculateTargetCmd(const SequenceStep& step);
    void updateLiftControl(const SequenceStep& step);
    float calculateTiltValue();  // Returns desired tilt value: 0=down, 1=up, -1=no change
    void finalizeStep(const SequenceStep& step);

    geometry_msgs::PoseStamped transformSensorToRobot(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool transformRobotToMap(const geometry_msgs::PoseStamped& robot_frame_pose, geometry_msgs::PoseStamped& map_frame_pose);
    void saveRetryState();
    void RetryCommand();
    NaviFra::SimplePos PosConvert(const nav_msgs::Odometry::ConstPtr& input);
    void PublishExternalPos(const geometry_msgs::PoseStamped& pos);
};

}  // namespace nc_dcs

#endif  // NC_DCS_DOCKING_CONTROL_NODE_H