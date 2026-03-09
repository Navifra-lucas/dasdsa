#include "nc_dcs/ros_interface.h"

namespace nc_dcs {

RosInterface::RosInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
{
}

void RosInterface::setupPublishers()
{
    perception_pub_ = nh_.advertise<std_msgs::String>("/perception/cmd", 10);
    wingbody_perception_pub_ = nh_.advertise<std_msgs::String>("/wingbody/cmd", 10);

    path_plan_cmd_pub_ = nh_.advertise<std_msgs::String>("/path_plan/cmd", 10);
    path_plan_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/path_plan/pose", 10);

    lift_cmd_pub_ = nh_.advertise<core_msgs::WiaForkInfo>("/forkinfo", 10);
    action_reached_pub_ = nh_.advertise<std_msgs::Bool>("/fork_lift_reached", 10);  // task done

    send_live_path_pub_ = nh_.advertise<move_msgs::CoreCommand>("navifra/live_path", 1);

    pallet_detect_trigger_pub_ = nh_.advertise<std_msgs::String>("/pallet_id_read_cmd", 10);

    fork_obs_trigger_pub_ = nh_.advertise<std_msgs::Bool>("/fork_check_trigger", 10);

    intensity_cmd_pub_ = nh_.advertise<std_msgs::String>("/nc_intensity_detector/cmd", 10);
    wingbody_pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/nc_dcs/wingbody_poses", 10);
    wingbody_offset_pub_ = nh_.advertise<geometry_msgs::Pose>("/nc_dcs/wingbody_offset", 10);
    speed_percent_pub_ = nh_.advertise<std_msgs::Float64>("navifra/speed_percent", 10);
    alarm_pub_ = nh_.advertise<std_msgs::Int64>("navifra/error", 10);
    external_pos_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/external/robot_pos", 10);

    // Initialize service client
    path_plan_done_client_ = nh_.serviceClient<std_srvs::Trigger>("/path_plan/done");

    LOG_INFO("[DCS] ROS publishers initialized");
}

void RosInterface::setupSubscribers()
{
    perception_sub_ =
        nh_.subscribe<geometry_msgs::PoseStamped>("/perception/pose", 10, boost::bind(&RosInterface::perceptionCallback, this, _1));
    wingbody_perception_sub_ =
        nh_.subscribe<geometry_msgs::PoseArray>("/wingbody/pose", 10, boost::bind(&RosInterface::wingbodyPerceptionCallback, this, _1));

    path_plan_status_sub_ =
        nh_.subscribe<std_msgs::String>("/path_plan/status", 10, boost::bind(&RosInterface::pathPlanStatusCallback, this, _1));

    lift_status_sub_ = nh_.subscribe<core_msgs::CheonilReadRegister>(
        "/cheonil/read_register", 10, boost::bind(&RosInterface::liftStatusCallback, this, _1));

    action_sub_ =
        nh_.subscribe<core_msgs::ForkLift>("/nc_task_manager/fork_docking", 10, boost::bind(&RosInterface::forkLiftCallback, this, _1));

    task_cmd_sub_ = nh_.subscribe<std_msgs::String>("/nc_task_manager/task_cmd", 10, boost::bind(&RosInterface::taskCmdCallback, this, _1));

    robot_pos_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        "/localization/robot_pos", 10, boost::bind(&RosInterface::robotPosCallback, this, _1));

    live_path_sub_ =
        nh_.subscribe<move_msgs::CoreCommand>("/navifra/live_path", 10, boost::bind(&RosInterface::livePathCallback, this, _1));

    path_plan_sub_ =
        nh_.subscribe<geometry_msgs::PoseArray>("/path_plan/docking_path", 10, boost::bind(&RosInterface::dockingPathPlanCallback, this, _1));

    return_path_sub_ =
        nh_.subscribe<geometry_msgs::PoseArray>("/path_plan/return_path", 10, boost::bind(&RosInterface::returnPathPlanCallback, this, _1));

    navi_alarm_sub_ = nh_.subscribe<core_msgs::NaviAlarm>("/navifra/alarm", 10, boost::bind(&RosInterface::naviAlarmCallback, this, _1));

    fork_obstacle_sub_ = nh_.subscribe<std_msgs::Bool>("/forklift_pause", 10, boost::bind(&RosInterface::forkPauseCallback, this, _1));

    intensity_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/nc_intensity_detector/pose", 10, boost::bind(&RosInterface::intensityPoseCallback, this, _1));

    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 10, boost::bind(&RosInterface::odomCallback, this, _1));

    LOG_INFO("[DCS] ROS subscribers initialized");
}

void RosInterface::setPerceptionCallback(const boost::function<void(const geometry_msgs::PoseStamped::ConstPtr&)>& callback)
{
    perception_callback_ = callback;
}

void RosInterface::setWingbodyPerceptionCallback(const boost::function<void(const geometry_msgs::PoseArray::ConstPtr&)>& callback)
{
    wingbody_perception_callback_ = callback;
}

void RosInterface::setPathPlanStatusCallback(const boost::function<void(const std_msgs::String::ConstPtr&)>& callback)
{
    path_plan_status_callback_ = callback;
}

void RosInterface::setLiftStatusCallback(const boost::function<void(const core_msgs::CheonilReadRegister::ConstPtr&)>& callback)
{
    lift_status_callback_ = callback;
}

void RosInterface::setForkLiftCallback(const boost::function<void(const core_msgs::ForkLift::ConstPtr&)>& callback)
{
    fork_lift_callback_ = callback;
}

void RosInterface::setTaskCmdCallback(const boost::function<void(const std_msgs::String::ConstPtr&)>& callback)
{
    task_cmd_callback_ = callback;
}

void RosInterface::setRobotPosCallback(const boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)>& callback)
{
    robot_pos_callback_ = callback;
}

void RosInterface::setLivePathCallback(const boost::function<void(const move_msgs::CoreCommand::ConstPtr&)>& callback)
{
    live_path_callback_ = callback;
}

void RosInterface::setDockingPathPlanCallback(const boost::function<void(const geometry_msgs::PoseArray::ConstPtr&)>& callback)
{
    path_plan_callback_ = callback;
}

void RosInterface::setReturnPathPlanCallback(const boost::function<void(const geometry_msgs::PoseArray::ConstPtr&)>& callback)
{
    return_path_callback_ = callback;
}

void RosInterface::setNaviAlarmCallback(const boost::function<void(const core_msgs::NaviAlarm::ConstPtr&)>& callback)
{
    navi_alarm_callback_ = callback;
}

void RosInterface::setForkPauseCallback(const boost::function<void(const std_msgs::Bool::ConstPtr&)>& callback)
{
    fork_pause_callback_ = callback;
}

void RosInterface::setIntensityPoseCallback(const boost::function<void(const geometry_msgs::PoseStamped::ConstPtr&)>& callback)
{
    intensity_pose_callback_ = callback;
}

void RosInterface::setOdomCallback(const boost::function<void(const nav_msgs::Odometry::ConstPtr&)>& callback)
{
    odom_callback_ = callback;
}

// Internal callback methods
void RosInterface::perceptionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Forward perception pose to path_plan/pose
    path_plan_pose_pub_.publish(msg);

    if (perception_callback_) {
        perception_callback_(msg);
    }
}

void RosInterface::wingbodyPerceptionCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    if (wingbody_perception_callback_) {
        wingbody_perception_callback_(msg);
    }
}

void RosInterface::pathPlanStatusCallback(const std_msgs::String::ConstPtr& msg)
{
    if (path_plan_status_callback_) {
        path_plan_status_callback_(msg);
    }
}

void RosInterface::forkPauseCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (fork_pause_callback_) {
        fork_pause_callback_(msg);
    }
}

void RosInterface::intensityPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (intensity_pose_callback_) {
        intensity_pose_callback_(msg);
    }
}

void RosInterface::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (odom_callback_) {
        odom_callback_(msg);
    }
}

void RosInterface::liftStatusCallback(const core_msgs::CheonilReadRegister::ConstPtr& msg)
{
    if (lift_status_callback_) {
        lift_status_callback_(msg);
    }
}

void RosInterface::forkLiftCallback(const core_msgs::ForkLift::ConstPtr& msg)
{
    if (fork_lift_callback_) {
        fork_lift_callback_(msg);
    }
}

void RosInterface::taskCmdCallback(const std_msgs::String::ConstPtr& msg)
{
    if (task_cmd_callback_) {
        task_cmd_callback_(msg);
    }
}

void RosInterface::robotPosCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    if (robot_pos_callback_) {
        robot_pos_callback_(msg);
    }
}

void RosInterface::livePathCallback(const move_msgs::CoreCommand::ConstPtr& msg)
{
    if (live_path_callback_) {
        live_path_callback_(msg);
    }
}

void RosInterface::dockingPathPlanCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    if (path_plan_callback_) {
        path_plan_callback_(msg);
    }
}

void RosInterface::returnPathPlanCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    if (return_path_callback_) {
        return_path_callback_(msg);
    }
}

void RosInterface::naviAlarmCallback(const core_msgs::NaviAlarm::ConstPtr& msg)
{
    if (navi_alarm_callback_) {
        navi_alarm_callback_(msg);
    }
}

bool RosInterface::callPathPlanDone()
{
    std_srvs::Trigger srv;

    if (path_plan_done_client_.call(srv)) {
        if (srv.response.success) {
            LOG_INFO("[DCS] Successfully called /path_plan/done service: %s", srv.response.message.c_str());
            return true;
        }
        else {
            LOG_WARNING("[DCS] /path_plan/done service call failed: %s", srv.response.message.c_str());
            return false;
        }
    }
    else {
        LOG_ERROR("[DCS] Failed to call /path_plan/done service");
        return false;
    }
}

}  // namespace nc_dcs
