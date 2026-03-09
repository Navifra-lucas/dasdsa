#ifndef NC_DCS_ROS_INTERFACE_H
#define NC_DCS_ROS_INTERFACE_H

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_srvs/Trigger.h>
// #include <dcs_msgs/DockingInfo.h>
#include "core_msgs/CheonilReadRegister.h"
#include "core_msgs/ForkLift.h"
#include "core_msgs/WiaForkInfo.h"
#include "nc_dcs/mock_msgs.h"
#include "nc_dcs/sequence_loader.h"
#include "std_msgs/Bool.h"
#include "util/logger.hpp"

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <core_msgs/NaviAlarm.h>
#include <move_msgs/CoreCommand.h>

#include <atomic>
#include <mutex>

namespace nc_dcs {

class RosInterface {
public:
    RosInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    void setupPublishers();
    void setupSubscribers();

    // Getters for publishers
    ros::Publisher& getPerceptionPub() { return perception_pub_; }
    ros::Publisher& getWingbodyPerceptionPub() { return wingbody_perception_pub_; }
    ros::Publisher& getPathPlanCmdPub() { return path_plan_cmd_pub_; }
    ros::Publisher& getPathPlanGobackPub() { return path_plan_pose_pub_; }
    ros::Publisher& getPathPlanPosePub() { return path_plan_pose_pub_; }
    ros::Publisher& getLiftCmdPub() { return lift_cmd_pub_; }
    ros::Publisher& getActionReachedPub() { return action_reached_pub_; }
    ros::Publisher& getSendLivePathPub() { return send_live_path_pub_; }
    ros::Publisher& getSendPalletDetectTrigger() { return pallet_detect_trigger_pub_; }
    ros::Publisher& getSentForkObsTrigger() { return fork_obs_trigger_pub_; }
    ros::Publisher& getIntensityCmdPub() { return intensity_cmd_pub_; }
    ros::Publisher& getWingbodyPoseArrayPub() { return wingbody_pose_array_pub_; }
    ros::Publisher& getWingbodyOffsetPub() { return wingbody_offset_pub_; }
    ros::Publisher& getSpeedPercentPub() { return speed_percent_pub_; }
    ros::Publisher& getAlarmPub() { return alarm_pub_; }
    ros::Publisher& getExternalPosPub() { return external_pos_pub_; }

    // Service clients
    bool callPathPlanDone();

    // Callback setters
    void setPerceptionCallback(const boost::function<void(const geometry_msgs::PoseStamped::ConstPtr&)>& callback);
    void setWingbodyPerceptionCallback(const boost::function<void(const geometry_msgs::PoseArray::ConstPtr&)>& callback);
    void setPathPlanStatusCallback(const boost::function<void(const std_msgs::String::ConstPtr&)>& callback);
    void setLiftStatusCallback(const boost::function<void(const core_msgs::CheonilReadRegister::ConstPtr&)>& callback);
    void setForkLiftCallback(const boost::function<void(const core_msgs::ForkLift::ConstPtr&)>& callback);
    void setTaskCmdCallback(const boost::function<void(const std_msgs::String::ConstPtr&)>& callback);
    void setRobotPosCallback(const boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)>& callback);
    void setLivePathCallback(const boost::function<void(const move_msgs::CoreCommand::ConstPtr&)>& callback);
    void setDockingPathPlanCallback(const boost::function<void(const geometry_msgs::PoseArray::ConstPtr&)>& callback);
    void setReturnPathPlanCallback(const boost::function<void(const geometry_msgs::PoseArray::ConstPtr&)>& callback);
    void setNaviAlarmCallback(const boost::function<void(const core_msgs::NaviAlarm::ConstPtr&)>& callback);
    void setForkPauseCallback(const boost::function<void(const std_msgs::Bool::ConstPtr&)>& callback);
    void setIntensityPoseCallback(const boost::function<void(const geometry_msgs::PoseStamped::ConstPtr&)>& callback);
    void setOdomCallback(const boost::function<void(const nav_msgs::Odometry::ConstPtr&)>& callback);

private:
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;

    // Publishers
    ros::Publisher perception_pub_;
    ros::Publisher wingbody_perception_pub_;
    ros::Publisher path_plan_cmd_pub_;
    ros::Publisher path_plan_pose_pub_;
    ros::Publisher lift_cmd_pub_;
    ros::Publisher action_reached_pub_;
    ros::Publisher send_live_path_pub_;
    ros::Publisher pallet_detect_trigger_pub_;
    ros::Publisher fork_obs_trigger_pub_;
    ros::Publisher intensity_cmd_pub_;
    ros::Publisher wingbody_pose_array_pub_;
    ros::Publisher wingbody_offset_pub_;
    ros::Publisher speed_percent_pub_;
    ros::Publisher alarm_pub_;
    ros::Publisher external_pos_pub_;

    // Service clients
    ros::ServiceClient path_plan_done_client_;

    // Subscribers
    ros::Subscriber perception_sub_;
    ros::Subscriber wingbody_perception_sub_;
    ros::Subscriber path_plan_status_sub_;
    ros::Subscriber lift_status_sub_;
    ros::Subscriber action_sub_;
    ros::Subscriber task_cmd_sub_;
    ros::Subscriber robot_pos_sub_;
    ros::Subscriber live_path_sub_;
    ros::Subscriber path_plan_sub_;
    ros::Subscriber return_path_sub_;
    ros::Subscriber navi_alarm_sub_;
    ros::Subscriber fork_obstacle_sub_;
    ros::Subscriber intensity_pose_sub_;
    ros::Subscriber odom_sub_;

    // Callback storage
    boost::function<void(const geometry_msgs::PoseStamped::ConstPtr&)> perception_callback_;
    boost::function<void(const geometry_msgs::PoseArray::ConstPtr&)> wingbody_perception_callback_;
    boost::function<void(const std_msgs::String::ConstPtr&)> path_plan_status_callback_;
    boost::function<void(const core_msgs::CheonilReadRegister::ConstPtr&)> lift_status_callback_;
    boost::function<void(const core_msgs::ForkLift::ConstPtr&)> fork_lift_callback_;
    boost::function<void(const std_msgs::String::ConstPtr&)> task_cmd_callback_;
    boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)> robot_pos_callback_;
    boost::function<void(const move_msgs::CoreCommand::ConstPtr&)> live_path_callback_;
    boost::function<void(const geometry_msgs::PoseArray::ConstPtr&)> path_plan_callback_;
    boost::function<void(const geometry_msgs::PoseArray::ConstPtr&)> return_path_callback_;
    boost::function<void(const core_msgs::NaviAlarm::ConstPtr&)> navi_alarm_callback_;
    boost::function<void(const std_msgs::Bool::ConstPtr&)> fork_pause_callback_;
    boost::function<void(const geometry_msgs::PoseStamped::ConstPtr&)> intensity_pose_callback_;
    boost::function<void(const nav_msgs::Odometry::ConstPtr&)> odom_callback_;

    // Internal callback methods
    void perceptionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void wingbodyPerceptionCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void pathPlanStatusCallback(const std_msgs::String::ConstPtr& msg);
    void liftStatusCallback(const core_msgs::CheonilReadRegister::ConstPtr& msg);
    void forkLiftCallback(const core_msgs::ForkLift::ConstPtr& msg);
    void taskCmdCallback(const std_msgs::String::ConstPtr& msg);
    void robotPosCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void livePathCallback(const move_msgs::CoreCommand::ConstPtr& msg);
    void dockingPathPlanCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void returnPathPlanCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void naviAlarmCallback(const core_msgs::NaviAlarm::ConstPtr& msg);
    void forkPauseCallback(const std_msgs::Bool::ConstPtr& msg);
    void intensityPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

}  // namespace nc_dcs

#endif  // NC_DCS_ROS_INTERFACE_H
