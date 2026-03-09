#ifndef DOCKING_PATH_GENERATOR_ROS1_HPP_
#define DOCKING_PATH_GENERATOR_ROS1_HPP_

#include <iostream>
#include <memory>
#include <chrono>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"

#include "answer_msgs/CommonService.h"


#include "logger/logger.h"
#include "common/keys.h"
#include "common/path.h"
#include "common/configurator.h"

#include "utils/debug_visualizer.hpp"
#include "utils/data_manager.hpp"
#include "utils/publish_callback.hpp"
#include "utils/loop_thread_handler.hpp"
#include "utils/pose.hpp"
#include "interface_common/topic_key.hpp"

#include "ros1/navi_common/msg_converter.hpp"
#include "ros1/navi_common/visualizer.hpp"

#include "ros1/docking_path_generator/dpg_param.hpp"
#include "ros1/docking_path_generator/dpg_param_manager.hpp"
#include "ros1/docking_path_generator/publish_manager.hpp"
#include "ros1/docking_path_generator/docking_info.hpp"
#include "ros1/docking_path_generator/docking_nodes.hpp"
#include "ros1/docking_path_generator/pallet_mission.hpp"
#include "ros1/docking_path_generator/wingbody_mission.hpp"
#include "ros1/docking_path_generator/go_back_mission.hpp"

namespace NVFR {

inline const std::string CMD_DOCKING = "path_plan/cmd";
inline const std::string OBJECT_POSE = "path_plan/pose";
inline const std::string DOCKING_PATH = "path_plan/docking_path";
inline const std::string RETURN_PATH = "path_plan/return_path";


class DockingPathGeneratorRos1
{
public:
    DockingPathGeneratorRos1(const std::string &node_name);
    ~DockingPathGeneratorRos1();

    bool Initialized() { return b_initialized_; };
    void Terminator();

private:
    void Initialize();

    void LoadParam();

    void RegistPublisher();
    void RegistSubscriber();
    void RegistServiceServer();
    void RegistServiceClient();

    // subscriber
    void GetRobotPose(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
    void GetOdom(const nav_msgs::Odometry::ConstPtr msg);
    void GetParamReadSignal(const std_msgs::Bool::ConstPtr msg);
    void GetLogLv(const std_msgs::String::ConstPtr msg);
    void GetObjectPose(const geometry_msgs::PoseStamped::ConstPtr msg);
    void GetCmdDocking(const std_msgs::String::ConstPtr msg);
    void DoResetMission();
    void DoPalletMission(DockingInfo& o_docking_info);
    void DoWingBodyMission(DockingInfo& o_docking_info);
    void DoGoBackMission();

    // publisher
    void SendDockingPath(const Path& path);
    void SendReturnPath(const Path& path);

    // ros
    ros::NodeHandle nh_;

    // subscriber
    std::map<std::string, std::shared_ptr<ros::Subscriber> > subscriptions_;

    // publisher
    std::map<std::string, std::shared_ptr<ros::Publisher> > publishers_;

    // service-server
    std::map<std::string, std::shared_ptr<ros::ServiceServer> > srv_servers_;

    // service-server
    std::map<std::string, std::shared_ptr<ros::ServiceClient> > srv_clients_;

    // system-variable
    const std::string s_node_name_;
    bool b_initialized_;
    // system-object
    Visualizer visualizer_;
    std::shared_ptr<ANSWER::Logger> logger_;

    // obj
    std::shared_ptr<VirtualMission> o_mission_ptr_;
    GoBackMission o_go_back_mission_;

    // mission type
    DataManager<DockingInfo::CMD> n_mission_type_;

    // param
    DataManager<DPG_Param_t> st_dpg_param_;

    // data
    DataManager<Arr3d> o_robot_pose_;
    DataManager<Arr3d> o_robot_vel_;

};

}  // namespace NVFR

#endif
