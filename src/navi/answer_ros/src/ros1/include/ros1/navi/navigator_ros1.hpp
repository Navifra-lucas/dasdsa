#ifndef NAVIGATOR_ROS1_HPP_
#define NAVIGATOR_ROS1_HPP_

#include "answer_msgs/DriveInfo.h"
#include "answer_msgs/GlobalMap.h"
#include "answer_msgs/MissionAlign.h"
#include "answer_msgs/MissionExplore.h"
#include "answer_msgs/MissionNodePath.h"
#include "answer_msgs/MissionResponse.h"
#include "answer_msgs/NaviInfo.h"
#include "common/keys.h"
#include "common/path.h"
#include "common/configurator.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "interface_common/navi_param_manager.hpp"
#include "interface_common/topic_key.hpp"
#include "logger/logger.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "navigator/navigator.hpp"
#include "ros/ros.h"
#include "ros1/navi_common/msg_converter.hpp"
#include "ros1/navi_common/visualizer.hpp"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "tf2_ros/transform_broadcaster.h"
#include "utils/data_storage.hpp"
#include "utils/grid_map/grid_map_calculator.hpp"
#include "utils/key_list.hpp"
#include "utils/navi_info.hpp"
#include "utils/publish_callback.hpp"
#include "utils/robot_config.hpp"
#include "visualization_msgs/Marker.h"

#include <stdlib.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace NVFR {

class NavigatorRos1
{
public:
    NavigatorRos1(const std::string &node_name);
    ~NavigatorRos1();

    bool Initialized() const { return b_initialized_; };
    void Terminator();

private:
    bool Initialize();

    void LoadParam();

    bool RegistPrimarySub();
    bool RegistSecondarySub();
    bool RegistOtherSub();

    bool WaitPrimaryDataSet();
    bool WaitSecondaryDataSet();

    // subscriber
    void GetRobotPose(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
    void GetOdom(const nav_msgs::Odometry::ConstPtr msg);
    void GetGlobalMap(const nav_msgs::OccupancyGrid &msg);
    // void GetGlobalMap(const nav_msgs::OccupancyGrid::ConstPtr msg);
    void GetLidarScan(const sensor_msgs::LaserScan::ConstPtr msg);
    void GetPointCloud2D(const sensor_msgs::PointCloud::ConstPtr msg);
    void GetCmdSignal(const std_msgs::String::ConstPtr msg);
    void GetParamReadSignal(const std_msgs::Bool::ConstPtr msg);
    void GetLogLv(const std_msgs::String::ConstPtr msg);
    void GetMissionAlign(const answer_msgs::MissionAlign::ConstPtr msg);
    void GetMissionNodePath(const answer_msgs::MissionNodePath::ConstPtr msg);
    void GetMissionExplore(const answer_msgs::MissionExplore::ConstPtr msg);

    // publisher
    void SendCmdVelT(
        const Pose &o_control_input, const DriveInfo &o_drive_info);
    void SendCmdVelTS(
        const Pose &o_control_input, const DriveInfo &o_drive_info);
    void SendMissionResponse(const std::string &s_uuid, const bool &b_received);
    void SendNaviStatus(const int &n_status);
    void SendNaviError(const int &n_error);
    void SendNaviInfo(const NaviInfo &o_navi_info);

    // ros
    ros::NodeHandle nh_;

    // subscriber
    std::map<KEY::TOPICS, std::shared_ptr<ros::Subscriber> > subscriptions_;

    // publisher
    std::map<KEY::TOPICS, std::shared_ptr<ros::Publisher> > publishers_;

    // server-client
    std::map<KEY::SERVICES, std::shared_ptr<ros::ServiceClient> > srv_clients_;

    // system-variable
    const std::string s_node_name_;
    bool b_initialized_;
    // system-object
    Visualizer visualizer_;
    std::shared_ptr<ANSWER::Logger> logger_;

    // navigator
    std::unique_ptr<Navigator> o_navigator_ptr_;

};

}  // namespace NVFR

#endif
