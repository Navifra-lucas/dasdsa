#ifndef EXPLORER_ROS1_HPP_
#define EXPLORER_ROS1_HPP_

#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <filesystem>
#include <thread>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <condition_variable>
#include <mutex>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Polygon.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
#include "answer_msgs/MissionAlign.h"
#include "answer_msgs/MissionNodePath.h"
#include "answer_msgs/MissionExplore.h"
#include "answer_msgs/MissionResponse.h"
#include "answer_msgs/GlobalMap.h"
#include "tf2_ros/transform_broadcaster.h"

#include "logger/logger.h"

#include "common/keys.h"
#include "common/path.h"
#include "common/configurator.h"
#include "common/service_client_handler.h"

#include "utils/data_storage.hpp"
#include "utils/grid_map/grid_map_calculator.hpp"
#include "utils/key_list.hpp"
#include "utils/publish_callback.hpp"
#include "utils/robot_config.hpp"
#include "utils/motion_planner_type_list.hpp"

#include "explorer/explorer.hpp"

#include "interface_common/topic_key.hpp"
#include "interface_common/file_system.hpp"
#include "interface_common/expr_param_manager.hpp"

#include "ros1/navi_common/msg_converter.hpp"
#include "ros1/navi_common/visualizer.hpp"

namespace NVFR {

class ExplorerRos1 {
public:
    ExplorerRos1(const std::string &node_name);
    ~ExplorerRos1();

    void Terminator();
    bool Initialized() const { return b_initialized_; };

private:
    void Initialize();

    bool WaitDataSet();
    bool FirstMovement();

    void LoadParam();

    void Draw(const std::string& s_name, const cv::Mat& cv_image);

    // subscriber
    void GetRobotPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
    void GetNaviMissionResponse(const answer_msgs::MissionResponse::ConstPtr msg);
    void GetNaviStatus(const std_msgs::Int32::ConstPtr msg);
    void GetNaviError(const std_msgs::Int32::ConstPtr msg);
    void GetSlamNodes(const geometry_msgs::Polygon::ConstPtr msg);
    void GetParamReadSignal(const std_msgs::Bool::ConstPtr msg);
    void GetLogLv(const std_msgs::String::ConstPtr msg);

    // publisher
    void SendStatus(const int& n_status);
    void SendAlignMission(const double& d_yaw_rad);
    void SendPathMission(const Pose& o_start_pose, const Pose& o_goal_pose);
    void SendExploreMission(const Pose& o_target_pose);

    // server-response

    // server-request
    void ReqMapImage();

    // ros
    ros::NodeHandle nh_;

    // subscriber
    std::map< KEY::TOPICS, std::shared_ptr<ros::Subscriber> >
        subscriptions_;

    // publisher
    std::map< KEY::TOPICS, std::shared_ptr<ros::Publisher> >
        publishers_;

    // server-server
    std::map< KEY::SERVICES, std::shared_ptr<ros::ServiceClient> >
        srv_servers_;

    // server-client
    std::map< KEY::SERVICES, std::shared_ptr<ros::ServiceClient> >
        srv_clients_;

    // system-variable
    const std::string s_node_name_;
    const std::string s_image_directory_;
    bool b_initialized_;
    // system-object
    Visualizer visualizer_;
    std::shared_ptr<ANSWER::Logger> logger_;

    // explorer
    std::unique_ptr<Explorer> o_explorer_ptr_;

    // service client
    ANSWER::ServiceClientHandler o_service_client_handler_;

};

} // namespace NVFR

#endif
