#ifndef GRAPH_MAP_ROS1_HPP_
#define GRAPH_MAP_ROS1_HPP_

#include <iostream>
#include <memory>
#include <chrono>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "answer_msgs/MissionResponse.h"
#include "answer_msgs/EdgeInfo.h"
#include "answer_msgs/MissionNodePath.h"
#include "answer_msgs/CommonService.h"
#include "answer_msgs/GraphLocalization.h"
#include "answer_msgs/MissionGraphNode.h"
#include "answer_msgs/MissionGraphNodeList.h"

#include "logger/logger.h"
#include "common/keys.h"
#include "common/path.h"
#include "common/configurator.h"

#include "interface_common/topic_key.hpp"
#include "graph_map/graph_map.hpp"

#include "ros1/navi_common/msg_converter.hpp"

namespace NVFR {

class GraphMapRos1
{
public:
    GraphMapRos1(const std::string &node_name);
    ~GraphMapRos1();

    bool Initialized() { return b_initialized_; };
    void Terminator();

private:
    void Initialize();

    void LoadParam();

    void RegistPublisher();
    void RegistSubscriber();
    void RegistServiceServer();

    // subscriber
    void GetRobotPose(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
    void GetParamReadSignal(const std_msgs::Bool::ConstPtr msg);
    void GetLogLv(const std_msgs::String::ConstPtr msg);

    // service-server
    bool ResponseLoadMap(
        answer_msgs::CommonService::Request  &req,
        answer_msgs::CommonService::Response &res);
    bool ResponseLocalization(
        answer_msgs::GraphLocalization::Request  &req,
        answer_msgs::GraphLocalization::Response &res);
    bool ResponseMissionGraphNode(
        answer_msgs::MissionGraphNode::Request  &req,
        answer_msgs::MissionGraphNode::Response &res);
    bool ResponseMissionGraphNodeList(
        answer_msgs::MissionGraphNodeList::Request  &req,
        answer_msgs::MissionGraphNodeList::Response &res);

    // service-client
    bool RequestMapDirectory(std::string& s_map_file);

    // ros
    ros::NodeHandle nh_;

    // subscriber
    std::map<KEY::TOPICS, std::shared_ptr<ros::Subscriber> > subscriptions_;

    // publisher
    std::map<KEY::TOPICS, std::shared_ptr<ros::Publisher> > publishers_;

    // service-server
    std::map<KEY::SERVICES, std::shared_ptr<ros::ServiceServer> > srv_servers_;

    // system-variable
    const std::string s_node_name_;
    bool b_initialized_;
    // system-object
    std::shared_ptr<ANSWER::Logger> logger_;

    // graph map
    std::unique_ptr<GraphMap> o_graph_map_ptr_;

};

}  // namespace NVFR

#endif
