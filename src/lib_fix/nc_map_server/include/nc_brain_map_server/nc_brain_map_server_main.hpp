#ifndef NC_BRAIN_SERVER_MAIN_HPP_
#define NC_BRAIN_SERVER_MAIN_HPP_

#include "core_msgs/JsonList.h"
#include "core_msgs/MapJson.h"
#include "util/json_stream.hpp"

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>

#include <fstream>

namespace NaviFra {
class BrainMapServer {
public:
    ~BrainMapServer();
    BrainMapServer(ros::NodeHandle& nh, ros::NodeHandle& nhp);

private:
    ros::NodeHandle& nh_;
    ros::NodeHandle& nhp_;

    ros::Publisher json_publisher_;
    ros::Publisher nodes_publisher_;
    ros::Publisher map_db_pub_;
    ros::Publisher cur_map_pub_;
    ros::Publisher map_db_cloud_pub_;

    ros::Subscriber map_request_sub_;

    ros::ServiceClient service_client_;

    std::string mapDir_;
    void LoadMapJson(const std::string& map_json_path, const std::string& cmd);

    core_msgs::JsonList GetNodeLinkData(rapidjson::Document& doc);
    core_msgs::JsonList GetTeachingData(const core_msgs::JsonList& json_list, rapidjson::Document& doc);

    void MapRequestCallback(const std_msgs::String::ConstPtr& msg);
};
};  // namespace NaviFra

#endif