#ifndef NC_TOTAL_TESTER_HPP
#define NC_TOTAL_TESTER_HPP

#include "util/json_stream.hpp"
#include "util/logger.hpp"

#include <Poco/Dynamic/Var.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <core/util/logger.hpp>
#include <core_msgs/CommonString.h>
#include <core_msgs/EtcInfo.h>
#include <core_msgs/JsonList.h>
#include <core_msgs/LidarInfoMsg.h>
#include <core_msgs/LocalizeInfo.h>
#include <core_msgs/MapDB.h>
#include <core_msgs/MapJson.h>
#include <core_msgs/MotionInfo.h>
#include <core_msgs/MotorInfo.h>
#include <core_msgs/NavicoreStatus.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <jsoncpp/json/json.h>
#include <rapidjson/document.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

class TotalTester {
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_;

    ros::Publisher json_publisher_;
    ros::Publisher nodes_publisher_;
    ros::Publisher map_db_pub_;
    ros::Publisher cur_map_pub_;
    ros::Publisher map_db_cloud_pub_;
    ros::Publisher param_pub_;
    ros::Publisher nav_cmd_pub_;
    ros::Publisher task_cmd_pub_;

    ros::Subscriber pc_log_sub_;
    std::string package_path_;

private:
    bool loadScenarios(const std::string& filepath, Json::Value& scenarios);
    void executeScenario(const Json::Value& scenario);
    void setInitialPose(const Json::Value& pose);
    void executeTask(const Json::Value& task);
    bool monitorDuringRun(const Json::Value& scenario);
    bool monitorAndEvaluate(const Json::Value& expected);
    void LoadMapJson();
    void StratTest();
    core_msgs::JsonList GetNodeLinkData(rapidjson::Document& doc);
    void RecvLog(const std_msgs::String::ConstPtr& msg);
    std::string getCurrentTime();
    bool b_stop_test_ = false;
    bool b_terminate_ = false;
    int n_total_scenario_count_ = 0;
    std::vector<std::string> vec_failed_scenario_name_;
    int n_kinematics_ = 0;
    int n_now_loop_ = 0;
    std::string s_kinematics_type_ = "DD";
    boost::thread th1_;
public:
    TotalTester();
    virtual ~TotalTester();
};
;

#endif
