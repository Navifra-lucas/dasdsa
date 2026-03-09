#pragma once

#include "answer/common_service.h"
#include "core_msgs/LocalizeInfo.h"
#include "core_msgs/NaviAlarm.h"
#include "geometry_msgs/PoseArray.h"
#include "ros/master.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <Poco/File.h>
#include <Poco/FileStream.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <Poco/UUID.h>
#include <Poco/UUIDGenerator.h>

#include "Eigen/Core"

struct Point2D {
    Point2D(float x = 0.0f, float y = 0.0f)
        : x(x)
        , y(y)
    {
    }
    float x;
    float y;
};

class AnswerCoreBridge {
private:
    ros::ServiceClient service_client_;
    ros::NodeHandle nh_;
    ros::Subscriber map_request_sub_;
    ros::Subscriber answer_status_sub_;
    ros::Subscriber answer_control_sub_;
    ros::Subscriber answer_report_sub_;
    ros::Subscriber answer_detected_reflectors_sub_;
    ros::Publisher localize_info_pub_;
    ros::Publisher reflectors_pub_;
    std::mutex status_mutex_;
    std::mutex slam_start_mutex_;
    bool b_localizer_ready_;
    bool slam_start_;
    std::string s_localizer_mode_;
    int localize_result_error_code_;
    Poco::JSON::Object reflector_map_json_;
    std::map<std::string, Point2D> reflector_map_;

public:
    AnswerCoreBridge();
    ~AnswerCoreBridge();

    void Initialize();
    void Terminate();
    void MapRequestCallback(const std_msgs::String::ConstPtr& msg);
    void AnswerReportCallback(const std_msgs::String::ConstPtr& msg);
    bool AnswerControlCallback(const std_msgs::String::ConstPtr& msg);
    void AnswerStatusCallback(const std_msgs::String::ConstPtr& msg);
    void AnswerDetectedReflectorsCallback(const std_msgs::String::ConstPtr& msg);
    void StartDefaultMode();
    void ReadReflectorMap();
};