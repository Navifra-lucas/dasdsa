/*
 * 	Copyright(C) 2024 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_NC_SITUATION_H_
#define NAVIFRA_NC_SITUATION_H_

#include "core/util/logger.hpp"
#include "core_msgs/CameraCmd.h"
#include "core_msgs/CameraRoiInfo.h"
#include "core_msgs/Goal.h"
#include "core_msgs/GoalList.h"
#include "core_msgs/HacsNode.h"
#include "core_msgs/HacsNodeList.h"
#include "core_msgs/LidarInfoMsg.h"
#include "core_msgs/LocalizeInfo.h"
#include "core_msgs/MapJson.h"
#include "core_msgs/MotorInfo.h"
#include "core_msgs/NaviAlarm.h"
#include "core_msgs/NaviStatus.h"
#include "core_msgs/NavicoreStatus.h"
#include "core_msgs/PGVPoseList.h"
#include "core_msgs/UpperInfo.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Odometry.h"
#include "nc_rest_api_client.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"

#include <Poco/DeflatingStream.h>
#include <Poco/File.h>
#include <Poco/FileStream.h>
#include <Poco/InflatingStream.h>
#include <Poco/JSON/Array.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <Poco/Net/FilePartSource.h>
#include <Poco/Net/HTMLForm.h>
#include <Poco/Net/HTTPClientSession.h>
#include <Poco/Net/HTTPRequest.h>
#include <Poco/Net/HTTPResponse.h>
#include <Poco/Net/HTTPSession.h>
#include <Poco/Path.h>
#include <Poco/Runnable.h>
#include <Poco/SignalHandler.h>
#include <Poco/StreamCopier.h>
#include <Poco/StreamTokenizer.h>
#include <Poco/StringTokenizer.h>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#define foreach BOOST_FOREACH

using namespace std;

class ncPlayback {
public:
    ncPlayback();
    virtual ~ncPlayback();

private:
    ros::NodeHandle node_handle_;

    ros::Publisher navifrainfo_pub;
    ros::Publisher motorinfo_pub;
    ros::Publisher scan_front_pub;
    ros::Publisher scan_rear_pub;
    ros::Publisher scan_left_pub;
    ros::Publisher scan_right_pub;
    ros::Publisher scan_v2v_pub;
    ros::Publisher scan_camera_pub;
    ros::Publisher global_path_pub;
    ros::Publisher local_path_pub;
    ros::Publisher robot_col_pre_pub;
    ros::Publisher robot_obs_pos_pub;
    ros::Publisher robot_uimsg_pub;
    ros::Publisher robot_col_pub;
    ros::Publisher time_pub;

    ros::Subscriber playback_sub_;
    ros::Subscriber cmdvel_sub_;

    ros::Subscriber navifraalarm_sub_;
    ros::Subscriber navifrainfo_sub_;
    ros::Subscriber motorinfo_sub_;
    ros::Subscriber scan_front_sub_;
    ros::Subscriber scan_rear_sub_;
    ros::Subscriber scan_left_sub_;
    ros::Subscriber scan_right_sub_;
    ros::Subscriber scan_v2v_sub_;
    ros::Subscriber scan_camera_sub_;
    ros::Subscriber global_path_sub_;
    ros::Subscriber local_path_sub_;
    ros::Subscriber robot_col_pre_sub;
    ros::Subscriber robot_obs_pos_sub;
    ros::Subscriber robot_uimsg_sub;
    ros::Subscriber robot_col_sub;
    ros::Subscriber time_sub_;

    std::thread th_play_;
    std::thread th_record_;
    std::thread th_download_;
    bool b_terminate_ = false;
    string s_dir_ = "";
    bool f_use_ = true;

public:
    string s_home = "";
    bool b_record_on_ = false;
    string s_dir_record_ = "";
    std::mutex mtx_record_;
    rosbag::Bag bag_record_;

    std::mutex mtx_play_;
    rosbag::Bag bag_play_;
    rosbag::View* bag_view_ = nullptr;
    rosbag::View::iterator view_;
    ros::Time bag_begin_time_;
    ros::Time bag_end_time_;
    string s_play_cmd_ = "";
    string s_dir_play_ = "";

    int n_lidar_skip_count_ = 2;
    float f_move_sec_ = 5;
    float f_change_sec_ = 2;
    float f_bag_size_ = 5000;  // 5GB
    float f_bag_onefile_size_ = 50;  // 100MB
    float f_speed_ = 1;
    double f_start_time_ = 0;
    float f_err_clear_sec_ = 1;

    bool b_play_open_ = false;
    bool b_play_close_ = false;
    bool b_error_start_ = false;
    string s_status_history_ = "";
    std::chrono::steady_clock::time_point tp_cmd_vel_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_err_start_time_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_err_pub_time_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_start_time_ = std::chrono::steady_clock::now();

    string s_download_date_ = "";
    string s_download_date_full_ = "";
    string s_download_start_ = "";
    string s_download_end_ = "";
    string s_file_ = "";
    string s_uuid_ = "";
    string s_type_ = "";

    NaviBrain::NcRESTAPIClient::Ptr apiClient_;

    int n_global_path_ = 0;
    int n_local_path_ = 0;
    nav_msgs::Path o_global_path_;
    nav_msgs::Path o_local_path_;

private:
    void RecvCmdVel(const geometry_msgs::Twist::ConstPtr& msg);
    void RecvCmd(const std_msgs::String::ConstPtr& msg);
    void RecordLoop();
    void PlayLoop();
    void DownloadLoop();
    void FileCheck();
    string MakeFileName();

    void RecvAlarm(const core_msgs::NaviAlarm::ConstPtr& msg);
    void RecordInfo(const core_msgs::NavicoreStatus::ConstPtr& msg);
    void RecordMotorInfo(const core_msgs::MotorInfo::ConstPtr& msg);
    void RecordFront(const sensor_msgs::PointCloud::ConstPtr& msg);
    void RecordRear(const sensor_msgs::PointCloud::ConstPtr& msg);
    void RecordLeft(const sensor_msgs::PointCloud::ConstPtr& msg);
    void RecordRight(const sensor_msgs::PointCloud::ConstPtr& msg);
    void RecordV2V(const sensor_msgs::PointCloud::ConstPtr& msg);
    void RecordCamera(const sensor_msgs::PointCloud::ConstPtr& msg);
    void RecordGPath(const nav_msgs::Path::ConstPtr& msg);
    void RecordLPath(const nav_msgs::Path::ConstPtr& msg);
    void RecordC(const geometry_msgs::PolygonStamped::ConstPtr& msg);
    void RecordCP(const geometry_msgs::PolygonStamped::ConstPtr& msg);
    void RecordOP(const geometry_msgs::PolygonStamped::ConstPtr& msg);
    void RecordMSG(const std_msgs::String::ConstPtr& msg);

    void RecordTime(const std_msgs::String::ConstPtr& msg);
    vector<string> Split(const string& data, char c_st);
    string MakeDate(const string& num_date);

    void APIConnect();
    std::string getBackendHostFromEnv();
    std::string getBackendPortFromEnv();
    std::string getToken();
    Poco::JSON::Object::Ptr loadJSON(std::string filename);
};

#endif
