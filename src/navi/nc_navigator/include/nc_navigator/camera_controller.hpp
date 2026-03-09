#ifndef CAMERA_CONTROLLER_HPP
#define CAMERA_CONTROLLER_HPP

#include "util/logger.hpp"

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <core/util/logger.hpp>
#include "core_msgs/CameraCmd.h"

#include <thread>
#include <vector>
#include <string>
#include <future>
#include <chrono>
#include <regex>

using namespace std;

enum CameraDirection {
    FRONT,
    REAR,
    LEFT,
    RIGHT
};

class CameraController {
private:
    ros::NodeHandle nh_;

    ros::Subscriber camera_info_sub_;
    ros::ServiceClient set_camera_client_;

    boost::thread camera_controller_thread_;

    vector<bool> vec_change_states_ = {false, false, false, false}; // {front, rear, left, right}

    vector<vector<int>> vec_vec_camera_index_{4}; // front, rear, left, right

    bool b_change_state_ = false;
    bool b_init_state_ = false;
    
public:
    CameraController ();
    virtual ~CameraController ();

    void SelectCamera(const geometry_msgs::Twist msg);

private:
    void SetParameter();
    void RegService();
    void RegSubscriber();
    void CameraInfoCallback(const std_msgs::String::ConstPtr& msg);
    void ControllCamera(vector<bool> vec_change_states);

    bool CallCameraService(int camera_index, bool use_camera, double timeout_sec = 1.0);
    std::vector<bool> ParseCameraStates(const std_msgs::String::ConstPtr& msg);


};

#endif
