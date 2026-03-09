#ifndef NAVIFRA_RSRSAFER_HPP_
#define NAVIFRA_RSRSAFER_HPP_

#include "pos/pos.hpp"
#include "sensor_msgs/PointCloud.h"
#include "core_calculator/core_calculator.hpp"

#include <asm/types.h>
#include <boost/any.hpp>
#include <float.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <signal.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

#include <bitset>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <list>
#include <map>
#include <mutex>
#include <sstream>
#include <boost/thread.hpp>
#include <core_msgs/NavicoreStatus.h>
#include <core_msgs/VehicleList.h>
#include <core_msgs/Vehicle.h>

using namespace std;
namespace NaviFra {
struct VehicleInfo {
    string id;
    std::chrono::system_clock::time_point update_time;
    float x_m;
    float y_m;
    float angle_deg;
    float f_linear_speed_x_ms;
    float f_linear_speed_y_ms;
    float f_angular_speed_z_degs;
    float f_robot_size_front_m;
    float f_robot_size_rear_m;
    float f_robot_size_left_m;
    float f_robot_size_right_m;
};

class V2V_SAFER {
public:
    V2V_SAFER(ros::NodeHandle& nh, ros::NodeHandle& nhp);
    ~V2V_SAFER();
    void ReceivedV2Vinfo(const core_msgs::VehicleList::ConstPtr& msg);
    void ReceivedRSRinfo(const core_msgs::Vehicle::ConstPtr& msg);

    void ReceivedNaviinfo(const core_msgs::NavicoreStatus::ConstPtr& msg);
    void MakePoint(sensor_msgs::PointCloud& msg_point, const VehicleInfo& o_vehicle, const core_msgs::NavicoreStatus& o_robot);
    sensor_msgs::PointCloud ToGlobal(sensor_msgs::PointCloud& vec_cloud);

    void MainLoop();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    boost::thread th_;
    std::mutex mtx_;
    std::mutex mtx_map_;

    float f_detect_sec_ = 1;
    float f_predict_sec_ = 3;
    float f_detect_range_m_ = 5;
    bool b_thread_run_ = true;
    float f_period_sec_ = 0.1;

    ros::Publisher pub_vehicle_point_;
    ros::Publisher pub_vehicle_point_global_;
    ros::Subscriber sub_v2v_info_;
    ros::Subscriber sub_rsr_info_;
    ros::Subscriber navi_info_;

    std::map<string, VehicleInfo> map_vehicle_;
    core_msgs::VehicleList o_vehicle_msg_;
    core_msgs::NavicoreStatus o_navi_msg_;
};

};  // namespace NaviFra
#endif
