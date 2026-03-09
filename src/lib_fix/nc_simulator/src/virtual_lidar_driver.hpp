
/*
 * @file	: virtual_lidar_driver.hpp
 * @date	: Aug. 13, 2020
 * @brief	: 가상의 라이다 드라이버
 * @remark	:
 * @warning	:
 */

#ifndef VIRTUAL_LIDAR_DRIVER_H
#define VIRTUAL_LIDAR_DRIVER_H

#include "core_msgs/MapJson.h"
#include "debug/debug_visualizer.hpp"
#include "distance_ray_calculator.hpp"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "msg_information/sensor_information/scan.hpp"
#include "nav_msgs/Odometry.h"
#include "pos/pos.hpp"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <chrono>
#define RADtoDEG (57.29577951f)  ///< 180.0/PI
#define DEGtoRAD (0.017453292f)  ///< PI/180.0
namespace NaviFra {
struct ObsInfo {
    bool b_active = false;
    float f_pos_x = 0;
    float f_pos_y = 0;
    float f_height = 0;
    float f_width = 0;
    float f_angle_deg = 0;
};

class VirtualLidarDriver {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;  //("~");

    ros::Publisher pub_scan_;
    ros::Subscriber ros_robot_pos_sub_;
    ros::Subscriber map_editor_node_info_sub_;
    ros::Subscriber map_db_sub_;
    ros::Subscriber initial_pose_sub_;
    ros::Subscriber initial_pose_sub2_;
    ros::Subscriber obstacle_pose_sub_;
    ros::Subscriber sub_param_;

    Pos o_robot_pos_;
    Pos o_prev_odom_pos_;
    Pos o_odom_pos_;
    Pos o_initial_pos_;

    bool b_map_loaded_;
    NaviFra::Scan_t st_scan_;
    int scan_size_;
    std::string header_frame_;
    sensor_msgs::LaserScan laser_scan_;
    geometry_msgs::TransformStamped tf_geom_;
    ros::Timer laser_timer_;

    std::chrono::steady_clock::time_point odom_time_;
    std::mutex mtx_lock_;
    std::mutex mtx_map_lock_;
    ObsInfo o_obs_info_;
    DistanchRayCalculator o_raymarch_;

    float f_map_size_x_ = 0;
    float f_map_size_y_ = 0;
    float f_map_origin_x_ = 0;
    float f_map_origin_y_ = 0;

    float f_map_min_x_ = 0;
    float f_map_min_y_ = 0;
    float f_map_max_x_ = 0;
    float f_map_max_y_ = 0;
    float f_map_resolution_ = 0.01;
    bool b_use_localization_ = true;

    std::vector<NaviFra::Pos> vec_pos_history_;

public:
    VirtualLidarDriver(ros::NodeHandle* nh);
    virtual ~VirtualLidarDriver();

    void readLastPos();
    void RecvInitialPos(const geometry_msgs::PoseWithCovarianceStamped msg);
    void RecvInitialPos_sim(const geometry_msgs::PoseWithCovarianceStamped msg);
    int ToSign_(float value) { return (value > 0) - (value < 0); }

    void RecvMapeditorNodeWithGrid(const core_msgs::MapJson& msg);
    void RecvRobotoPos(const nav_msgs::Odometry::ConstPtr& msg);
    void RecvObsPos(const std_msgs::String msg);
    void RecvDB(const core_msgs::MapDB::ConstPtr& msg);
    void ParamUpdateCallback(const std_msgs::String::ConstPtr& msg);

    void resetDefault();
    NaviFra::Pos ConvertToGlobalPosition_(const NaviFra::Pos& o_local_pos, const NaviFra::Pos& o_origin_pos);
    NaviFra::Pos PosConvert(const nav_msgs::Odometry::ConstPtr& ros_pos);
    NaviFra::Pos PosConvert(const geometry_msgs::PoseStamped& ros_pos);
    float WrapAnglePiToPiDeg_(float f_angle_deg);

private:
    /**
     * @brief 실제 라이다 데이터를 로봇좌표와, 지도정보를 생성
     *
     * @param o_robot_pos
     */
    void Read(const ros::TimerEvent& event);
};

}  // namespace NaviFra
#endif
