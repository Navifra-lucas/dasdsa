#ifndef LIDAR_STATIC_CHECK_NODE_HPP
#define LIDAR_STATIC_CHECK_NODE_HPP

#include "nc_lidar_static_check/lidar_static_check.hpp"

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <core_msgs/CheonilReadRegister.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <chrono>

class LidarStaticCheckNode {
public:
    LidarStaticCheckNode(ros::NodeHandle& nh, ros::NodeHandle& nhp);
    virtual ~LidarStaticCheckNode();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    bool LoadPolygonFromParam(ros::NodeHandle& nh, const std::string& param_name, NaviFra::Polygon& polygon);
    void ScanCallback(const sensor_msgs::PointCloud::ConstPtr& msg);

    void TimerCallback(const ros::TimerEvent&);

    void ParamUpdateCallback(const std_msgs::String::ConstPtr& msg);
    void ForkTriggerCallback(const std_msgs::Bool& msg);
    void CheonilCallback(const core_msgs::CheonilReadRegister::ConstPtr& msg);

    void Initialize();
    sensor_msgs::PointCloud::Ptr FilterLidarClusters(const sensor_msgs::PointCloud::ConstPtr& cloud_msg);

    NaviFra::LidarStaticCheck lidar_check_;
    ros::Subscriber scan_sub_;
    ros::Subscriber param_update_sub_;
    ros::Subscriber check_trigger_sub_;
    ros::Subscriber cheonil_sub_;
    ros::Publisher pub_status_;
    ros::Publisher pub_bool_status_;
    ros::Publisher pub_poly1_;
    ros::Publisher pub_poly2_;
    ros::Publisher pub_lift_state_;
    ros::Timer timer_;

    float f_lidar_timeout_sec_ = 0.5;
    double d_obstacle_hold_sec_;
    int n_fork_lift_position_;
    int n_consecutive_hits_;
    bool b_running_ = false;
    bool b_final_state_;
    bool prev_final_state_ = false;

    sensor_msgs::PointCloud::ConstPtr latest_cloud_;
    ros::Time last_cloud_time_;
    ros::Time final_state_changed_time_;

    std::mutex mutex_;

    // Clustering parameters
    int n_cluster_size_max_;
    float f_cluster_obs_check_dist_;
    float f_cluster_step_dist_;
    std::vector<float> vec_outline_dist_;
    std::vector<float> vec_max_cluster_distance_m_;
    std::vector<float> vec_cluster_size_min_;
};

#endif
