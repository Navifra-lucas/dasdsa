
/*
 * @file	: lidar_merger.hpp
 * @date	: Feb. 18, 2022
 * @brief	: lidar_merger.cpp
 * @remark	:
 * @warning	:
 */

#ifndef NAVIFRA_NC_LIDAR_MERGER__H
    #define NAVIFRA_NC_LIDAR_MERGER_H
    #include "debug/debug_visualizer.hpp"
    #include "geometry_msgs/PoseWithCovarianceStamped.h"
    #include "geometry_msgs/TransformStamped.h"
    #include "icp/point_to_plane_icp.hpp"
    #include "msg_information/sensor_information/scan.hpp"
    #include "nav_msgs/Odometry.h"
    #include "opencv2/opencv.hpp"
    #include "pcl_ros/point_cloud.h"
    #include "ros/ros.h"
    #include "sensor_msgs/ChannelFloat32.h"
    #include "sensor_msgs/LaserScan.h"
    #include "sensor_msgs/PointCloud.h"
    #include "tf/transform_convertor.hpp"
    #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
    #include "tf2_ros/buffer.h"
    #include "tf2_ros/transform_listener.h"

    #include <core_msgs/CommonString.h>
    #include <core_msgs/LidarInfoMsg.h>
    #include <core_msgs/NaviAlarm.h>
    #include <laser_geometry/laser_geometry.h>
    #include <message_filters/subscriber.h>
    #include <message_filters/sync_policies/approximate_time.h>
    #include <message_filters/synchronizer.h>
    #include <pcl/io/pcd_io.h>
    #include <pcl/point_cloud.h>
    #include <pcl/point_types.h>
    #include <pcl_ros/transforms.h>
    #include <sensor_msgs/PointCloud2.h>
    #include <sensor_msgs/point_cloud_conversion.h>
    #include <std_msgs/Bool.h>
    #include <std_msgs/String.h>
    #include <std_msgs/Int16.h>
    #include <tf/transform_listener.h>
    #include "util/license_check.hpp"

    #include <chrono>
    #include <cmath>
    #include <fstream>
    #include <random>
    #include <string>
// #define CAL_LIDAR_OFFSET

namespace NaviFra {
struct LidarData {
    sensor_msgs::LaserScan scan;
    NaviFra::Pos o_pos;
};

struct Lidar_Info_t {
    bool b_lidar_use = false;
    string s_lidar_frame_id = "";
    int n_lidar_error_code = 0;
    string s_lidar_error_text = "";
    int n_lidar_point_size = 0;
    std::chrono::steady_clock::time_point tp_lidar_timestamp;
};

struct Obstacle {
    NaviFra::Pos o_pos;
    float f_size = 0;
};

struct Interpolation {
    float f_correct_x = 0;
    float f_correct_y = 0;
    float f_theta_deg_sec = 0;
    float f_sin = 0;
    float f_cos = 1;
};

struct LidarOffset {
    float front_x_m_ = 0;
    float front_y_m_ = 0;
    float front_z_m_ = 0;
    float front_roll_deg_ = 0;
    float front_pitch_deg_ = 0;
    float front_yaw_deg_ = 0;
    float front_angle_increment_ = 0;
    float front_angle_min_ = 0;
    float front_range_offset_m_ = 0;
    float rear_x_m_ = 0;
    float rear_y_m_ = 0;
    float rear_z_m_ = 0;
    float rear_roll_deg_ = 0;
    float rear_pitch_deg_ = 0;
    float rear_yaw_deg_ = 0;
    float rear_angle_increment_ = 0;
    float rear_angle_min_ = 0;
    float rear_range_offset_m_ = 0;
    float left_x_m_ = 0;
    float left_y_m_ = 0;
    float left_z_m_ = 0;
    float left_roll_deg_ = 0;
    float left_pitch_deg_ = 0;
    float left_yaw_deg_ = 0;
    float left_angle_increment_ = 0;
    float left_angle_min_ = 0;
    float left_range_offset_m_ = 0;
    float right_x_m_ = 0;
    float right_y_m_ = 0;
    float right_z_m_ = 0;
    float right_roll_deg_ = 0;
    float right_pitch_deg_ = 0;
    float right_yaw_deg_ = 0;
    float right_angle_increment_ = 0;
    float right_angle_min_ = 0;
    float right_range_offset_m_ = 0;
};

class LidarMergerFilter {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;  //("~");
    ros::Subscriber sub_speed_;
    ros::Subscriber sub_pose_;

    ros::Subscriber sub_scan1_;
    ros::Subscriber sub_scan2_;
    ros::Subscriber sub_offset_;
    ros::Subscriber sub_obstacle_;
    ros::Subscriber sub_offset_front_;
    ros::Subscriber sub_param_;
    ros::Subscriber sub_loaded_;
    ros::Subscriber sub_fork_position_;
    ros::Publisher pub_scan_;
    ros::Publisher pub_merge_scan_;
    ros::Publisher pub_lidar_info_;

    ros::Publisher pub_scan1_;
    ros::Publisher pub_scan2_;
    ros::Publisher pub_scan3_;
    ros::Publisher pub_scan4_;

    ros::Publisher pub_scan1_global_;
    ros::Publisher pub_scan2_global_;
    ros::Publisher pub_scan3_global_;
    ros::Publisher pub_scan4_global_;

    ros::ServiceServer quality_service_;

    Pos o_pf_pos_;

    LidarOffset st_offset_;

    string s_target_scan_topic_;
    string s_target_frame_id;
    vector<string> vec_input_topics;

    std::vector<Lidar_Info_t> vec_lidar_info_;
    std::vector<std::vector<std::chrono::steady_clock::time_point>> vec_tp_lidar_timestamp_;
    std::chrono::steady_clock::time_point checktime_odom_ = std::chrono::steady_clock::now();

    std::string target_frame_front_id_;
    std::string header_frame_front_id_;
    std::string target_frame_rear_id_;
    std::string header_frame_rear_id_;
    vector<std::string> vec_target_frame_id_;

    bool b_full_lidar_ = false;
    bool b_lidar_direction_;
    float f_angle_min_;
    float f_angle_max_;
    float f_angle_increment_;
    float f_scan_time_;
    float f_range_min_;
    float f_range_max_;

    vector<float> vec_f_increment_;
    vector<float> vec_f_angle_min_;
    vector<float> vec_range_offset_m_;
    vector<vector<float>> vec_f_dist_;

    float f_front_range_min_;
    float f_rear_range_min_;
    double d_scan_front_time_diff_;
    double d_scan_rear_time_diff_;

    bool b_offset_init_ = true;
    bool b_loaded_ = false;
    bool b_load_diff_range_use_;

    float f_x_front_m_;
    float f_y_left_m_;
    float f_x_back_m_;
    float f_y_right_m_;
    float f_x_load_front_m_;
    float f_y_load_left_m_;
    float f_x_load_back_m_;
    float f_y_load_right_m_;

    float f_lidar_merge_range_diff_;

    float f_dist_min_m_;
    float f_dist_max_m_;
    float f_max_msec_;
    float f_time_bound_;

    int n_max_lidar_num_;
    int n_lidar_temp_;
    float f_lidar_timeout_threshold_;

    double d_odometry_time_;
    bool b_param_init_ = false;
    bool b_lidar_update_ = false;

    int n_full_lidar_m_ = 2.5;
    int n_skip_lidar_std_ = 10;
    int n_fork_position_ = 0;

    NaviFra::Scan_t st_laser_scan1_;
    NaviFra::Scan_t st_laser_scan2_;

    tf::StampedTransform req_to_front_trans_;
    tf::StampedTransform req_to_rear_trans_;

    sensor_msgs::LaserScan o_laser_scan1_;
    sensor_msgs::LaserScan o_laser_scan2_;

    std::mutex mtx_lock_;
    std::mutex mtx_lock_correct_;
    std::mutex mtx_lock_pos_;

    std::thread th_lidar_info_pub_;

    tf::TransformListener tfListener1_;
    tf::TransformListener tfListener2_;
    std::vector<std::shared_ptr<tf::TransformListener>> vec_tf_listener_;
    laser_geometry::LaserProjection projector_;

    std::vector<tf::StampedTransform> vec_tf_stamped_transform_;

    NaviFra::Pos o_pos_;
    NaviFra::Pos o_correct_info_;
    nav_msgs::Odometry nav_odom_;
    Lidar_Info_t o_lidar_info_;
    Obstacle o_obs_info_;

    bool b_cal_on_ = false;

    ofstream merge_gap;
    ofstream front_gap;
    ofstream rear_gap;

    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan>>*
        synchronizer2_ = nullptr;

    message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan>>*
        synchronizer3_ = nullptr;

    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan>>* synchronizer4_ = nullptr;

    std::vector<message_filters::Subscriber<sensor_msgs::LaserScan>*> message_filter_subscribers_;
    std::vector<bool> vec_check_transform_;

    void messageFilterCallback(const sensor_msgs::LaserScan::ConstPtr& scan1);

    void messageFilterCallback(const sensor_msgs::LaserScan::ConstPtr& scan1, const sensor_msgs::LaserScan::ConstPtr& scan2);

    void messageFilterCallback(
        const sensor_msgs::LaserScan::ConstPtr& scan1, const sensor_msgs::LaserScan::ConstPtr& scan2,
        const sensor_msgs::LaserScan::ConstPtr& scan3);

    void messageFilterCallback(
        const sensor_msgs::LaserScan::ConstPtr& scan1, const sensor_msgs::LaserScan::ConstPtr& scan2,
        const sensor_msgs::LaserScan::ConstPtr& scan3, const sensor_msgs::LaserScan::ConstPtr& scan4);

    bool LidarMerge(const std::vector<sensor_msgs::LaserScan::ConstPtr>& current_scans);

    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;

    void PosCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void AppendUIData(float f_dist, sensor_msgs::PointCloud& vec_cloud, geometry_msgs::Point32 point);
    bool CQcallback(core_msgs::CommonString::Request& request, core_msgs::CommonString::Response& response);

public:
    LidarMergerFilter(ros::NodeHandle& nh, ros::NodeHandle& nhp);
    virtual ~LidarMergerFilter();

private:
    sensor_msgs::PointCloud ToGlobal(sensor_msgs::PointCloud& vec_cloud);

    void SpeedCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void TransformListener(tf::StampedTransform& req_to_trans, float fx, float fy, float fz, float fRx, float fRy, float fRz);
    void VirtualObsCallback(const std_msgs::String::ConstPtr& msg);

    void UpdateOffset(const std::vector<sensor_msgs::LaserScan::ConstPtr>& current_scans);
    void LoadedCallback(const std_msgs::Bool::ConstPtr& msg);
    void ForkPositionCallback(const std_msgs::Int16::ConstPtr& msg);

    void SetTransformListener(tf::StampedTransform& req_to_trans, float fx, float fy, float fz, float fRx, float fRy, float fRz);
    void ParamUpdateCallback(const std_msgs::String::ConstPtr& msg);
    void Initialize();
    void MergeTransform(const sensor_msgs::PointCloud& scan_cloud);
    bool CheckData(const float& x, const float& y, const float& f_dist);
    void DefineCallbackFunction(const vector<string>& vec_input_topics);
    void MeasureTimeGap(std::vector<sensor_msgs::LaserScan::ConstPtr>& current_scan);
    float GetHz(std::vector<std::chrono::steady_clock::time_point>& vec_time);
    vector<Interpolation> CalcInterPolation(const std::vector<sensor_msgs::LaserScan::ConstPtr>& current_scan);
    geometry_msgs::Point32 CalcIPpoint(const geometry_msgs::Point32& point, const Interpolation& o_IP);
    NaviFra::Pos PosConvert(const geometry_msgs::PoseStamped& ros_pos);

    Pos TransformRotationRad_(const Pos& o_pos, float f_rad);
    Pos TransformRotationDeg_(const Pos& o_pos, float f_deg);
};

}  // namespace NaviFra
#endif
