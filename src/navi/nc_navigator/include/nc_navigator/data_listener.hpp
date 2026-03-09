/*
 * @file	: data_listener.hpp
 * @date	: June 9, "Joongtae, Park(brain)" (founder@navifra.com)"
 * @author	:"Joongtae, Park(brain)" (founder@navifra.com)"
 * @brief	:  navigtaion_node로 들어오는 센서데이터를 취합
 * @remark	:
 * @warning	:
 * 	Copyright(C) "Joongtae, Park(brain)" (founder@navifra.com)" NAVIFRA.
 * 	All Rights are Reserved.
 */

#ifndef NAVIFRA_SENSOR_SUPERVISOR_H_
#define NAVIFRA_SENSOR_SUPERVISOR_H_

class DataListener {
public:
    DataListener();
    virtual ~DataListener();

    bool Notify(const std::string& str_cbf_name, const boost::any& any_type_var);

    bool RegisteCallbackFunc(const std::string& str_cbf_name, const std::function<void(const boost::any&)>& pt_func_);

private:
    ros::NodeHandle node_handle_;

    ros::Publisher local_tf_all_data_pub_;

    ros::Subscriber cloud_sub_;
    ros::Subscriber cloud2_sub_;
    ros::Subscriber cloud3_sub_;
    ros::Subscriber robot_speed_sub_;
    ros::Subscriber robot_loaded_sub_;
    ros::Subscriber obstacle_sub_;
    ros::Subscriber v2v_sub_;

    geometry_msgs::TransformStamped tf_geom_;
    NaviFra::TransformConvertor o_transformer_;
    NaviFra::SensorMsg_t st_sensor_msg_;
    std::vector<NaviFra::SimplePos> vec_vision_obs_;
    std::vector<NaviFra::SimplePos> vec_v2v_obs_;
    std::vector<NaviFra::SimplePos> vec_cloud2_obs_;
    std::vector<NaviFra::SimplePos> vec_cloud_vanjee_obs_;

    std::mutex mtx_lock_vision_;
    std::mutex mtx_lock_v2v_;
    std::mutex mtx_lock_sensor_;
    std::mutex mtx_lock_cloud2_;
    std::mutex mtx_lock_cloud_vanjee_;

    std::map<std::string, std::function<void(const boost::any&)>> map_pt_;

    std::string s_laser_link_name_;
    std::string s_base_link_name_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tfl_;

    bool b_not_use_localization_ = false;
    bool b_cluster_use_ = false;
    int n_cluster_size_max_ = 0;
    float f_cluster_step_dist_ = 1.0;
    float f_cluster_obs_check_dist_ = 0.0;
    std::vector<float> vec_outline_dist_;
    std::vector<float> vec_max_cluster_distance_m_;
    std::vector<float> vec_cluster_size_min_;

private:
    void CloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    void Cloud2Callback(const sensor_msgs::PointCloud::ConstPtr& msg);
    void CloudVanjeeCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    void RobotSpeedCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void loadedCallback(const std_msgs::Bool::ConstPtr& msg);
    void ObstacleCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    void V2VCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    pcl::PointCloud<pcl::PointXYZRGB> FilterLidarClusters(const sensor_msgs::PointCloud& cloud);
    void UpdateSensorMsgCloud(const std::vector<geometry_msgs::Point32>& points);
    void UpdateSensorMsgCloud(const pcl::PointCloud<pcl::PointXYZRGB>& points);
};

#endif