#include "nc_navigator/nc_navigator_pch.h"

#include <nc_navigator/data_listener.hpp>

DataListener::DataListener()
    : tfl_(buffer_)
{
    buffer_.setUsingDedicatedThread(true);
    node_handle_.param<string>("laser_link_name", s_laser_link_name_, "laser_link");
    node_handle_.param<string>("base_link_name", s_base_link_name_, "base_link");
    ros::param::param<bool>("obstacle/b_cluster_use", b_cluster_use_, false);
    ros::param::param<int>("obstacle/n_cluster_size_max", n_cluster_size_max_, 10000);
    ros::param::param<float>("obstacle/f_cluster_obs_check_dist", f_cluster_obs_check_dist_, 0.05);
    ros::param::param<float>("obstacle/f_cluster_step_dist", f_cluster_step_dist_, 1.0);
    ros::param::get("obstacle/data_cluster_distance_m", vec_max_cluster_distance_m_);
    ros::param::get("obstacle/data_cluster_size_min", vec_cluster_size_min_);
    ros::param::get("obstacle/outline", vec_outline_dist_);

    cloud_sub_ = node_handle_.subscribe("scan_cloud", 1000, &DataListener::CloudCallback, this);
    cloud2_sub_ = node_handle_.subscribe("scan_cloud2", 1000, &DataListener::Cloud2Callback, this);
    cloud3_sub_ = node_handle_.subscribe("scan_cloud_vanjee", 1000, &DataListener::CloudVanjeeCallback, this);
    robot_speed_sub_ = node_handle_.subscribe("odom", 1000, &DataListener::RobotSpeedCallback, this);
    robot_loaded_sub_ = node_handle_.subscribe("navifra/loaded", 1000, &DataListener::loadedCallback, this);
    obstacle_sub_ = node_handle_.subscribe("/nc_obstacle_detector/vision_all_data", 2, &DataListener::ObstacleCallback, this);
    v2v_sub_ = node_handle_.subscribe("v2v_cloud", 2, &DataListener::V2VCallback, this);

    local_tf_all_data_pub_ = node_handle_.advertise<geometry_msgs::PoseArray>("navifra/local_tf_all_data", 1);

    ros::Duration timeout(1.0);
    try {
        tf_geom_ = buffer_.lookupTransform(s_base_link_name_, s_laser_link_name_, ros::Time(0), timeout);
        LOG_INFO("laser_link tf OK from: %s to: %s", s_base_link_name_.c_str(), s_laser_link_name_.c_str());
    }
    catch (tf2::TransformException& e) {
        LOG_ERROR("laser_link tf FAILED from: %s to: %s", s_base_link_name_.c_str(), s_laser_link_name_.c_str());
    }
}

DataListener::~DataListener()
{
}

bool DataListener::Notify(const std::string& str_cbf_name, const boost::any& any_type_var)
{
    if (map_pt_.find(str_cbf_name) == map_pt_.end()) {
        return false;
    }
    else {
        map_pt_[str_cbf_name](any_type_var);
        return true;
    }
}

bool DataListener::RegisteCallbackFunc(const std::string& str_cbf_name, const std::function<void(const boost::any&)>& pt_func)
{
    if (map_pt_.find(str_cbf_name) == map_pt_.end()) {
        map_pt_[str_cbf_name] = pt_func;
        return true;
    }
    else {
        return false;
    }
}

pcl::PointCloud<pcl::PointXYZRGB> DataListener::FilterLidarClusters(const sensor_msgs::PointCloud& cloud)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vec_obs_check_cloud(3);
    for (size_t i = 0; i < vec_obs_check_cloud.size(); ++i) {
        vec_obs_check_cloud[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    }

    LidarClustering::RGB color;
    color.r = rand() % 256;
    color.g = rand() % 256;
    color.b = rand() % 256;

    pcl::PointCloud<pcl::PointXYZRGB> cloud_total;
    for (int i = 0; i < cloud.points.size(); i++) {
        pcl::PointXYZ point_temp(cloud.points[i].x, cloud.points[i].y, 0.0);
        // Clutering시 거리별 체크 조건을 다르게하기 위하여 vector로 나눠서 저장
        for (int j = 0; j < 3; j++) {
            if ((cloud.points[i].x < (vec_outline_dist_[0] + f_cluster_step_dist_ * (j + 1)) &&
                 cloud.points[i].x > (vec_outline_dist_[1] - f_cluster_step_dist_ * (j + 1)) &&
                 cloud.points[i].y < (vec_outline_dist_[3] + f_cluster_step_dist_ * (j + 1)) &&
                 cloud.points[i].y > (vec_outline_dist_[2] - f_cluster_step_dist_ * (j + 1))) &&
                (cloud.points[i].x > (vec_outline_dist_[0] + (f_cluster_step_dist_ * j) - 0.15) ||
                 cloud.points[i].x < (vec_outline_dist_[1] - (f_cluster_step_dist_ * j) + 0.15) ||
                 cloud.points[i].y > (vec_outline_dist_[3] + (f_cluster_step_dist_ * j) - 0.15) ||
                 cloud.points[i].y < (vec_outline_dist_[2] - (f_cluster_step_dist_ * j) + 0.15))) {
                vec_obs_check_cloud[j]->points.emplace_back(point_temp);
            }
        }
        // 체크 거리 이상의 포인트는 바로 return 변수인 cloud_total로 저장
        if (cloud.points[i].x >= (vec_outline_dist_[0] + f_cluster_step_dist_ * 3) ||
            cloud.points[i].x <= (vec_outline_dist_[1] - f_cluster_step_dist_ * 3) ||
            cloud.points[i].y >= (vec_outline_dist_[3] + f_cluster_step_dist_ * 3) ||
            cloud.points[i].y <= (vec_outline_dist_[2] - f_cluster_step_dist_ * 3)) {
            pcl::PointXYZRGB point_rgb_temp;
            point_rgb_temp.x = cloud.points[i].x;
            point_rgb_temp.y = cloud.points[i].y;
            point_rgb_temp.z = 0.0;
            point_rgb_temp.r = color.r;
            point_rgb_temp.g = color.g;
            point_rgb_temp.b = color.b;
            cloud_total.points.emplace_back(point_rgb_temp);
        }
    }

    std::chrono::steady_clock::time_point timer = std::chrono::steady_clock::now();
    std::chrono::duration<double> eclipse;
    // Filtering 함수
    for (int a = 0; a < vec_obs_check_cloud.size(); a++) {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        if (!vec_obs_check_cloud[a]->points.empty()) {
            tree->setInputCloud(vec_obs_check_cloud[a]);
        }
        std::vector<pcl::PointIndices> cluster_indices;

        // perform clustering on 2d cloud
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(vec_max_cluster_distance_m_[a]);  // d_max_cluster_distance_m
        ec.setMinClusterSize(vec_cluster_size_min_[a]);  // n_cluster_size_min
        ec.setMaxClusterSize(n_cluster_size_max_);  // n_cluster_size_max
        ec.setSearchMethod(tree);
        ec.setInputCloud(vec_obs_check_cloud[a]);
        ec.extract(cluster_indices);

        eclipse = std::chrono::steady_clock::now() - timer;
        std_msgs::Header header = cloud.header;
        // cluster
        unsigned int id = 0;
        timer = std::chrono::steady_clock::now();
        std::vector<LidarClustering::ClusterPtr> o_clusters;
        for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
            LidarClustering::ClusterPtr cluster(new LidarClustering::Cluster());
            cluster->SetCloud(vec_obs_check_cloud[a], it->indices, header, id, color, "");
            o_clusters.push_back(cluster);

            id++;
        }
        for (const auto& cluster : o_clusters) {
            pcl::PointCloud<pcl::PointXYZRGB> cluster_check = *(cluster->GetCloud());
            float f_max_dist = 0;
            for (int i = 0; i < cluster_check.size() - 1; i++) {
                f_max_dist =
                    max(fabs(f_max_dist),
                        hypot(fabs(cluster_check[0].x - cluster_check[i + 1].x), fabs(cluster_check[0].y - cluster_check[i + 1].y)));
            }
            if (f_max_dist > f_cluster_obs_check_dist_)
                cloud_total += *(cluster->GetCloud());
        }
    }
    return cloud_total;
}

void DataListener::UpdateSensorMsgCloud(const std::vector<geometry_msgs::Point32>& points)
{
    st_sensor_msg_.vec_cloud.clear();
    st_sensor_msg_.vec_cloud.resize(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        st_sensor_msg_.vec_cloud[i].SetXm(points[i].x);
        st_sensor_msg_.vec_cloud[i].SetYm(points[i].y);
        st_sensor_msg_.vec_cloud[i].SetZm(4);
    }
}

void DataListener::UpdateSensorMsgCloud(const pcl::PointCloud<pcl::PointXYZRGB>& points)
{
    st_sensor_msg_.vec_cloud.clear();
    st_sensor_msg_.vec_cloud.resize(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        st_sensor_msg_.vec_cloud[i].SetXm(points[i].x);
        st_sensor_msg_.vec_cloud[i].SetYm(points[i].y);
        st_sensor_msg_.vec_cloud[i].SetZm(4);
    }
}

void DataListener::CloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    const std::lock_guard<std::mutex> lock(mtx_lock_sensor_);

    sensor_msgs::PointCloud cloud = *msg;
    if (b_cluster_use_ && vec_outline_dist_.size() < 4) {
        NLOG(info) << "vec_outline_dist_ size low...not cluster";
    }
    if (b_cluster_use_ && vec_outline_dist_.size() >= 4) {
        pcl::PointCloud<pcl::PointXYZRGB> cloud_total = FilterLidarClusters(cloud);
        UpdateSensorMsgCloud(cloud_total);
    }
    else {
        UpdateSensorMsgCloud(cloud.points);
    }

    {
        const float f_angle_increment_deg = 1.f;
        st_sensor_msg_.vec_sensors_relative_robot.clear();
        // cloud
        std::vector<NaviFra::SimplePos> vec_vision_obs;
        {
            const std::lock_guard<std::mutex> lock(mtx_lock_vision_);
            vec_vision_obs = vec_vision_obs_;
            st_sensor_msg_.vec_vision_obs = vec_vision_obs;
        }
        std::vector<NaviFra::SimplePos> vec_v2v_obs;
        {
            const std::lock_guard<std::mutex> lock(mtx_lock_v2v_);
            vec_v2v_obs = vec_v2v_obs_;
        }
        std::vector<NaviFra::SimplePos> vec_cloud2_obs;
        {
            const std::lock_guard<std::mutex> lock(mtx_lock_cloud2_);
            vec_cloud2_obs = vec_cloud2_obs_;
        }
        std::vector<NaviFra::SimplePos> vec_cloud_vanjee_obs;
        {
            const std::lock_guard<std::mutex> lock(mtx_lock_cloud_vanjee_);
            vec_cloud_vanjee_obs = vec_cloud_vanjee_obs_;
        }


        st_sensor_msg_.vec_sensors_relative_robot.insert(
            st_sensor_msg_.vec_sensors_relative_robot.end(), st_sensor_msg_.vec_cloud.begin(), st_sensor_msg_.vec_cloud.end());
        st_sensor_msg_.vec_sensors_relative_robot.insert(
            st_sensor_msg_.vec_sensors_relative_robot.end(), vec_vision_obs.begin(), vec_vision_obs.end());
        st_sensor_msg_.vec_sensors_relative_robot.insert(
            st_sensor_msg_.vec_sensors_relative_robot.end(), vec_v2v_obs.begin(), vec_v2v_obs.end());
        st_sensor_msg_.vec_sensors_relative_robot.insert(
            st_sensor_msg_.vec_sensors_relative_robot.end(), vec_cloud2_obs.begin(), vec_cloud2_obs.end());
        st_sensor_msg_.vec_sensors_relative_robot.insert(
            st_sensor_msg_.vec_sensors_relative_robot.end(), vec_cloud_vanjee_obs.begin(), vec_cloud_vanjee_obs.end());

        Notify("all_sensor_data_callback", st_sensor_msg_);
    }
    if (local_tf_all_data_pub_.getNumSubscribers() >= 1) {
        geometry_msgs::PoseArray pose_array;
        pose_array.header.stamp = ros::Time::now();
        pose_array.header.frame_id = s_base_link_name_;
        for (int i = 0; i < st_sensor_msg_.vec_sensors_relative_robot.size(); i++) {
            geometry_msgs::Pose p;
            p.position.x = st_sensor_msg_.vec_sensors_relative_robot[i].GetXm();
            p.position.y = st_sensor_msg_.vec_sensors_relative_robot[i].GetYm();
            p.position.z = st_sensor_msg_.vec_sensors_relative_robot[i].GetZm();

            pose_array.poses.emplace_back(std::move(p));
        }
        local_tf_all_data_pub_.publish(pose_array);
    }
}

void DataListener::RobotSpeedCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    const std::lock_guard<std::mutex> lock(mtx_lock_sensor_);

    st_sensor_msg_.o_robot_speed.SetXm(msg->twist.twist.linear.x);
    st_sensor_msg_.o_robot_speed.SetYm(msg->twist.twist.linear.y);
    st_sensor_msg_.o_robot_speed.SetRad(msg->twist.twist.angular.z);
    // Notify("all_sensor_data_callback", st_sensor_msg_);
}

void DataListener::loadedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    const std::lock_guard<std::mutex> lock(mtx_lock_sensor_);

    st_sensor_msg_.b_robot_loaded = msg->data;
}

void DataListener::ObstacleCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    std::vector<NaviFra::SimplePos> vec_pos;
    for (const auto& p : msg->points) {
        NaviFra::SimplePos q;
        q.SetXm(p.x);
        q.SetYm(p.y);
        q.SetZm(1);

        if(q.GetXm() != 0 && q.GetYm() != 0) {
            vec_pos.push_back(q);
        }
    }

    {
        const std::lock_guard<std::mutex> lock(mtx_lock_vision_);
        vec_vision_obs_.clear();
        vec_vision_obs_ = std::move(vec_pos);
    }
}

void DataListener::V2VCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    return;
    std::vector<NaviFra::SimplePos> vec_pos;
    for (const auto& p : msg->points) {
        NaviFra::SimplePos q;
        q.SetXm(p.x);
        q.SetYm(p.y);
        q.SetZm(2);
        vec_pos.push_back(q);
    }

    {
        const std::lock_guard<std::mutex> lock(mtx_lock_v2v_);
        vec_v2v_obs_.clear();
        vec_v2v_obs_ = std::move(vec_pos);
    }
}

void DataListener::Cloud2Callback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    std::vector<NaviFra::SimplePos> vec_pos;
    
    sensor_msgs::PointCloud cloud = *msg;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_total = FilterLidarClusters(cloud);
    for (const auto& p : cloud_total) {
        NaviFra::SimplePos q;
        q.SetXm(p.x);
        q.SetYm(p.y);
        q.SetZm(0);
        vec_pos.push_back(q);
    }

    {
        const std::lock_guard<std::mutex> lock(mtx_lock_cloud2_);
        vec_cloud2_obs_.clear();
        vec_cloud2_obs_ = std::move(vec_pos);
    }
}

void DataListener::CloudVanjeeCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    std::vector<NaviFra::SimplePos> vec_pos;
    
    sensor_msgs::PointCloud cloud = *msg;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_total = FilterLidarClusters(cloud);
    for (const auto& p : cloud_total) {
        NaviFra::SimplePos q;
        q.SetXm(p.x);
        q.SetYm(p.y);
        q.SetZm(0);
        vec_pos.push_back(q);
    }

    {
        const std::lock_guard<std::mutex> lock(mtx_lock_cloud_vanjee_);
        vec_cloud_vanjee_obs_.clear();
        vec_cloud_vanjee_obs_ = std::move(vec_pos);
    }
}