#include "nc_lidar_static_check/lidar_static_check_node.hpp"

#include "core_msgs/WiaForkInfo.h"

LidarStaticCheckNode::LidarStaticCheckNode(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    : nh_(nh)
    , nhp_(nhp)
{
    ros::param::param<int>("obstacle/n_cluster_size_max", n_cluster_size_max_, 10000);
    ros::param::param<float>("obstacle/f_cluster_obs_check_dist", f_cluster_obs_check_dist_, 0.05);
    ros::param::param<float>("obstacle/f_cluster_step_dist", f_cluster_step_dist_, 1.0);
    ros::param::get("obstacle/data_cluster_distance_m", vec_max_cluster_distance_m_);
    ros::param::get("obstacle/data_cluster_size_min", vec_cluster_size_min_);

    NLOG(info) << "LidarStaticCheckNode initialized.";
    NLOG(info) << "n_cluster_size_max_: " << n_cluster_size_max_;
    NLOG(info) << "f_cluster_obs_check_dist_: " << f_cluster_obs_check_dist_;
    NLOG(info) << "f_cluster_step_dist_: " << f_cluster_step_dist_;
    NLOG(info) << "vec_max_cluster_distance_m_ size: " << vec_max_cluster_distance_m_.size();
    NLOG(info) << "vec_cluster_size_min_ size: " << vec_cluster_size_min_.size();
    
    d_obstacle_hold_sec_ = 0.3;
    NLOG(info) << d_obstacle_hold_sec_;
    n_consecutive_hits_ = 0;
    b_final_state_ = false;

    // Load polygon
    std::vector<NaviFra::Polygon> vec_polygons;

    NaviFra::Polygon outline_poly;
    if (LoadPolygonFromParam(nh_, "/obstacle/outline", outline_poly))
        vec_polygons.push_back(outline_poly);

    NaviFra::Polygon collision_poly;
    if (LoadPolygonFromParam(nh_, "/obstacle/collision2", collision_poly))
        vec_polygons.push_back(collision_poly);

    lidar_check_.SetPolygon(vec_polygons);

    // Subscribe
    scan_sub_ = nh_.subscribe("/scan_cloud_lift_check", 1, &LidarStaticCheckNode::ScanCallback, this);
    param_update_sub_ = nh_.subscribe("navifra/param_update", 10, &LidarStaticCheckNode::ParamUpdateCallback, this);
    cheonil_sub_ = nh_.subscribe("cheonil/read_register", 10, &LidarStaticCheckNode::CheonilCallback, this);
    check_trigger_sub_ = nh_.subscribe("fork_check_trigger", 10, &LidarStaticCheckNode::ForkTriggerCallback, this);

    // Publishers
    pub_bool_status_ = nh_.advertise<std_msgs::Bool>("is_lift_obstacle", 1);
    pub_status_ = nh_.advertise<std_msgs::String>("test_status", 1);
    pub_poly1_ = nh_.advertise<geometry_msgs::PolygonStamped>("visualize/polygon1", 1, true);
    pub_poly2_ = nh_.advertise<geometry_msgs::PolygonStamped>("visualize/polygon2", 1, true);

    pub_lift_state_ = nh.advertise<std_msgs::Bool>("forklift_pause", 1, true);

    if (vec_polygons.size() > 0)
        pub_poly1_.publish(lidar_check_.GetPolygonMsg(0));
    if (vec_polygons.size() > 1)
        pub_poly2_.publish(lidar_check_.GetPolygonMsg(1));

    timer_ = nh_.createTimer(ros::Duration(0.05), &LidarStaticCheckNode::TimerCallback, this);

    last_cloud_time_ = ros::Time(0);
}

LidarStaticCheckNode::~LidarStaticCheckNode()
{
}

bool LidarStaticCheckNode::LoadPolygonFromParam(ros::NodeHandle& nh, const std::string& param_name, NaviFra::Polygon& polygon)
{
    std::vector<float> vec_param;
    if (nh_.getParam(param_name, vec_param)) {
        if (vec_param.size() == 4) {
            for (int i = 0; i < vec_param.size(); i++) {
                NLOG(info) << vec_param[i];
            }
            polygon.AddVertexList(vec_param);
            return true;
        }
        ROS_ERROR("Param '%s' size %lu, expected 4", param_name.c_str(), vec_param.size());
    }
    else {
        ROS_ERROR("Param '%s' not found.", param_name.c_str());
    }
    return false;
}

void LidarStaticCheckNode::ParamUpdateCallback(const std_msgs::String::ConstPtr& msg)
{
    Initialize();
}

void LidarStaticCheckNode::Initialize()
{
    std::lock_guard<std::mutex> lock(mutex_);

    nh_.param("/obstacle/f_obstacle_change_delay", d_obstacle_hold_sec_, 0.3);

    n_consecutive_hits_ = 0;
    b_final_state_ = false;

    last_cloud_time_ = ros::Time(0);

    std::vector<NaviFra::Polygon> vec_polygons;

    NaviFra::Polygon outline_poly;
    if (LoadPolygonFromParam(nh_, "/obstacle/outline", outline_poly))
        vec_polygons.push_back(outline_poly);

    NaviFra::Polygon collision_poly;
    if (LoadPolygonFromParam(nh_, "/obstacle/collision2", collision_poly))
        vec_polygons.push_back(collision_poly);

    lidar_check_.SetPolygon(vec_polygons);

    if (vec_polygons.size() > 0)
        pub_poly1_.publish(lidar_check_.GetPolygonMsg(0));
    if (vec_polygons.size() > 1)
        pub_poly2_.publish(lidar_check_.GetPolygonMsg(1));
}

void LidarStaticCheckNode::ScanCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cloud_ = msg;
    last_cloud_time_ = ros::Time::now();
}

void LidarStaticCheckNode::ForkTriggerCallback(const std_msgs::Bool& msg)
{
    NLOG(info) << "ForkTriggerCallback" << static_cast<int>(msg.data);
    b_running_ = msg.data;
}

void LidarStaticCheckNode::CheonilCallback(const core_msgs::CheonilReadRegister::ConstPtr& msg)
{
    n_fork_lift_position_ = msg->fork_up_down_position;
}

sensor_msgs::PointCloud::Ptr LidarStaticCheckNode::FilterLidarClusters(const sensor_msgs::PointCloud::ConstPtr& cloud_msg)
{
    if (!cloud_msg || cloud_msg->points.empty()) {
        return sensor_msgs::PointCloud::Ptr();
    }

    // Load outline parameters if not loaded
    if (vec_outline_dist_.empty()) {
        ros::param::get("/obstacle/outline", vec_outline_dist_);
    }

    // Need at least 4 values for outline
    if (vec_outline_dist_.size() < 4) {
        NLOG(info) << "vec_outline_dist_ size low...not cluster";
        // Return original cloud without clustering
        sensor_msgs::PointCloud::Ptr result(new sensor_msgs::PointCloud(*cloud_msg));
        return result;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vec_obs_check_cloud(3);
    for (size_t i = 0; i < vec_obs_check_cloud.size(); ++i) {
        vec_obs_check_cloud[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_total(new pcl::PointCloud<pcl::PointXYZ>());

    for (int i = 0; i < cloud_msg->points.size(); i++) {
        pcl::PointXYZ point_temp(cloud_msg->points[i].x, cloud_msg->points[i].y, 0.0);
        // Clutering시 거리별 체크 조건을 다르게하기 위하여 vector로 나눠서 저장
        for (int j = 0; j < 3; j++) {
            if ((cloud_msg->points[i].x < (vec_outline_dist_[0] + f_cluster_step_dist_ * (j + 1)) &&
                 cloud_msg->points[i].x > (vec_outline_dist_[1] - f_cluster_step_dist_ * (j + 1)) &&
                 cloud_msg->points[i].y < (vec_outline_dist_[3] + f_cluster_step_dist_ * (j + 1)) &&
                 cloud_msg->points[i].y > (vec_outline_dist_[2] - f_cluster_step_dist_ * (j + 1))) &&
                (cloud_msg->points[i].x > (vec_outline_dist_[0] + (f_cluster_step_dist_ * j) - 0.15) ||
                 cloud_msg->points[i].x < (vec_outline_dist_[1] - (f_cluster_step_dist_ * j) + 0.15) ||
                 cloud_msg->points[i].y > (vec_outline_dist_[3] + (f_cluster_step_dist_ * j) - 0.15) ||
                 cloud_msg->points[i].y < (vec_outline_dist_[2] - (f_cluster_step_dist_ * j) + 0.15))) {
                vec_obs_check_cloud[j]->points.emplace_back(point_temp);
            }
        }
        // 체크 거리 이상의 포인트는 바로cloud_total로 저장
        if (cloud_msg->points[i].x >= (vec_outline_dist_[0] + f_cluster_step_dist_ * 3) ||
            cloud_msg->points[i].x <= (vec_outline_dist_[1] - f_cluster_step_dist_ * 3) ||
            cloud_msg->points[i].y >= (vec_outline_dist_[3] + f_cluster_step_dist_ * 3) ||
            cloud_msg->points[i].y <= (vec_outline_dist_[2] - f_cluster_step_dist_ * 3)) {
            cloud_total->points.emplace_back(point_temp);
        }
    }

    // Filtering 함수
    for (int a = 0; a < vec_obs_check_cloud.size(); a++) {
        if (vec_obs_check_cloud[a]->points.empty()) {
            continue;
        }

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(vec_obs_check_cloud[a]);
        
        std::vector<pcl::PointIndices> cluster_indices;

        // perform clustering on 2d cloud
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(vec_max_cluster_distance_m_[a]);  // d_max_cluster_distance_m
        ec.setMinClusterSize(vec_cluster_size_min_[a]);  // n_cluster_size_min
        ec.setMaxClusterSize(n_cluster_size_max_);  // n_cluster_size_max
        ec.setSearchMethod(tree);
        ec.setInputCloud(vec_obs_check_cloud[a]);
        ec.extract(cluster_indices);

        // Process each cluster
        for (const auto& cluster : cluster_indices) {
            if (cluster.indices.empty()) {
                continue;
            }

            // Check cluster size (max distance within cluster)
            float f_max_dist = 0;
            for (size_t i = 0; i < cluster.indices.size() - 1; i++) {
                const auto& p0 = vec_obs_check_cloud[a]->points[cluster.indices[0]];
                const auto& p1 = vec_obs_check_cloud[a]->points[cluster.indices[i + 1]];
                f_max_dist = std::max(f_max_dist, 
                    std::hypot(std::fabs(p0.x - p1.x), std::fabs(p0.y - p1.y)));
            }

            // If cluster is large enough, add all points to result
            if (f_max_dist > f_cluster_obs_check_dist_) {
                for (const auto& idx : cluster.indices) {
                    cloud_total->points.push_back(vec_obs_check_cloud[a]->points[idx]);
                }
            }
        }
    }

    // Convert PCL cloud back to sensor_msgs::PointCloud
    sensor_msgs::PointCloud::Ptr result(new sensor_msgs::PointCloud());
    result->header = cloud_msg->header;
    result->points.resize(cloud_total->points.size());

    for (size_t i = 0; i < cloud_total->points.size(); i++) {
        result->points[i].x = cloud_total->points[i].x;
        result->points[i].y = cloud_total->points[i].y;
        result->points[i].z = cloud_total->points[i].z;
    }

    return result;
}


void LidarStaticCheckNode::TimerCallback(const ros::TimerEvent&)
{
    if (!b_running_ || n_fork_lift_position_ < 500) {
        return;
    }

    ros::Time now = ros::Time::now();
    sensor_msgs::PointCloud::ConstPtr cloud;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        cloud = latest_cloud_;
    }

    bool hit = false;
    sensor_msgs::PointCloud::Ptr cluster_cloud = FilterLidarClusters(cloud);

    if ((now - last_cloud_time_).toSec() > f_lidar_timeout_sec_) {
        hit = true;
    }
    else if (cluster_cloud && !cluster_cloud->points.empty()) {
        hit = lidar_check_.CheckObstacle(cluster_cloud);
    }

    if (hit) {
        n_consecutive_hits_++;
        NLOG(info) << "n_consecutive_hits_ size is" << n_consecutive_hits_;
        if (n_consecutive_hits_ >= 2) {
            // 장애물 확정
            if (!b_final_state_) {
                b_final_state_ = true;
                final_state_changed_time_ = now;

                std_msgs::String s;
                s.data = "Obstacle detected (confirm 2 hits)";
                pub_status_.publish(s);
            }
        }
    }
    else {
        n_consecutive_hits_ = 0;  // 감지 끊김 → 다시 카운트
    }

    if (b_final_state_ && !hit) {
        if ((now - final_state_changed_time_).toSec() > d_obstacle_hold_sec_) {
            b_final_state_ = false;  // 해제
        }
    }

    if (b_final_state_ != prev_final_state_) {
        std_msgs::Bool msg;
        msg.data = b_final_state_;
        pub_lift_state_.publish(msg);  // lift_pause topic

        NLOG(info) << "LiftPause changed → " << (b_final_state_ ? "TRUE" : "FALSE");

        prev_final_state_ = b_final_state_;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nc_lidar_static_check");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    LidarStaticCheckNode nc_lidar_static_check(nh, nhp);
    ros::spin();

    return 0;
}
