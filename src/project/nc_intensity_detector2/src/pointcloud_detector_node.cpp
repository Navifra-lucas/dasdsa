#include "nc_intensity_detector2/pointcloud_detector_node.h"
#include "nc_intensity_detector2/transform_manager.h"
#include "nc_intensity_detector2/visualization_manager.h"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <util/logger.hpp>
#include <cmath>
#include <algorithm>
#include <limits>
#include <filesystem>
#include <iomanip>
#include <vector>

PointCloudDetectorNode::PointCloudDetectorNode() 
  : nh_("~"), pnh_("~"), tf_listener_(tf_buffer_),
    has_previous_cluster_(false), has_robot_pose_(false),
    detection_enabled_(false), has_previous_yaw_(false), previous_yaw_(0.0) {
  
  loadParameters();
  
  // Initialize managers
  transform_manager_ = std::make_unique<nc_intensity_detector2::TransformManager>(
    camera_tf_x_, camera_tf_y_, camera_tf_z_, camera_tf_yaw_);
  
  // Publishers (must be created before VisualizationManager)
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 1);
  global_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(global_pose_topic_, 1);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 1);
  all_clusters_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(all_clusters_pc_topic_, 1);
  selected_cluster_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(selected_cluster_pc_topic_, 1);
  plane_inliers_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(plane_inliers_pc_topic_, 1);
  high_intensity_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(high_intensity_pc_topic_, 1);
  filtered_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(filtered_pc_topic_, 1);

  // image_transport::ImageTransport it(nh_);
  // wall_image_pub_ = it.advertise("/nc_intensity_detector/wall_image", 1);
  last_image_save_time_ = ros::Time::now();

  initializeImageSaveDirectory();
  
  visualization_manager_ = std::make_unique<nc_intensity_detector2::VisualizationManager>(
    marker_pub_, all_clusters_pc_pub_, selected_cluster_pc_pub_, plane_inliers_pc_pub_, high_intensity_pc_pub_,
    base_frame_, camera_tf_x_, camera_tf_y_, camera_tf_yaw_);
  
  // Subscribers
  pointcloud_sub_ = nh_.subscribe(pointcloud_topic_, 1,
                                   &PointCloudDetectorNode::pointcloudCallback, this);
  cmd_sub_ = nh_.subscribe(cmd_topic_, 1,
                           &PointCloudDetectorNode::cmdCallback, this);
  robot_pose_sub_ = nh_.subscribe("/localization/robot_pos", 1,
                                   &PointCloudDetectorNode::robotPoseCallback, this);
  
  LOG_INFO("nc_intensity_detector2 node initialized");
  LOG_INFO("Subscribing to pointcloud: %s", pointcloud_topic_.c_str());
  LOG_INFO("Subscribing to cmd: %s", cmd_topic_.c_str());
  LOG_INFO("Publishing to: %s", pose_topic_.c_str());
  LOG_INFO("Tape dimensions: %.2fm x %.2fm", tape_width_, tape_height_);
}

void PointCloudDetectorNode::loadParameters() {
  // Topics
  pnh_.param<std::string>("topics/pointcloud", pointcloud_topic_, "/sick_visionary_t_mini/points");
  pnh_.param<std::string>("topics/output", pose_topic_, "/nc_intensity_detector/pose");
  pnh_.param<std::string>("topics/global_pose", global_pose_topic_, "/nc_intensity_detector/global_pose");
  pnh_.param<std::string>("topics/cmd", cmd_topic_, "/nc_intensity_detector/cmd");
  pnh_.param<std::string>("topics/marker", marker_topic_, "/nc_intensity_detector/marker");
  pnh_.param<std::string>("topics/all_clusters_pc", all_clusters_pc_topic_, "/nc_intensity_detector/all_clusters_pc");
  pnh_.param<std::string>("topics/selected_cluster_pc", selected_cluster_pc_topic_, "/nc_intensity_detector/selected_cluster_pc");
  pnh_.param<std::string>("topics/plane_inliers_pc", plane_inliers_pc_topic_, "/nc_intensity_detector/plane_inliers_pc");
  pnh_.param<std::string>("topics/high_intensity_pc", high_intensity_pc_topic_, "/nc_intensity_detector/high_intensity_pc");
  pnh_.param<std::string>("topics/filtered_pc", filtered_pc_topic_, "/nc_intensity_detector/filtered_pc");
  
  // Frames
  pnh_.param<std::string>("frames/camera", camera_frame_, "sick_visionary_t_mini");
  pnh_.param<std::string>("frames/base", base_frame_, "base_link");
  
  // Tape dimensions
  pnh_.param<double>("tape/width", tape_width_, 0.60);
  pnh_.param<double>("tape/height", tape_height_, 0.40);
  pnh_.param<double>("tape/size_tolerance", tape_size_tolerance_, 0.15);
  
  // Point cloud filtering
  pnh_.param<double>("pointcloud/min_distance", min_distance_, 1.0);
  pnh_.param<double>("pointcloud/max_distance", max_distance_, 10.0);
  pnh_.param<double>("pointcloud/min_height", min_height_, -1.0);
  pnh_.param<double>("pointcloud/max_height", max_height_, 1.0);
  pnh_.param<double>("pointcloud/min_lateral", min_lateral_, -2.0);
  pnh_.param<double>("pointcloud/max_lateral", max_lateral_, 2.0);
  pnh_.param<double>("pointcloud/cluster_tolerance", cluster_tolerance_, 0.05);
  pnh_.param<int>("pointcloud/min_cluster_size", min_cluster_size_, 100);
  pnh_.param<int>("pointcloud/max_cluster_size", max_cluster_size_, 10000);
  
  // Camera transform (base_link -> camera)
  pnh_.param<double>("frames/camera_tf/x", camera_tf_x_, -0.363);
  pnh_.param<double>("frames/camera_tf/y", camera_tf_y_, 0.02);
  pnh_.param<double>("frames/camera_tf/z", camera_tf_z_, 0.0);
  pnh_.param<double>("frames/camera_tf/yaw", camera_tf_yaw_, 3.14159);
  
  // Tracking
  pnh_.param<double>("tracking/distance_threshold", tracking_distance_threshold_, 1.0);
  
  // Yaw filtering (low-pass filter)
  // alpha: 0.0 = no filtering (use raw value), 1.0 = no smoothing (use previous value)
  // Recommended: 0.1-0.3 for smoothing (smaller = more smoothing)
  pnh_.param<double>("filtering/yaw_alpha", yaw_filter_alpha_, 0.2);
  
  LOG_INFO("Parameters loaded: tape=%.2fx%.2fm, distance=[%.1f,%.1f]m, height=[%.1f,%.1f]m, lateral=[%.1f,%.1f]m, cluster_tolerance=%.3fm, yaw_filter_alpha=%.2f",
           tape_width_, tape_height_, min_distance_, max_distance_, min_height_, max_height_, min_lateral_, max_lateral_, cluster_tolerance_, yaw_filter_alpha_);
}

void PointCloudDetectorNode::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  // LOG_INFO("Point cloud callback: received point cloud (size=%dx%d, frame_id=%s, enabled=%d)",
  //           msg->width, msg->height, msg->header.frame_id.c_str(), detection_enabled_);
  
  if (!detection_enabled_) {
    LOG_DEBUG("Point cloud: Detection disabled, skipping");
    return;
  }
  
  try {
    ros::Time t_start = ros::Time::now();
    
    // Convert ROS PointCloud2 to PCL
    ros::Time t0 = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    double dt_convert = (ros::Time::now() - t0).toSec() * 1000.0;
    
    if (cloud->empty()) {
      LOG_WARNING("Point cloud: Empty point cloud received");
      return;
    }
    
    // Transform to camera frame first (if needed), then filter NaN and distance together
    ros::Time t1 = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);
    if (msg->header.frame_id != camera_frame_) {
      // Use camera_tf parameters for manual transformation
      if (msg->header.frame_id == base_frame_) {
        transform_manager_->transformPointCloudToCamera(cloud, cloud_camera);
      } else {
        LOG_WARNING("Point cloud: Cannot transform from %s to %s (only base_link -> camera supported)",
                    msg->header.frame_id.c_str(), camera_frame_.c_str());
        return;
      }
    } else {
      cloud_camera = cloud;
    }
    double dt_transform = (ros::Time::now() - t1).toSec() * 1000.0;
    
    // Filter NaN and invalid points using PCL filter (more efficient)
    ros::Time t2 = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_valid(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_camera, *cloud_valid, indices);
    double dt_nan_filter = (ros::Time::now() - t2).toSec() * 1000.0;
    
    if (cloud_valid->empty()) {
      LOG_DEBUG("Point cloud: No valid points after NaN filtering");
      return;
    }
    
    // Distance filtering (Z is forward in camera frame)
    ros::Time t3 = ros::Time::now();
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud_valid);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(min_distance_, max_distance_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_distance_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass_z.filter(*cloud_distance_filtered);
    double dt_distance_filter = (ros::Time::now() - t3).toSec() * 1000.0;
    
    if (cloud_distance_filtered->empty()) {
      LOG_DEBUG("Point cloud: No points after distance filtering");
      return;
    }
    
    // Height filtering (Y is vertical in camera frame)
    ros::Time t3_5 = ros::Time::now();
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud_distance_filtered);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(min_height_, max_height_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_height_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass_y.filter(*cloud_height_filtered);
    double dt_height_filter = (ros::Time::now() - t3_5).toSec() * 1000.0;
    
    if (cloud_height_filtered->empty()) {
      LOG_DEBUG("Point cloud: No points after height filtering");
      return;
    }
    
    // Lateral filtering (X is left/right in camera frame)
    ros::Time t3_6 = ros::Time::now();
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud_height_filtered);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(min_lateral_, max_lateral_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass_x.filter(*cloud_filtered);
    double dt_lateral_filter = (ros::Time::now() - t3_6).toSec() * 1000.0;
    
    if (cloud_filtered->empty()) {
      LOG_DEBUG("Point cloud: No points after lateral filtering");
      return;
    }
    
    // Publish filtered point cloud for visualization
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_base(new pcl::PointCloud<pcl::PointXYZ>);
    visualization_manager_->transformPointCloudToBase(cloud_filtered, cloud_filtered_base);
    sensor_msgs::PointCloud2 filtered_pc_msg;
    pcl::toROSMsg(*cloud_filtered_base, filtered_pc_msg);
    filtered_pc_msg.header.frame_id = base_frame_;
    filtered_pc_msg.header.stamp = msg->header.stamp;
    filtered_pc_pub_.publish(filtered_pc_msg);
    
    // Downsample for faster clustering (VoxelGrid filter)
    ros::Time t4 = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud_filtered);
    voxel_filter.setLeafSize(0.01f, 0.01f, 0.01f);  // 2cm voxel size
    voxel_filter.filter(*cloud_downsampled);
    double dt_downsample = (ros::Time::now() - t4).toSec() * 1000.0;
    
    if (cloud_downsampled->empty()) {
      LOG_DEBUG("Point cloud: No points after downsampling");
      return;
    }
    
    // Publish 2D wall image
    // convertAndPublishImage(cloud_downsampled, msg->header);
    saveImage(cloud_downsampled);

    // Find tape clusters
    ros::Time t5 = ros::Time::now();
    std::vector<nc_intensity_detector2::TapeCluster> clusters = findTapeClusters(cloud_downsampled);
    double dt_clustering = (ros::Time::now() - t5).toSec() * 1000.0;
    
    if (clusters.empty()) {
      LOG_DEBUG("Point cloud: No valid tape clusters found");
      visualization_manager_->clearMarker();
      return;
    }
    
    // Select best cluster (closest to center)
    ros::Time t6 = ros::Time::now();
    nc_intensity_detector2::TapeCluster best_cluster = selectBestCluster(clusters);
    double dt_selection = (ros::Time::now() - t6).toSec() * 1000.0;
    
    // 정밀한 각도 계산을 위해 선택된 군집에 대해 다시 평면 피팅 수행
    ros::Time t6_5 = ros::Time::now();
    double precise_yaw, precise_distance;
    pcl::PointCloud<pcl::PointXYZ>::Ptr precise_inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (fitPlaneToCluster(best_cluster.cloud, precise_yaw, precise_distance, precise_inlier_cloud, true)) {
      // Apply yaw filter to smooth out noise
      best_cluster.yaw = filterYaw(precise_yaw);
      best_cluster.distance = precise_distance;
      best_cluster.inlier_cloud = precise_inlier_cloud;
    }
    double dt_precise_angle = (ros::Time::now() - t6_5).toSec() * 1000.0;
    
    // Track cluster
    trackCluster(best_cluster);
    
    // Publish pose
    ros::Time t7 = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = camera_frame_;
    pose.header.stamp = msg->header.stamp;
    pose.pose.position.x = best_cluster.center.z;  // 전방 거리 (Z축)
    pose.pose.position.y = best_cluster.center.x;   // 횡 방향 (X축)
    pose.pose.position.z = best_cluster.center.y;   // 수직 (Y축)
    
    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, best_cluster.yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    
    // Transform to base_link using camera_tf parameters
    geometry_msgs::PoseStamped pose_base;
    transform_manager_->transformPoseToBase(pose, pose_base);
    pose_pub_.publish(pose_base);
    
    // Publish global pose if available
    geometry_msgs::PoseStamped global_pose;
    bool has_global_pose = false;
    if (has_robot_pose_) {
      global_pose = calculateGlobalPose(pose_base);
      global_pose_pub_.publish(global_pose);
      has_global_pose = true;
    }
    double dt_pose = (ros::Time::now() - t7).toSec() * 1000.0;
    
    // Publish visualization (in base_link frame)
    ros::Time t8 = ros::Time::now();
    visualization_manager_->publishMarker(best_cluster);
    visualization_manager_->publishClusterMarkers(clusters);
    visualization_manager_->publishAllClustersPointCloud(clusters, msg->header.stamp);
    visualization_manager_->publishSelectedClusterPointCloud(best_cluster, msg->header.stamp);
    visualization_manager_->publishPlaneInliersPointCloud(best_cluster, msg->header.stamp);
    double dt_viz = (ros::Time::now() - t8).toSec() * 1000.0;
    
    // Log timing information
    static ros::Time last_log_time(0);
    ros::Time now = ros::Time::now();
    // if ((now - last_log_time).toSec() >= 0.5) {  // Log at max 2Hz
      double dt_total = (now - t_start).toSec() * 1000.0;
      LOG_INFO("Point cloud: Timing (ms) - convert=%.1f, transform=%.1f, nan_filter=%.1f, dist_filter=%.1f, height_filter=%.1f, lateral_filter=%.1f, downsample=%.1f, clustering=%.1f, selection=%.1f, precise_angle=%.1f, pose=%.1f, viz=%.1f, TOTAL=%.1f",
               dt_convert, dt_transform, dt_nan_filter, dt_distance_filter, dt_height_filter, dt_lateral_filter, dt_downsample, dt_clustering, dt_selection, dt_precise_angle, dt_pose, dt_viz, dt_total);
      LOG_INFO("Point cloud: Point counts - original=%zu, after_filter=%zu, after_downsample=%zu",
               cloud->size(), cloud_filtered->size(), cloud_downsampled->size());
      LOG_INFO("Point cloud: Found %zu valid tape clusters, selected - x=%.3fm, y=%.3fm, z=%.3fm, yaw=%.3fdeg",
               clusters.size(), best_cluster.center.x, best_cluster.center.y, best_cluster.center.z,
               best_cluster.yaw * 180.0 / M_PI);
      // Extract yaw from pose_base quaternion
      tf2::Quaternion q_base(pose_base.pose.orientation.x, pose_base.pose.orientation.y,
                            pose_base.pose.orientation.z, pose_base.pose.orientation.w);
      tf2::Matrix3x3 m_base(q_base);
      double roll_base, pitch_base, yaw_base;
      m_base.getRPY(roll_base, pitch_base, yaw_base);
      
      LOG_INFO("Point cloud: Transformed pose (base_link) - x=%.3fm, y=%.3fm, z=%.3fm, yaw=%.3fdeg",
               pose_base.pose.position.x, pose_base.pose.position.y, pose_base.pose.position.z,
               yaw_base * 180.0 / M_PI);
      
      if (has_global_pose) {
        // Extract yaw from global_pose quaternion
        tf2::Quaternion q_global(global_pose.pose.orientation.x, global_pose.pose.orientation.y,
                                global_pose.pose.orientation.z, global_pose.pose.orientation.w);
        tf2::Matrix3x3 m_global(q_global);
        double roll_global, pitch_global, yaw_global;
        m_global.getRPY(roll_global, pitch_global, yaw_global);
        
        LOG_INFO("Point cloud: Global pose - x=%.3fm, y=%.3fm, z=%.3fm, yaw=%.3fdeg",
                 global_pose.pose.position.x, global_pose.pose.position.y, global_pose.pose.position.z,
                 yaw_global * 180.0 / M_PI);
      }
      last_log_time = now;
    // }
    
  } catch (const std::exception& e) {
    LOG_WARNING("Point cloud processing error: %s", e.what());
  }
}

std::vector<nc_intensity_detector2::TapeCluster> PointCloudDetectorNode::findTapeClusters(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  
  std::vector<nc_intensity_detector2::TapeCluster> valid_clusters;
  
  // Euclidean clustering (optimized: use organized search if possible)
  ros::Time t_cluster_start = ros::Time::now();
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);
  double dt_euclidean = (ros::Time::now() - t_cluster_start).toSec() * 1000.0;
  
  // Early exit if too many clusters (likely noise)
  if (cluster_indices.size() > 20) {
    LOG_DEBUG("Point cloud: Too many clusters (%zu), likely noise, skipping", cluster_indices.size());
    return valid_clusters;
  }
  
  // Process each cluster
  ros::Time t_process_start = ros::Time::now();
  for (size_t i = 0; i < cluster_indices.size(); i++) {
    ros::Time t_cluster_i = ros::Time::now();
    // Extract cluster (optimized: reserve memory first)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cluster_cloud->points.reserve(cluster_indices[i].indices.size());
    for (const auto& idx : cluster_indices[i].indices) {
      cluster_cloud->points.push_back(cloud->points[idx]);
    }
    cluster_cloud->width = cluster_cloud->points.size();
    cluster_cloud->height = 1;
    cluster_cloud->is_dense = false;
    
    // 빠른 필터링: 클러스터 크기가 1.5m 이상이면 제외 (속도 개선)
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);
    double cluster_width = max_pt.x - min_pt.x;
    double cluster_height = max_pt.y - min_pt.y;
    double max_size = 1.5;  // 1.5m
    if (cluster_width > max_size || cluster_height > max_size) {
      continue;  // 너무 큰 클러스터는 타겟이 아니므로 제외
    }
    
    // Validate cluster size (60cm x 40cm) using original cluster first
    ros::Time t_validate = ros::Time::now();
    if (!validateClusterSize(cluster_cloud, tape_width_, tape_height_)) {
      continue;
    }
    double dt_validate = (ros::Time::now() - t_validate).toSec() * 1000.0;
    
    // Remove outer edge (3cm margin) for visualization only (after validation)
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_filtered = removeClusterEdge(cluster_cloud, 0.03);
    
    // if (cluster_filtered->empty()) {
    //   // If filtering removes all points, use original cluster for visualization
    //   cluster_filtered = cluster_cloud;
    // }
    // else 
    // {
    //     cluster_cloud = cluster_filtered;
    // }
    
    // Fit plane to cluster and calculate yaw (빠른 탐색을 위해 optimize_coefficients=false 사용)
    ros::Time t_plane = ros::Time::now();
    double yaw, distance;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!fitPlaneToCluster(cluster_cloud, yaw, distance, inlier_cloud, false)) {
      continue;
    }
    double dt_plane = (ros::Time::now() - t_plane).toSec() * 1000.0;
    
    // Calculate cluster center (use original cluster for stable center calculation)
    ros::Time t_center = ros::Time::now();
    pcl::PointXYZ center;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster_cloud, centroid);
    center.x = centroid[0];
    center.y = centroid[1];
    center.z = centroid[2];
    double dt_center = (ros::Time::now() - t_center).toSec() * 1000.0;
    
    // Calculate distance from center (for selection)
    // In camera frame: center is at (0, 0, z)
    // For lateral alignment, use only X (lateral) distance, Y (vertical) is less important
    double distance_from_center = std::abs(center.x);  // Lateral distance only
    
    // Create TapeCluster (use filtered cluster for visualization, but calculations use original)
    nc_intensity_detector2::TapeCluster cluster;
    cluster.cloud = cluster_cloud;  // Use filtered cluster for visualization
    cluster.inlier_cloud = inlier_cloud;  // 평면 찾기에 사용된 포인트 (원본 클러스터에서 계산)
    cluster.center = center;  // 원본 클러스터에서 계산된 중심
    cluster.distance = distance;  // 원본 클러스터에서 계산된 거리
    cluster.y = center.x;  // Lateral position (X in camera frame: Z forward, X lateral, Y vertical)
    cluster.yaw = yaw;  // 원본 클러스터에서 계산된 각도
    cluster.distance_from_center = distance_from_center;
    
    // Calculate actual width and height from bounding box (use original cluster)
    // min_pt, max_pt는 이미 위에서 계산됨 (빠른 필터링에서)
    cluster.width = max_pt.x - min_pt.x;
    cluster.height = max_pt.y - min_pt.y;
    
    valid_clusters.push_back(cluster);
    
    double dt_cluster_i = (ros::Time::now() - t_cluster_i).toSec() * 1000.0;
    // LOG_DEBUG("Point cloud: Cluster %zu processing: validate=%.1fms, plane=%.1fms, center=%.1fms, total=%.1fms",
    //          i, dt_validate, dt_plane, dt_center, dt_cluster_i);
  }
  
  double dt_process = (ros::Time::now() - t_process_start).toSec() * 1000.0;
  static ros::Time last_cluster_log(0);
  ros::Time now_cluster = ros::Time::now();
  if ((now_cluster - last_cluster_log).toSec() >= 1.0) {  // Log at max 1Hz
    LOG_INFO("Point cloud: Clustering timing - euclidean=%.1fms, process_all=%.1fms, clusters=%zu",
             dt_euclidean, dt_process, cluster_indices.size());
    last_cluster_log = now_cluster;
  }
  
  return valid_clusters;
}

bool PointCloudDetectorNode::validateClusterSize(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
    double expected_width, double expected_height) {
  
  if (cluster->empty()) {
    return false;
  }
  
  // Get bounding box
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cluster, min_pt, max_pt);
  
  double width = max_pt.x - min_pt.x;
  double height = max_pt.y - min_pt.y;
  
  // Expected size validation (removed unnecessary average distance calculation)
  double tolerance = tape_size_tolerance_;
  double min_width = expected_width * (1.0 - tolerance);
  double max_width = expected_width * (1.0 + tolerance);
  double min_height = expected_height * (1.0 - tolerance);
  double max_height = expected_height * (1.0 + tolerance);
  
  bool width_ok = (width >= min_width && width <= max_width);
  bool height_ok = (height >= min_height && height <= max_height);
  
  // LOG_DEBUG("Point cloud: Cluster size validation: width=%.3fm (expected=%.3fm±%.1f%%), height=%.3fm (expected=%.3fm±%.1f%%)",
  //           width, expected_width, tolerance * 100, height, expected_height, tolerance * 100);
  
  return width_ok && height_ok;
}

bool PointCloudDetectorNode::fitPlaneToCluster(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
    double& yaw, double& distance,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& inlier_cloud,
    bool optimize_coefficients) {
  
  ros::Time t_start = ros::Time::now();
  
  if (cluster->size() < 3) {
    return false;
  }
  
  // Use RANSAC to fit plane
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  
  seg.setOptimizeCoefficients(optimize_coefficients);  // 빠른 탐색(false) 또는 정밀한 각도 계산(true)
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);  // Further reduced for faster processing
  seg.setDistanceThreshold(0.05);  // 3cm
  seg.setProbability(0.99);  // High probability with fewer iterations
  seg.setInputCloud(cluster);
  seg.segment(*inliers, *coefficients);
  
  // Evaluate plane fitting quality
  size_t total_points = cluster->size();
  size_t inlier_count = inliers->indices.size();
  double inlier_ratio = (total_points > 0) ? (static_cast<double>(inlier_count) / total_points) : 0.0;
  
  if (inlier_count < total_points * 0.5) {
    LOG_DEBUG("Point cloud: Plane fitting failed - not enough inliers (%zu/%zu, %.1f%%)",
              inlier_count, total_points, inlier_ratio * 100.0);
    return false;
  }
  
  // Extract inlier points
  inlier_cloud->points.reserve(inlier_count);
  for (const auto& idx : inliers->indices) {
    inlier_cloud->points.push_back(cluster->points[idx]);
  }
  inlier_cloud->width = inlier_cloud->points.size();
  inlier_cloud->height = 1;
  inlier_cloud->is_dense = false;
  
  // Log plane fitting quality (throttled)
  static ros::Time last_plane_log(0);
  ros::Time now_plane = ros::Time::now();
  if ((now_plane - last_plane_log).toSec() >= 1.0) {  // Log at max 1Hz
    LOG_INFO("Point cloud: Plane fitting - inliers=%zu/%zu (%.1f%%), distance_threshold=%.3fm",
             inlier_count, total_points, inlier_ratio * 100.0, 0.03);
    last_plane_log = now_plane;
  }
  
  // Extract plane equation: ax + by + cz + d = 0
  double a = coefficients->values[0];
  double b = coefficients->values[1];
  double c = coefficients->values[2];
  double d = coefficients->values[3];
  
  // Normalize normal vector to check verticality
  double norm_before = std::sqrt(a * a + b * b + c * c);
  if (norm_before < 1e-6) {
    return false;
  }
  double a_norm = a / norm_before;
  double b_norm = b / norm_before;
  double c_norm = c / norm_before;
  
  // Check if plane is vertical (Y component should be small)
  // Camera frame: Z forward, X lateral, Y vertical
  // For vertical plane: |b| (vertical component) should be small (< 0.2, about 11.5 degrees)
  double vertical_threshold = 0.2;  // ~11.5 degrees from vertical
  if (std::abs(b_norm) > vertical_threshold) {
    // LOG_DEBUG("Point cloud: Plane is not vertical enough (b=%.3f, threshold=%.3f)",
    //           std::abs(b_norm), vertical_threshold);
    return false;
  }
  
  // Force vertical plane (no vertical component in camera frame)
  // For vertical plane: b (vertical) should be 0
  b = 0.0;
  
  // Re-normalize
  double norm = std::sqrt(a * a + b * b + c * c);
  if (norm < 1e-6) {
    return false;
  }
  a /= norm;
  b /= norm;
  c /= norm;
  
  // Recalculate d using cluster center
  pcl::PointXYZ center;
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cluster, centroid);
  center.x = centroid[0];
  center.y = centroid[1];
  center.z = centroid[2];
  d = -(a * center.x + b * center.y + c * center.z);
  
  // Calculate distance (Z component in camera frame)
  distance = std::abs(center.z);
  
  // Calculate yaw from plane normal
  // For vertical plane: yaw = atan2(-a, |c|) where a is lateral, c is forward
  yaw = std::atan2(-a, std::abs(c));
  
  // Normalize yaw to [-PI, PI]
  while (yaw > M_PI) yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;
  
  // Log execution time
  double dt_total = (ros::Time::now() - t_start).toSec() * 1000.0;
  static ros::Time last_fit_log(0);
  ros::Time now_fit = ros::Time::now();
  if ((now_fit - last_fit_log).toSec() >= 1.0) {  // Log at max 1Hz
    LOG_INFO("Point cloud: fitPlaneToCluster - time=%.2fms, optimize=%s, points=%zu",
             dt_total, optimize_coefficients ? "true" : "false", cluster->size());
    last_fit_log = now_fit;
  }
  
  return true;
}

nc_intensity_detector2::TapeCluster PointCloudDetectorNode::selectBestCluster(
    const std::vector<nc_intensity_detector2::TapeCluster>& clusters) {
  
  if (clusters.empty()) {
    nc_intensity_detector2::TapeCluster empty;
    return empty;
  }
  
  // If we have previous cluster, use tracking
  if (has_previous_cluster_) {
    double min_distance = std::numeric_limits<double>::max();
    size_t best_idx = 0;
    
    for (size_t i = 0; i < clusters.size(); i++) {
      double dist = std::sqrt(
        std::pow(clusters[i].center.x - previous_center_.x, 2) +
        std::pow(clusters[i].center.y - previous_center_.y, 2) +
        std::pow(clusters[i].center.z - previous_center_.z, 2)
      );
      
      if (dist < min_distance && dist < tracking_distance_threshold_) {
        min_distance = dist;
        best_idx = i;
      }
    }
    
    if (min_distance < tracking_distance_threshold_) {
      // LOG_DEBUG("Point cloud: Selected cluster %zu based on tracking (distance=%.3fm)",
      //           best_idx, min_distance);
      return clusters[best_idx];
    }
  }
  
  // Otherwise, select cluster closest to center
  size_t best_idx = 0;
  double min_center_dist = clusters[0].distance_from_center;
  
  for (size_t i = 1; i < clusters.size(); i++) {
    if (clusters[i].distance_from_center < min_center_dist) {
      min_center_dist = clusters[i].distance_from_center;
      best_idx = i;
    }
  }
  
  // LOG_DEBUG("Point cloud: Selected cluster %zu closest to center (distance=%.3fm)",
  //           best_idx, min_center_dist);
  
  return clusters[best_idx];
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudDetectorNode::removeClusterEdge(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
    double margin) {
  
  if (cluster->empty() || margin <= 0.0) {
    return cluster;
  }
  
  // Get bounding box
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cluster, min_pt, max_pt);
  
  // Shrink bounding box by margin on each side
  double min_x = min_pt.x + margin;
  double max_x = max_pt.x - margin;
  double min_y = min_pt.y + margin;
  double max_y = max_pt.y - margin;
  double min_z = min_pt.z + margin;
  double max_z = max_pt.z - margin;
  
  // Check if bounding box is still valid after shrinking
  if (min_x >= max_x || min_y >= max_y || min_z >= max_z) {
    // If margin is too large, return empty cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    return empty_cloud;
  }
  
  // Filter points inside the shrunk bounding box
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  filtered_cloud->points.reserve(cluster->size());
  
  for (const auto& pt : cluster->points) {
    if (pt.x >= min_x && pt.x <= max_x &&
        pt.y >= min_y && pt.y <= max_y &&
        pt.z >= min_z && pt.z <= max_z) {
      filtered_cloud->points.push_back(pt);
    }
  }
  
  filtered_cloud->width = filtered_cloud->points.size();
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = false;
  
  return filtered_cloud;
}

void PointCloudDetectorNode::trackCluster(const nc_intensity_detector2::TapeCluster& cluster) {
  previous_center_ = cluster.center;
  has_previous_cluster_ = true;
}

void PointCloudDetectorNode::filterAndPublishHighIntensityPoints(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_camera) {
  
  // Check if intensity field exists
  int intensity_idx = -1;
  for (size_t i = 0; i < msg->fields.size(); i++) {
    if (msg->fields[i].name == "intensity") {
      intensity_idx = i;
      break;
    }
  }
  
  if (intensity_idx < 0) {
    // No intensity field, skip
    return;
  }
  
  // Extract intensity field information
  const sensor_msgs::PointField& intensity_field = msg->fields[intensity_idx];
  int intensity_offset = intensity_field.offset;
  int intensity_datatype = intensity_field.datatype;
  int point_step = msg->point_step;
  
  // Filter points with intensity >= 10000
  pcl::PointCloud<pcl::PointXYZ>::Ptr high_intensity_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  high_intensity_cloud->points.reserve(cloud_camera->size());
  
  const uint8_t* data_ptr = msg->data.data();
  size_t point_idx = 0;
  
  for (size_t i = 0; i < msg->width * msg->height; i++) {
    if (point_idx >= cloud_camera->size()) {
      break;
    }
    
    const uint8_t* point_data = data_ptr + i * point_step + intensity_offset;
    float intensity = 0.0f;
    
    // Read intensity based on datatype
    if (intensity_datatype == sensor_msgs::PointField::FLOAT32) {
      intensity = *reinterpret_cast<const float*>(point_data);
    } else if (intensity_datatype == sensor_msgs::PointField::UINT16) {
      intensity = static_cast<float>(*reinterpret_cast<const uint16_t*>(point_data));
    } else if (intensity_datatype == sensor_msgs::PointField::UINT32) {
      intensity = static_cast<float>(*reinterpret_cast<const uint32_t*>(point_data));
    } else {
      point_idx++;
      continue;
    }
    
    // Check if intensity >= 10000 and point is valid
    if (intensity >= 10.0f && 
        std::isfinite(cloud_camera->points[point_idx].x) &&
        std::isfinite(cloud_camera->points[point_idx].y) &&
        std::isfinite(cloud_camera->points[point_idx].z)) {
      high_intensity_cloud->points.push_back(cloud_camera->points[point_idx]);
    }
    
    point_idx++;
  }
  
  high_intensity_cloud->width = high_intensity_cloud->points.size();
  high_intensity_cloud->height = 1;
  high_intensity_cloud->is_dense = false;
  
  if (high_intensity_cloud->empty()) {
    return;
  }
  
  // Transform to base_link and publish
  pcl::PointCloud<pcl::PointXYZ>::Ptr high_intensity_base(new pcl::PointCloud<pcl::PointXYZ>);
  visualization_manager_->transformPointCloudToBase(high_intensity_cloud, high_intensity_base);
  
  sensor_msgs::PointCloud2 high_intensity_msg;
  pcl::toROSMsg(*high_intensity_base, high_intensity_msg);
  high_intensity_msg.header.frame_id = base_frame_;
  high_intensity_msg.header.stamp = msg->header.stamp;
  
  high_intensity_pc_pub_.publish(high_intensity_msg);
}

void PointCloudDetectorNode::saveImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  if (cloud->empty()) return;

  ros::Time now = ros::Time::now();
  if ((now - last_image_save_time_).toSec() < 3.0) {
      return;
  }
  last_image_save_time_ = now;

  // 1. Define Image Resolution (e.g. 1cm/pixel)
  double resolution = 0.01; // 1cm per pixel
  int width = std::ceil((max_lateral_ - min_lateral_) / resolution);
  int height = std::ceil((max_height_ - min_height_) / resolution);

  if (width <= 0 || height <= 0) return;

  // 2. Create cv::Mat (Initialize with black)
  cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3); // Use color image for better visualization if needed

  // 3. Project Points to Image
  // Coordinate Mapping:
  // Image X (col) <- Point Cloud X (Lateral)
  // Image Y (row) <- Point Cloud Y (Vertical) - Inverted (Image Y increases downwards)
  for (const auto& pt : cloud->points) {
    int col = static_cast<int>((pt.x - min_lateral_) / resolution);
    // Invert Y axis for image coordinates (max_height is top of image, min_height is bottom)
    int row = static_cast<int>((max_height_ - pt.y) / resolution);

    // Flip horizontally
    col = width - 1 - col;

    if (row >= 0 && row < height && col >= 0 && col < width) {
        // Draw white point
        image.at<cv::Vec3b>(row, col) = cv::Vec3b(255, 255, 255);
    }
  }

  // 4. Save Image
  std::time_t t = std::time(nullptr);
  std::tm* now_tm = std::localtime(&t);
  std::stringstream ss;
  ss << image_save_dir_ << "/intensity_" 
     << std::put_time(now_tm, "%Y%m%d_%H%M%S") 
     << ".png";
  std::string filename = ss.str();

  if (cv::imwrite(filename, image)) {
      // LOG_INFO("Saved wall image to %s", filename.c_str());
  } else {
      LOG_ERROR("Failed to save wall image to %s", filename.c_str());
      return;
  }

  // 5. File Rotation (Max 20 files)
  namespace fs = std::filesystem;
  std::vector<std::string> files;
  
  try {
      for (const auto& entry : fs::directory_iterator(image_save_dir_)) {
          if (entry.is_regular_file()) {
              std::string fname = entry.path().filename().string();
              if (fname.rfind("intensity_", 0) == 0 && fname.find(".png") != std::string::npos) {
                  files.push_back(entry.path().string());
              }
          }
      }
  } catch (const std::exception& ex) {
       LOG_ERROR("Filesystem error: %s", ex.what());
  }

  if (files.size() > 20) {
      std::sort(files.begin(), files.end()); // Sort by name (timestamp)
      int files_to_delete = files.size() - 20;
      for (int i = 0; i < files_to_delete; ++i) {
          try {
              if (fs::remove(files[i])) {
                 // LOG_INFO("Removed old image file: %s", files[i].c_str());
              }
          } catch (const std::exception& ex) {
               LOG_ERROR("Failed to remove file %s: %s", files[i].c_str(), ex.what());
          }
      }
  }
}

void PointCloudDetectorNode::initializeImageSaveDirectory() {
  namespace fs = std::filesystem;
  const char* home_env = std::getenv("HOME");
  std::string base_dir;

  if (home_env) {
    base_dir = std::string(home_env);
  } else {
    base_dir = ".";
    LOG_WARNING("HOME environment variable not set, using current directory");
  }

  image_save_dir_ = base_dir + "/navifra_solution/navicore/nc_intensity_detector_images";

  try {
    if (!fs::exists(image_save_dir_)) {
      if (fs::create_directories(image_save_dir_)) {
        LOG_INFO("Created image save directory: %s", image_save_dir_.c_str());
      } else {
        LOG_ERROR("Failed to create image save directory: %s", image_save_dir_.c_str());
      }
    } else {
      LOG_INFO("Using image save directory: %s", image_save_dir_.c_str());
    }
  } catch (const std::exception& ex) {
    LOG_ERROR("Filesystem error while initializing directory: %s", ex.what());
  }
}

void PointCloudDetectorNode::cmdCallback(const std_msgs::String::ConstPtr& msg) {
  std::string cmd = msg->data;
  std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
  
  if (cmd == "start") {
    detection_enabled_ = true;
    has_previous_cluster_ = false;
    has_previous_yaw_ = false;  // Reset yaw filter
    LOG_INFO("Detection STARTED");
  } else if (cmd == "stop") {
    detection_enabled_ = false;
    has_previous_cluster_ = false;
    has_previous_yaw_ = false;  // Reset yaw filter
    visualization_manager_->clearMarker();
    LOG_INFO("Detection STOPPED");
  }
}

void PointCloudDetectorNode::robotPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(robot_pose_mutex_);
  robot_pose_ = *msg;
  has_robot_pose_ = true;
}

geometry_msgs::PoseStamped PointCloudDetectorNode::calculateGlobalPose(
    const geometry_msgs::PoseStamped& local_pose) {
  
  geometry_msgs::PoseStamped global_pose;
  
  if (!has_robot_pose_) {
    return local_pose;
  }
  
  std::lock_guard<std::mutex> lock(robot_pose_mutex_);
  
  // Extract robot position and orientation
  double robot_x = robot_pose_.pose.pose.position.x;
  double robot_y = robot_pose_.pose.pose.position.y;
  double robot_z = robot_pose_.pose.pose.position.z;
  
  tf2::Quaternion robot_q(
    robot_pose_.pose.pose.orientation.x,
    robot_pose_.pose.pose.orientation.y,
    robot_pose_.pose.pose.orientation.z,
    robot_pose_.pose.pose.orientation.w
  );
  tf2::Matrix3x3 robot_m(robot_q);
  double robot_roll, robot_pitch, robot_yaw;
  robot_m.getRPY(robot_roll, robot_pitch, robot_yaw);
  
  // Transform local pose to global frame
  // local_pose는 base_link 좌표계에서의 타겟 위치 (로봇 기준 상대 위치)
  // 로봇의 회전(robot_yaw)만 고려하여 global 좌표계로 변환
  // 타겟의 방향(local_pose의 yaw)은 global 위치에 영향을 주지 않음
  double cos_yaw = std::cos(robot_yaw);
  double sin_yaw = std::sin(robot_yaw);
  
  // base_link 좌표계의 타겟 위치를 global 좌표계로 변환
  // 로봇의 위치 + (로봇의 회전 행렬 * base_link에서의 타겟 위치)
  // 주의: local_pose.pose.position.x, y는 base_link 좌표계에서의 타겟 위치이므로
  // 로봇이 회전하면 이 값들은 변하지 않지만, global 좌표계로 변환할 때는
  // 로봇의 회전을 고려해야 함
  double local_x = local_pose.pose.position.x;
  double local_y = local_pose.pose.position.y;
  double local_z = local_pose.pose.position.z;
  
  global_pose.pose.position.x = robot_x + local_x * cos_yaw - local_y * sin_yaw;
  global_pose.pose.position.y = robot_y + local_x * sin_yaw + local_y * cos_yaw;
  global_pose.pose.position.z = robot_z + local_z;
  
  // Transform orientation
  tf2::Quaternion local_q(
    local_pose.pose.orientation.x,
    local_pose.pose.orientation.y,
    local_pose.pose.orientation.z,
    local_pose.pose.orientation.w
  );
  tf2::Matrix3x3 local_m(local_q);
  double local_roll, local_pitch, local_yaw;
  local_m.getRPY(local_roll, local_pitch, local_yaw);
  
  double global_yaw = robot_yaw + local_yaw;
  
  // Normalize yaw to [-PI, PI]
  while (global_yaw > M_PI) global_yaw -= 2.0 * M_PI;
  while (global_yaw < -M_PI) global_yaw += 2.0 * M_PI;
  
  tf2::Quaternion global_q;
  global_q.setRPY(local_roll, local_pitch, global_yaw);
  
  global_pose.pose.orientation.x = global_q.x();
  global_pose.pose.orientation.y = global_q.y();
  global_pose.pose.orientation.z = global_q.z();
  global_pose.pose.orientation.w = global_q.w();
  
  global_pose.header.stamp = local_pose.header.stamp;
  global_pose.header.frame_id = robot_pose_.header.frame_id;
  
  return global_pose;
}

double PointCloudDetectorNode::filterYaw(double raw_yaw) {
  // Exponential moving average (low-pass) filter for yaw
  // filtered_yaw = alpha * raw_yaw + (1 - alpha) * previous_yaw
  
  if (!has_previous_yaw_) {
    // First measurement: use raw value
    previous_yaw_ = raw_yaw;
    has_previous_yaw_ = true;
    return raw_yaw;
  }
  
  // Handle angle wrapping: ensure difference is in [-PI, PI]
  double yaw_diff = raw_yaw - previous_yaw_;
  
  // Normalize difference to [-PI, PI]
  while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
  while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
  
  // Apply filter: interpolate between previous and new value
  double filtered_yaw = previous_yaw_ + yaw_filter_alpha_ * yaw_diff;
  
  // Normalize filtered yaw to [-PI, PI]
  while (filtered_yaw > M_PI) filtered_yaw -= 2.0 * M_PI;
  while (filtered_yaw < -M_PI) filtered_yaw += 2.0 * M_PI;
  
  // Update previous value
  previous_yaw_ = filtered_yaw;
  
  return filtered_yaw;
}

int main(int argc, char** argv) {
  NaviFra::Logger::get().SetSeverityMin(severity_level::info);
  ros::init(argc, argv, "nc_intensity_detector2_node");
  
  PointCloudDetectorNode node;
  
  ros::spin();
  
  return 0;
}

