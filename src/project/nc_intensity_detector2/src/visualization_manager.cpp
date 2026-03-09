#include "nc_intensity_detector2/visualization_manager.h"

#include <tf2/LinearMath/Quaternion.h>
#include <util/logger.hpp>
#include <pcl/filters/voxel_grid.h>
#include <cmath>

namespace nc_intensity_detector2 {

VisualizationManager::VisualizationManager(ros::Publisher& marker_pub,
                                           ros::Publisher& all_clusters_pc_pub,
                                           ros::Publisher& selected_cluster_pc_pub,
                                           ros::Publisher& plane_inliers_pc_pub,
                                           ros::Publisher& high_intensity_pc_pub,
                                           const std::string& base_frame,
                                           double camera_tf_x, double camera_tf_y, double camera_tf_yaw)
  : marker_pub_(marker_pub), 
    all_clusters_pc_pub_(all_clusters_pc_pub),
    selected_cluster_pc_pub_(selected_cluster_pc_pub),
    plane_inliers_pc_pub_(plane_inliers_pc_pub),
    base_frame_(base_frame),
    camera_tf_x_(camera_tf_x),
    camera_tf_y_(camera_tf_y),
    camera_tf_yaw_(camera_tf_yaw),
    marker_id_(0) {
  // high_intensity_pc_pub_ is not stored as member, but passed for consistency
  (void)high_intensity_pc_pub;
}

void VisualizationManager::transformPointCloudToBase(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_camera,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_base) {
  
  // Transform from camera frame to base_link
  // camera_tf: base_link -> camera (translation: x, y, rotation: yaw)
  // Inverse: camera -> base_link
  double cos_yaw = std::cos(-camera_tf_yaw_);  // Negative for inverse
  double sin_yaw = std::sin(-camera_tf_yaw_);
  
  cloud_base->points.resize(cloud_camera->points.size());
  cloud_base->width = cloud_camera->width;
  cloud_base->height = cloud_camera->height;
  cloud_base->is_dense = cloud_camera->is_dense;
  
  for (size_t i = 0; i < cloud_camera->points.size(); ++i) {
    // Subtract translation first
    double x_rel = cloud_camera->points[i].x - camera_tf_x_;
    double y_rel = cloud_camera->points[i].y - camera_tf_y_;
    double z_rel = cloud_camera->points[i].z;
    
    // Rotate by -yaw (inverse rotation)
    cloud_base->points[i].x = x_rel * cos_yaw - y_rel * sin_yaw;
    cloud_base->points[i].y = x_rel * sin_yaw + y_rel * cos_yaw;
    cloud_base->points[i].z = z_rel;
  }
}

void VisualizationManager::publishMarker(const TapeCluster& cluster) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = base_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "tape_detection";
  marker.id = marker_id_++;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  
  // Transform center from camera frame to base_link
  double cos_yaw = std::cos(-camera_tf_yaw_);
  double sin_yaw = std::sin(-camera_tf_yaw_);
  double x_rel = cluster.center.x - camera_tf_x_;
  double y_rel = cluster.center.y - camera_tf_y_;
  double x_base = x_rel * cos_yaw - y_rel * sin_yaw;
  double y_base = x_rel * sin_yaw + y_rel * cos_yaw;
  double z_base = cluster.center.z;
  
  marker.pose.position.x = x_base;
  marker.pose.position.y = y_base;
  marker.pose.position.z = z_base;
  
  // Orientation (yaw in base_link frame)
  tf2::Quaternion q;
  q.setRPY(0, 0, cluster.yaw - camera_tf_yaw_);  // Adjust yaw for base_link
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();
  
  // Scale
  marker.scale.x = 0.5;  // Arrow length
  marker.scale.y = 0.1;  // Arrow width
  marker.scale.z = 0.1;  // Arrow height
  
  // Color (red for selected)
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  
  marker_pub_.publish(marker);
}

void VisualizationManager::publishClusterMarkers(
    const std::vector<TapeCluster>& clusters) {
  
  for (size_t i = 0; i < clusters.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "tape_clusters";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // Transform center from camera frame to base_link
    double cos_yaw = std::cos(-camera_tf_yaw_);
    double sin_yaw = std::sin(-camera_tf_yaw_);
    double x_rel = clusters[i].center.x - camera_tf_x_;
    double y_rel = clusters[i].center.y - camera_tf_y_;
    double x_base = x_rel * cos_yaw - y_rel * sin_yaw;
    double y_base = x_rel * sin_yaw + y_rel * cos_yaw;
    double z_base = clusters[i].center.z;
    
    marker.pose.position.x = x_base;
    marker.pose.position.y = y_base;
    marker.pose.position.z = z_base;
    
    // Orientation
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    // Scale (tape dimensions)
    marker.scale.x = clusters[i].width;
    marker.scale.y = clusters[i].height;
    marker.scale.z = 0.05;  // Thickness
    
    // Color (different for each cluster)
    marker.color.r = (i % 3 == 0) ? 1.0 : 0.0;
    marker.color.g = (i % 3 == 1) ? 1.0 : 0.0;
    marker.color.b = (i % 3 == 2) ? 1.0 : 0.0;
    marker.color.a = 0.5;
    
    marker_pub_.publish(marker);
  }
}

void VisualizationManager::applyVoxelFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output) {
  
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(input);
  voxel_filter.setLeafSize(0.02f, 0.02f, 0.02f);  // 2cm voxel size (same as clustering)
  voxel_filter.filter(*output);
}

void VisualizationManager::publishAllClustersPointCloud(
    const std::vector<TapeCluster>& clusters,
    const ros::Time& stamp) {
  
  if (clusters.empty()) {
    return;
  }
  
  // Combine all cluster point clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);
  
  for (const auto& cluster : clusters) {
    if (cluster.cloud && !cluster.cloud->empty()) {
      *combined_cloud_camera += *cluster.cloud;
    }
  }
  
  if (combined_cloud_camera->empty()) {
    return;
  }
  
  // Apply voxel filter for visualization (ensure downsampled)
  pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
  applyVoxelFilter(combined_cloud_camera, combined_cloud_voxel);
  
  if (combined_cloud_voxel->empty()) {
    return;
  }
  
  // Transform to base_link
  pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud_base(new pcl::PointCloud<pcl::PointXYZ>);
  transformPointCloudToBase(combined_cloud_voxel, combined_cloud_base);
  
  // Convert to ROS message
  sensor_msgs::PointCloud2 cluster_pc_msg;
  pcl::toROSMsg(*combined_cloud_base, cluster_pc_msg);
  cluster_pc_msg.header.frame_id = base_frame_;
  cluster_pc_msg.header.stamp = stamp;
  
  all_clusters_pc_pub_.publish(cluster_pc_msg);
}

void VisualizationManager::publishSelectedClusterPointCloud(
    const TapeCluster& cluster,
    const ros::Time& stamp) {
  
  if (!cluster.cloud || cluster.cloud->empty()) {
    return;
  }
  
  // Apply voxel filter for visualization (ensure downsampled)
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_voxel(new pcl::PointCloud<pcl::PointXYZ>);
  applyVoxelFilter(cluster.cloud, cluster_voxel);
  
  if (cluster_voxel->empty()) {
    return;
  }
  
  // Transform to base_link
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_base(new pcl::PointCloud<pcl::PointXYZ>);
  transformPointCloudToBase(cluster_voxel, cluster_base);
  
  // Convert to ROS message
  sensor_msgs::PointCloud2 cluster_pc_msg;
  pcl::toROSMsg(*cluster_base, cluster_pc_msg);
  cluster_pc_msg.header.frame_id = base_frame_;
  cluster_pc_msg.header.stamp = stamp;
  
  selected_cluster_pc_pub_.publish(cluster_pc_msg);
}

void VisualizationManager::publishPlaneInliersPointCloud(
    const TapeCluster& cluster,
    const ros::Time& stamp) {
  
  if (!cluster.inlier_cloud || cluster.inlier_cloud->empty()) {
    return;
  }
  
  // Apply voxel filter for visualization (ensure downsampled)
  pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_voxel(new pcl::PointCloud<pcl::PointXYZ>);
  applyVoxelFilter(cluster.inlier_cloud, inlier_voxel);
  
  if (inlier_voxel->empty()) {
    return;
  }
  
  // Transform to base_link
  pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_base(new pcl::PointCloud<pcl::PointXYZ>);
  transformPointCloudToBase(inlier_voxel, inlier_base);
  
  // Convert to ROS message
  sensor_msgs::PointCloud2 inlier_pc_msg;
  pcl::toROSMsg(*inlier_base, inlier_pc_msg);
  inlier_pc_msg.header.frame_id = base_frame_;
  inlier_pc_msg.header.stamp = stamp;
  
  plane_inliers_pc_pub_.publish(inlier_pc_msg);
}

void VisualizationManager::clearMarker() {
  visualization_msgs::Marker marker;
  marker.header.frame_id = base_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "tape_detection";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker_pub_.publish(marker);
  
  visualization_msgs::Marker cluster_marker;
  cluster_marker.header.frame_id = base_frame_;
  cluster_marker.header.stamp = ros::Time::now();
  cluster_marker.ns = "tape_clusters";
  cluster_marker.id = 0;
  cluster_marker.action = visualization_msgs::Marker::DELETEALL;
  marker_pub_.publish(cluster_marker);
}

} // namespace nc_intensity_detector2
