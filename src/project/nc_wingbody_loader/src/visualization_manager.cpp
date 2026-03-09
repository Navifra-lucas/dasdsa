#include "nc_wingbody_loader/visualization_manager.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <util/logger.hpp>

namespace nc_wingbody_loader {

VisualizationManager::VisualizationManager(ros::Publisher& marker_array_pub,
                                           ros::Publisher& classified_pc_pub,
                                           ros::Publisher& filtered_pc_pub,
                                           ros::Publisher& target_marker_pub,
                                           const std::string& base_frame,
                                           TransformManager* transform_manager)
  : marker_array_pub_(marker_array_pub),
    classified_pc_pub_(classified_pc_pub),
    filtered_pc_pub_(filtered_pc_pub),
    target_marker_pub_(target_marker_pub),
    base_frame_(base_frame),
    transform_manager_(transform_manager) {
}

void VisualizationManager::getColorForType(PlaneType type, float& r, float& g, float& b) {
  switch (type) {
    case DOOR:             r = 1.0f; g = 0.0f; b = 0.0f; break;  // Red
    case HEAD_WALL:        r = 0.0f; g = 1.0f; b = 0.0f; break;  // Green
    case SIDE_WALL_LEFT:   r = 0.0f; g = 0.0f; b = 1.0f; break;  // Blue
    case SIDE_WALL_RIGHT:  r = 0.0f; g = 0.5f; b = 1.0f; break;  // Light blue
    case FLOOR:            r = 1.0f; g = 1.0f; b = 0.0f; break;  // Yellow
    case EXISTING_PALLET:  r = 0.0f; g = 1.0f; b = 1.0f; break;  // Cyan
    default:               r = 0.5f; g = 0.5f; b = 0.5f; break;  // Gray
  }
}

void VisualizationManager::publishPlaneMarkers(const std::vector<DetectedPlane>& planes,
                                                const ros::Time& stamp) {
  visualization_msgs::MarkerArray marker_array;

  for (size_t i = 0; i < planes.size(); ++i) {
    const auto& plane = planes[i];

    visualization_msgs::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = stamp;
    marker.ns = "wingbody_planes";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // Transform centroid from camera frame to base_link
    pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cam(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pt_base(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ p;
    p.x = plane.centroid.x();
    p.y = plane.centroid.y();
    p.z = plane.centroid.z();
    pt_cam->push_back(p);
    transform_manager_->transformPointCloudToBase(pt_cam, pt_base);

    marker.pose.position.x = pt_base->points[0].x;
    marker.pose.position.y = pt_base->points[0].y;
    marker.pose.position.z = pt_base->points[0].z;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Scale based on bounding box dimensions
    marker.scale.x = std::max(0.05, static_cast<double>(plane.width));
    marker.scale.y = std::max(0.05, static_cast<double>(plane.height));
    marker.scale.z = std::max(0.05, static_cast<double>(plane.depth));

    float r, g, b;
    getColorForType(plane.type, r, g, b);
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.5f;

    marker.lifetime = ros::Duration(2.0);

    marker_array.markers.push_back(marker);
  }

  marker_array_pub_.publish(marker_array);
}

void VisualizationManager::publishClassifiedPointCloud(const std::vector<DetectedPlane>& planes,
                                                        const ros::Time& stamp) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  for (const auto& plane : planes) {
    if (!plane.cloud || plane.cloud->empty()) continue;

    float r, g, b;
    getColorForType(plane.type, r, g, b);
    uint8_t ri = static_cast<uint8_t>(r * 255);
    uint8_t gi = static_cast<uint8_t>(g * 255);
    uint8_t bi = static_cast<uint8_t>(b * 255);

    // Transform plane cloud to base_link
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZ>);
    transform_manager_->transformPointCloudToBase(plane.cloud, cloud_base);

    for (const auto& pt : cloud_base->points) {
      pcl::PointXYZRGB colored_pt;
      colored_pt.x = pt.x;
      colored_pt.y = pt.y;
      colored_pt.z = pt.z;
      colored_pt.r = ri;
      colored_pt.g = gi;
      colored_pt.b = bi;
      colored_cloud->push_back(colored_pt);
    }
  }

  if (colored_cloud->empty()) return;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*colored_cloud, msg);
  msg.header.frame_id = base_frame_;
  msg.header.stamp = stamp;
  classified_pc_pub_.publish(msg);
}

void VisualizationManager::publishFilteredPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const ros::Time& stamp) {

  if (!cloud || cloud->empty()) return;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZ>);
  transform_manager_->transformPointCloudToBase(cloud, cloud_base);

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud_base, msg);
  msg.header.frame_id = base_frame_;
  msg.header.stamp = stamp;
  filtered_pc_pub_.publish(msg);
}

void VisualizationManager::publishTargetMarker(const PlacementResult& result,
                                                const ros::Time& stamp,
                                                double pallet_depth,
                                                double pallet_width,
                                                double pallet_height) {
  if (!result.success) return;

  tf2::Quaternion q;
  q.setRPY(0, 0, result.yaw);

  // 1) Arrow: placement pose direction
  visualization_msgs::Marker arrow;
  arrow.header.frame_id = base_frame_;
  arrow.header.stamp = stamp;
  arrow.ns = "wingbody_target";
  arrow.id = 0;
  arrow.type = visualization_msgs::Marker::ARROW;
  arrow.action = visualization_msgs::Marker::ADD;
  arrow.pose.position.x = result.x;
  arrow.pose.position.y = result.y;
  arrow.pose.position.z = result.z;
  arrow.pose.orientation.x = q.x();
  arrow.pose.orientation.y = q.y();
  arrow.pose.orientation.z = q.z();
  arrow.pose.orientation.w = q.w();
  arrow.scale.x = 1.0;
  arrow.scale.y = 0.15;
  arrow.scale.z = 0.15;
  arrow.color.r = 1.0f;
  arrow.color.g = 0.0f;
  arrow.color.b = 1.0f;
  arrow.color.a = 1.0f;
  arrow.lifetime = ros::Duration(10.0);
  target_marker_pub_.publish(arrow);

  // 2) Pallet box: virtual pallet size at placement pose (base_link: X=forward, Y=lateral, Z=up)
  visualization_msgs::Marker box;
  box.header.frame_id = base_frame_;
  box.header.stamp = stamp;
  box.ns = "wingbody_target";
  box.id = 1;
  box.type = visualization_msgs::Marker::CUBE;
  box.action = visualization_msgs::Marker::ADD;
  box.pose.position.x = result.x;
  box.pose.position.y = result.y;
  box.pose.position.z = result.z + pallet_height * 0.5;  // box base at z, center at z + half height
  box.pose.orientation.x = q.x();
  box.pose.orientation.y = q.y();
  box.pose.orientation.z = q.z();
  box.pose.orientation.w = q.w();
  box.scale.x = pallet_depth;   // depth along X (forward)
  box.scale.y = pallet_width;  // width along Y (lateral)
  box.scale.z = pallet_height; // height along Z
  box.color.r = 0.0f;
  box.color.g = 0.8f;
  box.color.b = 0.2f;
  box.color.a = 0.5f;
  box.lifetime = ros::Duration(10.0);
  target_marker_pub_.publish(box);
}

void VisualizationManager::clearMarkers() {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker delete_marker;
  delete_marker.header.frame_id = base_frame_;
  delete_marker.header.stamp = ros::Time::now();
  delete_marker.ns = "wingbody_planes";
  delete_marker.id = 0;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);
  marker_array_pub_.publish(marker_array);

  visualization_msgs::Marker target_delete;
  target_delete.header.frame_id = base_frame_;
  target_delete.header.stamp = ros::Time::now();
  target_delete.ns = "wingbody_target";
  target_delete.id = 0;
  target_delete.action = visualization_msgs::Marker::DELETEALL;
  target_marker_pub_.publish(target_delete);
}

} // namespace nc_wingbody_loader
