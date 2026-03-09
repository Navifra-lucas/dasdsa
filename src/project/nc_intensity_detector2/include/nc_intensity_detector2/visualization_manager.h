#ifndef NC_INTENSITY_DETECTOR2_VISUALIZATION_MANAGER_H
#define NC_INTENSITY_DETECTOR2_VISUALIZATION_MANAGER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "nc_intensity_detector2/tape_cluster.h"
#include <string>
#include <vector>

namespace nc_intensity_detector2 {

class VisualizationManager {
public:
  VisualizationManager(ros::Publisher& marker_pub, 
                      ros::Publisher& all_clusters_pc_pub,
                      ros::Publisher& selected_cluster_pc_pub,
                      ros::Publisher& plane_inliers_pc_pub,
                      ros::Publisher& high_intensity_pc_pub,
                      const std::string& base_frame,
                      double camera_tf_x, double camera_tf_y, double camera_tf_yaw);
  
  // Publish marker for best cluster (in base_link)
  void publishMarker(const TapeCluster& cluster);
  
  // Publish markers for all clusters (in base_link)
  void publishClusterMarkers(const std::vector<TapeCluster>& clusters);
  
  // Publish point cloud for all clusters (in base_link)
  void publishAllClustersPointCloud(const std::vector<TapeCluster>& clusters, 
                                    const ros::Time& stamp);
  
  // Publish point cloud for selected cluster (in base_link)
  void publishSelectedClusterPointCloud(const TapeCluster& cluster,
                                        const ros::Time& stamp);
  
  // Publish point cloud for plane inliers (in base_link)
  void publishPlaneInliersPointCloud(const TapeCluster& cluster,
                                     const ros::Time& stamp);
  
  // Clear markers
  void clearMarker();
  
  // Transform point cloud from camera frame to base_link (public for use in node)
  void transformPointCloudToBase(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_camera,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_base);

private:
  
  // Apply voxel grid filter for visualization
  void applyVoxelFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& output);
  
  ros::Publisher& marker_pub_;
  ros::Publisher& all_clusters_pc_pub_;
  ros::Publisher& selected_cluster_pc_pub_;
  ros::Publisher& plane_inliers_pc_pub_;
  std::string base_frame_;
  double camera_tf_x_;
  double camera_tf_y_;
  double camera_tf_yaw_;
  int marker_id_;
};

} // namespace nc_intensity_detector2

#endif // NC_INTENSITY_DETECTOR2_VISUALIZATION_MANAGER_H

