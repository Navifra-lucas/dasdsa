#ifndef NC_INTENSITY_DETECTOR2_POINTCLOUD_DETECTOR_NODE_H
#define NC_INTENSITY_DETECTOR2_POINTCLOUD_DETECTOR_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include "nc_intensity_detector2/tape_cluster.h"
#include "nc_intensity_detector2/transform_manager.h"
#include "nc_intensity_detector2/visualization_manager.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <mutex>
#include <string>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class PointCloudDetectorNode {
public:
  PointCloudDetectorNode();
  
private:
  // Parameter loading
  void loadParameters();
  
  // Callbacks
  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void cmdCallback(const std_msgs::String::ConstPtr& msg);
  void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  
  // Point cloud processing
  std::vector<nc_intensity_detector2::TapeCluster> findTapeClusters(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  bool validateClusterSize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
                          double expected_width, double expected_height);
  bool fitPlaneToCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
                        double& yaw, double& distance,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& inlier_cloud,
                        bool optimize_coefficients = false);
  // Remove outer edge points from cluster (remove margin from boundary)
  pcl::PointCloud<pcl::PointXYZ>::Ptr removeClusterEdge(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
      double margin);
  nc_intensity_detector2::TapeCluster selectBestCluster(
      const std::vector<nc_intensity_detector2::TapeCluster>& clusters);
  void trackCluster(const nc_intensity_detector2::TapeCluster& cluster);

  // Image save directory initialization
  void initializeImageSaveDirectory();
  
  // High intensity point filtering
  void filterAndPublishHighIntensityPoints(const sensor_msgs::PointCloud2::ConstPtr& msg,
                                           const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_camera);
  
  // Image generation
  void saveImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  
  // Pose calculation
  geometry_msgs::PoseStamped calculateGlobalPose(const geometry_msgs::PoseStamped& local_pose);
  
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  
  // Subscribers
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber robot_pose_sub_;
  
  // Publishers
  ros::Publisher pose_pub_;
  ros::Publisher global_pose_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher all_clusters_pc_pub_;
  ros::Publisher selected_cluster_pc_pub_;
  ros::Publisher plane_inliers_pc_pub_;
  ros::Publisher high_intensity_pc_pub_;

  ros::Publisher filtered_pc_pub_;  // Filtered point cloud for visualization
  // image_transport::Publisher wall_image_pub_;
  
  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  // Parameters
  std::string pointcloud_topic_;
  std::string pose_topic_;
  std::string global_pose_topic_;
  std::string cmd_topic_;
  std::string marker_topic_;
  std::string all_clusters_pc_topic_;
  std::string selected_cluster_pc_topic_;
  std::string plane_inliers_pc_topic_;
  std::string high_intensity_pc_topic_;
  std::string filtered_pc_topic_;
  std::string base_frame_;
  std::string camera_frame_;
  
  // Tape dimensions (meters)
  double tape_width_;   // 0.60m
  double tape_height_;  // 0.40m
  double tape_size_tolerance_;  // 크기 허용 오차
  
  // Point cloud filtering
  double min_distance_;
  double max_distance_;
  double min_height_;  // 최소 높이 (Y축, 카메라 좌표계)
  double max_height_;  // 최대 높이 (Y축, 카메라 좌표계)
  double min_lateral_;  // 최소 좌우 거리 (X축, 카메라 좌표계)
  double max_lateral_;  // 최대 좌우 거리 (X축, 카메라 좌표계)
  double cluster_tolerance_;  // 클러스터링 거리 임계값
  int min_cluster_size_;
  int max_cluster_size_;
  
  // Camera transform (base_link -> camera)
  double camera_tf_x_;   // Translation X
  double camera_tf_y_;   // Translation Y
  double camera_tf_z_;   // Translation Z
  double camera_tf_yaw_;  // Rotation Yaw
  
  // Tracking
  bool has_previous_cluster_;
  pcl::PointXYZ previous_center_;
  double tracking_distance_threshold_;
  
  // Yaw filtering
  bool has_previous_yaw_;
  double previous_yaw_;
  double yaw_filter_alpha_;  // Low-pass filter coefficient (0.0 = no filtering, 1.0 = no smoothing)
  
  // Robot pose
  geometry_msgs::PoseWithCovarianceStamped robot_pose_;
  bool has_robot_pose_;
  std::mutex robot_pose_mutex_;
  
  // Detection state
  bool detection_enabled_;
  // Image saving
  ros::Time last_image_save_time_;
  std::string image_save_dir_;
  
  // Helper function for yaw filtering
  double filterYaw(double raw_yaw);
  
  // Managers
  std::unique_ptr<nc_intensity_detector2::TransformManager> transform_manager_;
  std::unique_ptr<nc_intensity_detector2::VisualizationManager> visualization_manager_;
};

#endif // NC_INTENSITY_DETECTOR2_POINTCLOUD_DETECTOR_NODE_H

