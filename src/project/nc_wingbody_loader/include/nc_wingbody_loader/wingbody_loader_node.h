#ifndef NC_WINGBODY_LOADER_NODE_H
#define NC_WINGBODY_LOADER_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "nc_wingbody_loader/detected_plane.h"
#include "nc_wingbody_loader/transform_manager.h"
#include "nc_wingbody_loader/plane_detector.h"
#include "nc_wingbody_loader/cargo_classifier.h"
#include "nc_wingbody_loader/pose_calculator.h"
#include "nc_wingbody_loader/visualization_manager.h"

#include <memory>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

enum NodeState {
  IDLE,
  DETECTING,
  STOPPED
};

class WingbodyLoaderNode {
public:
  WingbodyLoaderNode();

private:
  void loadParameters();

  // Callbacks
  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void cmdCallback(const std_msgs::String::ConstPtr& msg);

  // Processing
  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(
      const sensor_msgs::PointCloud2::ConstPtr& msg);

  void publishResult(const nc_wingbody_loader::PlacementResult& result,
                     const ros::Time& stamp);
  void publishStatus(const std::string& state, const std::string& detail);

  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Subscribers
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber cmd_sub_;

  // Publishers
  ros::Publisher pose_pub_;
  ros::Publisher status_pub_;
  ros::Publisher marker_array_pub_;
  ros::Publisher classified_pc_pub_;
  ros::Publisher filtered_pc_pub_;
  ros::Publisher target_marker_pub_;

  // Parameters - topics
  std::string pointcloud_topic_;
  std::string cmd_topic_;
  std::string pose_topic_;
  std::string status_topic_;
  std::string base_frame_;
  std::string camera_frame_;

  // Parameters - camera transform
  double camera_tf_x_, camera_tf_y_, camera_tf_z_, camera_tf_yaw_;

  // Parameters - pointcloud filtering
  double min_distance_, max_distance_;
  double min_height_, max_height_;
  double min_lateral_, max_lateral_;
  double voxel_leaf_size_;

  // Parameters - plane detection
  int max_planes_;
  double ransac_distance_threshold_;
  int ransac_max_iterations_;
  int min_plane_points_;

  // Parameters - classification
  double door_max_distance_;
  double cargo_width_, cargo_depth_;

  // Parameters - pose calculation
  double pallet_size_;
  double pallet_width_;
  double pallet_height_;
  double safety_margin_;
  double door_to_floor_offset_;

  // Parameters - detection
  int max_retry_attempts_;

  // State
  NodeState state_;
  int retry_count_;
  std::string loading_mode_;  // "first" or "next"

  // Managers
  std::unique_ptr<nc_wingbody_loader::TransformManager> transform_manager_;
  std::unique_ptr<nc_wingbody_loader::PlaneDetector> plane_detector_;
  std::unique_ptr<nc_wingbody_loader::CargoClassifier> cargo_classifier_;
  std::unique_ptr<nc_wingbody_loader::PoseCalculator> pose_calculator_;
  std::unique_ptr<nc_wingbody_loader::VisualizationManager> visualization_manager_;
};

#endif // NC_WINGBODY_LOADER_NODE_H
