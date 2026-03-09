#include "nc_wingbody_loader/wingbody_loader_node.h"
#include "nc_wingbody_loader/json.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <util/logger.hpp>
#include <cmath>

using json = nlohmann::json;

WingbodyLoaderNode::WingbodyLoaderNode()
  : nh_("~"), pnh_("~"), state_(IDLE), retry_count_(0), loading_mode_("first") {

  loadParameters();

  // Initialize managers
  transform_manager_ = std::make_unique<nc_wingbody_loader::TransformManager>(
      camera_tf_x_, camera_tf_y_, camera_tf_z_, camera_tf_yaw_);

  plane_detector_ = std::make_unique<nc_wingbody_loader::PlaneDetector>(
      max_planes_, ransac_distance_threshold_, ransac_max_iterations_, min_plane_points_);

  cargo_classifier_ = std::make_unique<nc_wingbody_loader::CargoClassifier>(
      door_max_distance_, cargo_width_, cargo_depth_, pallet_size_);

  // Publishers (create before VisualizationManager)
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 1);
  status_pub_ = nh_.advertise<std_msgs::String>(status_topic_, 1);
  marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/nc_wingbody_loader/plane_markers", 1);
  classified_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/nc_wingbody_loader/classified_pc", 1);
  filtered_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/nc_wingbody_loader/filtered_pc", 1);
  target_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "/nc_wingbody_loader/target_marker", 1);

  pose_calculator_ = std::make_unique<nc_wingbody_loader::PoseCalculator>(
      pallet_size_, safety_margin_, door_to_floor_offset_, cargo_depth_,
      transform_manager_.get());

  visualization_manager_ = std::make_unique<nc_wingbody_loader::VisualizationManager>(
      marker_array_pub_, classified_pc_pub_, filtered_pc_pub_, target_marker_pub_,
      base_frame_, transform_manager_.get());

  // Subscribers
  pointcloud_sub_ = nh_.subscribe(pointcloud_topic_, 1,
                                   &WingbodyLoaderNode::pointcloudCallback, this);
  cmd_sub_ = nh_.subscribe(cmd_topic_, 1,
                            &WingbodyLoaderNode::cmdCallback, this);

  LOG_INFO("nc_wingbody_loader node initialized");
  LOG_INFO("Subscribing to pointcloud: %s", pointcloud_topic_.c_str());
  LOG_INFO("Subscribing to cmd: %s", cmd_topic_.c_str());
  LOG_INFO("Publishing pose to: %s", pose_topic_.c_str());
  LOG_INFO("Publishing status to: %s", status_topic_.c_str());
}

void WingbodyLoaderNode::loadParameters() {
  // Topics
  pnh_.param<std::string>("topics/pointcloud", pointcloud_topic_, "/sick_visionary_t_mini_bt_bottom/points");
  pnh_.param<std::string>("topics/cmd", cmd_topic_, "/nc_wingbody_loader/cmd");
  pnh_.param<std::string>("topics/pose", pose_topic_, "/nc_wingbody_loader/pose");
  pnh_.param<std::string>("topics/status", status_topic_, "/nc_wingbody_loader/status");

  // Frames
  pnh_.param<std::string>("frames/base", base_frame_, "base_link");
  pnh_.param<std::string>("frames/camera", camera_frame_, "sick_visionary_t_mini_bt_bottom");

  // Camera transform
  pnh_.param<double>("frames/camera_tf/x", camera_tf_x_, 0.0);
  pnh_.param<double>("frames/camera_tf/y", camera_tf_y_, 0.0);
  pnh_.param<double>("frames/camera_tf/z", camera_tf_z_, 0.0);
  pnh_.param<double>("frames/camera_tf/yaw", camera_tf_yaw_, 0.0);

  // Point cloud filtering
  pnh_.param<double>("pointcloud/min_distance", min_distance_, 1.0);
  pnh_.param<double>("pointcloud/max_distance", max_distance_, 12.0);
  pnh_.param<double>("pointcloud/min_height", min_height_, -2.0);
  pnh_.param<double>("pointcloud/max_height", max_height_, 2.0);
  pnh_.param<double>("pointcloud/min_lateral", min_lateral_, -2.5);
  pnh_.param<double>("pointcloud/max_lateral", max_lateral_, 2.5);
  pnh_.param<double>("pointcloud/voxel_leaf_size", voxel_leaf_size_, 0.02);

  // Plane detection
  pnh_.param<int>("plane_detection/max_planes", max_planes_, 10);
  pnh_.param<double>("plane_detection/ransac_distance_threshold", ransac_distance_threshold_, 0.03);
  pnh_.param<int>("plane_detection/ransac_max_iterations", ransac_max_iterations_, 1000);
  pnh_.param<int>("plane_detection/min_plane_points", min_plane_points_, 50);

  // Classification
  pnh_.param<double>("classification/door_max_distance", door_max_distance_, 5.0);
  pnh_.param<double>("classification/cargo_width", cargo_width_, 2.35);
  pnh_.param<double>("classification/cargo_depth", cargo_depth_, 9.5);

  // Pose calculation
  pnh_.param<double>("pose_calculation/pallet_size", pallet_size_, 2.2);
  pnh_.param<double>("pose_calculation/pallet_width", pallet_width_, 1.2);
  pnh_.param<double>("pose_calculation/pallet_height", pallet_height_, 0.15);
  pnh_.param<double>("pose_calculation/safety_margin", safety_margin_, 0.1);
  pnh_.param<double>("pose_calculation/door_to_floor_offset", door_to_floor_offset_, 0.05);

  // Detection
  pnh_.param<int>("detection/max_retry_attempts", max_retry_attempts_, 3);

  LOG_INFO("Parameters loaded: distance=[%.1f,%.1f], voxel=%.3f, max_planes=%d, ransac_thresh=%.3f",
           min_distance_, max_distance_, voxel_leaf_size_, max_planes_, ransac_distance_threshold_);
}

void WingbodyLoaderNode::cmdCallback(const std_msgs::String::ConstPtr& msg) {
  try {
    json cmd_json = json::parse(msg->data);

    std::string command = cmd_json.value("command", "");
    loading_mode_ = cmd_json.value("loading_mode", "first");

    if (command == "start") {
      state_ = DETECTING;
      retry_count_ = 0;
      LOG_INFO("CMD: start (loading_mode=%s)", loading_mode_.c_str());
      publishStatus("detecting", "Detection started");
    } else if (command == "stop") {
      state_ = IDLE;
      retry_count_ = 0;
      visualization_manager_->clearMarkers();
      LOG_INFO("CMD: stop");
      publishStatus("idle", "Detection stopped");
    } else {
      LOG_WARNING("CMD: Unknown command '%s'", command.c_str());
    }
  } catch (const json::parse_error& e) {
    // Fallback: treat as plain text command
    std::string cmd = msg->data;
    if (cmd == "start") {
      state_ = DETECTING;
      retry_count_ = 0;
      loading_mode_ = "first";
      LOG_INFO("CMD: start (plain text, loading_mode=first)");
      publishStatus("detecting", "Detection started");
    } else if (cmd == "stop") {
      state_ = IDLE;
      retry_count_ = 0;
      visualization_manager_->clearMarkers();
      LOG_INFO("CMD: stop (plain text)");
      publishStatus("idle", "Detection stopped");
    }
  }
}

void WingbodyLoaderNode::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  if (state_ != DETECTING) {
    return;
  }

  try {
    ros::Time t_start = ros::Time::now();

    // Preprocess
    ros::Time t_pre = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = preprocessPointCloud(msg);
    double dt_preprocess = (ros::Time::now() - t_pre).toSec() * 1000.0;

    if (!cloud || cloud->empty()) {
      LOG_WARNING("PointCloud: Empty after preprocessing");
      retry_count_++;
      if (retry_count_ >= max_retry_attempts_) {
        state_ = STOPPED;
        publishStatus("fail", "Empty point cloud after preprocessing");
        LOG_ERROR("Detection FAILED: Empty point cloud after %d retries", retry_count_);
      }
      return;
    }

    // Publish filtered point cloud for debug
    visualization_manager_->publishFilteredPointCloud(cloud, msg->header.stamp);

    // Detect planes
    ros::Time t_detect = ros::Time::now();
    auto planes = plane_detector_->detectPlanes(cloud);
    double dt_detect = (ros::Time::now() - t_detect).toSec() * 1000.0;

    if (planes.empty()) {
      LOG_WARNING("PointCloud: No planes detected");
      retry_count_++;
      if (retry_count_ >= max_retry_attempts_) {
        state_ = STOPPED;
        publishStatus("fail", "No planes detected");
        LOG_ERROR("Detection FAILED: No planes after %d retries", retry_count_);
      }
      return;
    }

    // Classify planes
    ros::Time t_classify = ros::Time::now();
    cargo_classifier_->classify(planes);
    double dt_classify = (ros::Time::now() - t_classify).toSec() * 1000.0;

    if (!cargo_classifier_->getDoor()) {
      LOG_WARNING("PointCloud: No door found in classification");
      retry_count_++;
      if (retry_count_ >= max_retry_attempts_) {
        state_ = STOPPED;
        publishStatus("fail", "No door detected");
        LOG_ERROR("Detection FAILED: No door after %d retries", retry_count_);
      }
      return;
    }

    // Publish visualization
    visualization_manager_->publishPlaneMarkers(planes, msg->header.stamp);
    visualization_manager_->publishClassifiedPointCloud(planes, msg->header.stamp);

    // Calculate placement pose
    ros::Time t_pose = ros::Time::now();
    auto result = pose_calculator_->calculate(*cargo_classifier_, loading_mode_);
    double dt_pose = (ros::Time::now() - t_pose).toSec() * 1000.0;

    if (!result.success) {
      LOG_WARNING("PointCloud: Pose calculation failed - %s", result.error_msg.c_str());
      retry_count_++;
      if (retry_count_ >= max_retry_attempts_) {
        state_ = STOPPED;
        publishStatus("fail", result.error_msg);
        LOG_ERROR("Detection FAILED: %s after %d retries", result.error_msg.c_str(), retry_count_);
      }
      return;
    }

    // Success
    publishResult(result, msg->header.stamp);
    visualization_manager_->publishTargetMarker(result, msg->header.stamp,
                                                pallet_size_, pallet_width_, pallet_height_);
    publishStatus("success", "Placement pose calculated");
    state_ = IDLE;

    double dt_total = (ros::Time::now() - t_start).toSec() * 1000.0;
    LOG_INFO("Detection SUCCESS: x=%.3f, y=%.3f, z=%.3f, yaw=%.1fdeg (timing: pre=%.1f, detect=%.1f, classify=%.1f, pose=%.1f, total=%.1fms)",
             result.x, result.y, result.z, result.yaw * 180.0 / M_PI,
             dt_preprocess, dt_detect, dt_classify, dt_pose, dt_total);

  } catch (const std::exception& e) {
    LOG_ERROR("PointCloud processing error: %s", e.what());
    retry_count_++;
    if (retry_count_ >= max_retry_attempts_) {
      state_ = STOPPED;
      publishStatus("fail", std::string("Exception: ") + e.what());
    }
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr WingbodyLoaderNode::preprocessPointCloud(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {

  // Convert ROS -> PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  if (cloud->empty()) return nullptr;

  // Transform to camera frame if needed
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);
  if (msg->header.frame_id != camera_frame_) {
    if (msg->header.frame_id == base_frame_) {
      transform_manager_->transformPointCloudToCamera(cloud, cloud_camera);
    } else {
      LOG_WARNING("Cannot transform from %s (only %s supported)",
                  msg->header.frame_id.c_str(), base_frame_.c_str());
      return nullptr;
    }
  } else {
    cloud_camera = cloud;
  }

  // Remove NaN
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_valid(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_camera, *cloud_valid, indices);

  if (cloud_valid->empty()) return nullptr;

  // PassThrough Z (forward distance)
  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud(cloud_valid);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(min_distance_, max_distance_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z(new pcl::PointCloud<pcl::PointXYZ>);
  pass_z.filter(*cloud_z);

  if (cloud_z->empty()) return nullptr;

  // PassThrough Y (height)
  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud(cloud_z);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(min_height_, max_height_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y(new pcl::PointCloud<pcl::PointXYZ>);
  pass_y.filter(*cloud_y);

  if (cloud_y->empty()) return nullptr;

  // PassThrough X (lateral)
  pcl::PassThrough<pcl::PointXYZ> pass_x;
  pass_x.setInputCloud(cloud_y);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(min_lateral_, max_lateral_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pass_x.filter(*cloud_filtered);

  if (cloud_filtered->empty()) return nullptr;

  // VoxelGrid downsampling
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud_filtered);
  float leaf = static_cast<float>(voxel_leaf_size_);
  voxel.setLeafSize(leaf, leaf, leaf);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  voxel.filter(*cloud_downsampled);

  LOG_DEBUG("Preprocess: %zu -> nan=%zu -> z=%zu -> y=%zu -> x=%zu -> voxel=%zu",
            cloud->size(), cloud_valid->size(), cloud_z->size(),
            cloud_y->size(), cloud_filtered->size(), cloud_downsampled->size());

  return cloud_downsampled;
}

void WingbodyLoaderNode::publishResult(const nc_wingbody_loader::PlacementResult& result,
                                        const ros::Time& stamp) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = base_frame_;
  pose_msg.header.stamp = stamp;
  pose_msg.pose.position.x = result.x;
  pose_msg.pose.position.y = result.y;
  pose_msg.pose.position.z = result.z;

  tf2::Quaternion q;
  q.setRPY(0, 0, result.yaw);
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  pose_pub_.publish(pose_msg);
}

void WingbodyLoaderNode::publishStatus(const std::string& state, const std::string& detail) {
  json status;
  status["state"] = state;
  status["detail"] = detail;
  status["retry_count"] = retry_count_;

  std_msgs::String msg;
  msg.data = status.dump();
  status_pub_.publish(msg);
}

int main(int argc, char** argv) {
  NaviFra::Logger::get().SetSeverityMin(severity_level::info);
  ros::init(argc, argv, "nc_wingbody_loader_node");

  WingbodyLoaderNode node;

  ros::spin();

  return 0;
}
