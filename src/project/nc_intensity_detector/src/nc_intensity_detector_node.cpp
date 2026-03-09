#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <mutex>
#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>
#include <util/logger.hpp>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <dirent.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "nc_intensity_detector/intensity_detector.h"

// Structure to store statistics for each test case
struct TestCaseStats {
  std::string name;
  std::vector<double> distances;
  std::vector<double> angles;
  std::vector<double> x_positions;
  std::vector<double> y_positions;
  std::vector<double> z_positions;
  size_t count;
  
  TestCaseStats() : count(0) {}
  TestCaseStats(const std::string& n) : name(n), count(0) {}
};

class IntensityDetectorNode {
public:
  IntensityDetectorNode() : nh_("~"), it_(nh_), tf_listener_(tf_buffer_), detection_active_(false),
                           stats_count_(0), last_stats_time_(ros::Time::now()),
                           saved_image_count_(0), detection_frame_count_(0), last_save_time_(ros::Time(0)),
                           image_save_interval_(5),
                           filtered_distance_(0.0), filtered_angle_(0.0), filtered_y_(0.0),
                           has_previous_measurement_(false), temporal_filter_alpha_(0.7),
                           y_deadzone_(0.01), yaw_deadzone_(0.5), max_y_rate_(0.05), max_yaw_rate_(2.0),
                           current_test_case_index_(-1),
                           distortion_k1_(0.0), distortion_k2_(0.0), distortion_k3_(0.0),
                           distortion_p1_(0.0), distortion_p2_(0.0),
                           height_correction_slope_(0.0), height_correction_offset_(0.0),
                           reference_height_(0.0), y_position_offset_(0.0), y_scale_factor_(1.0),
                           pnp_x_reference_distance_(1.77), pnp_x_error_per_meter_(0.1),
                           pnp_y_error_per_meter_(0.05),
                           has_robot_pose_(false),
                           camera_tf_x_(-0.363), camera_tf_y_(0.05), camera_tf_yaw_(M_PI),
                           use_odom_correction_(false), has_odom_(false),
                           odom_correction_gain_(1.0), odom_angular_vel_threshold_(0.01),
                           odom_time_diff_threshold_(0.001), odom_max_yaw_correction_(0.5),
                           odom_filter_alpha_(0.7), has_filtered_angular_vel_(false),
                           odom_max_angular_accel_(2.0), previous_angular_vel_(0.0), last_odom_time_(ros::Time(0)),
                           f_target_angle_(0.0) {
    loadParameters();
    initializeDetector();
    initializeImageSaveDirectory();
    
    // Subscribers
    intensity_sub_ = it_.subscribe(intensity_topic_, 1, 
                                   &IntensityDetectorNode::intensityCallback, this);
    cmd_sub_ = nh_.subscribe(cmd_topic_, 1, 
                             &IntensityDetectorNode::cmdCallback, this);
    robot_pose_sub_ = nh_.subscribe("/localization/robot_pos", 1,
                                     &IntensityDetectorNode::robotPoseCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 1,
                              &IntensityDetectorNode::odomCallback, this);
    
    // Publishers
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(output_topic_, 1);
    global_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/nc_intensity_detector/global_pose", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/nc_intensity_detector/marker", 1);
    
    // Subscriber for resetting statistics
    reset_stats_sub_ = nh_.subscribe("/nc_intensity_detector/reset_stats", 1,
                                     &IntensityDetectorNode::resetStatsCallback, this);
    
    LOG_INFO("nc_intensity_detector node initialized");
    LOG_INFO("Subscribing to intensity: %s", intensity_topic_.c_str());
    LOG_INFO("Subscribing to cmd: %s", cmd_topic_.c_str());
    LOG_INFO("Subscribing to robot pose: /localization/robot_pos");
    LOG_INFO("Publishing to: %s", output_topic_.c_str());
    LOG_INFO("Publishing global pose to: /nc_intensity_detector/global_pose");
    LOG_INFO("Detection is currently: %s", detection_active_ ? "ACTIVE" : "INACTIVE");
    LOG_INFO("Image save interval: %zu (1 = every frame, 5 = every 5th frame)", image_save_interval_);
  }
  
  // Destructor to print final results when node shuts down
  ~IntensityDetectorNode() {
    // Note: Don't save here - test cases are already saved by resetStatsCallback
    // Print final statistics for all test cases when node is shutting down
    printAllTestCasesStatistics();
  }

private:
  void loadParameters() {
    // Get parameter file path
    std::string param_file;
    nh_.param<std::string>("param_file", param_file, "");
    
    if (!param_file.empty()) {
      loadParametersFromFile(param_file);
    } else {
      loadParametersFromROS();
    }
  }

  void loadParametersFromFile(const std::string& file_path) {
    try {
      YAML::Node config = YAML::LoadFile(file_path);
      
      // Topic parameters
      intensity_topic_ = config["topics"]["intensity"].as<std::string>(intensity_topic_);
      output_topic_ = config["topics"]["output"].as<std::string>(output_topic_);
      cmd_topic_ = config["topics"]["cmd"].as<std::string>(cmd_topic_);
      camera_frame_ = config["frames"]["camera"].as<std::string>(camera_frame_);
      base_frame_ = config["frames"]["base"].as<std::string>(base_frame_);
      
      // Camera TF parameters (base_link to camera frame)
      if (config["frames"]["camera_tf"]) {
        camera_tf_x_ = config["frames"]["camera_tf"]["x"].as<double>(camera_tf_x_);
        camera_tf_y_ = config["frames"]["camera_tf"]["y"].as<double>(camera_tf_y_);
        camera_tf_yaw_ = config["frames"]["camera_tf"]["yaw"].as<double>(camera_tf_yaw_);
      }
      
      // Detection parameters
      intensity_threshold_ = config["detection"]["intensity_threshold"].as<double>(intensity_threshold_);
      min_visible_ratio_ = config["detection"]["min_visible_ratio"].as<double>(min_visible_ratio_);
      crop_top_ratio_ = config["detection"]["crop_top_ratio"].as<double>(0.0);
      crop_bottom_ratio_ = config["detection"]["crop_bottom_ratio"].as<double>(0.0);
      tape_width_ = config["tape"]["width"].as<double>(tape_width_);
      tape_height_ = config["tape"]["height"].as<double>(tape_height_);
      tape_thickness_ = config["tape"]["thickness"].as<double>(tape_thickness_);
      min_distance_ = config["distance"]["min"].as<double>(min_distance_);
      max_distance_ = config["distance"]["max"].as<double>(max_distance_);
      
      // Camera parameters
      focal_length_x_ = config["camera"]["focal_length_x"].as<double>(focal_length_x_);
      focal_length_y_ = config["camera"]["focal_length_y"].as<double>(focal_length_y_);
      principal_point_x_ = config["camera"]["principal_point_x"].as<double>(principal_point_x_);
      principal_point_y_ = config["camera"]["principal_point_y"].as<double>(principal_point_y_);
      
      // Camera distortion coefficients
      if (config["camera"]["distortion"]) {
        distortion_k1_ = config["camera"]["distortion"]["k1"].as<double>(0.0);
        distortion_k2_ = config["camera"]["distortion"]["k2"].as<double>(0.0);
        distortion_k3_ = config["camera"]["distortion"]["k3"].as<double>(0.0);
        distortion_p1_ = config["camera"]["distortion"]["p1"].as<double>(0.0);
        distortion_p2_ = config["camera"]["distortion"]["p2"].as<double>(0.0);
      } else {
        distortion_k1_ = 0.0;
        distortion_k2_ = 0.0;
        distortion_k3_ = 0.0;
        distortion_p1_ = 0.0;
        distortion_p2_ = 0.0;
      }
      
      // Calibration parameters
      distance_calibration_factor_ = config["calibration"]["distance_calibration_factor"].as<double>(1.0);
      angle_calibration_offset_ = config["calibration"]["angle_calibration_offset"].as<double>(0.0);
      pnp_yaw_multiplier_ = config["calibration"]["pnp_yaw_multiplier"].as<double>(5.0);
      y_position_offset_ = config["calibration"]["y_position_offset"].as<double>(0.0);
      y_scale_factor_ = config["calibration"]["y_scale_factor"].as<double>(1.0);
      max_y_change_ = config["calibration"]["max_y_change"].as<double>(0.3);
      occlusion_distance_penalty_ = config["calibration"]["occlusion_distance_penalty"].as<double>(0.0);
      temporal_filter_alpha_ = config["calibration"]["temporal_filter_alpha"].as<double>(0.7);
      
      // PnP position error correction (distance-dependent)
      // 기준 거리: 오차 보정을 시작하는 거리 (이 거리 이상에서만 보정 적용)
      // x 오차율: 거리 1m당 x를 몇 m 보정할지 (예: 0.1 = 1m당 0.1m 보정)
      // y 오차율: 거리 1m당 y를 몇 m 보정할지 (예: 0.05 = 1m당 0.05m 보정)
      pnp_x_reference_distance_ = config["calibration"]["pnp_x_reference_distance"].as<double>(1.77);
      pnp_x_error_per_meter_ = config["calibration"]["pnp_x_error_per_meter"].as<double>(0.1);
      pnp_y_error_per_meter_ = config["calibration"]["pnp_y_error_per_meter"].as<double>(0.05);
      
      // Deadzone and rate limiting parameters
      y_deadzone_ = config["calibration"]["y_deadzone"].as<double>(0.01);
      yaw_deadzone_ = config["calibration"]["yaw_deadzone"].as<double>(0.5);
      max_y_rate_ = config["calibration"]["max_y_rate"].as<double>(0.05);
      max_yaw_rate_ = config["calibration"]["max_yaw_rate"].as<double>(2.0);
      
      // Height correction parameters
      height_correction_slope_ = config["calibration"]["height_correction_slope"].as<double>(0.0);
      height_correction_offset_ = config["calibration"]["height_correction_offset"].as<double>(0.0);
      reference_height_ = config["calibration"]["reference_height"].as<double>(0.0);
      
      // Odom-based angular velocity correction
      use_odom_correction_ = config["calibration"]["use_odom_correction"].as<bool>(false);
      odom_correction_gain_ = config["calibration"]["odom_correction_gain"].as<double>(1.0);
      odom_angular_vel_threshold_ = config["calibration"]["odom_angular_vel_threshold"].as<double>(0.01);
      odom_time_diff_threshold_ = config["calibration"]["odom_time_diff_threshold"].as<double>(0.001);
      odom_max_yaw_correction_ = config["calibration"]["odom_max_yaw_correction"].as<double>(0.5);
      odom_filter_alpha_ = config["calibration"]["odom_filter_alpha"].as<double>(0.7);
      
      // Image save parameters
      if (config["image_save"]) {
        if (config["image_save"]["interval"]) {
          int temp_interval = config["image_save"]["interval"].as<int>();
          image_save_interval_ = static_cast<size_t>(temp_interval);
          LOG_INFO("Loaded image_save_interval from file: %zu (raw value: %d)", image_save_interval_, temp_interval);
        } else {
          LOG_WARNING("image_save.interval not found in config, using default: %zu", image_save_interval_);
        }
      } else {
        LOG_WARNING("image_save section not found in config, using default: %zu", image_save_interval_);
      }
      
      LOG_INFO("Parameters loaded from file: %s", file_path.c_str());
    } catch (const std::exception& e) {
      LOG_WARNING("Failed to load parameters from file: %s. Using ROS parameters or defaults.", e.what());
      loadParametersFromROS();
    }
  }

  void loadParametersFromROS() {
    // Topic parameters
    nh_.param<std::string>("intensity_topic", intensity_topic_, "/sick_visionary_t_mini/intensity");
    nh_.param<std::string>("output_topic", output_topic_, "/nc_intensity_detector/pose");
    nh_.param<std::string>("cmd_topic", cmd_topic_, "/nc_intensity_detector/cmd");
    nh_.param<std::string>("camera_frame", camera_frame_, "sick_visionary_t_mini");
    nh_.param<std::string>("base_frame", base_frame_, "base_link");
    
    // Camera TF parameters (base_link to camera frame)
    nh_.param<double>("camera_tf_x", camera_tf_x_, -0.363);
    nh_.param<double>("camera_tf_y", camera_tf_y_, 0.05);
    nh_.param<double>("camera_tf_yaw", camera_tf_yaw_, M_PI);  // 180 degrees in radians
    
    // Detection parameters
    nh_.param<double>("intensity_threshold", intensity_threshold_, 5000.0);
    nh_.param<double>("min_visible_ratio", min_visible_ratio_, 0.5);
    nh_.param<double>("crop_top_ratio", crop_top_ratio_, 0.0);
    nh_.param<double>("crop_bottom_ratio", crop_bottom_ratio_, 0.0);
    nh_.param<double>("tape_width", tape_width_, 0.70);
    nh_.param<double>("tape_height", tape_height_, 0.50);
    nh_.param<double>("tape_thickness", tape_thickness_, 0.05);
    nh_.param<double>("distance/min", min_distance_, 1.0);
    nh_.param<double>("distance/max", max_distance_, 3.0);
    
    // Camera parameters (YAML structure: camera/focal_length_x)
    nh_.param<double>("camera/focal_length_x", focal_length_x_, 365.6);  // Default from 70deg FOV
    nh_.param<double>("camera/focal_length_y", focal_length_y_, 367.2);  // Default from 60deg FOV
    nh_.param<double>("camera/principal_point_x", principal_point_x_, 0.0);
    nh_.param<double>("camera/principal_point_y", principal_point_y_, 0.0);
    
    // Camera distortion coefficients
    nh_.param<double>("camera/distortion/k1", distortion_k1_, 0.0);
    nh_.param<double>("camera/distortion/k2", distortion_k2_, 0.0);
    nh_.param<double>("camera/distortion/k3", distortion_k3_, 0.0);
    nh_.param<double>("camera/distortion/p1", distortion_p1_, 0.0);
    nh_.param<double>("camera/distortion/p2", distortion_p2_, 0.0);
    
    // Calibration parameters
    nh_.param<double>("calibration/distance_calibration_factor", distance_calibration_factor_, 1.0);
    nh_.param<double>("calibration/angle_calibration_offset", angle_calibration_offset_, 0.0);
    nh_.param<double>("calibration/pnp_yaw_multiplier", pnp_yaw_multiplier_, 5.0);
    nh_.param<double>("calibration/y_position_offset", y_position_offset_, 0.0);
    nh_.param<double>("calibration/y_scale_factor", y_scale_factor_, 1.0);
    nh_.param<double>("calibration/max_y_change", max_y_change_, 0.3);
    nh_.param<double>("calibration/occlusion_distance_penalty", occlusion_distance_penalty_, 0.0);
    nh_.param<double>("calibration/temporal_filter_alpha", temporal_filter_alpha_, 0.7);
    
    // PnP position error correction (distance-dependent)
    // 기준 거리: 오차 보정을 시작하는 거리 (이 거리 이상에서만 보정 적용)
    // x 오차율: 거리 1m당 x를 몇 m 보정할지 (예: 0.1 = 1m당 0.1m 보정)
    // y 오차율: 거리 1m당 y를 몇 m 보정할지 (예: 0.05 = 1m당 0.05m 보정)
    nh_.param<double>("calibration/pnp_x_reference_distance", pnp_x_reference_distance_, 1.77);
    nh_.param<double>("calibration/pnp_x_error_per_meter", pnp_x_error_per_meter_, 0.1);
    nh_.param<double>("calibration/pnp_y_error_per_meter", pnp_y_error_per_meter_, 0.05);
    
    // Deadzone and rate limiting parameters
    nh_.param<double>("calibration/y_deadzone", y_deadzone_, 0.01);
    nh_.param<double>("calibration/yaw_deadzone", yaw_deadzone_, 0.5);
    nh_.param<double>("calibration/max_y_rate", max_y_rate_, 0.05);
    nh_.param<double>("calibration/max_yaw_rate", max_yaw_rate_, 2.0);
    
    // Image save parameters
    int temp_interval = 5;
    nh_.param<int>("image_save/interval", temp_interval, 5);
    image_save_interval_ = static_cast<size_t>(temp_interval);
    LOG_INFO("Loaded image_save_interval from ROS: %zu (raw value: %d)", image_save_interval_, temp_interval);
    
    // Height correction parameters
    nh_.param<double>("calibration/height_correction_slope", height_correction_slope_, 0.0);
    nh_.param<double>("calibration/height_correction_offset", height_correction_offset_, 0.0);
    nh_.param<double>("calibration/reference_height", reference_height_, 0.0);
    
    // Odom-based angular velocity correction
    nh_.param<bool>("calibration/use_odom_correction", use_odom_correction_, false);
    nh_.param<double>("calibration/odom_correction_gain", odom_correction_gain_, 1.0);
    nh_.param<double>("calibration/odom_angular_vel_threshold", odom_angular_vel_threshold_, 0.01);
    nh_.param<double>("calibration/odom_time_diff_threshold", odom_time_diff_threshold_, 0.001);
    nh_.param<double>("calibration/odom_max_yaw_correction", odom_max_yaw_correction_, 0.5);  // Maximum correction in radians (default: ~28.6 degrees)
  }

  void initializeDetector() {
    detector_.initialize(intensity_threshold_,
                         tape_width_,
                         tape_height_,
                         tape_thickness_,
                         min_distance_,
                         max_distance_,
                         focal_length_x_,
                         focal_length_y_,
                         principal_point_x_,
                         principal_point_y_,
                         min_visible_ratio_,
                         distance_calibration_factor_,
                         angle_calibration_offset_,
                         pnp_yaw_multiplier_,
                         y_position_offset_,
                         y_scale_factor_,
                         max_y_change_,
                         occlusion_distance_penalty_,
                         crop_top_ratio_,
                         crop_bottom_ratio_,
                         distortion_k1_,
                         distortion_k2_,
                         distortion_k3_,
                         distortion_p1_,
                         distortion_p2_,
                         height_correction_slope_,
                         height_correction_offset_,
                         reference_height_);
  }

  void initializeImageSaveDirectory() {
    // Create directory for saving images
    const char* home_env = getenv("HOME");
    if (home_env == nullptr) {
      // Fallback to current directory
      image_save_dir_ = "./nc_intensity_detector_images";
      LOG_WARNING("HOME environment variable not set, using current directory");
    } else {
      image_save_dir_ = std::string(home_env) + "/navifra_solution/navicore/nc_intensity_detector_images";
    }
    
    // Create directory if it doesn't exist
    struct stat info;
    if (stat(image_save_dir_.c_str(), &info) != 0) {
      // Directory doesn't exist, create it
      std::string cmd = "mkdir -p " + image_save_dir_;
      int result = system(cmd.c_str());
      if (result == 0) {
        LOG_INFO("Created image save directory: %s", image_save_dir_.c_str());
      } else {
        LOG_WARNING("Failed to create image save directory: %s", image_save_dir_.c_str());
      }
    } else if (info.st_mode & S_IFDIR) {
      LOG_INFO("Using image save directory: %s", image_save_dir_.c_str());
    }
  }

  void cleanupOldImages() {
    // Get list of all PNG files in the directory
    std::vector<std::pair<std::string, std::time_t>> image_files;
    
    DIR* dir = opendir(image_save_dir_.c_str());
    if (dir == nullptr) {
      LOG_WARNING("Cannot open directory for cleanup: %s", image_save_dir_.c_str());
      return;
    }
    
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
      std::string filename = entry->d_name;
      // Check if it's a PNG file
      if (filename.length() > 4 && filename.substr(filename.length() - 4) == ".png") {
        std::string full_path = image_save_dir_ + "/" + filename;
        struct stat file_stat;
        if (stat(full_path.c_str(), &file_stat) == 0) {
          image_files.push_back(std::make_pair(full_path, file_stat.st_mtime));
        }
      }
    }
    closedir(dir);
    
    // Sort by modification time (oldest first)
    std::sort(image_files.begin(), image_files.end(), 
              [](const std::pair<std::string, std::time_t>& a, 
                 const std::pair<std::string, std::time_t>& b) {
                return a.second < b.second;
              });
    
    // Delete oldest files if we have more than 100
    const size_t max_images = 100;
    if (image_files.size() >= max_images) {
      size_t files_to_delete = image_files.size() - max_images + 1;  // +1 to make room for new image
      for (size_t i = 0; i < files_to_delete; i++) {
        if (std::remove(image_files[i].first.c_str()) == 0) {
          LOG_DEBUG("Deleted old image: %s", image_files[i].first.c_str());
        } else {
          LOG_WARNING("Failed to delete old image: %s", image_files[i].first.c_str());
        }
      }
    }
  }

  void saveVisualizationImage(const cv::Mat& intensity_image, 
                             const nc_intensity_detector::TapeDetection& detection,
                             const geometry_msgs::PoseStamped& pose = geometry_msgs::PoseStamped(),
                             const ros::Time& image_stamp = ros::Time(0),
                             double pnp_angle = 0.0) {
    // Save every N detections (configurable via image_save_interval_), up to 100 images total
    detection_frame_count_++;
    
    // Only save every Nth detection (1 = every frame, 5 = every 5th frame)
    // For interval=1: save every frame (1, 2, 3, ...)
    // For interval=5: save every 5th frame (5, 10, 15, ...)
    if (image_save_interval_ > 0 && (detection_frame_count_ % image_save_interval_ != 0)) {
      LOG_DEBUG("Skipping image save (frame %zu, save every %zu frames)", 
                detection_frame_count_, image_save_interval_);
      return;
    }
    
    LOG_INFO("Saving image (frame %zu, interval: %zu)", detection_frame_count_, image_save_interval_);
    
    // Check if we've reached the maximum number of images (100)
    if (saved_image_count_ >= 100) {
      LOG_DEBUG("Maximum image count reached (%zu/100), skipping save", saved_image_count_);
      return;
    }
    
    ros::Time now = ros::Time::now();
    
    // Clean up old images before saving new one (maintain max 100 images)
    cleanupOldImages();
    
    // Count current images in directory
    size_t current_image_count = 0;
    DIR* dir = opendir(image_save_dir_.c_str());
    if (dir != nullptr) {
      struct dirent* entry;
      while ((entry = readdir(dir)) != nullptr) {
        std::string filename = entry->d_name;
        if (filename.length() > 4 && filename.substr(filename.length() - 4) == ".png") {
          current_image_count++;
        }
      }
      closedir(dir);
    }
    
    LOG_DEBUG("Saving visualization image (current: %zu/30)", current_image_count);
    
    // Check if intensity image is valid
    if (intensity_image.empty()) {
      LOG_WARNING("Cannot save image: intensity image is empty");
      return;
    }
    
    // Convert 16-bit intensity image to 8-bit color image for visualization
    cv::Mat image_8bit;
    cv::Mat image_color;
    
    if (intensity_image.type() == CV_16UC1) {
      // Normalize 16-bit to 8-bit
      intensity_image.convertTo(image_8bit, CV_8UC1, 255.0 / 65535.0);
    } else {
      image_8bit = intensity_image;
    }
    
    // Convert to color (BGR)
    cv::cvtColor(image_8bit, image_color, cv::COLOR_GRAY2BGR);
    
    // Check if conversion was successful
    if (image_color.empty()) {
      LOG_WARNING("Cannot save image: failed to convert to color image");
      return;
    }
    
    // Create threshold mask for high-intensity regions (for semi-transparent overlay)
    cv::Mat threshold_mask;
    if (intensity_image.type() == CV_16UC1) {
      // Threshold the original 16-bit image
      double threshold_16bit = intensity_threshold_;
      cv::Mat temp_mask;
      cv::threshold(intensity_image, temp_mask, threshold_16bit, 65535, cv::THRESH_BINARY);
      temp_mask.convertTo(threshold_mask, CV_8UC1, 255.0 / 65535.0);
    } else {
      // Threshold the 8-bit image
      double threshold_8bit = (intensity_threshold_ / 65535.0) * 255.0;
      cv::threshold(image_8bit, threshold_mask, threshold_8bit, 255, cv::THRESH_BINARY);
    }
    
    // Create semi-transparent overlay for high-intensity regions
    // Use blue color with 30% opacity
    cv::Mat overlay = image_color.clone();
    cv::Mat colored_mask(image_color.size(), CV_8UC3, cv::Scalar(255, 100, 0));  // Blue in BGR
    
    // Apply semi-transparent overlay only where threshold_mask is non-zero
    double alpha = 0.3;  // 30% opacity
    for (int y = 0; y < image_color.rows; y++) {
      for (int x = 0; x < image_color.cols; x++) {
        if (threshold_mask.at<uchar>(y, x) > 0) {
          cv::Vec3b& pixel = overlay.at<cv::Vec3b>(y, x);
          cv::Vec3b color = colored_mask.at<cv::Vec3b>(y, x);
          pixel[0] = cv::saturate_cast<uchar>(pixel[0] * (1.0 - alpha) + color[0] * alpha);
          pixel[1] = cv::saturate_cast<uchar>(pixel[1] * (1.0 - alpha) + color[1] * alpha);
          pixel[2] = cv::saturate_cast<uchar>(pixel[2] * (1.0 - alpha) + color[2] * alpha);
        }
      }
    }
    
    // Copy overlay back to image_color
    overlay.copyTo(image_color);
    
    // Different visualization for PnP method vs fallback method
    // Always use PnP visualization now
    bool use_pnp_visualization = true;
    
    // Draw detected lines (only for fallback method - PnP doesn't use lines)
    // Removed legacy line drawing code
    
    // Draw detected corners (actual detected positions) - RED
    if (detection.detected && !detection.corners.empty() && detection.corners.size() == 4) {
      // Draw actual detected corners as red circles
      for (size_t i = 0; i < detection.corners.size(); i++) {
        const auto& corner = detection.corners[i];
        cv::Point2i corner_pt(static_cast<int>(corner.position.x), 
                             static_cast<int>(corner.position.y));
        cv::circle(image_color, corner_pt, 5, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);  // Red circle
      }
      
      // Draw rectangle connecting actual detected corners - RED
      if (use_pnp_visualization) {
        cv::Point2i detected_pts[4];
        for (size_t i = 0; i < 4; i++) {
          detected_pts[i] = cv::Point2i(static_cast<int>(detection.corners[i].position.x),
                                       static_cast<int>(detection.corners[i].position.y));
        }
        // Draw rectangle edges in order: TL->TR->BR->BL->TL
        cv::line(image_color, detected_pts[0], detected_pts[1], cv::Scalar(0, 0, 255), 1, cv::LINE_AA);  // Top edge (red)
        cv::line(image_color, detected_pts[1], detected_pts[2], cv::Scalar(0, 0, 255), 1, cv::LINE_AA);  // Right edge (red)
        cv::line(image_color, detected_pts[2], detected_pts[3], cv::Scalar(0, 0, 255), 1, cv::LINE_AA);  // Bottom edge (red)
        cv::line(image_color, detected_pts[3], detected_pts[0], cv::Scalar(0, 0, 255), 1, cv::LINE_AA);  // Left edge (red)
      }
    }
    
    // Draw outer rectangle (frame bounding box) - BLUE (PnP reconstructed)
    if (detection.detected && detection.bounding_box.width > 0 && detection.bounding_box.height > 0) {
      cv::Rect bbox = detection.bounding_box;
      // PnP method: Blue rectangle (reconstructed from corners), very thin lines (thickness 1)
      // This shows the bounding box reconstructed from detected corners
      cv::rectangle(image_color, bbox, cv::Scalar(255, 100, 0), 1, cv::LINE_AA);  // Blue, thin
    }
    
    // Draw inner rectangle if inner corners are detected - different color, very thin lines
    if (detection.detected && !detection.inner_corners.empty() && detection.inner_corners.size() == 4) {
      // Draw inner rectangle by connecting inner corners
      const auto& ic = detection.inner_corners;
      cv::Point2i inner_pts[4];
      for (size_t i = 0; i < 4; i++) {
        inner_pts[i] = cv::Point2i(static_cast<int>(ic[i].position.x), 
                                    static_cast<int>(ic[i].position.y));
      }
      
      // Draw inner rectangle with green color, very thin lines (thickness 1)
      cv::line(image_color, inner_pts[0], inner_pts[1], cv::Scalar(0, 255, 0), 1, cv::LINE_AA);  // Top edge (green)
      cv::line(image_color, inner_pts[1], inner_pts[2], cv::Scalar(0, 255, 0), 1, cv::LINE_AA);  // Right edge (green)
      cv::line(image_color, inner_pts[2], inner_pts[3], cv::Scalar(0, 255, 0), 1, cv::LINE_AA);  // Bottom edge (green)
      cv::line(image_color, inner_pts[3], inner_pts[0], cv::Scalar(0, 255, 0), 1, cv::LINE_AA);  // Left edge (green)
    }
    
    // Draw center point (only if detection was successful)
    if (detection.detected) {
      cv::Point2i center(static_cast<int>(detection.center.x), 
                         static_cast<int>(detection.center.y));
      // PnP method: Blue filled circle
      cv::circle(image_color, center, 3, cv::Scalar(255, 100, 0), -1);  // Blue
    }
    
    // Add text information
    std::stringstream ss;
    if (detection.detected) {
      // Status line with method indicator
      ss << "Status: DETECTED (PnP)";
      cv::putText(image_color, ss.str(), cv::Point(10, 30), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 100, 0), 1);  // Blue
      
      ss.str("");
      ss << "Dist: " << std::fixed << std::setprecision(2) << detection.distance << "m";
      cv::putText(image_color, ss.str(), cv::Point(10, 50), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
      
      ss.str("");
      ss << "Angle: " << std::fixed << std::setprecision(1) 
         << (detection.angle * 180.0 / M_PI) << "deg";
      cv::putText(image_color, ss.str(), cv::Point(10, 70), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
      
      // Add position information (X, Y, Z) if pose is available
      if (pose.header.stamp.toSec() > 0) {
        ss.str("");
        ss << "X: " << std::fixed << std::setprecision(3) << pose.pose.position.x << "m";
        cv::putText(image_color, ss.str(), cv::Point(10, 90), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        
        ss.str("");
        ss << "Y: " << std::fixed << std::setprecision(3) << pose.pose.position.y << "m";
        cv::putText(image_color, ss.str(), cv::Point(10, 110), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        
        ss.str("");
        ss << "Z: " << std::fixed << std::setprecision(3) << pose.pose.position.z << "m";
        cv::putText(image_color, ss.str(), cv::Point(10, 130), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        
        // Display height (corrected Z value)
        ss.str("");
        ss << "Height: " << std::fixed << std::setprecision(3) << pose.pose.position.z << "m";
        cv::putText(image_color, ss.str(), cv::Point(10, 150), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);  // Cyan color
        
        // Display roll, pitch, yaw
        ss.str("");
        ss << "Roll: " << std::fixed << std::setprecision(2) 
           << (detection.roll * 180.0 / M_PI) << "deg";
        cv::putText(image_color, ss.str(), cv::Point(10, 170), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        
        ss.str("");
        ss << "Pitch: " << std::fixed << std::setprecision(2) 
           << (detection.pitch * 180.0 / M_PI) << "deg";
        cv::putText(image_color, ss.str(), cv::Point(10, 190), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        
        ss.str("");
        ss << "Yaw: " << std::fixed << std::setprecision(2) 
           << (detection.angle * 180.0 / M_PI) << "deg";
        cv::putText(image_color, ss.str(), cv::Point(10, 210), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        
      } else {
        // If pose is not available, still show roll/pitch/yaw from detection
        ss.str("");
        ss << "Roll: " << std::fixed << std::setprecision(2) 
           << (detection.roll * 180.0 / M_PI) << "deg";
        cv::putText(image_color, ss.str(), cv::Point(10, 90), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        
        ss.str("");
        ss << "Pitch: " << std::fixed << std::setprecision(2) 
           << (detection.pitch * 180.0 / M_PI) << "deg";
        cv::putText(image_color, ss.str(), cv::Point(10, 110), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        
        ss.str("");
        ss << "Yaw: " << std::fixed << std::setprecision(2) 
           << (detection.angle * 180.0 / M_PI) << "deg";
        cv::putText(image_color, ss.str(), cv::Point(10, 130), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
      }
    } else {
      ss << "Status: NOT DETECTED";
      cv::putText(image_color, ss.str(), cv::Point(10, 30), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
    }
    
    // Generate filename with timestamp (including milliseconds)
    // Always use current system time for filename to ensure accuracy
    ros::Time save_time = ros::Time::now();
    
    // Convert ros::Time to system time (ros::Time uses Unix timestamp)
    std::time_t time_sec = static_cast<std::time_t>(save_time.toSec());
    std::tm* time_info = std::localtime(&time_sec);
    
    if (time_info == nullptr) {
      LOG_WARNING("Failed to convert time, using system time");
      std::time_t now = std::time(nullptr);
      time_info = std::localtime(&now);
      if (time_info == nullptr) {
        LOG_ERROR("Failed to get local time");
        return;
      }
      time_sec = now;
      // Recalculate save_time from system time for milliseconds
      save_time = ros::Time(time_sec, 0);
    }
    
    char date_part[32];
    std::strftime(date_part, sizeof(date_part), "%Y%m%d", time_info);
    
    // Extract hour, minute, second from local time
    int hour = time_info->tm_hour;
    int minute = time_info->tm_min;
    int second = time_info->tm_sec;
    
    // Extract milliseconds (3 digits) from ros::Time
    int64_t nsec = save_time.toNSec();
    int64_t msec = (nsec / 1000000) % 1000;  // Extract milliseconds
    
    LOG_DEBUG("Time conversion: ros::Time=%.6f, localtime: %02d:%02d:%02d.%03lld", 
              save_time.toSec(), hour, minute, second, msec);
    
    // Save visualization image
    // Format: detection_YYYYMMDD_HHMMSSMMM.png (time + milliseconds combined)
    std::stringstream filename_ss;
    filename_ss << image_save_dir_ << "/detection_" << date_part << "_"
                << std::setfill('0') << std::setw(2) << hour
                << std::setfill('0') << std::setw(2) << minute
                << std::setfill('0') << std::setw(2) << second
                << std::setfill('0') << std::setw(3) << msec << ".png";
    std::string full_path = filename_ss.str();
    LOG_DEBUG("Attempting to save visualization image to: %s", full_path.c_str());
    LOG_DEBUG("Image size: %dx%d, type: %d", image_color.cols, image_color.rows, image_color.type());
    
    bool saved_vis = false;
    if (cv::imwrite(full_path, image_color)) {
      saved_vis = true;
      LOG_DEBUG("Saved visualization image: %s", full_path.c_str());
    } else {
      LOG_ERROR("Failed to save visualization image: %s (check permissions and path)", full_path.c_str());
    }
    
    // Original intensity image saving disabled (as requested)
    
    if (saved_vis) {
      last_save_time_ = now;
      saved_image_count_++;
      
      // Count images again after saving
      size_t final_count = 0;
      DIR* dir = opendir(image_save_dir_.c_str());
      if (dir != nullptr) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
          std::string filename = entry->d_name;
          if (filename.length() > 4 && filename.substr(filename.length() - 4) == ".png") {
            final_count++;
          }
        }
        closedir(dir);
      }
      
      LOG_INFO("Image save complete (total saved: %zu, directory total: %zu)", saved_image_count_, final_count);
      LOG_DEBUG("Image save directory: %s", image_save_dir_.c_str());
    }
  }

  void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(robot_pose_mutex_);
    robot_pose_ = *msg;
    has_robot_pose_ = true;
    LOG_DEBUG("Received robot pose: x=%.3f, y=%.3f, z=%.3f, frame_id=%s",
             msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
             msg->header.frame_id.c_str());
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_ = *msg;
    has_odom_ = true;
    
    ros::Time current_time = msg->header.stamp;
    if (current_time.toSec() == 0) {
      current_time = ros::Time::now();
    }
    
    // Extract angular velocity (z-component is yaw rate)
    double raw_angular_velocity = msg->twist.twist.angular.z;
    
    // Calculate angular acceleration if we have previous measurement
    double angular_accel = 0.0;
    if (has_filtered_angular_vel_ && last_odom_time_.toSec() > 0) {
      double dt = (current_time - last_odom_time_).toSec();
      if (dt > 0.0 && dt < 1.0) {  // Valid time difference
        angular_accel = (raw_angular_velocity - previous_angular_vel_) / dt;
      }
    }
    
    // Apply filtering to smooth out rapid changes
    if (has_filtered_angular_vel_) {
      // Exponential Moving Average (EMA) filter
      angular_velocity_ = odom_filter_alpha_ * raw_angular_velocity + 
                         (1.0 - odom_filter_alpha_) * angular_velocity_;
    } else {
      // First measurement, initialize
      angular_velocity_ = raw_angular_velocity;
      has_filtered_angular_vel_ = true;
    }
    
    // Update previous values for next calculation
    previous_angular_vel_ = raw_angular_velocity;
    last_odom_time_ = current_time;
    
    LOG_DEBUG("Received odom: raw_angular_vel=%.4f rad/s (%.2f deg/s), filtered=%.4f rad/s (%.2f deg/s), "
             "angular_accel=%.4f rad/s^2, linear_vel=%.3f m/s",
             raw_angular_velocity, raw_angular_velocity * 180.0 / M_PI,
             angular_velocity_, angular_velocity_ * 180.0 / M_PI,
             angular_accel,
             std::sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x + 
                      msg->twist.twist.linear.y * msg->twist.twist.linear.y));
  }

  void cmdCallback(const std_msgs::String::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    std::string raw_cmd = msg->data;
    std::string cmd;
    float new_angle = 0.f;
    // Check for angle separator '/'
    size_t slash_pos = raw_cmd.find('/');
    if (slash_pos != std::string::npos) {
      cmd = raw_cmd.substr(0, slash_pos);
      std::string angle_str = raw_cmd.substr(slash_pos + 1);
      try {
        new_angle = std::stof(angle_str) * M_PI / 180.f;
        LOG_INFO("Updated target angle to: %.3f deg", new_angle);
      } catch (const std::exception& e) {
        LOG_WARNING("Failed to parse angle from command: %s", raw_cmd.c_str());
      }
    } else {
      cmd = raw_cmd;
    }
    
    // Convert to lowercase for case-insensitive comparison
    std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
    
    if (cmd == "start" && !detection_active_) {
      detection_active_ = true;
      f_target_angle_ = new_angle;
      // Reset image save counter and frame count when starting new detection session
      saved_image_count_ = 0;
      detection_frame_count_ = 0;
      last_save_time_ = ros::Time(0);
      // Reset temporal filter
      has_previous_measurement_ = false;
      filtered_distance_ = 0.0;
      filtered_angle_ = 0.0;
      filtered_y_ = 0.0;
      // Set selection mode to center for "start" command
      detector_.setSelectionMode("center");
      LOG_INFO("Detection STARTED (image save counter and temporal filter reset, selection mode: center)");
    } else if (cmd == "stop" && detection_active_) {
      detection_active_ = false;
      // Note: Don't save here - last test case is already saved by resetStatsCallback
      // Reset temporal filter
      has_previous_measurement_ = false;
      filtered_distance_ = 0.0;
      filtered_angle_ = 0.0;
      filtered_y_ = 0.0;
      LOG_INFO("Detection STOPPED (temporal filter reset)");
      // Print final statistics with summary format for all test cases
      printAllTestCasesStatistics();
    } else if (cmd == "left") {
      // Set selection mode to left
      detector_.setSelectionMode("left");
      LOG_INFO("Selection mode changed to: left");
    } else if (cmd == "right") {
      // Set selection mode to right
      detector_.setSelectionMode("right");
      LOG_INFO("Selection mode changed to: right");
    } else if (cmd == "center") {
      // Set selection mode to center
      detector_.setSelectionMode("center");
      LOG_INFO("Selection mode changed to: center");
    } else if (cmd != "start" && cmd != "stop") {
      LOG_WARNING("Unknown command received: %s (expected 'start', 'stop', 'center', 'left', or 'right')", msg->data.c_str());
    }
  }

  void resetStatsCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
      // Save current test case statistics before resetting
      if (stats_count_ > 0) {
        TestCaseStats current_stats;
        current_stats.name = "Test Case " + std::to_string(test_case_stats_.size() + 1);
        current_stats.distances = distances_;
        current_stats.angles = angles_;
        current_stats.x_positions = x_positions_;
        current_stats.y_positions = y_positions_;
        current_stats.z_positions = z_positions_;
        current_stats.count = stats_count_;
        test_case_stats_.push_back(current_stats);
        
        LOG_INFO("Saved statistics for %s (%zu detections)", 
                 current_stats.name.c_str(), current_stats.count);
      }
      
      // Reset current statistics
      distances_.clear();
      angles_.clear();
      x_positions_.clear();
      y_positions_.clear();
      z_positions_.clear();
      stats_count_ = 0;
      current_test_case_index_++;
      LOG_INFO("Statistics reset for next test case");
    }
  }

  void intensityCallback(const sensor_msgs::ImageConstPtr& msg) {
    // Check if detection is active
    {
      std::lock_guard<std::mutex> lock(detection_mutex_);
      if (!detection_active_) {
        return;  // Skip processing if detection is not active
      }
    }

    try {
      // Convert ROS image to OpenCV
      cv_bridge::CvImagePtr cv_ptr;
      if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      } else {
        LOG_WARNING("Unsupported image encoding: %s", msg->encoding.c_str());
        return;
      }

      cv::Mat intensity_image = cv_ptr->image;

      // Detect tape
      nc_intensity_detector::TapeDetection detection = detector_.detectTape(intensity_image);

      if (detection.detected) {
        // Override PnP yaw with global angle difference if robot pose is available
        // Also force roll and pitch to 0 as requested
        {
          std::lock_guard<std::mutex> lock(robot_pose_mutex_);
          if (has_robot_pose_) {
            // Calculate robot yaw from quaternion
            tf2::Quaternion q(
              robot_pose_.pose.pose.orientation.x,
              robot_pose_.pose.pose.orientation.y,
              robot_pose_.pose.pose.orientation.z,
              robot_pose_.pose.pose.orientation.w
            );
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            
            // Calculate relative angle: target_global_angle - robot_global_yaw
            // This represents the tape's angle relative to the robot
            double relative_yaw = f_target_angle_ - yaw;
            
            // Normalize to [-PI, PI]
            while (relative_yaw > M_PI) relative_yaw -= 2.0 * M_PI;
            while (relative_yaw < -M_PI) relative_yaw += 2.0 * M_PI;
            
            // Override detection angle
            detection.angle = relative_yaw;
            
            LOG_DEBUG("Overriding PnP angle: PnP=%.3f deg, Robot=%.3f deg, Target=%.3f deg -> New=%.3f deg",
                     detection.angle * 180.0 / M_PI, yaw * 180.0 / M_PI, 
                     f_target_angle_ * 180.0 / M_PI, relative_yaw * 180.0 / M_PI);
          }
        }
        
        // Force roll and pitch to 0
        detection.roll = 0.0;
        detection.pitch = 0.0;

        // Apply temporal filtering to reduce noise
        double filtered_distance = detection.distance;
        double filtered_angle = detection.angle;
        
        if (has_previous_measurement_) {
          // Exponential Moving Average (EMA) filter
          // alpha = 0.7 means 70% weight to current, 30% to previous
          filtered_distance = temporal_filter_alpha_ * detection.distance + 
                             (1.0 - temporal_filter_alpha_) * filtered_distance_;
          filtered_angle = temporal_filter_alpha_ * detection.angle + 
                          (1.0 - temporal_filter_alpha_) * filtered_angle_;
          
        } else {
          // First measurement, initialize filter
          has_previous_measurement_ = true;
        }
        
        // Store PnP angle for visualization
        double pnp_angle_for_viz = filtered_angle;

        // Update filtered values
        filtered_distance_ = filtered_distance;
        filtered_angle_ = filtered_angle;
        
        // Update detection with filtered values
        detection.distance = filtered_distance;
        detection.angle = filtered_angle;
        
        // Use actual frame_id from image message (more reliable than config)
        std::string actual_camera_frame = msg->header.frame_id;
        if (actual_camera_frame.empty()) {
          actual_camera_frame = camera_frame_;  // Fallback to config if empty
        }
        
        // Calculate pose in camera frame
        geometry_msgs::PoseStamped pose_camera = detector_.calculatePose(detection, actual_camera_frame, 
                                                                          intensity_image.cols, intensity_image.rows);
        
        // Apply distance-dependent error correction for X and Y positions
        // 기준 거리 이상에서만 보정 적용, 거리당 오차율로 계산
        double x_raw = pose_camera.pose.position.x;
        double y_raw = pose_camera.pose.position.y;
        double distance = detection.distance;  // x 거리
        
        // Calculate error correction based on distance and error rate per meter
        // 기준 거리 이상에서만 보정 적용
        double x_error = 0.0;
        double y_error = 0.0;
        
        if (distance >= pnp_x_reference_distance_) {
          // 기준 거리 이상: 거리 × 오차율로 보정
          // 예: 거리 2m, x 오차율 0.1 m/m → x 오차 = 2 × 0.1 = 0.2m
          x_error = distance * pnp_x_error_per_meter_;
          y_error = distance * pnp_y_error_per_meter_;
        }
        // 기준 거리 미만: 보정 안 함 (x_error = 0, y_error = 0)
        
        // Apply correction (subtract error from measured value)
        double x_corrected = x_raw - x_error;
        double y_corrected = y_raw - y_error;
        
        LOG_DEBUG("PnP position correction: x_dist=%.3fm (ref=%.3fm, correction_enabled=%s), "
                 "x_raw=%.4fm, x_error=%.4fm (rate=%.4f m/m), x_corrected=%.4fm, "
                 "y_raw=%.4fm, y_error=%.4fm (rate=%.4f m/m), y_corrected=%.4fm",
                 distance, pnp_x_reference_distance_, 
                 (distance >= pnp_x_reference_distance_) ? "yes" : "no",
                 x_raw, x_error, pnp_x_error_per_meter_, x_corrected,
                 y_raw, y_error, pnp_y_error_per_meter_, y_corrected);
        
        // Update pose with corrected X and Y
        pose_camera.pose.position.x = x_corrected;
        pose_camera.pose.position.y = y_corrected;
        
        // Additional Y position correction by removing rotation-induced offset (if needed)
        // When the robot rotates, the tape appears shifted in the camera view
        // This correction removes that virtual shift to get the true lateral offset
        // Y_corrected = Y_raw - (Distance * tan(Angle))
        // Note: y_corrected already has distance-dependent error correction applied above
        double y_after_distance_correction = pose_camera.pose.position.y;
        // double y_correction = detection.distance * std::tan(detection.angle);
        double y_correction = 0;
        double y_final = y_after_distance_correction - y_correction;
        
        LOG_DEBUG("Y rotation correction: after_dist_corr=%.4fm, angle=%.3fdeg, dist=%.3fm, correction=%.4fm, final=%.4fm",
                 y_after_distance_correction, detection.angle * 180.0 / M_PI, detection.distance, y_correction, y_final);
        
        // Update pose with final corrected Y
        pose_camera.pose.position.y = y_final;
        
        // Apply temporal filtering to Y position for smooth compensation
        double filtered_y = y_final;
        
        if (has_previous_measurement_) {
          // Exponential Moving Average (EMA) filter for Y position
          // Use same alpha as distance/angle for consistency
          filtered_y = temporal_filter_alpha_ * y_final + 
                      (1.0 - temporal_filter_alpha_) * filtered_y_;
          
        } else {
          // First measurement, initialize filter
          filtered_y_ = y_final;
        }
        
        // Update filtered Y value
        filtered_y_ = filtered_y;
        
        // Apply filtered Y to pose
        geometry_msgs::PoseStamped pose_camera_filtered = pose_camera;
        pose_camera_filtered.pose.position.y = filtered_y;
        
        // Transform to base frame using camera TF parameters
        // Always use camera_tf parameters for consistent transformation
        geometry_msgs::PoseStamped pose_base;
        bool transform_success = false;
        
        // Use camera TF parameters to transform from camera frame to base_link frame
        // Camera TF: base_link -> camera (x=camera_tf_x, y=camera_tf_y, yaw=camera_tf_yaw)
        // Inverse: camera -> base_link
        double cos_yaw = std::cos(camera_tf_yaw_);
        double sin_yaw = std::sin(camera_tf_yaw_);
        
        // Camera position in base_link frame (inverse of camera_tf)
        double cam_x_in_base = -camera_tf_x_;
        double cam_y_in_base = -camera_tf_y_;
        
        // Rotate camera position by -yaw (inverse rotation)
        double cam_x_rotated = cam_x_in_base * cos_yaw + cam_y_in_base * sin_yaw;
        double cam_y_rotated = -cam_x_in_base * sin_yaw + cam_y_in_base * cos_yaw;
        
        // Object position in camera frame
        double obj_x_cam = pose_camera_filtered.pose.position.x;
        double obj_y_cam = pose_camera_filtered.pose.position.y;
        double obj_z_cam = pose_camera_filtered.pose.position.z;
        
        // Rotate object position by camera yaw (camera is rotated relative to base_link)
        double obj_x_rotated = obj_x_cam * cos_yaw - obj_y_cam * sin_yaw;
        double obj_y_rotated = obj_x_cam * sin_yaw + obj_y_cam * cos_yaw;
        
        // Object position in base_link frame
        pose_base.pose.position.x = cam_x_rotated + obj_x_rotated;
        pose_base.pose.position.y = cam_y_rotated + obj_y_rotated;
        pose_base.pose.position.z = obj_z_cam;
        
        // Transform orientation
        tf2::Quaternion obj_q_cam(
          pose_camera_filtered.pose.orientation.x,
          pose_camera_filtered.pose.orientation.y,
          pose_camera_filtered.pose.orientation.z,
          pose_camera_filtered.pose.orientation.w
        );
        tf2::Quaternion camera_rot;
        camera_rot.setRPY(0, 0, camera_tf_yaw_);
        tf2::Quaternion obj_q_base = camera_rot * obj_q_cam;
        pose_base.pose.orientation.x = obj_q_base.x();
        pose_base.pose.orientation.y = obj_q_base.y();
        pose_base.pose.orientation.z = obj_q_base.z();
        pose_base.pose.orientation.w = obj_q_base.w();
        pose_base.header.frame_id = base_frame_;
        pose_base.header.stamp = pose_camera_filtered.header.stamp;
        transform_success = true;
        
        // Calculate local yaw from quaternion
        tf2::Quaternion q_base(pose_base.pose.orientation.x, pose_base.pose.orientation.y,
                               pose_base.pose.orientation.z, pose_base.pose.orientation.w);
        double roll_base, pitch_base, yaw_base;
        tf2::Matrix3x3(q_base).getRPY(roll_base, pitch_base, yaw_base);

        // Publish local pose (base_link frame)
        pose_pub_.publish(pose_base);

        // Calculate and publish global pose if robot pose is available
        // Note: calculateGlobalPose locks robot_pose_mutex_ internally, so don't lock here
        {
          bool should_calculate = false;
          {
            std::lock_guard<std::mutex> lock(robot_pose_mutex_);
            should_calculate = has_robot_pose_;
          }
          
          if (should_calculate) {
            // Get target frame from robot_pose_ (usually "map" or "odom")
            std::string target_frame;
            ros::Time robot_pose_stamp;
            {
              std::lock_guard<std::mutex> lock(robot_pose_mutex_);
              target_frame = robot_pose_.header.frame_id;
              robot_pose_stamp = robot_pose_.header.stamp;
            }
            
            // Use default frame if frame_id is empty
            if (target_frame.empty()) {
              target_frame = "map";  // Default to "map" frame
              LOG_DEBUG("Robot pose frame_id is empty, using default frame: %s", target_frame.c_str());
            }
            
            // Check timestamp synchronization and apply angular velocity correction
            // Camera image timestamp
            ros::Time image_stamp = pose_camera_filtered.header.stamp;
            double time_diff = 0.0;
            if (image_stamp.toSec() > 0 && robot_pose_stamp.toSec() > 0) {
              time_diff = (image_stamp - robot_pose_stamp).toSec();
              if (std::abs(time_diff) > 0.1) {  // More than 100ms difference
                LOG_WARNING("Timestamp mismatch: image_stamp=%.3f, robot_pose_stamp=%.3f, diff=%.3f sec. "
                           "Applying angular velocity correction.",
                           image_stamp.toSec(), robot_pose_stamp.toSec(), time_diff);
              }
            }
            
            {
              // Transform from camera frame to map/odom frame using TF
              // Step 1: Use pose_base (already transformed using camera_tf parameters)
              geometry_msgs::PoseStamped pose_base_for_global = pose_base;
              
              // pose_base is already transformed from camera to base_link using camera_tf parameters
              // No need for additional transformation here
              
              // Step 2: Transform base_link -> map/odom using TF (with timestamp interpolation)
              // This automatically handles timestamp synchronization and provides smooth interpolation
              geometry_msgs::PoseStamped global_pose;
              bool tf_transform_success = false;
              
              try {
                // Use image timestamp for TF lookup to get interpolated transform
                ros::Time transform_time = pose_base_for_global.header.stamp;
                
                // Check if TF transform is available
                if (tf_buffer_.canTransform(target_frame, base_frame_, transform_time, ros::Duration(0.1))) {
                  // Transform using TF (automatic timestamp interpolation)
                  tf_buffer_.transform(pose_base_for_global, global_pose, target_frame, ros::Duration(0.1));
                  tf_transform_success = true;
                  
                } else {
                  LOG_WARNING("TF transform not available: %s -> %s at time %.3f. Falling back to robot_pose method.",
                             base_frame_.c_str(), target_frame.c_str(), transform_time.toSec());
                }
              } catch (tf2::TransformException& ex) {
                LOG_WARNING("TF transform failed: %s. Falling back to robot_pose method.", ex.what());
              }
              
              // Fallback: Use robot_pose_ if TF transform failed
              if (!tf_transform_success) {
                std::lock_guard<std::mutex> lock(robot_pose_mutex_);
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
                
                // Apply angular velocity correction if enabled (for fallback method)
                double corrected_robot_yaw = robot_yaw;
                if (use_odom_correction_) {
                  double angular_vel = 0.0;
                  bool has_odom_data = false;
                  {
                    std::lock_guard<std::mutex> lock_odom(odom_mutex_);
                    has_odom_data = has_odom_;
                    if (has_odom_) {
                      angular_vel = angular_velocity_;
                    }
                  }
                  
                  if (has_odom_data && 
                      std::abs(angular_vel) > odom_angular_vel_threshold_) {
                    double yaw_correction = odom_correction_gain_ * angular_vel * time_diff;
                    if (std::abs(yaw_correction) > odom_max_yaw_correction_) {
                      double sign = (yaw_correction > 0) ? 1.0 : -1.0;
                      yaw_correction = sign * odom_max_yaw_correction_;
                    }
                    corrected_robot_yaw = robot_yaw + yaw_correction;
                  }
                }
                
                // Manual transformation
                double cos_robot_yaw = std::cos(corrected_robot_yaw);
                double sin_robot_yaw = std::sin(corrected_robot_yaw);
                
                global_pose.pose.position.x = robot_x + pose_base_for_global.pose.position.x * cos_robot_yaw - 
                                             pose_base_for_global.pose.position.y * sin_robot_yaw;
                global_pose.pose.position.y = robot_y + pose_base_for_global.pose.position.x * sin_robot_yaw + 
                                             pose_base_for_global.pose.position.y * cos_robot_yaw;
                global_pose.pose.position.z = robot_z + pose_base_for_global.pose.position.z;
                
                // Transform orientation
                tf2::Quaternion obj_q_base(
                  pose_base_for_global.pose.orientation.x,
                  pose_base_for_global.pose.orientation.y,
                  pose_base_for_global.pose.orientation.z,
                  pose_base_for_global.pose.orientation.w
                );
                tf2::Quaternion corrected_robot_q;
                corrected_robot_q.setRPY(robot_roll, robot_pitch, corrected_robot_yaw);
                tf2::Quaternion obj_q_global = corrected_robot_q * obj_q_base;
                
                global_pose.pose.orientation.x = obj_q_global.x();
                global_pose.pose.orientation.y = obj_q_global.y();
                global_pose.pose.orientation.z = obj_q_global.z();
                global_pose.pose.orientation.w = obj_q_global.w();
                
                global_pose.header.stamp = pose_base_for_global.header.stamp;
                global_pose.header.frame_id = target_frame;
              }
              
              // Calculate global yaw from quaternion
              tf2::Quaternion q_global(global_pose.pose.orientation.x, global_pose.pose.orientation.y,
                                       global_pose.pose.orientation.z, global_pose.pose.orientation.w);
              double roll_global, pitch_global, yaw_global;
              tf2::Matrix3x3(q_global).getRPY(roll_global, pitch_global, yaw_global);
              
              global_pose_pub_.publish(global_pose);
              LOG_INFO("Local: x=%.3f, y=%.3f, yaw=%.3f deg | Global: x=%.3f, y=%.3f, yaw=%.3f deg",
                       pose_base.pose.position.x, pose_base.pose.position.y, yaw_base * 180.0 / M_PI,
                       global_pose.pose.position.x, global_pose.pose.position.y, yaw_global * 180.0 / M_PI);
            }
          } else {
            LOG_WARNING("Robot pose not available, skipping global pose publication. Subscribe to /localization/robot_pos topic.");
          }
        }

        // Publish visualization marker
        publishMarker(pose_base, detection);

        // Save visualization image
        saveVisualizationImage(intensity_image, detection, pose_base, msg->header.stamp, pnp_angle_for_viz);

        // Collect statistics for accuracy evaluation
        collectStatistics(pose_base, detection);

        // Print statistics every 5 seconds
        ros::Time now = ros::Time::now();
        if ((now - last_stats_time_).toSec() >= 5.0) {
          printStatistics();
          last_stats_time_ = now;
        }
      } else {
        LOG_DEBUG("No tape detected");
        // Reset when detection is lost
        has_previous_measurement_ = false;
        filtered_distance_ = 0.0;
        filtered_angle_ = 0.0;
        
        // Clear marker when no detection
        clearMarker();
        
        // Save image even when no detection (for debugging)
        nc_intensity_detector::TapeDetection empty_detection;
        empty_detection.detected = false;
        saveVisualizationImage(intensity_image, empty_detection, geometry_msgs::PoseStamped(), msg->header.stamp);
      }

    } catch (cv_bridge::Exception& e) {
      LOG_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void publishMarker(const geometry_msgs::PoseStamped& pose, 
                     const nc_intensity_detector::TapeDetection& detection) {
    // Publish frame bounding box (yellow cube)
    visualization_msgs::Marker frame_marker;
    frame_marker.header = pose.header;
    frame_marker.ns = "nc_intensity_detector";
    frame_marker.id = 0;
    frame_marker.type = visualization_msgs::Marker::CUBE;
    frame_marker.action = visualization_msgs::Marker::ADD;
    
    frame_marker.pose = pose.pose;
    frame_marker.scale.x = tape_width_;
    frame_marker.scale.y = tape_thickness_;
    frame_marker.scale.z = tape_height_;
    
    frame_marker.color.r = 1.0;
    frame_marker.color.g = 1.0;
    frame_marker.color.b = 0.0;
    frame_marker.color.a = 0.7;
    frame_marker.lifetime = ros::Duration(0.5);
    
    marker_pub_.publish(frame_marker);
    
    // Publish arrow pointing to the tape
    visualization_msgs::Marker arrow;
    arrow.header = pose.header;
    arrow.ns = "nc_intensity_detector";
    arrow.id = 1;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    
    arrow.pose.position.x = 0;
    arrow.pose.position.y = 0;
    arrow.pose.position.z = 0;
    arrow.pose.orientation.w = 1.0;
    
    arrow.points.resize(2);
    arrow.points[0].x = 0;
    arrow.points[0].y = 0;
    arrow.points[0].z = 0;
    arrow.points[1] = pose.pose.position;
    
    arrow.scale.x = 0.05;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    
    arrow.color.r = 0.0;
    arrow.color.g = 1.0;
    arrow.color.b = 0.0;
    arrow.color.a = 0.8;
    arrow.lifetime = ros::Duration(0.5);
    
    marker_pub_.publish(arrow);
    
    // Publish L-shape corners as spheres
    // Convert corner pixel positions to 3D positions in camera frame
    double image_center_x = 256.0;  // 512 / 2
    double image_center_y = 212.0;  // 424 / 2
    
    std::vector<geometry_msgs::Point> corner_3d_positions;
    
    for (size_t i = 0; i < detection.corners.size(); i++) {
      const auto& corner = detection.corners[i];
      
      // Calculate pixel offset from image center
      double pixel_offset_x = corner.position.x - image_center_x - principal_point_x_;
      double pixel_offset_y = corner.position.y - image_center_y - principal_point_y_;
      
      // Convert to 3D position in camera frame
      // Camera coordinate: x forward, y left, z up
      // Using pinhole camera model: x = distance, y = -x * (u - cx) / fx, z = -x * (v - cy) / fy
      double corner_x = detection.distance;
      double corner_y = -(pixel_offset_x * detection.distance / focal_length_x_);
      double corner_z = -(pixel_offset_y * detection.distance / focal_length_y_);
      
      // Transform to base frame (if pose_base is in base frame)
      // For now, use camera frame coordinates
      geometry_msgs::Point corner_3d;
      corner_3d.x = corner_x;
      corner_3d.y = corner_y;
      corner_3d.z = corner_z;
      corner_3d_positions.push_back(corner_3d);
      
      // Publish corner as sphere marker
      visualization_msgs::Marker corner_marker;
      corner_marker.header = pose.header;
      corner_marker.ns = "nc_intensity_detector_corners";
      corner_marker.id = static_cast<int>(i);
      corner_marker.type = visualization_msgs::Marker::SPHERE;
      corner_marker.action = visualization_msgs::Marker::ADD;
      
      corner_marker.pose.position = corner_3d;
      corner_marker.pose.orientation.w = 1.0;
      
      corner_marker.scale.x = 0.1;  // 10cm sphere
      corner_marker.scale.y = 0.1;
      corner_marker.scale.z = 0.1;
      
      // Color based on confidence (red = low, green = high)
      corner_marker.color.r = 1.0 - corner.confidence;
      corner_marker.color.g = corner.confidence;
      corner_marker.color.b = 0.0;
      corner_marker.color.a = 0.9;
      
      corner_marker.lifetime = ros::Duration(0.5);
      
      marker_pub_.publish(corner_marker);
    }
    
    // Publish frame edges as line list (connect corners to show frame structure)
    if (detection.corners.size() >= 2) {
      visualization_msgs::Marker frame_lines;
      frame_lines.header = pose.header;
      frame_lines.ns = "nc_intensity_detector";
      frame_lines.id = 2;
      frame_lines.type = visualization_msgs::Marker::LINE_LIST;
      frame_lines.action = visualization_msgs::Marker::ADD;
      
      frame_lines.pose.orientation.w = 1.0;
      frame_lines.scale.x = 0.02;  // Line width (2cm)
      
      frame_lines.color.r = 1.0;
      frame_lines.color.g = 0.5;
      frame_lines.color.b = 0.0;
      frame_lines.color.a = 0.8;
      
      frame_lines.lifetime = ros::Duration(0.5);
      
      // Connect corners to form frame edges
      // For 2+ corners, connect adjacent corners
      if (corner_3d_positions.size() >= 2) {
        // Connect all pairs of corners (for frame structure)
        for (size_t i = 0; i < corner_3d_positions.size(); i++) {
          for (size_t j = i + 1; j < corner_3d_positions.size(); j++) {
            frame_lines.points.push_back(corner_3d_positions[i]);
            frame_lines.points.push_back(corner_3d_positions[j]);
          }
        }
      }
      
      marker_pub_.publish(frame_lines);
    }
    
    // Publish detected lines (edges) as line list markers
    // Removed legacy line visualization
  }
  
  void clearMarker() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = ros::Time::now();
    
    // Clear frame markers (cube, arrow, lines)
    marker.ns = "nc_intensity_detector";
    for (int i = 0; i < 3; i++) {
      marker.id = i;
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub_.publish(marker);
    }
    
    // Clear corner markers (up to 10 corners)
    marker.ns = "nc_intensity_detector_corners";
    for (int i = 0; i < 10; i++) {
      marker.id = i;
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub_.publish(marker);
    }
    
    // Clear line markers (horizontal and vertical)
    // Removed legacy line visualization clearing
  }

  geometry_msgs::PoseStamped calculateGlobalPose(const geometry_msgs::PoseStamped& local_pose) {
    geometry_msgs::PoseStamped global_pose;
    
    
    if (!has_robot_pose_) {
      LOG_WARNING("Robot pose not available, cannot calculate global pose");
      return local_pose;  // Return local pose if robot pose is not available
    }
    
    // Extract robot position and orientation
    // PoseWithCovarianceStamped uses pose.pose.pose for the actual pose
    double robot_x = robot_pose_.pose.pose.position.x;
    double robot_y = robot_pose_.pose.pose.position.y;
    double robot_z = robot_pose_.pose.pose.position.z;
    
    // Extract robot orientation (quaternion to yaw)
    tf2::Quaternion robot_q(
      robot_pose_.pose.pose.orientation.x,
      robot_pose_.pose.pose.orientation.y,
      robot_pose_.pose.pose.orientation.z,
      robot_pose_.pose.pose.orientation.w
    );
    tf2::Matrix3x3 robot_m(robot_q);
    double robot_roll, robot_pitch, robot_yaw;
    robot_m.getRPY(robot_roll, robot_pitch, robot_yaw);
    
    // Extract local pose (tape position relative to base_link)
    double local_x = local_pose.pose.position.x;
    double local_y = local_pose.pose.position.y;
    double local_z = local_pose.pose.position.z;
    
    // Extract local orientation
    tf2::Quaternion local_q(
      local_pose.pose.orientation.x,
      local_pose.pose.orientation.y,
      local_pose.pose.orientation.z,
      local_pose.pose.orientation.w
    );
    tf2::Matrix3x3 local_m(local_q);
    double local_roll, local_pitch, local_yaw;
    local_m.getRPY(local_roll, local_pitch, local_yaw);
    
    // Transform local pose to global frame
    // Rotation: Rotate local position by robot yaw
    double cos_yaw = std::cos(robot_yaw);
    double sin_yaw = std::sin(robot_yaw);
    
    // Global position = Robot position + Rotated local position
    global_pose.pose.position.x = robot_x + local_x * cos_yaw - local_y * sin_yaw;
    global_pose.pose.position.y = robot_y + local_x * sin_yaw + local_y * cos_yaw;
    global_pose.pose.position.z = robot_z + local_z;
    
    // Global orientation = Robot orientation * Local orientation
    double global_yaw = robot_yaw + local_yaw;
    tf2::Quaternion global_q;
    global_q.setRPY(local_roll, local_pitch, global_yaw);
    
    global_pose.pose.orientation.x = global_q.x();
    global_pose.pose.orientation.y = global_q.y();
    global_pose.pose.orientation.z = global_q.z();
    global_pose.pose.orientation.w = global_q.w();
    
    // Set header
    global_pose.header.stamp = local_pose.header.stamp;
    global_pose.header.frame_id = robot_pose_.header.frame_id;  // Use robot pose frame (usually "map" or "odom")
    
    return global_pose;
  }

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber intensity_sub_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber reset_stats_sub_;
  ros::Subscriber robot_pose_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher global_pose_pub_;
  ros::Publisher marker_pub_;
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  nc_intensity_detector::IntensityDetector detector_;

  // Detection control
  bool detection_active_;
  std::mutex detection_mutex_;
  
  // Robot pose for global pose calculation
  geometry_msgs::PoseWithCovarianceStamped robot_pose_;
  bool has_robot_pose_;
  std::mutex robot_pose_mutex_;
  
  // Odom for angular velocity correction
  nav_msgs::Odometry odom_;
  bool has_odom_;
  double angular_velocity_;  // Filtered angular velocity
  std::mutex odom_mutex_;
  bool use_odom_correction_;
  double odom_correction_gain_;
  double odom_angular_vel_threshold_;  // Minimum angular velocity to apply correction (rad/s)
  double odom_time_diff_threshold_;    // Minimum time difference to apply correction (sec)
  double odom_max_yaw_correction_;     // Maximum yaw correction in radians
  double odom_filter_alpha_;           // EMA filter coefficient for angular velocity (0.0-1.0, higher = more responsive)
  bool has_filtered_angular_vel_;      // Flag to track if filtered angular velocity is initialized
  double odom_max_angular_accel_;     // Maximum angular acceleration (rad/s^2) - if exceeded, reduce correction
  double previous_angular_vel_;       // Previous angular velocity for acceleration calculation
  ros::Time last_odom_time_;          // Last odom callback time for acceleration calculation

  // Statistics for accuracy evaluation (current test case)
  std::vector<double> distances_;
  std::vector<double> angles_;
  std::vector<double> x_positions_;
  std::vector<double> y_positions_;
  std::vector<double> z_positions_;
  
  // Target angle for global yaw override
  double f_target_angle_ = 0.f;  // Global target angle of the tape (radians)
  size_t stats_count_;
  ros::Time last_stats_time_;
  
  // Statistics for all test cases
  std::vector<TestCaseStats> test_case_stats_;
  int current_test_case_index_;

  void collectStatistics(const geometry_msgs::PoseStamped& pose, 
                         const nc_intensity_detector::TapeDetection& detection) {
    distances_.push_back(detection.distance);
    angles_.push_back(detection.angle * 180.0 / M_PI);
    x_positions_.push_back(pose.pose.position.x);
    y_positions_.push_back(pose.pose.position.y);
    z_positions_.push_back(pose.pose.position.z);
    stats_count_++;
    
    // Keep only last 100 samples to avoid memory issues
    if (distances_.size() > 100) {
      distances_.erase(distances_.begin());
      angles_.erase(angles_.begin());
      x_positions_.erase(x_positions_.begin());
      y_positions_.erase(y_positions_.begin());
      z_positions_.erase(z_positions_.begin());
    }
  }

  void printStatistics(bool is_final = false) {
    if (stats_count_ == 0) {
      if (is_final) {
        LOG_INFO("\n");
        LOG_INFO("╔════════════════════════════════════════════════════════════╗");
        LOG_INFO("║          FINAL TEST RESULTS - No Detections              ║");
        LOG_INFO("╚════════════════════════════════════════════════════════════╝");
      } else {
        LOG_INFO("=== Accuracy Evaluation: No detections yet ===");
      }
      return;
    }

    // Calculate statistics
    auto calcStats = [](const std::vector<double>& values) -> std::tuple<double, double, double, double, double> {
      if (values.empty()) return std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0);
      
      double sum = 0.0;
      for (double v : values) sum += v;
      double mean = sum / values.size();
      
      double variance = 0.0;
      for (double v : values) {
        double diff = v - mean;
        variance += diff * diff;
      }
      double stddev = std::sqrt(variance / values.size());
      
      double min_val = *std::min_element(values.begin(), values.end());
      double max_val = *std::max_element(values.begin(), values.end());
      
      // Calculate RMSE (Root Mean Square Error) - using mean as reference
      double rmse = std::sqrt(variance);
      
      return std::make_tuple(mean, stddev, min_val, max_val, rmse);
    };

    auto [dist_mean, dist_std, dist_min, dist_max, dist_rmse] = calcStats(distances_);
    auto [angle_mean, angle_std, angle_min, angle_max, angle_rmse] = calcStats(angles_);
    auto [x_mean, x_std, x_min, x_max, x_rmse] = calcStats(x_positions_);
    auto [y_mean, y_std, y_min, y_max, y_rmse] = calcStats(y_positions_);
    auto [z_mean, z_std, z_min, z_max, z_rmse] = calcStats(z_positions_);
    
    // Calculate stability (coefficient of variation)
    double dist_cv = (dist_mean > 1e-6) ? (dist_std / dist_mean) * 100.0 : 0.0;
    double angle_cv = (std::abs(angle_mean) > 1e-6) ? (angle_std / std::abs(angle_mean)) * 100.0 : angle_std;
    
    // Calculate accuracy (assuming target is known from test cases)
    // For now, just show precision (stddev)
    double dist_precision_mm = dist_std * 1000.0;  // Convert to mm
    double angle_precision_deg = angle_std;

    if (is_final) {
      // Final summary with better formatting
      LOG_INFO("\n");
      LOG_INFO("╔════════════════════════════════════════════════════════════╗");
      LOG_INFO("║              FINAL TEST RESULTS SUMMARY                    ║");
      LOG_INFO("╚════════════════════════════════════════════════════════════╝");
      LOG_INFO("");
      LOG_INFO("  Total Detections: %zu", stats_count_);
      LOG_INFO("");
      LOG_INFO("  ┌──────────────────────────────────────────────────────────┐");
      LOG_INFO("  │ DISTANCE (meters)                                        │");
      LOG_INFO("  ├──────────────────────────────────────────────────────────┤");
      LOG_INFO("  │  Mean:      %8.3f m", dist_mean);
      LOG_INFO("  │  Std Dev:   %8.3f m  (Precision: %6.2f mm)", dist_std, dist_precision_mm);
      LOG_INFO("  │  RMSE:      %8.3f m", dist_rmse);
      LOG_INFO("  │  Range:     [%6.3f, %6.3f] m", dist_min, dist_max);
      LOG_INFO("  │  Stability: %6.2f%% (CV)", dist_cv);
      LOG_INFO("  └──────────────────────────────────────────────────────────┘");
      LOG_INFO("");
      LOG_INFO("  ┌──────────────────────────────────────────────────────────┐");
      LOG_INFO("  │ ANGLE (degrees)                                          │");
      LOG_INFO("  ├──────────────────────────────────────────────────────────┤");
      LOG_INFO("  │  Mean:      %8.3f deg", angle_mean);
      LOG_INFO("  │  Std Dev:   %8.3f deg  (Precision: %6.3f deg)", angle_std, angle_precision_deg);
      LOG_INFO("  │  RMSE:      %8.3f deg", angle_rmse);
      LOG_INFO("  │  Range:     [%6.3f, %6.3f] deg", angle_min, angle_max);
      LOG_INFO("  │  Stability: %6.2f%% (CV)", angle_cv);
      LOG_INFO("  └──────────────────────────────────────────────────────────┘");
      LOG_INFO("");
      LOG_INFO("  ┌──────────────────────────────────────────────────────────┐");
      LOG_INFO("  │ POSITION (meters)                                        │");
      LOG_INFO("  ├──────────────────────────────────────────────────────────┤");
      LOG_INFO("  │  X:  mean=%.3f, std=%.3f, RMSE=%.3f, range=[%.3f, %.3f]", 
               x_mean, x_std, x_rmse, x_min, x_max);
      LOG_INFO("  │  Y:  mean=%.3f, std=%.3f, RMSE=%.3f, range=[%.3f, %.3f]", 
               y_mean, y_std, y_rmse, y_min, y_max);
      LOG_INFO("  │  Z:  mean=%.3f, std=%.3f, RMSE=%.3f, range=[%.3f, %.3f]", 
               z_mean, z_std, z_rmse, z_min, z_max);
      LOG_INFO("  └──────────────────────────────────────────────────────────┘");
      LOG_INFO("");
      LOG_INFO("╔════════════════════════════════════════════════════════════╗");
      LOG_INFO("║  Accuracy Target: Distance < 10mm, Angle < 0.1deg         ║");
      LOG_INFO("║  Current Precision: Distance %.2f mm, Angle %.3f deg     ║", 
               dist_precision_mm, angle_precision_deg);
      if (dist_precision_mm < 10.0 && angle_precision_deg < 0.1) {
        LOG_INFO("║  Status: ✓ TARGET ACHIEVED                                ║");
      } else {
        LOG_INFO("║  Status: ⚠ TARGET NOT MET                                 ║");
      }
      LOG_INFO("╚════════════════════════════════════════════════════════════╝");
      LOG_INFO("");
    } else {
      // Regular periodic output
      LOG_INFO("=== Accuracy Evaluation (last %zu detections) ===", stats_count_);
      LOG_INFO("Distance:  mean=%.3fm, std=%.3fm, RMSE=%.3fm, range=[%.3f, %.3f]m", 
               dist_mean, dist_std, dist_rmse, dist_min, dist_max);
      LOG_INFO("Angle:     mean=%.3fdeg, std=%.3fdeg, RMSE=%.3fdeg, range=[%.3f, %.3f]deg", 
               angle_mean, angle_std, angle_rmse, angle_min, angle_max);
      LOG_INFO("Position X: mean=%.3fm, std=%.3fm, RMSE=%.3fm, range=[%.3f, %.3f]m", 
               x_mean, x_std, x_rmse, x_min, x_max);
      LOG_INFO("Position Y: mean=%.3fm, std=%.3fm, RMSE=%.3fm, range=[%.3f, %.3f]m", 
               y_mean, y_std, y_rmse, y_min, y_max);
      LOG_INFO("Position Z: mean=%.3fm, std=%.3fm, RMSE=%.3fm, range=[%.3f, %.3f]m", 
               z_mean, z_std, z_rmse, z_min, z_max);
      LOG_INFO("Stability: Distance CV=%.2f%%, Angle CV=%.2f%%", dist_cv, angle_cv);
      LOG_INFO("================================================");
    }
  }

  void printAllTestCasesStatistics() {
    if (test_case_stats_.empty() && stats_count_ == 0) {
      LOG_INFO("\n");
      LOG_INFO("╔════════════════════════════════════════════════════════════╗");
      LOG_INFO("║          FINAL TEST RESULTS - No Detections              ║");
      LOG_INFO("╚════════════════════════════════════════════════════════════╝");
      return;
    }
    
    // Save current test case if it has data
    if (stats_count_ > 0) {
      TestCaseStats current_stats;
      current_stats.name = "Test Case " + std::to_string(test_case_stats_.size() + 1);
      current_stats.distances = distances_;
      current_stats.angles = angles_;
      current_stats.x_positions = x_positions_;
      current_stats.y_positions = y_positions_;
      current_stats.z_positions = z_positions_;
      current_stats.count = stats_count_;
      test_case_stats_.push_back(current_stats);
    }
    
    if (test_case_stats_.empty()) {
      LOG_INFO("\n");
      LOG_INFO("╔════════════════════════════════════════════════════════════╗");
      LOG_INFO("║          FINAL TEST RESULTS - No Test Cases              ║");
      LOG_INFO("╚════════════════════════════════════════════════════════════╝");
      return;
    }
    
    // Calculate statistics helper function
    auto calcStats = [](const std::vector<double>& values) -> std::tuple<double, double, double, double, double> {
      if (values.empty()) return std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0);
      
      double sum = 0.0;
      for (double v : values) sum += v;
      double mean = sum / values.size();
      
      double variance = 0.0;
      for (double v : values) {
        double diff = v - mean;
        variance += diff * diff;
      }
      double stddev = std::sqrt(variance / values.size());
      
      double min_val = *std::min_element(values.begin(), values.end());
      double max_val = *std::max_element(values.begin(), values.end());
      double rmse = std::sqrt(variance);
      
      return std::make_tuple(mean, stddev, min_val, max_val, rmse);
    };
    
    // Test case names and true values (matching test_intensity_publisher.py)
    struct TestCaseInfo {
      std::string name;
      double true_distance;  // meters
      double true_angle;       // degrees
    };
    
    std::vector<TestCaseInfo> test_case_infos = {
      {"정면 1.5m", 1.5, 0.0},
      {"정면 2.0m", 2.0, 0.0},
      {"정면 2.5m", 2.5, 0.0},
      {"왼쪽 10도 2.0m", 2.0, -10.0},
      {"오른쪽 10도 2.0m", 2.0, 10.0},
      {"정면 2.0m (30% 가려짐)", 2.0, 0.0}
    };
    
    LOG_INFO("\n");
    LOG_INFO("╔════════════════════════════════════════════════════════════╗");
    LOG_INFO("║         FINAL TEST RESULTS - ALL TEST CASES                ║");
    LOG_INFO("╚════════════════════════════════════════════════════════════╝");
    LOG_INFO("");
    
    // Print statistics for each test case
    for (size_t i = 0; i < test_case_stats_.size(); i++) {
      const auto& stats = test_case_stats_[i];
      std::string case_name = (i < test_case_infos.size()) ? test_case_infos[i].name : stats.name;
      double true_distance = (i < test_case_infos.size()) ? test_case_infos[i].true_distance : 0.0;
      double true_angle = (i < test_case_infos.size()) ? test_case_infos[i].true_angle : 0.0;
      
      if (stats.count == 0) {
        LOG_INFO("  ┌──────────────────────────────────────────────────────────┐");
        LOG_INFO("  │ %s", case_name.c_str());
        LOG_INFO("  ├──────────────────────────────────────────────────────────┤");
        LOG_INFO("  │  No detections                                           │");
        LOG_INFO("  └──────────────────────────────────────────────────────────┘");
        LOG_INFO("");
        continue;
      }
      
      auto [dist_mean, dist_std, dist_min, dist_max, dist_rmse] = calcStats(stats.distances);
      auto [angle_mean, angle_std, angle_min, angle_max, angle_rmse] = calcStats(stats.angles);
      auto [x_mean, x_std, x_min, x_max, x_rmse] = calcStats(stats.x_positions);
      auto [y_mean, y_std, y_min, y_max, y_rmse] = calcStats(stats.y_positions);
      auto [z_mean, z_std, z_min, z_max, z_rmse] = calcStats(stats.z_positions);
      
      // Calculate accuracy (bias) and RMSE with respect to true values
      double dist_bias = dist_mean - true_distance;
      double dist_bias_mm = dist_bias * 1000.0;
      double angle_bias = angle_mean - true_angle;
      
      // Calculate RMSE (Root Mean Square Error) with respect to true values
      double dist_rmse_true = 0.0;
      double angle_rmse_true = 0.0;
      for (size_t j = 0; j < stats.distances.size(); j++) {
        double dist_error = stats.distances[j] - true_distance;
        dist_rmse_true += dist_error * dist_error;
      }
      for (size_t j = 0; j < stats.angles.size(); j++) {
        double angle_error = stats.angles[j] - true_angle;
        angle_rmse_true += angle_error * angle_error;
      }
      dist_rmse_true = std::sqrt(dist_rmse_true / stats.distances.size()) * 1000.0;  // in mm
      angle_rmse_true = std::sqrt(angle_rmse_true / stats.angles.size());  // in deg
      
      double dist_precision_mm = dist_std * 1000.0;
      double angle_precision_deg = angle_std;
      double y_precision_mm = y_std * 1000.0;
      double z_precision_mm = z_std * 1000.0;
      double dist_cv = (dist_mean > 1e-6) ? (dist_std / dist_mean) * 100.0 : 0.0;
      
      // Check if this test case meets accuracy targets (both precision and accuracy)
      bool dist_precision_met = dist_precision_mm < 10.0;
      bool dist_accuracy_met = std::abs(dist_bias_mm) < 10.0;
      bool dist_rmse_met = dist_rmse_true < 10.0;
      bool dist_target_met = dist_precision_met && dist_accuracy_met;
      
      bool angle_precision_met = angle_precision_deg < 0.1;
      bool angle_accuracy_met = std::abs(angle_bias) < 0.1;
      bool angle_rmse_met = angle_rmse_true < 0.1;
      bool angle_target_met = angle_precision_met && angle_accuracy_met;
      
      bool y_target_met = y_precision_mm < 10.0;
      bool z_target_met = z_precision_mm < 10.0;
      bool target_met = dist_target_met && angle_target_met && y_target_met && z_target_met;
      
      LOG_INFO("  ┌──────────────────────────────────────────────────────────┐");
      LOG_INFO("  │ Test Case %zu: %s", i + 1, case_name.c_str());
      LOG_INFO("  ├──────────────────────────────────────────────────────────┤");
      LOG_INFO("  │  Detections: %zu", stats.count);
      LOG_INFO("  │  Distance:   mean=%.3fm (true=%.3fm), bias=%.2fmm, std=%.3fm (%.2fmm), RMSE=%.2fmm", 
               dist_mean, true_distance, dist_bias_mm, dist_std, dist_precision_mm, dist_rmse_true);
      LOG_INFO("  │  Angle:      mean=%.3fdeg (true=%.3fdeg), bias=%.3fdeg, std=%.3fdeg, RMSE=%.3fdeg", 
               angle_mean, true_angle, angle_bias, angle_std, angle_rmse_true);
      LOG_INFO("  │  Position:   X=%.3f±%.3f, Y=%.3f±%.3f (%.2fmm), Z=%.3f±%.3f (%.2fmm)", 
               x_mean, x_std, y_mean, y_std, y_precision_mm, z_mean, z_std, z_precision_mm);
      LOG_INFO("  │  Stability:  Distance CV=%.2f%%", dist_cv);
      LOG_INFO("  │  Precision:  Distance %s (%.2fmm), Y %s (%.2fmm), Z %s (%.2fmm), Angle %s (%.3fdeg)", 
               dist_precision_met ? "✓" : "✗", dist_precision_mm,
               y_target_met ? "✓" : "✗", y_precision_mm,
               z_target_met ? "✓" : "✗", z_precision_mm,
               angle_precision_met ? "✓" : "✗", angle_precision_deg);
      LOG_INFO("  │  Accuracy:   Distance %s (bias=%.2fmm, RMSE=%.2fmm), Angle %s (bias=%.3fdeg, RMSE=%.3fdeg)", 
               dist_accuracy_met ? "✓" : "✗", dist_bias_mm, dist_rmse_true,
               angle_accuracy_met ? "✓" : "✗", angle_bias, angle_rmse_true);
      LOG_INFO("  │  Status:     %s", target_met ? "✓ TARGET ACHIEVED" : "⚠ TARGET NOT MET");
      LOG_INFO("  └──────────────────────────────────────────────────────────┘");
      LOG_INFO("");
    }
    
    // Calculate overall statistics (average precision and accuracy across test cases)
    // Since each test case has different distance conditions, we should average the precisions
    size_t total_detections = 0;
    std::vector<double> dist_precisions_mm;
    std::vector<double> angle_precisions_deg;
    std::vector<double> y_precisions_mm;
    std::vector<double> z_precisions_mm;
    std::vector<double> dist_biases_mm;
    std::vector<double> angle_biases_deg;
    std::vector<double> dist_rmses_mm;
    std::vector<double> angle_rmses_deg;
    int cases_meeting_target = 0;
    int total_cases = 0;
    
    for (size_t i = 0; i < test_case_stats_.size(); i++) {
      const auto& stats = test_case_stats_[i];
      if (stats.count > 0) {
        double true_distance = (i < test_case_infos.size()) ? test_case_infos[i].true_distance : 0.0;
        double true_angle = (i < test_case_infos.size()) ? test_case_infos[i].true_angle : 0.0;
        
        auto [dist_mean, dist_std, dist_min, dist_max, dist_rmse] = calcStats(stats.distances);
        auto [angle_mean, angle_std, angle_min, angle_max, angle_rmse] = calcStats(stats.angles);
        auto [y_mean, y_std, y_min, y_max, y_rmse] = calcStats(stats.y_positions);
        auto [z_mean, z_std, z_min, z_max, z_rmse] = calcStats(stats.z_positions);
        
        double dist_precision_mm = dist_std * 1000.0;
        double angle_precision_deg = angle_std;
        double y_precision_mm = y_std * 1000.0;
        double z_precision_mm = z_std * 1000.0;
        
        // Calculate bias and RMSE
        double dist_bias_mm = (dist_mean - true_distance) * 1000.0;
        double angle_bias_deg = angle_mean - true_angle;
        
        double dist_rmse_true = 0.0;
        double angle_rmse_true = 0.0;
        for (size_t j = 0; j < stats.distances.size(); j++) {
          double dist_error = stats.distances[j] - true_distance;
          dist_rmse_true += dist_error * dist_error;
        }
        for (size_t j = 0; j < stats.angles.size(); j++) {
          double angle_error = stats.angles[j] - true_angle;
          angle_rmse_true += angle_error * angle_error;
        }
        dist_rmse_true = std::sqrt(dist_rmse_true / stats.distances.size()) * 1000.0;
        angle_rmse_true = std::sqrt(angle_rmse_true / stats.angles.size());
        
        dist_precisions_mm.push_back(dist_precision_mm);
        angle_precisions_deg.push_back(angle_precision_deg);
        y_precisions_mm.push_back(y_precision_mm);
        z_precisions_mm.push_back(z_precision_mm);
        dist_biases_mm.push_back(dist_bias_mm);
        angle_biases_deg.push_back(angle_bias_deg);
        dist_rmses_mm.push_back(dist_rmse_true);
        angle_rmses_deg.push_back(angle_rmse_true);
        total_detections += stats.count;
        total_cases++;
        
        // Check if both precision and accuracy targets are met
        bool dist_precision_met = dist_precision_mm < 10.0;
        bool dist_accuracy_met = std::abs(dist_bias_mm) < 10.0;
        bool angle_precision_met = angle_precision_deg < 0.1;
        bool angle_accuracy_met = std::abs(angle_bias_deg) < 0.1;
        
        if (dist_precision_met && dist_accuracy_met && angle_precision_met && angle_accuracy_met &&
            y_precision_mm < 10.0 && z_precision_mm < 10.0) {
          cases_meeting_target++;
        }
      }
    }
    
    if (!dist_precisions_mm.empty()) {
      // Calculate average precision and accuracy across test cases
      double avg_dist_precision = 0.0;
      double avg_angle_precision = 0.0;
      double avg_y_precision = 0.0;
      double avg_z_precision = 0.0;
      double avg_dist_bias = 0.0;
      double avg_angle_bias = 0.0;
      double avg_dist_rmse = 0.0;
      double avg_angle_rmse = 0.0;
      double max_dist_precision = 0.0;
      double max_angle_precision = 0.0;
      double max_y_precision = 0.0;
      double max_z_precision = 0.0;
      double max_dist_bias = 0.0;
      double max_angle_bias = 0.0;
      
      for (double p : dist_precisions_mm) {
        avg_dist_precision += p;
        max_dist_precision = std::max(max_dist_precision, p);
      }
      for (double p : angle_precisions_deg) {
        avg_angle_precision += p;
        max_angle_precision = std::max(max_angle_precision, p);
      }
      for (double p : y_precisions_mm) {
        avg_y_precision += p;
        max_y_precision = std::max(max_y_precision, p);
      }
      for (double p : z_precisions_mm) {
        avg_z_precision += p;
        max_z_precision = std::max(max_z_precision, p);
      }
      for (double b : dist_biases_mm) {
        avg_dist_bias += b;
        max_dist_bias = std::max(max_dist_bias, std::abs(b));
      }
      for (double b : angle_biases_deg) {
        avg_angle_bias += b;
        max_angle_bias = std::max(max_angle_bias, std::abs(b));
      }
      for (double r : dist_rmses_mm) {
        avg_dist_rmse += r;
      }
      for (double r : angle_rmses_deg) {
        avg_angle_rmse += r;
      }
      
      avg_dist_precision /= dist_precisions_mm.size();
      avg_angle_precision /= angle_precisions_deg.size();
      avg_y_precision /= y_precisions_mm.size();
      avg_z_precision /= z_precisions_mm.size();
      avg_dist_bias /= dist_biases_mm.size();
      avg_angle_bias /= angle_biases_deg.size();
      avg_dist_rmse /= dist_rmses_mm.size();
      avg_angle_rmse /= angle_rmses_deg.size();
      
      LOG_INFO("  ╔══════════════════════════════════════════════════════════╗");
      LOG_INFO("  ║ OVERALL STATISTICS (Average Across Test Cases)          ║");
      LOG_INFO("  ╠══════════════════════════════════════════════════════════╣");
      LOG_INFO("  ║  Total Test Cases: %d", total_cases);
      LOG_INFO("  ║  Total Detections: %zu", total_detections);
      LOG_INFO("  ║  Distance Precision: avg=%.2fmm (max=%.2fmm), Bias: avg=%.2fmm (max=%.2fmm), RMSE: avg=%.2fmm", 
               avg_dist_precision, max_dist_precision, avg_dist_bias, max_dist_bias, avg_dist_rmse);
      LOG_INFO("  ║  Y Position Precision: avg=%.2fmm (max=%.2fmm)", 
               avg_y_precision, max_y_precision);
      LOG_INFO("  ║  Z Position Precision: avg=%.2fmm (max=%.2fmm)", 
               avg_z_precision, max_z_precision);
      LOG_INFO("  ║  Angle Precision: avg=%.3fdeg (max=%.3fdeg), Bias: avg=%.3fdeg (max=%.3fdeg), RMSE: avg=%.3fdeg", 
               avg_angle_precision, max_angle_precision, avg_angle_bias, max_angle_bias, avg_angle_rmse);
      LOG_INFO("  ║  Cases Meeting Target:      %d / %d (%.1f%%)", 
               cases_meeting_target, total_cases, 
               100.0 * cases_meeting_target / total_cases);
      LOG_INFO("  ╠══════════════════════════════════════════════════════════╣");
      LOG_INFO("  ║  Accuracy Target: Distance < 10mm (precision & bias),    ║");
      LOG_INFO("  ║                Y < 10mm, Z < 10mm, Angle < 0.1deg       ║");
      LOG_INFO("  ║  Overall Precision: Distance %.2fmm, Y %.2fmm, Z %.2fmm,  ║", 
               avg_dist_precision, avg_y_precision, avg_z_precision);
      LOG_INFO("  ║         Angle %.3fdeg                                    ║", 
               avg_angle_precision);
      LOG_INFO("  ║  Overall Accuracy: Distance bias=%.2fmm, RMSE=%.2fmm,   ║", 
               avg_dist_bias, avg_dist_rmse);
      LOG_INFO("  ║         Angle bias=%.3fdeg, RMSE=%.3fdeg                 ║", 
               avg_angle_bias, avg_angle_rmse);
      if (cases_meeting_target == total_cases) {
        LOG_INFO("  ║  Status: ✓ ALL TEST CASES MEET TARGET                    ║");
      } else if (cases_meeting_target > 0) {
        LOG_INFO("  ║  Status: ⚠ PARTIAL (%d/%d cases meet target)            ║", 
                 cases_meeting_target, total_cases);
      } else {
        LOG_INFO("  ║  Status: ✗ NO TEST CASES MEET TARGET                    ║");
      }
      LOG_INFO("  ╚══════════════════════════════════════════════════════════╝");
    }
    LOG_INFO("");
  }

  // Parameters
  std::string intensity_topic_;
  std::string output_topic_;
  std::string cmd_topic_;
  std::string camera_frame_;
  std::string base_frame_;
  
  // Camera TF parameters (base_link to camera frame)
  // Used as fallback if TF transform is not available
  double camera_tf_x_;      // Translation X: -0.363m
  double camera_tf_y_;      // Translation Y: 0.05m
  double camera_tf_yaw_;    // Rotation Yaw: 180 degrees (in radians)
  
  double intensity_threshold_;
  double min_visible_ratio_;
  double crop_top_ratio_;
  double crop_bottom_ratio_;
  double tape_width_;
  double tape_height_;
  double tape_thickness_;
  double min_distance_;
  double max_distance_;
  
  double focal_length_x_;
  double focal_length_y_;
  double principal_point_x_;
  double principal_point_y_;
  
  // Camera distortion coefficients
  double distortion_k1_;
  double distortion_k2_;
  double distortion_k3_;
  double distortion_p1_;
  double distortion_p2_;
  
  // Calibration parameters
  double distance_calibration_factor_;
  double angle_calibration_offset_;
  double pnp_yaw_multiplier_;  // PnP yaw multiplier (multiply PnP yaw result by this value)
  double y_position_offset_;  // Y position correction offset in meters
  double y_scale_factor_;  // Y position scale factor (multiply Y result by this value, default: 1.0)
  double max_y_change_;  // Maximum allowed Y position change in meters for tracking validation
  double occlusion_distance_penalty_;
  
  // PnP position error correction (distance-dependent)
  double pnp_x_reference_distance_;  // 기준 거리: 오차 보정을 시작하는 거리 (meters, default: 1.77m)
                                       // 이 거리 이상에서만 보정 적용
  double pnp_x_error_per_meter_;      // X 오차율: 거리 1m당 x를 몇 m 보정할지 (m/m, default: 0.1)
                                       // 예: 0.1 = 1m당 0.1m 보정, 2m에서는 0.2m 보정
  double pnp_y_error_per_meter_;      // Y 오차율: 거리 1m당 y를 몇 m 보정할지 (m/m, default: 0.05)
                                       // 예: 0.05 = 1m당 0.05m 보정, 2m에서는 0.1m 보정
  
  // Height correction parameters
  double height_correction_slope_;
  double height_correction_offset_;
  double reference_height_;
  
  // Image saving
  std::string image_save_dir_;
  size_t saved_image_count_;
  size_t detection_frame_count_;  // Count of detection frames
  size_t image_save_interval_;    // Save image every N frames (1 = every frame, 5 = every 5th frame)
  ros::Time last_save_time_;
  
  // Temporal filtering for noise reduction
  double filtered_distance_;      // Filtered distance from previous frame
  double filtered_angle_;         // Filtered angle (yaw) from previous frame
  double filtered_y_;            // Filtered Y position from previous frame
  bool has_previous_measurement_; // Whether we have a previous measurement
  double temporal_filter_alpha_;  // EMA filter coefficient (0.0-1.0, higher = more responsive)
  
  // Deadzone and rate limiting parameters to prevent oscillation
  double y_deadzone_;            // Y position deadzone in meters (ignore changes smaller than this)
  double yaw_deadzone_;          // Yaw deadzone in degrees (ignore changes smaller than this)
  double max_y_rate_;            // Maximum Y change rate per frame in meters
  double max_yaw_rate_;          // Maximum yaw change rate per frame in degrees
};

int main(int argc, char** argv) {
  NaviFra::Logger::get().SetSeverityMin(severity_level::info);
  ros::init(argc, argv, "nc_intensity_detector_node");
  
  IntensityDetectorNode node;
  
  ros::spin();
  
  return 0;
}

