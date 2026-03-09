#ifndef NC_INTENSITY_DETECTOR_INTENSITY_DETECTOR_H
#define NC_INTENSITY_DETECTOR_INTENSITY_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <geometry_msgs/PoseStamped.h>

namespace nc_intensity_detector {

struct Corner {
  cv::Point2f position;  // Corner position in image
  double angle;          // Angle of the corner (typically 90 degrees for L-shape)
  double confidence;     // Confidence score (0.0 ~ 1.0)
};

struct TapeDetection {
  bool detected;
  cv::Rect bounding_box;
  cv::Point2f center;
  double distance;  // Estimated distance in meters
  double angle;     // Yaw angle in radians
  double pitch;     // Pitch angle in radians
  double roll;      // Roll angle in radians
  double pnp_y;     // Y position from PnP tvec[1] (camera coordinate system, in meters)
  std::vector<Corner> corners;  // Detected outer corners (4 corners for rectangle)
  std::vector<Corner> inner_corners;  // Detected inner corners (4 corners for inner rectangle, empty if not detected)
  geometry_msgs::PoseStamped pose;
};


class IntensityDetector {
public:
  IntensityDetector();
  ~IntensityDetector();

  // Initialize with parameters
  void initialize(double intensity_threshold,
                  double tape_width,
                  double tape_height,
                  double tape_thickness,
                  double min_distance,
                  double max_distance,
                  double focal_length_x,
                  double focal_length_y,
                  double principal_point_x,
                  double principal_point_y,
                  double min_visible_ratio = 0.5,
                  double distance_calibration_factor = 1.0,
                  double angle_calibration_offset = 0.0,
                  double pnp_yaw_multiplier = 1.0,
                  double y_position_offset = 0.0,
                  double y_scale_factor = 1.0,
                  double max_y_change = 0.3,
                  double occlusion_distance_penalty = 0.0,
                  double crop_top_ratio = 0.0,
                  double crop_bottom_ratio = 0.0,
                  double distortion_k1 = 0.0,
                  double distortion_k2 = 0.0,
                  double distortion_k3 = 0.0,
                  double distortion_p1 = 0.0,
                  double distortion_p2 = 0.0,
                  double height_correction_slope = 0.0,
                  double height_correction_offset = 0.0,
                  double reference_height = 0.0);

  // Process intensity image and detect reflective tape
  TapeDetection detectTape(const cv::Mat& intensity_image);

  // Set selection mode for multiple tape detection
  // "center": select contour closest to image center (default)
  // "left": select leftmost contour
  // "right": select rightmost contour
  void setSelectionMode(const std::string& mode);

  // Calculate pose from detection
  geometry_msgs::PoseStamped calculatePose(const TapeDetection& detection,
                                           const std::string& frame_id,
                                           int image_width = 512,
                                           int image_height = 424);

private:
  // Parameters
  double intensity_threshold_;
  double tape_width_;      // meters
  double tape_height_;     // meters
  double tape_thickness_;  // meters
  double min_distance_;    // meters
  double max_distance_;   // meters
  double crop_top_ratio_;    // Crop top portion ratio (0.0 ~ 1.0)
  double crop_bottom_ratio_; // Crop bottom portion ratio (0.0 ~ 1.0)
  
  // Camera parameters
  double focal_length_x_;
  double focal_length_y_;
  double principal_point_x_;
  double principal_point_y_;
  
  // Camera distortion coefficients
  double distortion_k1_;  // Radial distortion coefficient 1
  double distortion_k2_;  // Radial distortion coefficient 2
  double distortion_k3_;  // Radial distortion coefficient 3
  double distortion_p1_;  // Tangential distortion coefficient 1
  double distortion_p2_;  // Tangential distortion coefficient 2

  // Calibration parameters
  double distance_calibration_factor_;  // Distance correction factor (1.0 = no correction)
  double angle_calibration_offset_;     // Angle (yaw) correction offset in degrees
  double pnp_yaw_multiplier_;          // PnP yaw multiplier (multiply PnP yaw result by this value)
  double y_position_offset_;           // Y position correction offset in meters
  double y_scale_factor_;              // Y position scale factor (multiply Y result by this value, default: 1.0)
  double max_y_change_;                // Maximum allowed Y position change in meters for tracking validation
  double occlusion_distance_penalty_;   // Additional distance uncertainty when occlusion detected
  double min_visible_ratio_;           // Minimum visible ratio when partially occluded (0.0 ~ 1.0)
  
  // Height correction parameters (to compensate for distance-dependent Z measurement variation)
  double height_correction_slope_;     // Slope of height correction (m/m)
  double height_correction_offset_;    // Offset of height correction (m)
  double reference_height_;            // Reference height (m) - target height value
  
  // Statistics calculation counter (to reduce computation)
  mutable int intensity_stats_counter_;  // Counter for intensity statistics calculation
  
  // Selection mode for multiple tape detection
  std::string selection_mode_;  // "center", "left", or "right"
  
  // Previous detection state for temporal consistency
  bool has_previous_detection_;  // Whether we have a previous detection
  cv::Point2f previous_center_;  // Previous detection center in image coordinates
  double previous_distance_;     // Previous detection distance
  double previous_y_;            // Previous Y position in meters (for tracking validation)
  ros::Time last_detection_time_;  // Time of last successful detection

  // Helper functions (legacy, kept for backward compatibility)
  std::vector<std::pair<cv::Rect, double>> findHighIntensityRegions(const cv::Mat& intensity_image);
  bool validateTapeShape(const cv::Rect& bbox, double distance, double contour_area = -1.0);
  double estimateDistance(const cv::Rect& bbox, double contour_area = -1.0);
  cv::Point2f calculateCenter(const cv::Mat& intensity_image, const cv::Rect& bbox);
};

} // namespace nc_intensity_detector

#endif // NC_INTENSITY_DETECTOR_INTENSITY_DETECTOR_H

