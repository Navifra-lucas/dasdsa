#include "nc_intensity_detector/intensity_detector.h"
#include "nc_intensity_detector/corner_detection.h"
#include "nc_intensity_detector/frame_processing.h"
#include "nc_intensity_detector/distance_estimation.h"
#include <util/logger.hpp>
#include <ros/ros.h>
#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace nc_intensity_detector {

IntensityDetector::IntensityDetector()
  : intensity_threshold_(5000.0)
  , tape_width_(0.70)
  , tape_height_(0.50)
  , tape_thickness_(0.05)
  , min_distance_(1.0)
  , max_distance_(3.0)
  , crop_top_ratio_(0.0)
  , crop_bottom_ratio_(0.0)
  , focal_length_x_(365.6)  // Calculated from 70deg FOV: 256 / tan(35°) ≈ 365.6
  , focal_length_y_(367.2)  // Calculated from 60deg FOV: 212 / tan(30°) ≈ 367.2
  , principal_point_x_(0.0)
  , principal_point_y_(0.0)
  , distortion_k1_(0.0)
  , distortion_k2_(0.0)
  , distortion_k3_(0.0)
  , distortion_p1_(0.0)
  , distortion_p2_(0.0)
  , distance_calibration_factor_(1.0)
  , angle_calibration_offset_(0.0)
  , pnp_yaw_multiplier_(1.0)
  , y_position_offset_(0.0)
  , y_scale_factor_(1.0)
  , max_y_change_(0.3)
  , occlusion_distance_penalty_(0.0)
  , min_visible_ratio_(0.5)
  , height_correction_slope_(0.0)
  , height_correction_offset_(0.0)
  , reference_height_(0.0)
  , intensity_stats_counter_(0)
  , selection_mode_("center")
  , has_previous_detection_(false)
  , previous_center_(-1, -1)
  , previous_distance_(-1.0)
  , previous_y_(0.0)
  , last_detection_time_(ros::Time(0))
{
}

IntensityDetector::~IntensityDetector() {
}

void IntensityDetector::initialize(double intensity_threshold,
                                   double tape_width,
                                   double tape_height,
                                   double tape_thickness,
                                   double min_distance,
                                   double max_distance,
                                   double focal_length_x,
                                   double focal_length_y,
                                   double principal_point_x,
                                   double principal_point_y,
                                   double min_visible_ratio,
                                   double distance_calibration_factor,
                  double angle_calibration_offset,
                  double pnp_yaw_multiplier,
                  double y_position_offset,
                  double y_scale_factor,
                  double max_y_change,
                  double occlusion_distance_penalty,
                                   double crop_top_ratio,
                                   double crop_bottom_ratio,
                                   double distortion_k1,
                                   double distortion_k2,
                                   double distortion_k3,
                                   double distortion_p1,
                                   double distortion_p2,
                                   double height_correction_slope,
                                   double height_correction_offset,
                                   double reference_height) {
  intensity_threshold_ = intensity_threshold;
  tape_width_ = tape_width;
  tape_height_ = tape_height;
  tape_thickness_ = tape_thickness;
  min_distance_ = min_distance;
  max_distance_ = max_distance;
  focal_length_x_ = focal_length_x;
  focal_length_y_ = focal_length_y;
  principal_point_x_ = principal_point_x;
  principal_point_y_ = principal_point_y;
  distortion_k1_ = distortion_k1;
  distortion_k2_ = distortion_k2;
  distortion_k3_ = distortion_k3;
  distortion_p1_ = distortion_p1;
  distortion_p2_ = distortion_p2;
  distance_calibration_factor_ = std::max(0.5, std::min(1.5, distance_calibration_factor));  // Clamp between 0.5 and 1.5
  angle_calibration_offset_ = angle_calibration_offset;  // Allow -180 to +180 degrees
  pnp_yaw_multiplier_ = pnp_yaw_multiplier;
  y_position_offset_ = y_position_offset;  // Y position offset in meters
  y_scale_factor_ = std::max(0.1, std::min(2.0, y_scale_factor));  // Clamp between 0.1 and 2.0
  max_y_change_ = std::max(0.05, std::min(1.0, max_y_change));  // Clamp between 0.05m and 1.0m
  occlusion_distance_penalty_ = std::max(0.0, std::min(0.5, occlusion_distance_penalty));  // Clamp between 0.0 and 0.5
  min_visible_ratio_ = std::max(0.1, std::min(1.0, min_visible_ratio));  // Clamp between 0.1 and 1.0
  crop_top_ratio_ = std::max(0.0, std::min(0.5, crop_top_ratio));  // Clamp between 0.0 and 0.5
  crop_bottom_ratio_ = std::max(0.0, std::min(0.5, crop_bottom_ratio));  // Clamp between 0.0 and 0.5
  height_correction_slope_ = height_correction_slope;
  height_correction_offset_ = height_correction_offset;
  reference_height_ = reference_height;
}

TapeDetection IntensityDetector::detectTape(const cv::Mat& intensity_image) {
  TapeDetection result;
  result.detected = false;

  if (intensity_image.empty()) {
    return result;
  }

  // Crop image: remove top and bottom portions to focus on center region
  cv::Mat cropped_image = intensity_image;
  int crop_top = 0;
  int crop_bottom = 0;
  
  if (crop_top_ratio_ > 0.0 || crop_bottom_ratio_ > 0.0) {
    int img_height = intensity_image.rows;
    int img_width = intensity_image.cols;
    
    crop_top = static_cast<int>(img_height * crop_top_ratio_);
    crop_bottom = static_cast<int>(img_height * crop_bottom_ratio_);
    
    // Ensure valid crop region
    int crop_height = img_height - crop_top - crop_bottom;
    if (crop_height > 0 && crop_top >= 0 && crop_bottom >= 0) {
      cv::Rect crop_region(0, crop_top, img_width, crop_height);
      cropped_image = intensity_image(crop_region);
      LOG_DEBUG("Cropped image: original=%dx%d, cropped=%dx%d (top=%d, bottom=%d)", 
               img_width, img_height, cropped_image.cols, cropped_image.rows, crop_top, crop_bottom);
    } else {
      LOG_WARNING("Invalid crop parameters: top=%d, bottom=%d, height=%d, using original image", 
                  crop_top, crop_bottom, crop_height);
      cropped_image = intensity_image;
    }
  }

  // Convert 16-bit image to 8-bit for thresholding
  cv::Mat image_8bit;
  if (cropped_image.type() == CV_16UC1) {
    cropped_image.convertTo(image_8bit, CV_8UC1, 255.0 / 65535.0);
  } else {
    image_8bit = cropped_image;
  }

  // Analyze intensity distribution for debugging
  double min_val, max_val;
  cv::minMaxLoc(cropped_image, &min_val, &max_val);
  cv::Scalar mean_intensity = cv::mean(cropped_image);
  LOG_DEBUG("Intensity stats: min=%.0f, max=%.0f, mean=%.0f, threshold=%.0f", 
                    min_val, max_val, mean_intensity[0], intensity_threshold_);

  // Convert threshold value to 8-bit scale
  double threshold_8bit = (intensity_threshold_ / 65535.0) * 255.0;

  // Threshold the intensity image
  cv::Mat thresholded;
  cv::threshold(image_8bit, thresholded, threshold_8bit, 255, cv::THRESH_BINARY);
  
  int thresholded_pixels = cv::countNonZero(thresholded);
  LOG_INFO( "After threshold (%.0f): %d pixels (%.2f%% of image)", 
                    intensity_threshold_, thresholded_pixels, 
                    100.0 * thresholded_pixels / (thresholded.rows * thresholded.cols));
  
  // Morphological operations to clean up the image
  // For thin frame structures, use adaptive kernel size and iterations based on object size
  // Closer objects need larger kernel to connect fragmented pixels
  // Far objects (small) need fewer iterations to avoid over-smoothing
  int kernel_size = 3;
  int morph_iterations = 2;  // Reduced from 3 for small objects at far distance
  double estimated_pixel_size = (tape_width_ * focal_length_x_) / ((min_distance_ + max_distance_) / 2.0);
  if (estimated_pixel_size > 50) {  // Close distance: larger kernel and more iterations
    kernel_size = 5;
    morph_iterations = 3;
  } else if (estimated_pixel_size < 20) {  // Far distance: smaller kernel, fewer iterations
    kernel_size = 3;
    morph_iterations = 1;  // Minimal iterations for small objects
  }
  cv::Mat kernel_small = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
  cv::Mat cleaned;
  // Close operation to connect frame edges (important for hollow rectangles)
  // Adaptive iterations: fewer for small objects to preserve shape
  cv::morphologyEx(thresholded, cleaned, cv::MORPH_CLOSE, kernel_small, cv::Point(-1, -1), morph_iterations);
  
  int cleaned_pixels = cv::countNonZero(cleaned);
  LOG_INFO( "After morphology: %d pixels (%.2f%% of image)", 
                    cleaned_pixels, 100.0 * cleaned_pixels / (cleaned.rows * cleaned.cols));

  // Use PnP method only (no fallback)
  std::vector<Corner> all_corners;
  double pnp_distance = 0.0;
  double pnp_angle = 0.0;
  double pnp_pitch = 0.0;
  double pnp_roll = 0.0;
  double pnp_y = 0.0;
  
  LOG_DEBUG("Attempting PnP method...");
  std::vector<Corner> inner_corners;
  
  // Check if 5 seconds have passed since last successful detection
  // If so, reset previous detection state to allow finding a new tape
  ros::Time current_time = ros::Time::now();
  if (has_previous_detection_ && last_detection_time_.toSec() > 0) {
    double time_since_last = (current_time - last_detection_time_).toSec();
    if (time_since_last > 5.0) {
      LOG_INFO("No detection for %.1f seconds (> 5.0s). Resetting previous detection state to allow finding new tape.",
               time_since_last);
      has_previous_detection_ = false;
      previous_center_ = cv::Point2f(-1, -1);
      previous_distance_ = -1.0;
      last_detection_time_ = ros::Time(0);
    }
  }
  
  // Prepare previous center (adjust for crop offset if needed)
  cv::Point2f prev_center = previous_center_;
  if (has_previous_detection_ && crop_top > 0 && prev_center.y >= 0) {
    // Adjust previous center for current crop offset
    prev_center.y -= crop_top;
  }
  
  all_corners = nc_intensity_detector::detectCornersPnP(
      cropped_image, cleaned, tape_width_, tape_height_, tape_thickness_,
      focal_length_x_, focal_length_y_,
      distortion_k1_, distortion_k2_, distortion_k3_, distortion_p1_, distortion_p2_,
      principal_point_x_, principal_point_y_,
      pnp_distance, pnp_angle, pnp_pitch, pnp_roll, pnp_y,
      inner_corners, selection_mode_,
      has_previous_detection_ ? prev_center : cv::Point2f(-1, -1),
      has_previous_detection_ ? previous_distance_ : -1.0,
      has_previous_detection_ ? previous_y_ : 0.0);
  
  // Adjust inner corner positions for crop offset
  if (crop_top > 0 && !inner_corners.empty()) {
    for (auto& corner : inner_corners) {
      corner.position.y += crop_top;
    }
  }
  
  // Adjust corner positions for crop offset (add crop_top offset to y-coordinates)
  if (crop_top > 0 && !all_corners.empty()) {
    for (auto& corner : all_corners) {
      corner.position.y += crop_top;
    }
  }
  
  // Check if PnP method succeeded (need exactly 4 corners)
  if (all_corners.size() != 4 || pnp_distance <= 0.0) {
    LOG_WARNING("PnP method failed (corners=%zu, distance=%.2f). "
                "Try adjusting intensity_threshold (current: %.0f). "
                "Intensity range: [%.0f, %.0f], mean: %.0f", 
                all_corners.size(), pnp_distance, intensity_threshold_, 
                min_val, max_val, mean_intensity[0]);
    // Don't reset previous detection immediately on failure - wait for 5 second timeout
    // This allows the system to wait for the tracked tape to reappear
    return result;
  }
  
  // Store detected lines (empty for PnP method)
  // result.lines = detected_lines; // Removed
  
  // Use all detected corners
  std::vector<Corner> corners = all_corners;
  
  LOG_INFO("PnP method succeeded: %zu corners, distance=%.2fm, roll=%.3fdeg, pitch=%.3fdeg, yaw=%.3fdeg", 
           corners.size(), pnp_distance, pnp_roll * 180.0 / M_PI, pnp_pitch * 180.0 / M_PI, pnp_angle * 180.0 / M_PI);

  // Use PnP results directly
  double distance = pnp_distance;
  double pitch = pnp_pitch;
  double roll = pnp_roll;
  
  // Use PnP angle directly (object orientation from PnP)
  // PnP angle is more accurate and stable than pixel offset based calculation
  // Pixel offset includes both lateral movement and rotation effects, making it unreliable
  // Apply PnP yaw multiplier
  double angle = pnp_angle * pnp_yaw_multiplier_;
  
  // Calculate center from corners (needed for result.center and Y calculation)
  cv::Point2f center = calculateCenterFromCorners(corners);
  
  LOG_INFO("Using PnP results: distance=%.3fm, pnp_angle=%.3fdeg, y=%.3fm", 
           distance, angle * 180.0 / M_PI, pnp_y);
  
  // Calculate bounding box from corners (PnP method)
  cv::Rect bbox = reconstructFrameFromCorners(corners);
  
  // Apply distance calibration factor (only for real sensor calibration, not for simulation)
  double raw_distance = distance;
  if (std::abs(distance_calibration_factor_ - 1.0) > 0.001) {
    distance = raw_distance * distance_calibration_factor_;
    LOG_DEBUG( "Distance calibration: raw=%.3fm, factor=%.3f, calibrated=%.3fm", 
                      raw_distance, distance_calibration_factor_, distance);
  }
  
  // Check for occlusion and apply penalty if detected
  // Improved occlusion detection: check corner count, line completeness, and bounding box coverage
  double estimated_occlusion_ratio = 0.0;
  bool has_occlusion = false;
  
  // Method 1: Corner-based occlusion detection
  if (corners.size() < 4) {
    double corner_occlusion = 1.0 - (corners.size() / 4.0);
    estimated_occlusion_ratio = std::max(estimated_occlusion_ratio, corner_occlusion);
    has_occlusion = true;
  }
  
  // Method 2: Line-based occlusion detection (if we have lines)
  // Removed as lines are no longer used
  
  // Method 3: Bounding box coverage (if available)
  if (bbox.width > 0 && bbox.height > 0) {
    double expected_area = (tape_width_ * focal_length_x_ / raw_distance) * 
                          (tape_height_ * focal_length_y_ / raw_distance);
    double actual_area = bbox.width * bbox.height;
    if (expected_area > 0) {
      double coverage_ratio = actual_area / expected_area;
      if (coverage_ratio < 0.8) {  // Less than 80% coverage suggests occlusion
        double bbox_occlusion = 1.0 - coverage_ratio;
        estimated_occlusion_ratio = std::max(estimated_occlusion_ratio, bbox_occlusion);
        has_occlusion = true;
      }
    }
  }
  
  // Apply occlusion penalty (increased for better handling)
  // Note: PnP method typically detects full frame, so occlusion is less likely
  // For PnP, we skip occlusion penalty as it requires 4 corners (full frame)
  if (has_occlusion && estimated_occlusion_ratio > 0.1) {
    LOG_DEBUG("Occlusion detected but skipping penalty for PnP method (ratio=%.2f)", 
              estimated_occlusion_ratio);
  }
  
  LOG_INFO( "Estimated distance: %.2fm (range: [%.2f, %.2f])", 
                    distance, min_distance_, max_distance_);
  
  // Check if distance is within valid range (with small tolerance for edge cases)
  // Allow 10% margin for measurement uncertainty, especially at close distances
  double range_tolerance = (max_distance_ - min_distance_) * 0.1;
  double effective_min = min_distance_ - range_tolerance;
  double effective_max = max_distance_ + range_tolerance;
  
  if (distance < effective_min || distance > effective_max) {
    LOG_WARNING( "Distance %.2f out of range [%.2f, %.2f] (effective: [%.2f, %.2f])", 
                      distance, min_distance_, max_distance_, effective_min, effective_max);
    return result;
  }
  
  // Clamp distance to valid range for validation purposes
  distance = std::max(min_distance_, std::min(max_distance_, distance));

  // Validate corners (check if they form a valid frame structure)
  if (!validateCorners(corners, distance, tape_width_, tape_height_,
                       focal_length_x_, focal_length_y_)) {
    LOG_WARNING( "Corners do not form a valid frame structure (distance=%.2fm, %zu corners)", 
                      distance, corners.size());
    return result;
  }

  // Bbox is already calculated above for occlusion detection
  
  // Calculate intensity statistics for the detected rectangle region (every 10 frames to reduce computation)
  intensity_stats_counter_++;
  if (intensity_stats_counter_ % 10 == 0 && bbox.width > 0 && bbox.height > 0) {
    // Ensure bbox is within image bounds
    bbox.x = std::max(0, std::min(bbox.x, intensity_image.cols - 1));
    bbox.y = std::max(0, std::min(bbox.y, intensity_image.rows - 1));
    bbox.width = std::min(bbox.width, intensity_image.cols - bbox.x);
    bbox.height = std::min(bbox.height, intensity_image.rows - bbox.y);
    
    if (bbox.width > 0 && bbox.height > 0) {
      cv::Mat roi = intensity_image(bbox);
      cv::Scalar mean_intensity_roi = cv::mean(roi);
      double min_intensity_roi, max_intensity_roi;
      cv::minMaxLoc(roi, &min_intensity_roi, &max_intensity_roi);
      
      // Calculate standard deviation
      cv::Mat roi_float;
      roi.convertTo(roi_float, CV_32F);
      cv::Scalar mean_scalar, stddev_scalar;
      cv::meanStdDev(roi_float, mean_scalar, stddev_scalar);
      
      LOG_DEBUG("Detected rectangle intensity stats: bbox=(%d,%d,%d,%d), "
               "mean=%.0f, min=%.0f, max=%.0f, stddev=%.0f, pixels=%d",
               bbox.x, bbox.y, bbox.width, bbox.height,
               mean_intensity_roi[0], min_intensity_roi, max_intensity_roi,
               stddev_scalar[0], bbox.width * bbox.height);
    }
  }
  
  // Apply angle calibration offset (convert degrees to radians)
  // PnP already calculated angle, so we only apply calibration
  double angle_offset_rad = angle_calibration_offset_ * M_PI / 180.0;
  angle = angle + angle_offset_rad;
  
  LOG_DEBUG("Angle from pixel offset: raw=%.3fdeg, offset=%.3fdeg, final=%.3fdeg",
            (angle - angle_offset_rad) * 180.0 / M_PI,
            angle_calibration_offset_, angle * 180.0 / M_PI);

  // Center is already calculated above (line 239) for angle calculation
  // Reuse it here instead of recalculating

  result.detected = true;
  result.bounding_box = bbox;
  result.center = center;
  result.distance = distance;
  result.inner_corners = inner_corners;  // Store inner corners for visualization
  result.angle = angle;
  result.pitch = pitch;
  result.roll = roll;
  result.pnp_y = pnp_y;  // Store PnP Y position for accurate lateral position
  result.corners = corners;
  // result.lines = detected_lines; // Removed
  // result.used_pnp_method = true;  // Removed

  // Calculate Y position for tracking validation
  double center_x = result.center.x;
  double image_center_x = static_cast<double>(cropped_image.cols) / 2.0 + principal_point_x_;
  double pixel_offset_x = center_x - image_center_x;
  double y_raw = -(pixel_offset_x * result.distance) / focal_length_x_;
  double y = (y_raw + y_position_offset_) * y_scale_factor_;
  
  // Check Y value for tracking validation
  // If Y value changed too much, it's likely tracking a different tape
  if (has_previous_detection_ && previous_y_ != 0.0) {
    double y_diff = std::abs(y - previous_y_);
    
    if (y_diff > max_y_change_) {
      LOG_WARNING("Y value changed too much (%.4f -> %.4f, diff=%.4f > %.4f). "
                  "Resetting previous detection state to allow finding correct tape.",
                  previous_y_, y, y_diff, max_y_change_);
      // Reset previous detection to allow finding the correct tape
      has_previous_detection_ = false;
      previous_center_ = cv::Point2f(-1, -1);
      previous_distance_ = -1.0;
      previous_y_ = 0.0;
      last_detection_time_ = ros::Time(0);
      // Return detection failure to trigger re-detection
      result.detected = false;
      return result;
    }
    
    LOG_DEBUG("Y tracking validation: previous=%.4f, current=%.4f, diff=%.4f <= %.4f (OK)",
              previous_y_, y, y_diff, max_y_change_);
  }

  // Update previous detection state for temporal consistency
  has_previous_detection_ = true;
  previous_center_ = result.center;  // Already adjusted for crop offset
  previous_distance_ = result.distance;
  previous_y_ = y;  // Store Y position for tracking validation
  last_detection_time_ = ros::Time::now();  // Update last successful detection time

  return result;
}

std::vector<std::pair<cv::Rect, double>> IntensityDetector::findHighIntensityRegions(const cv::Mat& intensity_image) {
  std::vector<std::pair<cv::Rect, double>> regions;

  // Convert 16-bit image to 8-bit for thresholding and contour detection
  cv::Mat image_8bit;
  if (intensity_image.type() == CV_16UC1) {
    // Normalize 16-bit to 8-bit (0-65535 -> 0-255)
    intensity_image.convertTo(image_8bit, CV_8UC1, 255.0 / 65535.0);
  } else {
    image_8bit = intensity_image;
  }

  // Convert threshold value to 8-bit scale
  double threshold_8bit = (intensity_threshold_ / 65535.0) * 255.0;

  // Threshold the intensity image
  cv::Mat thresholded;
  cv::threshold(image_8bit, thresholded, threshold_8bit, 255, cv::THRESH_BINARY);

  // Morphological operations to clean up the image
  // For frame structure, use larger kernel to connect frame edges
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
  cv::Mat cleaned;
  // Close operation to connect frame edges (important for hollow rectangles)
  cv::morphologyEx(thresholded, cleaned, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 2);
  cv::morphologyEx(cleaned, cleaned, cv::MORPH_OPEN, kernel);

  // Find contours (now supports CV_8UC1)
  // Use RETR_EXTERNAL to get only outer contours (frame outline)
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(cleaned, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  LOG_INFO("Found %zu contours after thresholding", contours.size());

  // Filter contours by area and convert to bounding boxes
  // For frame structure (hollow rectangle), the contour area will be smaller
  // Minimum area: consider frame thickness (about 2-5% of full rectangle area)
  // Reduced minimum area to detect small tapes at far distances
  double min_area = intensity_image.rows * intensity_image.cols * 0.0001;  // At least 0.01% of image (reduced for far distance detection)
  double max_area = intensity_image.rows * intensity_image.cols * 0.1;   // At most 10% of image

  for (size_t i = 0; i < contours.size(); i++) {
    const auto& contour = contours[i];
    double area = cv::contourArea(contour);
    LOG_INFO("Contour %zu: area=%.1f (min=%.1f, max=%.1f)", i, area, min_area, max_area);
    
    if (area > min_area && area < max_area) {
      cv::Rect bbox = cv::boundingRect(contour);
      LOG_INFO("Valid contour %zu: bbox=(%d,%d,%d,%d), area=%.1f", i, bbox.x, bbox.y, bbox.width, bbox.height, area);
      
      // Calculate actual intensity-weighted area from original 16-bit image
      // Use the bounding box region from original image
      cv::Rect roi = bbox & cv::Rect(0, 0, intensity_image.cols, intensity_image.rows);
      if (roi.area() > 0) {
        cv::Mat roi_16bit = intensity_image(roi);
        cv::Mat mask_roi = cleaned(roi);
        
        // Calculate sum of intensities above threshold in the ROI
        cv::Mat roi_thresholded;
        cv::threshold(roi_16bit, roi_thresholded, intensity_threshold_, 0, cv::THRESH_TOZERO);
        double intensity_sum = cv::sum(roi_thresholded)[0];
        double intensity_area = intensity_sum / intensity_threshold_;  // Normalized area
        
        regions.push_back(std::make_pair(bbox, intensity_area));
      } else {
        regions.push_back(std::make_pair(bbox, area));
      }
    }
  }

  return regions;
}

bool IntensityDetector::validateTapeShape(const cv::Rect& bbox, double distance, double contour_area) {
  // Estimate expected pixel dimensions based on distance
  double expected_width_pixels = (tape_width_ * focal_length_x_) / distance;
  double expected_height_pixels = (tape_height_ * focal_length_y_) / distance;
  double expected_full_area = expected_width_pixels * expected_height_pixels;
  
  // For frame structure (hollow rectangle), calculate expected frame area
  // Frame area = 2 * thickness * (width + height - 2*thickness)
  double expected_thickness_pixels = (tape_thickness_ * focal_length_x_) / distance;
  double expected_frame_area = 2.0 * expected_thickness_pixels * 
                                (expected_width_pixels + expected_height_pixels - 2.0 * expected_thickness_pixels);
  
  // Use frame area for validation (more accurate for hollow rectangles)
  double expected_area = expected_frame_area;
  // But also consider full area as fallback (in case frame is detected as filled)
  double expected_area_fallback = expected_full_area * 0.03;  // At least 3% for frame structure

  // Calculate actual visible area
  double actual_area = (contour_area > 0) ? contour_area : bbox.area();
  
  // Check against both frame area and minimum frame area
  double visible_ratio = actual_area / expected_area;
  double visible_ratio_fallback = actual_area / expected_area_fallback;

  // Check if visible ratio meets minimum requirement (considering occlusion and frame structure)
  // For frame structure, use lower threshold (frame area is much smaller than full area)
  // Reduced threshold for far distance detection
  double frame_min_visible_ratio = min_visible_ratio_ * 0.2;  // Frame area is ~3% of full area, reduced to 0.2 for far distance
  double min_visible_ratio_reduced = min_visible_ratio_ * 0.3;  // Reduced fallback threshold for far distance
  if (visible_ratio < frame_min_visible_ratio && visible_ratio_fallback < min_visible_ratio_reduced) {
    LOG_DEBUG("Visible ratio too low: %.3f < %.3f (frame) or %.3f < %.3f (fallback) (expected frame: %.1f, full: %.1f, actual: %.1f, bbox: %dx%d)", 
             visible_ratio, frame_min_visible_ratio, visible_ratio_fallback, min_visible_ratio_reduced,
             expected_area, expected_full_area, actual_area, bbox.width, bbox.height);
    return false;
  }

  // For partially occluded tape, allow more tolerance in size
  // If fully visible (ratio > 0.9), use strict tolerance (30%)
  // If partially visible, use relaxed tolerance (up to 50% smaller)
  double size_tolerance_factor = 0.3 + (1.0 - visible_ratio) * 0.2;  // 0.3 ~ 0.5
  double width_tolerance = expected_width_pixels * size_tolerance_factor;
  double height_tolerance = expected_height_pixels * size_tolerance_factor;

  // For occluded cases, we allow the bbox to be smaller than expected
  // but not larger (unless there's noise)
  // Increase tolerance to account for distance estimation errors and far distance detection
  // Allow up to 80% smaller for far distance detection
  double width_tolerance_lower = size_tolerance_factor + 0.5;  // Allow up to 80-100% smaller for far distance
  double height_tolerance_lower = size_tolerance_factor + 0.5;
  
  // Also add absolute minimum size check (at least 5 pixels for very far distances)
  double min_width_pixels = std::max(5.0, expected_width_pixels * 0.1);  // At least 10% of expected or 5 pixels
  double min_height_pixels = std::max(5.0, expected_height_pixels * 0.1);  // At least 10% of expected or 5 pixels
  
  bool width_ok = (bbox.width <= expected_width_pixels * (1.0 + 0.3)) &&  // Not too large
                  (bbox.width >= std::max(expected_width_pixels * (1.0 - width_tolerance_lower), min_width_pixels));  // Can be smaller if occluded or distance error
  bool height_ok = (bbox.height <= expected_height_pixels * (1.0 + 0.3)) &&  // Not too large
                   (bbox.height >= std::max(expected_height_pixels * (1.0 - height_tolerance_lower), min_height_pixels));  // Can be smaller if occluded or distance error

  // Also check aspect ratio (should be roughly maintained even when occluded)
  double aspect_ratio = static_cast<double>(bbox.width) / bbox.height;
  double expected_aspect = tape_width_ / tape_height_;
  double aspect_tolerance = 0.4;  // Allow 40% deviation in aspect ratio for occluded cases
  bool aspect_ok = std::abs(aspect_ratio - expected_aspect) / expected_aspect < aspect_tolerance;

  if (!((width_ok || height_ok) && aspect_ok)) {
    LOG_INFO("Shape validation failed: width_ok=%d, height_ok=%d, aspect_ok=%d (expected: %.1fx%.1f, actual: %dx%d, aspect: %.2f vs %.2f)",
             width_ok, height_ok, aspect_ok, expected_width_pixels, expected_height_pixels,
             bbox.width, bbox.height, aspect_ratio, expected_aspect);
  }

  return (width_ok || height_ok) && aspect_ok;  // At least one dimension should match, and aspect should be reasonable
}

double IntensityDetector::estimateDistance(const cv::Rect& bbox, double contour_area) {
  // Estimate distance based on the size of the bounding box
  // Use both width and height for more robust estimation
  
  double distance;
  
  // Calculate distance using width
  double distance_width = (tape_width_ * focal_length_x_) / bbox.width;
  
  // Calculate distance using height
  double distance_height = (tape_height_ * focal_length_y_) / bbox.height;
  
  // Use average of both for more robust estimation
  distance = (distance_width + distance_height) / 2.0;
  
  // If contour area is available, use it as a sanity check
  if (contour_area > 0) {
    double expected_area_at_distance = (tape_width_ * focal_length_x_ / distance) * 
                                       (tape_height_ * focal_length_y_ / distance);
    double area_ratio = contour_area / expected_area_at_distance;
    
    // If area ratio is significantly different, adjust distance
    // (contour_area might be intensity-weighted, so use it carefully)
    if (area_ratio < 0.7) {
      // Possibly occluded or distance underestimated, adjust slightly
      distance = distance * 1.1;  // Increase distance estimate by 10%
    }
  }
  
  // Clamp to valid range
  distance = std::max(min_distance_, std::min(max_distance_, distance));
  
  return distance;
}

cv::Point2f IntensityDetector::calculateCenter(const cv::Mat& intensity_image, const cv::Rect& bbox) {
  // Calculate weighted center based on intensity values
  cv::Mat roi = intensity_image(bbox);
  cv::Moments moments = cv::moments(roi, true);
  
  if (moments.m00 > 0) {
    cv::Point2f center_in_roi(moments.m10 / moments.m00, moments.m01 / moments.m00);
    return cv::Point2f(bbox.x + center_in_roi.x, bbox.y + center_in_roi.y);
  } else {
    // Fallback to geometric center
    return cv::Point2f(bbox.x + bbox.width / 2.0, bbox.y + bbox.height / 2.0);
  }
}

geometry_msgs::PoseStamped IntensityDetector::calculatePose(const TapeDetection& detection,
                                                             const std::string& frame_id,
                                                             int image_width,
                                                             int image_height) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = ros::Time::now();

  if (!detection.detected) {
    return pose;
  }

  // Calculate position relative to camera
  // Assuming camera coordinate system: x forward, y left, z up
  double x = detection.distance;
  
  // Calculate Y position from image center offset (most accurate for lateral camera movement)
  // Similar to Z calculation: y = -(pixel_offset_x * distance) / focal_length_x
  // Image center is at (width/2, height/2) + principal_point
  // In camera frame: positive Y means object is to the left of image center
  // In image coordinates: x increases to the right, so we need to negate
  double center_x = detection.center.x;
  double image_center_x = static_cast<double>(image_width) / 2.0 + principal_point_x_;
  double pixel_offset_x = center_x - image_center_x;
  
  // Y = -(pixel_offset_x * distance) / focal_length_x
  // Negative because: if object is to the right of image center (pixel_offset_x > 0), 
  // Y should be negative (to the right in camera frame, which is negative Y)
  double y_raw = -(pixel_offset_x * detection.distance) / focal_length_x_;
  
  // Y value calculation
  // Use y_raw directly (calculated from pixel offset and distance)
  // No rotation correction needed - y_raw already represents the actual lateral position
  // Apply scale factor to adjust Y sensitivity
  double y = (y_raw + y_position_offset_) * y_scale_factor_;
  
  LOG_DEBUG("Y calculation: pixel_offset_x=%.1f, distance=%.3fm, "
           "y_raw=%.4fm, offset=%.4fm, scale=%.2f, y_final=%.4fm",
           pixel_offset_x, detection.distance, 
           y_raw, y_position_offset_, y_scale_factor_, y);
  
  // Calculate Z position based on bottom edge of the tape (not center)
  // This gives the actual height of the rectangle's bottom edge
  // Z = (pixel_offset_y * distance) / focal_length_y
  // Image center is at (width/2, height/2) + principal_point
  // In camera frame: positive Z means object is above image center, negative Z means below
  // In image coordinates: y increases downward, so we need to negate
  
  // Use bottom edge of bounding box (or find maximum y from corners)
  double bottom_y = detection.bounding_box.y + detection.bounding_box.height;
  
  // Alternative: use maximum y from corners (more accurate for frame structure)
  if (!detection.corners.empty()) {
    double max_corner_y = detection.corners[0].position.y;
    for (const auto& corner : detection.corners) {
      if (corner.position.y > max_corner_y) {
        max_corner_y = corner.position.y;
      }
    }
    // Use the larger value (should be similar, but corner-based is more accurate)
    bottom_y = std::max(bottom_y, max_corner_y);
  }
  
  double image_center_y = static_cast<double>(image_height) / 2.0 + principal_point_y_;
  double pixel_offset_y = bottom_y - image_center_y;
  // Z = -(pixel_offset_y * distance) / focal_length_y
  // Negative because: if object is below image center (pixel_offset_y > 0), Z should be negative (below in camera frame)
  // This Z represents the height of the bottom edge of the tape
  double z_raw = -(pixel_offset_y * detection.distance) / focal_length_y_;
  
  // Apply height correction to compensate for distance-dependent variation
  // Correction formula: height = z_raw + (slope * x + offset)
  // This ensures height remains constant regardless of distance (x)
  double height = z_raw;
  if (std::abs(height_correction_slope_) > 1e-6 || std::abs(height_correction_offset_) > 1e-6) {
    // Calculate correction based on distance (x)
    double correction = height_correction_slope_ * x + height_correction_offset_;
    height = z_raw + correction;
    
    LOG_DEBUG("Height correction: z_raw=%.3fm, x=%.3fm, correction=%.3fm, height=%.3fm (reference=%.3fm)",
              z_raw, x, correction, height, reference_height_);
  }
  
  // Store both raw z and corrected height
  double z = height;

  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;


  // Calculate orientation using roll, pitch, yaw from PnP detection
  // Use detection.angle for quaternion (includes calibration offset)
  double roll = detection.roll;
  double pitch = detection.pitch;
  double yaw = detection.angle;  // Use detection.angle for orientation
  
  // Convert RPY to quaternion using tf2
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  return pose;
}

void IntensityDetector::setSelectionMode(const std::string& mode) {
  std::string mode_lower = mode;
  std::transform(mode_lower.begin(), mode_lower.end(), mode_lower.begin(), ::tolower);
  
  if (mode_lower == "center" || mode_lower == "left" || mode_lower == "right") {
    selection_mode_ = mode_lower;
    LOG_INFO("Selection mode set to: %s", selection_mode_.c_str());
  } else {
    LOG_WARNING("Invalid selection mode: %s (expected 'center', 'left', or 'right'), keeping current: %s",
                mode.c_str(), selection_mode_.c_str());
  }
}

} // namespace nc_intensity_detector
