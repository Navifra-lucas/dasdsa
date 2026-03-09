#include "nc_intensity_detector/corner_detection.h"
#include <util/logger.hpp>
#include <algorithm>
#include <map>
#include <cmath>
#include <limits>
#include <string>
#include <cctype>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace nc_intensity_detector {

// Helper function to correct camera distortion for a point using OpenCV's undistortPoints
// This is more accurate than manual implementation
void correctDistortion(const std::vector<cv::Point2f>& distorted_points,
                       std::vector<cv::Point2f>& undistorted_points,
                       double k1, double k2, double k3,
                       double p1, double p2,
                       double cx, double cy,
                       double fx, double fy) {
  if (distorted_points.empty()) {
    undistorted_points.clear();
    return;
  }
  
  // Build camera matrix
  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    fx, 0.0, cx,
    0.0, fy, cy,
    0.0, 0.0, 1.0);
  
  // Build distortion coefficients
  cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
  
  // Use OpenCV's undistortPoints for accurate distortion correction
  cv::undistortPoints(distorted_points, undistorted_points, 
                      camera_matrix, dist_coeffs,
                      cv::noArray(), camera_matrix);
}

std::vector<Corner> detectCornersPnP(const cv::Mat& intensity_image,
                                      const cv::Mat& thresholded,
                                      double tape_width,
                                      double tape_height,
                                      double tape_thickness,
                                      double focal_length_x,
                                      double focal_length_y,
                                      double distortion_k1,
                                      double distortion_k2,
                                      double distortion_k3,
                                      double distortion_p1,
                                      double distortion_p2,
                                      double principal_point_x,
                                      double principal_point_y,
                                      double& distance,
                                      double& angle,
                                      double& pitch,
                                      double& roll,
                                      double& pnp_y,
                                      std::vector<Corner>& inner_corners,
                                      const std::string& selection_mode,
                                      const cv::Point2f& previous_center,
                                      double previous_distance,
                                      double previous_y) {
  std::vector<Corner> corners;
  distance = 0.0;
  angle = 0.0;
  pitch = 0.0;
  roll = 0.0;
  pnp_y = 0.0;
  inner_corners.clear();  // Initialize inner corners
  
  // Step 1: Find contours (only external contours to avoid detecting inner rectangle)
  // Use RETR_EXTERNAL to get only outer contours (frame outline)
  std::vector<std::vector<cv::Point>> contours;
  cv::Mat hierarchy;
  cv::findContours(thresholded, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  
  if (contours.empty()) {
    LOG_DEBUG("PnP method: No contours found");
    return corners;
  }

  
  // Step 2: Find all valid contours and process them to get Y values
  // Then select the best one based on Y-value tracking or center position
  double image_center_x = intensity_image.cols / 2.0;
  double image_center_y = intensity_image.rows / 2.0;
  
  // Minimum area threshold: adapt to image size for small objects at far distance
  double min_area = intensity_image.rows * intensity_image.cols * 0.00005;  // 0.005% of image (reduced for far distance)
  if (min_area < 50.0) {
    min_area = 50.0;  // Absolute minimum to avoid noise
  }
  
  // Structure to store valid contour information
  struct ContourInfo {
    size_t idx;
    double area;
    cv::Point2f center;
    double distance_from_center;
    double estimated_y;  // Y value calculated from pixel offset
  };
  
  std::vector<ContourInfo> valid_contours;
  
  // First pass: find all valid contours and calculate their Y values
  for (size_t i = 0; i < contours.size(); i++) {
    double area = cv::contourArea(contours[i]);
    if (area < min_area) {
      continue;  // Skip contours that are too small
    }
    
    // Calculate contour center
    cv::Moments moments = cv::moments(contours[i]);
    if (moments.m00 == 0.0) {
      continue;
    }
    double contour_center_x = moments.m10 / moments.m00;
    double contour_center_y = moments.m01 / moments.m00;
    
    // Calculate distance from image center
    double dx = contour_center_x - image_center_x;
    double dy = contour_center_y - image_center_y;
    double distance_from_center = std::sqrt(dx * dx + dy * dy);
    
    // Estimate distance from area (rough approximation for Y calculation)
    double estimated_distance = std::sqrt(tape_width * tape_height * focal_length_x * focal_length_y / area);
    
    // Calculate Y value from pixel offset
    double pixel_offset_x = contour_center_x - image_center_x;
    double cx = intensity_image.cols / 2.0 + principal_point_x;
    double image_center_x_corrected = cx;
    pixel_offset_x = contour_center_x - image_center_x_corrected;
    double estimated_y = -(pixel_offset_x * estimated_distance) / focal_length_x;
    
    ContourInfo info;
    info.idx = i;
    info.area = area;
    info.center = cv::Point2f(contour_center_x, contour_center_y);
    info.distance_from_center = distance_from_center;
    info.estimated_y = estimated_y;
    
    valid_contours.push_back(info);
  }
  
  if (valid_contours.empty()) {
    LOG_DEBUG("PnP method: No valid contour found (min area: %.1f)", min_area);
    return corners;
  }
  
  LOG_DEBUG("PnP method: Found %zu valid contours", valid_contours.size());
  
  // Step 3: Select the best contour
  size_t best_contour_idx = 0;
  
  // If we have previous Y value, prioritize Y-value based tracking
  bool use_y_tracking = (previous_y != 0.0);
  
  if (use_y_tracking) {
    // Find contour with Y value closest to previous Y
    double min_y_diff = std::numeric_limits<double>::max();
    for (size_t j = 0; j < valid_contours.size(); j++) {
      double y_diff = std::abs(valid_contours[j].estimated_y - previous_y);
      if (y_diff < min_y_diff) {
        min_y_diff = y_diff;
        best_contour_idx = j;
      }
    }
    LOG_DEBUG("PnP method: Selected contour %zu based on Y-value tracking (y=%.4f, prev_y=%.4f, diff=%.4f)",
              valid_contours[best_contour_idx].idx, valid_contours[best_contour_idx].estimated_y, 
              previous_y, min_y_diff);
  } else {
    // No previous Y value: select contour closest to center (sorted by distance from center)
    std::sort(valid_contours.begin(), valid_contours.end(),
              [](const ContourInfo& a, const ContourInfo& b) {
                return a.distance_from_center < b.distance_from_center;
              });
    best_contour_idx = 0;  // First one is closest to center
    LOG_DEBUG("PnP method: Selected contour %zu closest to center (dist=%.1f, y=%.4f)",
              valid_contours[best_contour_idx].idx, valid_contours[best_contour_idx].distance_from_center,
              valid_contours[best_contour_idx].estimated_y);
  }
  
  // Get the actual contour index
  size_t selected_contour_idx = valid_contours[best_contour_idx].idx;
  double selected_area = valid_contours[best_contour_idx].area;
  cv::Point2f selected_center = valid_contours[best_contour_idx].center;
  
  LOG_DEBUG("PnP method: Selected contour %zu (area=%.1f, center=(%.1f,%.1f), dist_from_center=%.1f, y=%.4f)",
            selected_contour_idx, selected_area, selected_center.x, selected_center.y,
            valid_contours[best_contour_idx].distance_from_center, valid_contours[best_contour_idx].estimated_y);
  
  // Step 4: Approximate contour to polygon (should be 4 points for rectangle)
  std::vector<cv::Point> approx_contour;
  // For small objects at far distance, use smaller epsilon to preserve corner details
  // Adaptive epsilon: smaller for smaller contours
  double contour_perimeter = cv::arcLength(contours[selected_contour_idx], true);
  double epsilon_ratio = 0.02;  // Default 2% of perimeter
  if (contour_perimeter < 100.0) {
    // For small contours (far distance), use smaller epsilon ratio
    epsilon_ratio = 0.03;  // Slightly larger ratio but still preserves corners
  }
  double epsilon = epsilon_ratio * contour_perimeter;
  cv::approxPolyDP(contours[selected_contour_idx], approx_contour, epsilon, true);
  
  if (approx_contour.size() != 4) {
    LOG_DEBUG("PnP method: Contour has %zu points, expected 4 (perimeter=%.1f, epsilon=%.2f, ratio=%.3f)", 
              approx_contour.size(), contour_perimeter, epsilon, epsilon_ratio);
    // Try with even smaller epsilon if first attempt failed
    if (approx_contour.size() > 4) {
      epsilon = 0.015 * contour_perimeter;  // Smaller epsilon
      cv::approxPolyDP(contours[selected_contour_idx], approx_contour, epsilon, true);
      LOG_DEBUG("PnP method: Retry with smaller epsilon (%.2f), got %zu points", epsilon, approx_contour.size());
    }
    if (approx_contour.size() != 4) {
      return corners;
    }
  }
  
  // Step 4: Convert to Point2f for SubPix
  std::vector<cv::Point2f> corner_points;
  for (const auto& pt : approx_contour) {
    corner_points.push_back(cv::Point2f(static_cast<float>(pt.x), static_cast<float>(pt.y)));
  }
  
  // Step 5: Apply Corner SubPix refinement for maximum precision
  // Convert 16-bit to 8-bit with better precision preservation
  cv::Mat image_for_subpix;
  if (intensity_image.type() == CV_16UC1) {
    // Use better scaling to preserve more information in high-intensity regions
    // Normalize to 8-bit while preserving relative intensity differences
    intensity_image.convertTo(image_for_subpix, CV_8UC1, 255.0 / 65535.0);
  } else {
    image_for_subpix = intensity_image;
  }
  
  // Enhanced SubPix parameters for maximum precision:
  // - Larger window (7x7) for better gradient estimation
  // - More iterations (40) for convergence
  // - Tighter epsilon (0.001) for sub-pixel accuracy
  cv::Size winSize(7, 7);  // Increased from 5x5 for better precision
  cv::Size zeroZone(-1, -1);
  cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.001);  // More iterations, tighter epsilon
  
  try {
    cv::cornerSubPix(image_for_subpix, corner_points, winSize, zeroZone, criteria);
    LOG_DEBUG("PnP method: Corner SubPix refinement applied (winSize=7x7, maxIter=40, epsilon=0.001)");
  } catch (const cv::Exception& e) {
    LOG_WARNING("PnP method: Corner SubPix failed: %s, trying with default parameters", e.what());
    // Fallback to default parameters if enhanced version fails
    winSize = cv::Size(5, 5);
    criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01);
    try {
      cv::cornerSubPix(image_for_subpix, corner_points, winSize, zeroZone, criteria);
      LOG_DEBUG("PnP method: Corner SubPix applied with default parameters");
    } catch (const cv::Exception& e2) {
      LOG_WARNING("PnP method: Corner SubPix failed even with default parameters: %s", e2.what());
      return corners;
    }
  }
  
  // Step 6: Apply distortion correction if needed
  bool has_distortion = (std::abs(distortion_k1) > 1e-6 || std::abs(distortion_k2) > 1e-6 || 
                         std::abs(distortion_k3) > 1e-6 || std::abs(distortion_p1) > 1e-6 || 
                         std::abs(distortion_p2) > 1e-6);
  
  if (has_distortion && !intensity_image.empty()) {
    double cx = intensity_image.cols / 2.0 + principal_point_x;
    double cy = intensity_image.rows / 2.0 + principal_point_y;
    
    std::vector<cv::Point2f> undistorted_points;
    correctDistortion(corner_points, undistorted_points,
                      distortion_k1, distortion_k2, distortion_k3,
                      distortion_p1, distortion_p2,
                      cx, cy, focal_length_x, focal_length_y);
    
    if (undistorted_points.size() == corner_points.size()) {
      corner_points = undistorted_points;
      LOG_DEBUG("PnP method: Distortion correction applied");
    }
  }
  
  // Step 7: Order corners (TL, TR, BR, BL) - Robust method using stable sorting
  // This prevents corner order from changing between frames due to small detection variations
  
  if (corner_points.size() != 4) {
    LOG_DEBUG("PnP method: Expected 4 corners, got %zu", corner_points.size());
    return corners;
  }
  
  // Method: Sort all corners by y-coordinate, then separate into top/bottom
  // This is more stable than using center.y as threshold
  std::vector<std::pair<float, cv::Point2f>> corners_with_y;
  for (const auto& pt : corner_points) {
    corners_with_y.push_back(std::make_pair(pt.y, pt));
  }
  
  // Sort by y-coordinate (ascending: top to bottom)
  std::sort(corners_with_y.begin(), corners_with_y.end(),
            [](const std::pair<float, cv::Point2f>& a, const std::pair<float, cv::Point2f>& b) {
              return a.first < b.first;
            });
  
  // Top 2 corners (smallest y) = top row
  // Bottom 2 corners (largest y) = bottom row
  std::vector<cv::Point2f> top_points, bottom_points;
  top_points.push_back(corners_with_y[0].second);      // Top-left or top-right
  top_points.push_back(corners_with_y[1].second);      // Top-right or top-left
  bottom_points.push_back(corners_with_y[2].second);   // Bottom-left or bottom-right
  bottom_points.push_back(corners_with_y[3].second);   // Bottom-right or bottom-left
  
  // Sort top row by x (left to right)
  if (top_points[0].x > top_points[1].x) {
    std::swap(top_points[0], top_points[1]);
  }
  
  // Sort bottom row by x (left to right)
  if (bottom_points[0].x > bottom_points[1].x) {
    std::swap(bottom_points[0], bottom_points[1]);
  }
  
  // Verify: top row should have smaller y than bottom row
  // If not, there might be an issue with corner detection
  float top_y_avg = (top_points[0].y + top_points[1].y) / 2.0f;
  float bottom_y_avg = (bottom_points[0].y + bottom_points[1].y) / 2.0f;
  
  if (top_y_avg >= bottom_y_avg) {
    LOG_WARNING("PnP method: Top/bottom row ordering issue (top_y_avg=%.1f >= bottom_y_avg=%.1f). "
                "This may cause inconsistent angle values.", top_y_avg, bottom_y_avg);
  }
  
  // Ordered corners: TL, TR, BR, BL
  std::vector<cv::Point2f> ordered_corners;
  ordered_corners.push_back(top_points[0]);      // TL
  ordered_corners.push_back(top_points[1]);      // TR
  ordered_corners.push_back(bottom_points[1]);  // BR (rightmost of bottom row)
  ordered_corners.push_back(bottom_points[0]);   // BL (leftmost of bottom row)
  
  LOG_DEBUG("PnP method: Ordered corners - TL=(%.1f,%.1f), TR=(%.1f,%.1f), BR=(%.1f,%.1f), BL=(%.1f,%.1f)",
            ordered_corners[0].x, ordered_corners[0].y,
            ordered_corners[1].x, ordered_corners[1].y,
            ordered_corners[2].x, ordered_corners[2].y,
            ordered_corners[3].x, ordered_corners[3].y);
  
  // Step 8: Define 3D object points in object coordinate system
  // Object coordinate system (standard OpenCV convention):
  //   X: right (image horizontal direction)
  //   Y: down (image vertical direction)
  //   Z: forward (toward camera, positive Z = closer to camera)
  // Camera coordinate system:
  //   X: forward (toward object)
  //   Y: left
  //   Z: up
  // solvePnP computes transformation from object frame to camera frame
  // For a flat tape facing camera, all points have Z=0 (on the object plane)
  // Tape center at origin, corners in order TL, TR, BR, BL
  std::vector<cv::Point3f> object_points;
  object_points.push_back(cv::Point3f(-tape_width / 2.0, -tape_height / 2.0, 0.0));  // TL
  object_points.push_back(cv::Point3f(tape_width / 2.0, -tape_height / 2.0, 0.0));   // TR
  object_points.push_back(cv::Point3f(tape_width / 2.0, tape_height / 2.0, 0.0));    // BR
  object_points.push_back(cv::Point3f(-tape_width / 2.0, tape_height / 2.0, 0.0));    // BL
  
  // Step 9: Build camera matrix
  double cx = intensity_image.cols / 2.0 + principal_point_x;
  double cy = intensity_image.rows / 2.0 + principal_point_y;
  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    focal_length_x, 0.0, cx,
    0.0, focal_length_y, cy,
    0.0, 0.0, 1.0);
  
  // Step 10: Build distortion coefficients
  cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << distortion_k1, distortion_k2, distortion_p1, distortion_p2, distortion_k3);
  
  // Step 11: Solve PnP for outer rectangle
  cv::Mat rvec_outer, tvec_outer;
  bool success_outer = cv::solvePnP(object_points, ordered_corners, camera_matrix, dist_coeffs, rvec_outer, tvec_outer, false, cv::SOLVEPNP_ITERATIVE);
  
  if (!success_outer) {
    LOG_DEBUG("PnP method: solvePnP failed for outer rectangle");
    return corners;
  }
  
  // Step 11b: Inner rectangle PnP processing - DISABLED (causing program hang)
  // Inner rectangle size: outer size minus 2 * thickness
  // double inner_width = tape_width - 2.0 * tape_thickness;
  // double inner_height = tape_height - 2.0 * tape_thickness;
  
  // Inner rectangle PnP processing is disabled to avoid program hang
  // TODO: Debug and re-enable inner rectangle processing if needed
  inner_corners.clear();  // Clear inner corners (not used)
  
  // Step 11c: Use outer rectangle results only
  cv::Mat rvec, tvec;
  // Use outer rectangle only
  rvec = rvec_outer;
  tvec = tvec_outer;
  LOG_DEBUG("PnP method: Using outer rectangle only (inner rectangle processing disabled)");
  
  // Extract distance and position from tvec
  // tvec[0] = X (forward), tvec[1] = Y (left), tvec[2] = Z (up) in camera coordinate system
  distance = std::sqrt(tvec.at<double>(0) * tvec.at<double>(0) + 
                       tvec.at<double>(1) * tvec.at<double>(1) + 
                       tvec.at<double>(2) * tvec.at<double>(2));
  
  // Extract Y position directly from tvec[1]
  // This is more accurate than calculating from angle, especially when camera moves laterally
  pnp_y = tvec.at<double>(1);
  
  // Convert rotation vector to rotation matrix, then extract roll, pitch, yaw
  cv::Mat rotation_matrix;
  cv::Rodrigues(rvec, rotation_matrix);
  
  // Extract Euler angles (roll, pitch, yaw) from rotation matrix
  // solvePnP returns rotation from object coordinate system to camera coordinate system
  // Object coordinate: X right, Y down, Z forward
  // Camera coordinate: X forward, Y left, Z up
  // 
  // For a flat object facing camera, when camera rotates by +theta around Z axis:
  // - Object appears rotated by -theta in camera frame
  // - This should be reflected in the yaw angle
  //
  // Using ZYX convention (yaw-pitch-roll) for camera coordinate system
  // R = Rz(yaw) * Ry(pitch) * Rx(roll)
  // Camera coordinate: X forward, Y left, Z up
  
  // Check if rotation matrix is valid (determinant should be close to 1)
  double det = cv::determinant(rotation_matrix);
  if (std::abs(det - 1.0) > 0.1) {
    LOG_WARNING("PnP method: Invalid rotation matrix (det=%.3f)", det);
  }
  
  // Extract Euler angles using standard ZYX convention
  // For a flat object, we're mainly interested in yaw (rotation around Z axis)
  // Pitch and roll should be small for a flat object facing camera
  double sy = std::sqrt(rotation_matrix.at<double>(0, 0) * rotation_matrix.at<double>(0, 0) + 
                        rotation_matrix.at<double>(1, 0) * rotation_matrix.at<double>(1, 0));
  
  bool singular = sy < 1e-6;  // Check for gimbal lock
  
  if (!singular) {
    pitch = std::atan2(-rotation_matrix.at<double>(2, 0), sy);
    roll = std::atan2(rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));
    // Yaw: rotation around Z axis (camera's up direction)
    // This represents how much the object is rotated relative to camera
    // When camera rotates +theta, object appears rotated -theta, so yaw should be -theta
    angle = std::atan2(rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(0, 0));
  } else {
    // Gimbal lock case - use alternative extraction
    pitch = std::atan2(-rotation_matrix.at<double>(2, 0), sy);
    roll = std::atan2(-rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(1, 1));
    angle = 0.0;
  }
  
  // Normalize angle to [-180, 180] degrees range
  // If angle is very large (close to ±180), it might be wrapped incorrectly
  if (angle > M_PI) {
    angle -= 2.0 * M_PI;
  } else if (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  
  // Log rotation matrix for debugging
  LOG_DEBUG("PnP rotation matrix:\n"
            "  [%.4f, %.4f, %.4f]\n"
            "  [%.4f, %.4f, %.4f]\n"
            "  [%.4f, %.4f, %.4f]",
            rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
            rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));
  
  // For a flat tape, roll and pitch should be close to zero
  // However, due to coordinate system mismatch between object frame and camera frame,
  // PnP may return large roll/pitch values even for a flat object
  // Solution: Clamp roll and pitch to small values (e.g., ±5 degrees) or set to zero
  // Since we're detecting a flat tape, roll and pitch are not meaningful and should be near zero
  
  // Store original values for logging
  double roll_original = roll;
  double pitch_original = pitch;
  
  // Clamp roll and pitch to reasonable range for flat tape (±5 degrees)
  // This prevents unrealistic values due to coordinate system issues
  const double max_roll_pitch = 5.0 * M_PI / 180.0;  // 5 degrees in radians
  if (std::abs(roll) > max_roll_pitch) {
    LOG_WARNING("PnP method: Roll angle too large (%.2f deg), clamping to %.2f deg. "
                "This may indicate coordinate system issue. Original: %.2f deg",
                roll * 180.0 / M_PI, max_roll_pitch * 180.0 / M_PI, roll_original * 180.0 / M_PI);
    roll = (roll > 0) ? max_roll_pitch : -max_roll_pitch;
  }
  if (std::abs(pitch) > max_roll_pitch) {
    LOG_WARNING("PnP method: Pitch angle too large (%.2f deg), clamping to %.2f deg. "
                "This may indicate coordinate system issue. Original: %.2f deg",
                pitch * 180.0 / M_PI, max_roll_pitch * 180.0 / M_PI, pitch_original * 180.0 / M_PI);
    pitch = (pitch > 0) ? max_roll_pitch : -max_roll_pitch;
  }
  
  // Alternative: Set roll and pitch to zero for flat tape (uncomment if clamping is not enough)
  // roll = 0.0;
  // pitch = 0.0;
  
  if (std::abs(angle) > M_PI / 2.0) {
    LOG_WARNING("PnP method: Yaw angle seems large (%.2f deg), may indicate coordinate system issue", angle * 180.0 / M_PI);
  }
  
  LOG_DEBUG("PnP orientation: roll=%.3f (original: %.3f), pitch=%.3f (original: %.3f), yaw=%.3f deg",
            roll * 180.0 / M_PI, roll_original * 180.0 / M_PI,
            pitch * 180.0 / M_PI, pitch_original * 180.0 / M_PI,
            angle * 180.0 / M_PI);
  
  // Step 12: Create Corner structures from ordered corners
  for (const auto& pt : ordered_corners) {
    Corner corner;
    corner.position = pt;
    corner.angle = 90.0;
    corner.confidence = 1.0;
    corners.push_back(corner);
  }
  
  LOG_INFO("PnP method: Successfully detected 4 corners, distance=%.2fm, roll=%.3fdeg, pitch=%.3fdeg, yaw=%.3fdeg",
           distance, roll * 180.0 / M_PI, pitch * 180.0 / M_PI, angle * 180.0 / M_PI);
  LOG_DEBUG("PnP corner positions: TL=(%.2f,%.2f), TR=(%.2f,%.2f), BR=(%.2f,%.2f), BL=(%.2f,%.2f)",
           corners[0].position.x, corners[0].position.y,
           corners[1].position.x, corners[1].position.y,
           corners[2].position.x, corners[2].position.y,
           corners[3].position.x, corners[3].position.y);
  
  return corners;
}

} // namespace nc_intensity_detector
