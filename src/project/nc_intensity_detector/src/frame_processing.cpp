#include "nc_intensity_detector/frame_processing.h"
#include "nc_intensity_detector/distance_estimation.h"
#include <util/logger.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace nc_intensity_detector {

std::vector<std::vector<Corner>> groupCornersIntoFrames(
    const std::vector<Corner>& corners,
    double distance,
    double tape_width,
    double tape_height,
    double focal_length_x,
    double focal_length_y) {
  std::vector<std::vector<Corner>> frame_groups;
  
  if (corners.size() < 2) {
    LOG_INFO("Cannot group corners: need at least 2 corners, found %zu", corners.size());
    return frame_groups;
  }

  // Check if initial distance is reasonable, if not use middle of range
  double min_distance = 1.0;
  double max_distance = 3.0;
  if (distance < min_distance * 0.5 || distance > max_distance * 1.5) {
    distance = (min_distance + max_distance) / 2.0;
    LOG_DEBUG("Initial distance estimate out of reasonable range, using middle value: %.2fm", distance);
  }

  // Calculate expected corner distances based on frame dimensions
  double expected_width_pixels = (tape_width * focal_length_x) / distance;
  double expected_height_pixels = (tape_height * focal_length_y) / distance;
  double max_corner_distance = std::sqrt(expected_width_pixels * expected_width_pixels + 
                                         expected_height_pixels * expected_height_pixels);
  double tolerance = max_corner_distance * 0.5;  // 50% tolerance

  LOG_DEBUG("Grouping corners: expected width=%.1f, height=%.1f, diagonal=%.1f pixels (distance=%.2fm)", 
           expected_width_pixels, expected_height_pixels, max_corner_distance, distance);

  // Improved grouping: try to find all corners that belong to the same frame
  // For a rectangular frame, we expect 4 corners, but can work with 2+
  // Strategy: group corners that form reasonable geometric relationships
  
  // If we have 4+ corners, try to find the best rectangle
  if (corners.size() >= 4) {
    // Try all combinations to find the best rectangle
    // For now, use a simpler approach: group all corners that are within reasonable bounds
    std::vector<Corner> frame;
    
    // Calculate bounding box of all corners
    float min_x = corners[0].position.x, max_x = corners[0].position.x;
    float min_y = corners[0].position.y, max_y = corners[0].position.y;
    for (const auto& c : corners) {
      min_x = std::min(min_x, c.position.x);
      max_x = std::max(max_x, c.position.x);
      min_y = std::min(min_y, c.position.y);
      max_y = std::max(max_y, c.position.y);
    }
    
    // Check if bounding box dimensions match expected frame size (with tolerance)
    double bbox_width = max_x - min_x;
    double bbox_height = max_y - min_y;
    double width_ratio = bbox_width / expected_width_pixels;
    double height_ratio = bbox_height / expected_height_pixels;
    
    // If dimensions are reasonable (within 50% tolerance), use all corners
    if (width_ratio > 0.5 && width_ratio < 1.5 && height_ratio > 0.5 && height_ratio < 1.5) {
      frame = corners;
      frame_groups.push_back(frame);
      LOG_DEBUG("Grouped all %zu corners into single frame (bbox: %.1fx%.1f, expected: %.1fx%.1f)", 
                corners.size(), bbox_width, bbox_height, expected_width_pixels, expected_height_pixels);
    }
  }
  
  // Fallback: original grouping logic for fewer corners or if above failed
  if (frame_groups.empty()) {
    std::vector<bool> used(corners.size(), false);

    for (size_t i = 0; i < corners.size(); i++) {
      if (used[i]) continue;

      std::vector<Corner> frame;
      frame.push_back(corners[i]);
      used[i] = true;

      // Find corners that are at expected distances from this corner
      // Use more lenient tolerance for grouping
      for (size_t j = i + 1; j < corners.size(); j++) {
        if (used[j]) continue;

        double dx = corners[i].position.x - corners[j].position.x;
        double dy = corners[i].position.y - corners[j].position.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        // Check if distance matches expected width, height, or diagonal (more lenient)
        double width_tolerance = expected_width_pixels * 0.5;  // Increased from 0.3
        double height_tolerance = expected_height_pixels * 0.5;  // Increased from 0.3
        bool matches_width = std::abs(dist - expected_width_pixels) < width_tolerance;
        bool matches_height = std::abs(dist - expected_height_pixels) < height_tolerance;
        bool matches_diagonal = std::abs(dist - max_corner_distance) < tolerance;

        if (matches_width || matches_height || matches_diagonal) {
          frame.push_back(corners[j]);
          used[j] = true;
        }
      }

      // A valid frame should have at least 2 corners
      if (frame.size() >= 2) {
        frame_groups.push_back(frame);
        LOG_DEBUG("Created frame group %zu with %zu corners", frame_groups.size() - 1, frame.size());
      }
    }
  }

  return frame_groups;
}

std::vector<Corner> selectCenterFrame(
    const std::vector<std::vector<Corner>>& frame_groups,
    const cv::Mat& intensity_image,
    double principal_point_x,
    double principal_point_y) {
  if (frame_groups.empty()) {
    return std::vector<Corner>();
  }

  // Calculate image center
  double image_center_x = intensity_image.cols / 2.0 + principal_point_x;
  double image_center_y = intensity_image.rows / 2.0 + principal_point_y;

  double min_distance_to_center = std::numeric_limits<double>::max();
  size_t best_frame_idx = 0;

  // Find the frame whose center is closest to image center
  for (size_t i = 0; i < frame_groups.size(); i++) {
    cv::Point2f frame_center = calculateCenterFromCorners(frame_groups[i]);
    
    double dx = frame_center.x - image_center_x;
    double dy = frame_center.y - image_center_y;
    double distance_to_center = std::sqrt(dx * dx + dy * dy);

    LOG_DEBUG("Frame %zu: center=(%.1f,%.1f), distance_to_image_center=%.1f pixels", 
             i, frame_center.x, frame_center.y, distance_to_center);

    if (distance_to_center < min_distance_to_center) {
      min_distance_to_center = distance_to_center;
      best_frame_idx = i;
    }
  }

  LOG_DEBUG("Selected frame %zu (closest to center, distance=%.1f pixels)", best_frame_idx, min_distance_to_center);
  return frame_groups[best_frame_idx];
}

bool validateCorners(
    const std::vector<Corner>& corners,
    double distance,
    double tape_width,
    double tape_height,
    double focal_length_x,
    double focal_length_y) {
  if (corners.size() < 2) {
    LOG_INFO("Corner validation failed: need at least 2 corners, found %zu", corners.size());
    return false;
  }

  // Check if corners form a reasonable rectangle
  // Calculate expected corner distances based on frame dimensions
  double expected_width_pixels = (tape_width * focal_length_x) / distance;
  double expected_height_pixels = (tape_height * focal_length_y) / distance;
  double expected_diagonal_pixels = std::sqrt(expected_width_pixels * expected_width_pixels + 
                                              expected_height_pixels * expected_height_pixels);
  
  LOG_DEBUG("Corner validation: expected width=%.1f, height=%.1f, diagonal=%.1f pixels (distance=%.2fm)", 
           expected_width_pixels, expected_height_pixels, expected_diagonal_pixels, distance);
  
  // Find corners that are approximately at expected distances
  int valid_pairs_width = 0;
  int valid_pairs_height = 0;
  int valid_pairs_diagonal = 0;
  
  // Store actual distances for debugging
  std::vector<double> actual_distances;
  
  for (size_t i = 0; i < corners.size(); i++) {
    for (size_t j = i + 1; j < corners.size(); j++) {
      double dx = corners[i].position.x - corners[j].position.x;
      double dy = corners[i].position.y - corners[j].position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      actual_distances.push_back(dist);
      
      // Check if distance matches expected width, height, or diagonal (with tolerance)
      // Increased tolerance from 30% to 50% for more robust detection, especially at close distances
      double width_tolerance = expected_width_pixels * 0.5;
      double height_tolerance = expected_height_pixels * 0.5;
      double diagonal_tolerance = expected_diagonal_pixels * 0.5;
      
      if (std::abs(dist - expected_width_pixels) < width_tolerance) {
        valid_pairs_width++;
        LOG_DEBUG("Valid width pair: dist=%.1f, expected=%.1f, diff=%.1f", 
                 dist, expected_width_pixels, std::abs(dist - expected_width_pixels));
      }
      if (std::abs(dist - expected_height_pixels) < height_tolerance) {
        valid_pairs_height++;
        LOG_DEBUG("Valid height pair: dist=%.1f, expected=%.1f, diff=%.1f", 
                 dist, expected_height_pixels, std::abs(dist - expected_height_pixels));
      }
      if (std::abs(dist - expected_diagonal_pixels) < diagonal_tolerance) {
        valid_pairs_diagonal++;
        LOG_DEBUG("Valid diagonal pair: dist=%.1f, expected=%.1f, diff=%.1f", 
                 dist, expected_diagonal_pixels, std::abs(dist - expected_diagonal_pixels));
      }
    }
  }
  
  // Log actual corner distances for debugging
  if (!actual_distances.empty()) {
    double min_dist = *std::min_element(actual_distances.begin(), actual_distances.end());
    double max_dist = *std::max_element(actual_distances.begin(), actual_distances.end());
    LOG_DEBUG("Corner validation: actual distances range [%.1f, %.1f] pixels, width_pairs=%d, height_pairs=%d, diagonal_pairs=%d", 
           min_dist, max_dist, valid_pairs_width, valid_pairs_height, valid_pairs_diagonal);
  } else {
    LOG_DEBUG("Corner validation: no corner pairs found");
  }
  
  // Need at least one valid pair (width, height, or diagonal)
  bool valid = (valid_pairs_width > 0 || valid_pairs_height > 0 || valid_pairs_diagonal > 0);
  
  if (!valid) {
    LOG_WARNING("Corner validation failed: no valid corner pairs found (expected: width=%.1f, height=%.1f, diagonal=%.1f)", 
               expected_width_pixels, expected_height_pixels, expected_diagonal_pixels);
  }
  
  return valid;
}

cv::Rect reconstructFrameFromCorners(const std::vector<Corner>& corners) {
  if (corners.empty()) {
    return cv::Rect();
  }

  // Find bounding box of all corners
  float min_x = corners[0].position.x;
  float max_x = corners[0].position.x;
  float min_y = corners[0].position.y;
  float max_y = corners[0].position.y;

  for (const auto& corner : corners) {
    min_x = std::min(min_x, corner.position.x);
    max_x = std::max(max_x, corner.position.x);
    min_y = std::min(min_y, corner.position.y);
    max_y = std::max(max_y, corner.position.y);
  }

  return cv::Rect(static_cast<int>(min_x), static_cast<int>(min_y),
                  static_cast<int>(max_x - min_x), static_cast<int>(max_y - min_y));
}

cv::Point2f calculateCenterFromCorners(const std::vector<Corner>& corners) {
  if (corners.empty()) {
    return cv::Point2f(0, 0);
  }

  // Calculate centroid of all corners
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  
  for (const auto& corner : corners) {
    sum_x += corner.position.x;
    sum_y += corner.position.y;
  }

  return cv::Point2f(sum_x / corners.size(), sum_y / corners.size());
}

} // namespace nc_intensity_detector

