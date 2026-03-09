#include "nc_intensity_detector/distance_estimation.h"
#include "nc_intensity_detector/frame_processing.h"
#include <util/logger.hpp>
#include <algorithm>
#include <cmath>

namespace nc_intensity_detector {

double estimateDistanceFromCorners(
    const std::vector<Corner>& corners,
    double tape_width,
    double tape_height,
    double tape_thickness,
    double focal_length_x,
    double focal_length_y,
    double min_distance,
    double max_distance) {
  if (corners.size() < 2) {
    return (min_distance + max_distance) / 2.0;  // Default to middle of range
  }

  // Calculate bounding box from corners (more reliable than corner-to-corner distance)
  // NOTE: This bbox represents the INNER boundary of the frame (detected corners)
  cv::Rect bbox = reconstructFrameFromCorners(corners);
  double bbox_width_pixels = static_cast<double>(bbox.width);
  double bbox_height_pixels = static_cast<double>(bbox.height);
  
  // Step 1: Initial distance estimate from inner boundary
  double initial_distance_from_width = 0.0;
  double initial_distance_from_height = 0.0;
  
  if (bbox_width_pixels > 5.0) {
    // Inner width = outer width - 2 * thickness
    // So: outer_width = inner_width + 2 * thickness
    // But we need distance first to convert thickness to pixels...
    // Use inner boundary as approximation for initial estimate
    initial_distance_from_width = (tape_width * focal_length_x) / bbox_width_pixels;
  }
  if (bbox_height_pixels > 5.0) {
    initial_distance_from_height = (tape_height * focal_length_y) / bbox_height_pixels;
  }
  
  // Get initial distance estimate (average of width and height)
  double initial_distance = 0.0;
  if (initial_distance_from_width > 0.0 && initial_distance_from_height > 0.0) {
    initial_distance = (initial_distance_from_width + initial_distance_from_height) / 2.0;
  } else if (initial_distance_from_width > 0.0) {
    initial_distance = initial_distance_from_width;
  } else if (initial_distance_from_height > 0.0) {
    initial_distance = initial_distance_from_height;
  } else {
    initial_distance = (min_distance + max_distance) / 2.0;
  }
  
  // Step 2: Convert frame thickness to pixels at this distance
  double thickness_pixels_x = (tape_thickness * focal_length_x) / initial_distance;
  double thickness_pixels_y = (tape_thickness * focal_length_y) / initial_distance;
  
  // Step 3: Estimate outer boundary (add thickness on both sides)
  double outer_width_pixels = bbox_width_pixels + 2.0 * thickness_pixels_x;
  double outer_height_pixels = bbox_height_pixels + 2.0 * thickness_pixels_y;
  
  // Step 4: Final distance estimate using outer boundary
  double distance_from_width = 0.0;
  double distance_from_height = 0.0;
  
  if (outer_width_pixels > 5.0) {
    distance_from_width = (tape_width * focal_length_x) / outer_width_pixels;
  }
  if (outer_height_pixels > 5.0) {
    distance_from_height = (tape_height * focal_length_y) / outer_height_pixels;
  }
  
  // Use weighted average for more accurate distance estimation
  // Weight based on measurement reliability (larger dimension is usually more stable)
  double distance = 0.0;
  if (distance_from_width > 0.0 && distance_from_height > 0.0) {
    // Weighted average: give more weight to the larger dimension (more stable)
    double width_weight = bbox_width_pixels / (bbox_width_pixels + bbox_height_pixels);
    double height_weight = bbox_height_pixels / (bbox_width_pixels + bbox_height_pixels);
    distance = distance_from_width * width_weight + distance_from_height * height_weight;
    
    // Alternative: use the more stable estimate (closer to expected aspect ratio)
    // For a 70cm x 50cm tape, expected aspect ratio is 1.4
    double expected_aspect = tape_width / tape_height;
    double actual_aspect = bbox_width_pixels / bbox_height_pixels;
    double aspect_error = std::abs(actual_aspect - expected_aspect) / expected_aspect;
    
    // If aspect ratio is close to expected, both estimates are reliable
    // If aspect ratio is off, prefer the dimension that matches better
    if (aspect_error > 0.1) {  // More than 10% aspect error
      // Prefer the dimension that gives better aspect ratio match
      double width_aspect = tape_width / (tape_height * (bbox_height_pixels / bbox_width_pixels));
      double height_aspect = (tape_width * (bbox_width_pixels / bbox_height_pixels)) / tape_height;
      if (std::abs(width_aspect - expected_aspect) < std::abs(height_aspect - expected_aspect)) {
        distance = distance_from_width;  // Width-based estimate is better
      } else {
        distance = distance_from_height;  // Height-based estimate is better
      }
    }
  } else if (distance_from_width > 0.0) {
    distance = distance_from_width;
  } else if (distance_from_height > 0.0) {
    distance = distance_from_height;
  } else {
    // Fallback: use maximum corner distance (diagonal method)
    double max_dist = 0.0;
    for (size_t i = 0; i < corners.size(); i++) {
      for (size_t j = i + 1; j < corners.size(); j++) {
        double dx = corners[i].position.x - corners[j].position.x;
        double dy = corners[i].position.y - corners[j].position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist > max_dist) {
          max_dist = dist;
        }
      }
    }
    if (max_dist > 0) {
      double frame_diagonal = std::sqrt(tape_width * tape_width + tape_height * tape_height);
      double avg_focal = (focal_length_x + focal_length_y) / 2.0;
      distance = (frame_diagonal * avg_focal) / max_dist;
    } else {
      // Last resort: use middle of range
      distance = (min_distance + max_distance) / 2.0;
    }
  }

  // Clamp to valid range
  distance = std::max(min_distance, std::min(max_distance, distance));
  
  LOG_INFO( "Distance estimation from %zu corners: inner_bbox=(%d,%d), "
                         "thickness_px=(%.1f,%.1f), outer_bbox=(%.1f,%.1f), "
                         "width_dist=%.2fm, height_dist=%.2fm, final=%.2fm",
                    corners.size(), bbox.width, bbox.height,
                    thickness_pixels_x, thickness_pixels_y,
                    outer_width_pixels, outer_height_pixels,
                    distance_from_width, distance_from_height, distance);
  
  return distance;
}

double estimateDistanceFromBBox(
    const cv::Rect& bbox,
    double tape_width,
    double tape_height,
    double focal_length_x,
    double focal_length_y,
    double min_distance,
    double max_distance) {
  double bbox_width_pixels = static_cast<double>(bbox.width);
  double bbox_height_pixels = static_cast<double>(bbox.height);
  
  // Estimate distance using bounding box dimensions
  double distance_from_width = 0.0;
  double distance_from_height = 0.0;
  
  if (bbox_width_pixels > 5.0) {
    distance_from_width = (tape_width * focal_length_x) / bbox_width_pixels;
  }
  if (bbox_height_pixels > 5.0) {
    distance_from_height = (tape_height * focal_length_y) / bbox_height_pixels;
  }
  
  double distance = 0.0;
  if (distance_from_width > 0.0 && distance_from_height > 0.0) {
    distance = (distance_from_width + distance_from_height) / 2.0;
  } else if (distance_from_width > 0.0) {
    distance = distance_from_width;
  } else if (distance_from_height > 0.0) {
    distance = distance_from_height;
  } else {
    distance = (min_distance + max_distance) / 2.0;
  }
  
  // Clamp to valid range
  distance = std::max(min_distance, std::min(max_distance, distance));
  
  return distance;
}

} // namespace nc_intensity_detector

