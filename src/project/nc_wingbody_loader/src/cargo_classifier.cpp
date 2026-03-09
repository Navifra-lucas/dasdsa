#include "nc_wingbody_loader/cargo_classifier.h"

#include <util/logger.hpp>
#include <cmath>
#include <algorithm>

namespace nc_wingbody_loader {

CargoClassifier::CargoClassifier(double door_max_distance, double cargo_width,
                                 double cargo_depth, double pallet_size)
  : door_max_distance_(door_max_distance),
    cargo_width_(cargo_width),
    cargo_depth_(cargo_depth),
    pallet_size_(pallet_size) {
}

void CargoClassifier::classify(std::vector<DetectedPlane>& planes) {
  // Reset state
  door_ = nullptr;
  head_wall_ = nullptr;
  side_wall_left_ = nullptr;
  side_wall_right_ = nullptr;
  floor_ = nullptr;
  existing_pallets_.clear();

  if (planes.empty()) {
    return;
  }

  // Camera frame: Z=forward, X=lateral, Y=vertical (down)
  // Vertical plane: |normal.y| should be small (normal mostly in XZ plane)
  // Horizontal plane: |normal.y| should be large

  // --- Pass 1: Find DOOR (closest large vertical plane facing camera) ---
  // Door normal should be mostly along Z (forward): |normal.z| > 0.7
  // Door should be close: distance < door_max_distance_
  // Door should be wide: width > 1.5m
  const DetectedPlane* best_door = nullptr;
  int best_door_inliers = 0;

  for (auto& plane : planes) {
    bool is_vertical = std::abs(plane.normal.y()) < 0.3f;
    bool faces_camera = std::abs(plane.normal.z()) > 0.7f;
    bool is_close = plane.distance < door_max_distance_;
    bool is_wide = plane.width > 1.5f;

    if (is_vertical && faces_camera && is_close && is_wide) {
      if (plane.inlier_count > best_door_inliers) {
        best_door = &plane;
        best_door_inliers = plane.inlier_count;
      }
    }
  }

  if (best_door) {
    const_cast<DetectedPlane*>(best_door)->type = DOOR;
    door_ = best_door;
    LOG_INFO("CargoClassifier: DOOR found - dist=%.2f, width=%.2f, inliers=%d, normal=(%.2f,%.2f,%.2f)",
             best_door->distance, best_door->width, best_door->inlier_count,
             best_door->normal.x(), best_door->normal.y(), best_door->normal.z());
  } else {
    LOG_WARNING("CargoClassifier: No DOOR detected");
    return;
  }

  float door_z = best_door->centroid.z();

  // --- Pass 2: Classify remaining planes ---
  for (auto& plane : planes) {
    if (plane.type != UNKNOWN) continue;

    float abs_ny = std::abs(plane.normal.y());
    float abs_nx = std::abs(plane.normal.x());
    float abs_nz = std::abs(plane.normal.z());

    // FLOOR: horizontal plane (normal mostly Y, i.e. |normal.y| > 0.7)
    if (abs_ny > 0.7f) {
      plane.type = FLOOR;
      floor_ = &plane;
      LOG_INFO("CargoClassifier: FLOOR found - centroid_y=%.2f, inliers=%d",
               plane.centroid.y(), plane.inlier_count);
      continue;
    }

    bool is_vertical = abs_ny < 0.3f;
    if (!is_vertical) continue;

    // HEAD_WALL: vertical, faces same direction as door (|normal.z|>0.7), farther away (distance > door + 2m)
    if (abs_nz > 0.7f && plane.distance > (door_z + 2.0f)) {
      plane.type = HEAD_WALL;
      head_wall_ = &plane;
      LOG_INFO("CargoClassifier: HEAD_WALL found - dist=%.2f, width=%.2f, inliers=%d",
               plane.distance, plane.width, plane.inlier_count);
      continue;
    }

    // SIDE_WALL: vertical, normal mostly lateral (|normal.x| > 0.7)
    if (abs_nx > 0.7f) {
      // Must be behind the door (centroid.z > door_z)
      if (plane.centroid.z() > door_z) {
        if (plane.centroid.x() < 0.0f) {
          plane.type = SIDE_WALL_LEFT;
          side_wall_left_ = &plane;
          LOG_INFO("CargoClassifier: SIDE_WALL_LEFT found - centroid_x=%.2f, inliers=%d",
                   plane.centroid.x(), plane.inlier_count);
        } else {
          plane.type = SIDE_WALL_RIGHT;
          side_wall_right_ = &plane;
          LOG_INFO("CargoClassifier: SIDE_WALL_RIGHT found - centroid_x=%.2f, inliers=%d",
                   plane.centroid.x(), plane.inlier_count);
        }
      }
      continue;
    }

    // EXISTING_PALLET: vertical, faces camera (|normal.z|>0.7), behind door,
    //   width similar to pallet_size (±30%), elevated above floor
    if (abs_nz > 0.7f && plane.centroid.z() > door_z) {
      float width_ratio = plane.width / static_cast<float>(pallet_size_);
      if (width_ratio > 0.7f && width_ratio < 1.3f) {
        plane.type = EXISTING_PALLET;
        existing_pallets_.push_back(&plane);
        LOG_INFO("CargoClassifier: EXISTING_PALLET found - dist=%.2f, width=%.2f, inliers=%d",
                 plane.distance, plane.width, plane.inlier_count);
      }
    }
  }

  // Sort existing pallets by distance (closest first)
  std::sort(existing_pallets_.begin(), existing_pallets_.end(),
            [](const DetectedPlane* a, const DetectedPlane* b) {
              return a->distance < b->distance;
            });

  LOG_INFO("CargoClassifier: Classification complete - door=%d, head_wall=%d, side_left=%d, side_right=%d, floor=%d, pallets=%zu",
           door_ != nullptr, head_wall_ != nullptr,
           side_wall_left_ != nullptr, side_wall_right_ != nullptr,
           floor_ != nullptr, existing_pallets_.size());
}

} // namespace nc_wingbody_loader
