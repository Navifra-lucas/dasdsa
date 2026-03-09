#include "nc_wingbody_loader/pose_calculator.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <util/logger.hpp>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace nc_wingbody_loader {

PoseCalculator::PoseCalculator(double pallet_size, double safety_margin,
                               double door_to_floor_offset, double cargo_depth,
                               TransformManager* transform_manager)
  : pallet_size_(pallet_size),
    safety_margin_(safety_margin),
    door_to_floor_offset_(door_to_floor_offset),
    cargo_depth_(cargo_depth),
    transform_manager_(transform_manager) {
}

PlacementResult PoseCalculator::calculate(const CargoClassifier& classifier,
                                          const std::string& loading_mode) {
  PlacementResult result;

  const DetectedPlane* door = classifier.getDoor();
  if (!door) {
    result.success = false;
    result.error_msg = "No door detected";
    return result;
  }

  // All calculations in camera frame, then convert to base_link at the end
  double yaw_camera = calculateYaw(classifier);
  double z_camera = calculateZ(classifier);

  double x_camera = 0.0, y_camera = 0.0;
  calculateXY(classifier, loading_mode, x_camera, y_camera);

  // Convert camera frame pose to base_link
  // Camera: Z=forward, X=lateral, Y=vertical
  // Robot:  X=forward, Y=lateral, Z=vertical
  geometry_msgs::PoseStamped pose_camera;
  pose_camera.header.frame_id = "camera";
  pose_camera.header.stamp = ros::Time::now();
  pose_camera.pose.position.x = x_camera;  // camera Z (forward) -> will become robot X
  pose_camera.pose.position.y = y_camera;  // camera X (lateral) -> will become robot Y
  pose_camera.pose.position.z = z_camera;  // camera Y (vertical) -> will become robot Z

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_camera);
  pose_camera.pose.orientation.x = q.x();
  pose_camera.pose.orientation.y = q.y();
  pose_camera.pose.orientation.z = q.z();
  pose_camera.pose.orientation.w = q.w();

  geometry_msgs::PoseStamped pose_base;
  transform_manager_->transformPoseToBase(pose_camera, pose_base);

  // Extract yaw from base pose
  tf2::Quaternion q_base(pose_base.pose.orientation.x, pose_base.pose.orientation.y,
                         pose_base.pose.orientation.z, pose_base.pose.orientation.w);
  tf2::Matrix3x3 m_base(q_base);
  double roll, pitch, yaw_base;
  m_base.getRPY(roll, pitch, yaw_base);

  result.success = true;
  result.x = pose_base.pose.position.x;
  result.y = pose_base.pose.position.y;
  result.z = pose_base.pose.position.z;
  result.yaw = yaw_base;

  LOG_INFO("PoseCalculator: Camera frame - forward=%.3f, lateral=%.3f, height=%.3f, yaw=%.1fdeg",
           x_camera, y_camera, z_camera, yaw_camera * 180.0 / M_PI);
  LOG_INFO("PoseCalculator: Base frame - x=%.3f, y=%.3f, z=%.3f, yaw=%.1fdeg",
           result.x, result.y, result.z, result.yaw * 180.0 / M_PI);

  return result;
}

double PoseCalculator::calculateYaw(const CargoClassifier& classifier) {
  const DetectedPlane* door = classifier.getDoor();
  if (!door) return 0.0;

  // Yaw from door normal: atan2(-a, |c|) where a=normal.x, c=normal.z
  // Same formula as nc_intensity_detector2
  double yaw_door = std::atan2(-door->normal.x(), std::abs(door->normal.z()));

  const DetectedPlane* head_wall = classifier.getHeadWall();
  if (head_wall) {
    double yaw_wall = std::atan2(-head_wall->normal.x(), std::abs(head_wall->normal.z()));
    // Weighted average: door 80%, head_wall 20%
    double yaw = 0.8 * yaw_door + 0.2 * yaw_wall;

    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;

    LOG_INFO("PoseCalculator: Yaw - door=%.1fdeg, wall=%.1fdeg, weighted=%.1fdeg",
             yaw_door * 180.0 / M_PI, yaw_wall * 180.0 / M_PI, yaw * 180.0 / M_PI);
    return yaw;
  }

  LOG_INFO("PoseCalculator: Yaw from door only = %.1fdeg", yaw_door * 180.0 / M_PI);
  return yaw_door;
}

double PoseCalculator::calculateZ(const CargoClassifier& classifier) {
  const DetectedPlane* door = classifier.getDoor();
  if (!door || !door->cloud || door->cloud->empty()) return 0.0;

  // Z (height): Use door points' Y coordinates (vertical in camera frame)
  // Find upper percentile (top of door) for floor height estimation
  // Camera Y axis: positive = down, so the most negative Y values are highest
  std::vector<float> y_values;
  y_values.reserve(door->cloud->size());
  for (const auto& pt : door->cloud->points) {
    y_values.push_back(pt.y);
  }
  std::sort(y_values.begin(), y_values.end());

  // Top 5% percentile (most negative Y = highest point)
  size_t idx = static_cast<size_t>(y_values.size() * 0.05);
  if (idx >= y_values.size()) idx = 0;
  float top_y = y_values[idx];

  // Convert: camera Y -> robot Z direction
  // The floor height relative to camera = top of door Y + offset
  double z_value = static_cast<double>(top_y) + door_to_floor_offset_;

  LOG_INFO("PoseCalculator: Z(height) - door top Y=%.3f, offset=%.3f, result=%.3f",
           top_y, door_to_floor_offset_, z_value);
  return z_value;
}

void PoseCalculator::calculateXY(const CargoClassifier& classifier,
                                  const std::string& loading_mode,
                                  double& x_out, double& y_out) {
  const DetectedPlane* door = classifier.getDoor();
  if (!door) {
    x_out = y_out = 0.0;
    return;
  }

  double door_z = door->centroid.z();  // forward distance in camera frame
  const auto& existing_pallets = classifier.getExistingPallets();

  // Y (lateral): door center X in camera frame
  y_out = door->centroid.x();

  if (loading_mode == "next" && !existing_pallets.empty()) {
    // After first pallet: place relative to closest existing pallet face
    const DetectedPlane* closest_pallet = existing_pallets.front();
    // Place at gap distance from pallet face toward camera
    double pallet_face_z = closest_pallet->centroid.z() - closest_pallet->depth / 2.0;
    x_out = pallet_face_z - safety_margin_ - pallet_size_ / 2.0;

    LOG_INFO("PoseCalculator: XY(next) - pallet_face=%.3f, gap=%.3f, target_z=%.3f, lateral=%.3f",
             pallet_face_z, safety_margin_, x_out, y_out);
  } else {
    // First pallet: use head_wall or fallback to cargo_depth
    const DetectedPlane* head_wall = classifier.getHeadWall();
    double back_z;
    if (head_wall) {
      back_z = head_wall->centroid.z();
      LOG_INFO("PoseCalculator: Using head_wall distance = %.3f", back_z);
    } else {
      back_z = door_z + cargo_depth_;
      LOG_INFO("PoseCalculator: No head_wall, fallback depth = %.3f (door=%.3f + depth=%.3f)",
               back_z, door_z, cargo_depth_);
    }

    x_out = back_z - safety_margin_ - pallet_size_ / 2.0;

    LOG_INFO("PoseCalculator: XY(first) - back=%.3f, margin=%.3f, pallet/2=%.3f, target_z=%.3f, lateral=%.3f",
             back_z, safety_margin_, pallet_size_ / 2.0, x_out, y_out);
  }
}

} // namespace nc_wingbody_loader
