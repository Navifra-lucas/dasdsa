#include "nc_intensity_detector2/transform_manager.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <util/logger.hpp>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace nc_intensity_detector2 {

TransformManager::TransformManager(double camera_tf_x, double camera_tf_y, double camera_tf_z, double camera_tf_yaw)
  : camera_tf_x_(camera_tf_x), camera_tf_y_(camera_tf_y), camera_tf_z_(camera_tf_z), camera_tf_yaw_(camera_tf_yaw) {
}

void TransformManager::transformPointCloudToCamera(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_base,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_camera) {
  
  // camera_tf is base_link -> camera transformation
  // To transform base_link point to camera: p_camera = R * p_base + t
  // where R is rotation by yaw, t is translation (x, y, 0)
  double cos_yaw = std::cos(camera_tf_yaw_);
  double sin_yaw = std::sin(camera_tf_yaw_);
  
  cloud_camera->points.resize(cloud_base->points.size());
  cloud_camera->width = cloud_base->width;
  cloud_camera->height = cloud_base->height;
  cloud_camera->is_dense = cloud_base->is_dense;
  
  for (size_t i = 0; i < cloud_base->points.size(); ++i) {
    // Rotate by yaw first
    double x_rot = cloud_base->points[i].x * cos_yaw - cloud_base->points[i].y * sin_yaw;
    double y_rot = cloud_base->points[i].x * sin_yaw + cloud_base->points[i].y * cos_yaw;
    double z_rot = cloud_base->points[i].z;
    
    // Then translate by camera position
    cloud_camera->points[i].x = x_rot + camera_tf_x_;
    cloud_camera->points[i].y = y_rot + camera_tf_y_;
    cloud_camera->points[i].z = z_rot + camera_tf_z_;
  }
  
  LOG_DEBUG("Transform: Transformed point cloud from base_link to camera (points: %zu)",
            cloud_camera->size());
}

void TransformManager::transformPoseToBase(
    const geometry_msgs::PoseStamped& pose_camera,
    geometry_msgs::PoseStamped& pose_base) {
  
  // Transform from camera frame to base_link using camera_tf
  // 카메라 좌표계: Z=정면, X=좌우, Y=상하
  // 로봇 좌표계: X=정면, Y=좌우, Z=상하
  // camera_tf: base_link -> camera (translation: x, y, rotation: yaw)
  // camera_tf는 로봇 좌표계에서 카메라의 위치와 회전을 나타냄
  // pose_camera는 이미 카메라 좌표계를 로봇 좌표계로 변환된 상태:
  //   pose_camera.x = camera Z (forward) = 로봇 X 방향
  //   pose_camera.y = camera X (lateral) = 로봇 Y 방향
  //   pose_camera.z = camera Y (vertical) = 로봇 Z 방향
  
  // Inverse: camera -> base_link
  double cos_yaw = std::cos(-camera_tf_yaw_);  // Negative for inverse
  double sin_yaw = std::sin(-camera_tf_yaw_);
  
  // Get position in camera frame (already converted to ROS standard/robot frame)
  // pose_camera.x = camera Z (forward) = 로봇 X 방향
  // pose_camera.y = camera X (lateral) = 로봇 Y 방향
  // pose_camera.z = camera Y (vertical) = 로봇 Z 방향
  double x_cam_robot = pose_camera.pose.position.x;  // 카메라 Z -> 로봇 X
  double y_cam_robot = pose_camera.pose.position.y;  // 카메라 X -> 로봇 Y
  double z_cam_robot = pose_camera.pose.position.z;  // 카메라 Y -> 로봇 Z
  
  // 카메라 위치(translation) 빼기 (camera_tf는 로봇 좌표계 기준)
  // camera_tf_x는 로봇 X 방향 (카메라 Z 방향과 같음)
  // camera_tf_y는 로봇 Y 방향 (카메라 X 방향과 같음)
  double x_rel = x_cam_robot - camera_tf_x_;  // Forward distance relative to camera position
  double y_rel = y_cam_robot - camera_tf_y_;  // Lateral distance relative to camera position
  double z_rel = z_cam_robot;  // Vertical (no translation in Z)
  
  // Rotate by -yaw (inverse rotation) - 카메라 회전 보정
  double x_base = x_rel * cos_yaw - y_rel * sin_yaw;
  double y_base = x_rel * sin_yaw + y_rel * cos_yaw;
  double z_base = z_rel;
  
  // Transform orientation
  // best_cluster.yaw는 카메라 좌표계에서의 각도 (카메라 Z축 기준)
  // 로봇 좌표계로 변환하려면: 로봇_yaw = 카메라_yaw + camera_tf_yaw_
  // (카메라가 로봇 좌표계에서 camera_tf_yaw_만큼 회전되어 있으므로)
  tf2::Quaternion q_cam(pose_camera.pose.orientation.x, pose_camera.pose.orientation.y,
                        pose_camera.pose.orientation.z, pose_camera.pose.orientation.w);
  tf2::Matrix3x3 m_cam(q_cam);
  double roll, pitch, yaw_cam;
  m_cam.getRPY(roll, pitch, yaw_cam);  // 카메라 좌표계에서의 yaw 추출
  
  // 로봇 좌표계로 변환: 로봇_yaw = 카메라_yaw + camera_tf_yaw_
  double yaw_base = yaw_cam + camera_tf_yaw_;
  
  // Normalize to [-PI, PI]
  while (yaw_base > M_PI) yaw_base -= 2.0 * M_PI;
  while (yaw_base < -M_PI) yaw_base += 2.0 * M_PI;
  
  tf2::Quaternion q_base;
  q_base.setRPY(roll, pitch, yaw_base);
  
  pose_base.header.frame_id = "base_link";
  pose_base.header.stamp = pose_camera.header.stamp;
  pose_base.pose.position.x = x_base;
  pose_base.pose.position.y = y_base;
  pose_base.pose.position.z = z_base;
  pose_base.pose.orientation.x = q_base.x();
  pose_base.pose.orientation.y = q_base.y();
  pose_base.pose.orientation.z = q_base.z();
  pose_base.pose.orientation.w = q_base.w();
  
  LOG_DEBUG("Transform: Transformed pose from camera to base_link (x=%.2f, y=%.2f)",
            x_base, y_base);
}

} // namespace nc_intensity_detector2

