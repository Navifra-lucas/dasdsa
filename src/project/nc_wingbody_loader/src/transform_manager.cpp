#include "nc_wingbody_loader/transform_manager.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <util/logger.hpp>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace nc_wingbody_loader {

TransformManager::TransformManager(double camera_tf_x, double camera_tf_y, double camera_tf_z, double camera_tf_yaw)
  : camera_tf_x_(camera_tf_x), camera_tf_y_(camera_tf_y), camera_tf_z_(camera_tf_z), camera_tf_yaw_(camera_tf_yaw) {
}

void TransformManager::transformPointCloudToCamera(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_base,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_camera) {

  double cos_yaw = std::cos(camera_tf_yaw_);
  double sin_yaw = std::sin(camera_tf_yaw_);

  cloud_camera->points.resize(cloud_base->points.size());
  cloud_camera->width = cloud_base->width;
  cloud_camera->height = cloud_base->height;
  cloud_camera->is_dense = cloud_base->is_dense;

  for (size_t i = 0; i < cloud_base->points.size(); ++i) {
    double x_rot = cloud_base->points[i].x * cos_yaw - cloud_base->points[i].y * sin_yaw;
    double y_rot = cloud_base->points[i].x * sin_yaw + cloud_base->points[i].y * cos_yaw;
    double z_rot = cloud_base->points[i].z;

    cloud_camera->points[i].x = x_rot + camera_tf_x_;
    cloud_camera->points[i].y = y_rot + camera_tf_y_;
    cloud_camera->points[i].z = z_rot + camera_tf_z_;
  }
}

void TransformManager::transformPointCloudToBase(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_camera,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_base) {

  // Camera frame: X=lateral, Y=vertical, Z=forward
  // Base frame:   X=forward, Y=lateral, Z=vertical
  // So base = (camera.z, camera.x, camera.y) then apply camera_tf inverse
  double cos_yaw = std::cos(-camera_tf_yaw_);
  double sin_yaw = std::sin(-camera_tf_yaw_);

  cloud_base->points.resize(cloud_camera->points.size());
  cloud_base->width = cloud_camera->width;
  cloud_base->height = cloud_camera->height;
  cloud_base->is_dense = cloud_camera->is_dense;

  for (size_t i = 0; i < cloud_camera->points.size(); ++i) {
    double forward = cloud_camera->points[i].z;
    double lateral = cloud_camera->points[i].x;
    double vertical = cloud_camera->points[i].y;

    double x_rel = forward - camera_tf_x_;
    double y_rel = lateral - camera_tf_y_;
    double z_rel = vertical - camera_tf_z_;

    cloud_base->points[i].x = x_rel * cos_yaw - y_rel * sin_yaw;
    cloud_base->points[i].y = x_rel * sin_yaw + y_rel * cos_yaw;
    cloud_base->points[i].z = z_rel;
  }
}

void TransformManager::transformPoseToBase(
    const geometry_msgs::PoseStamped& pose_camera,
    geometry_msgs::PoseStamped& pose_base) {

  double cos_yaw = std::cos(-camera_tf_yaw_);
  double sin_yaw = std::sin(-camera_tf_yaw_);

  double x_cam_robot = pose_camera.pose.position.x;
  double y_cam_robot = pose_camera.pose.position.y;
  double z_cam_robot = pose_camera.pose.position.z;

  double x_rel = x_cam_robot - camera_tf_x_;
  double y_rel = y_cam_robot - camera_tf_y_;
  double z_rel = z_cam_robot;

  double x_base = x_rel * cos_yaw - y_rel * sin_yaw;
  double y_base = x_rel * sin_yaw + y_rel * cos_yaw;
  double z_base = z_rel;

  tf2::Quaternion q_cam(pose_camera.pose.orientation.x, pose_camera.pose.orientation.y,
                        pose_camera.pose.orientation.z, pose_camera.pose.orientation.w);
  tf2::Matrix3x3 m_cam(q_cam);
  double roll, pitch, yaw_cam;
  m_cam.getRPY(roll, pitch, yaw_cam);

  double yaw_base = yaw_cam + camera_tf_yaw_;

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

  LOG_DEBUG("TransformManager: pose camera->base (x=%.2f, y=%.2f, z=%.2f)",
            x_base, y_base, z_base);
}

} // namespace nc_wingbody_loader
