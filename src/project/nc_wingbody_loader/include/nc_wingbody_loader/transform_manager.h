#ifndef NC_WINGBODY_LOADER_TRANSFORM_MANAGER_H
#define NC_WINGBODY_LOADER_TRANSFORM_MANAGER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>

namespace nc_wingbody_loader {

class TransformManager {
public:
  TransformManager(double camera_tf_x, double camera_tf_y, double camera_tf_z, double camera_tf_yaw);

  // base_link -> camera frame
  void transformPointCloudToCamera(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_base,
      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_camera);

  // camera frame -> base_link
  void transformPointCloudToBase(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_camera,
      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_base);

  // camera frame pose -> base_link pose
  void transformPoseToBase(
      const geometry_msgs::PoseStamped& pose_camera,
      geometry_msgs::PoseStamped& pose_base);

private:
  double camera_tf_x_, camera_tf_y_, camera_tf_z_, camera_tf_yaw_;
};

} // namespace nc_wingbody_loader

#endif // NC_WINGBODY_LOADER_TRANSFORM_MANAGER_H
