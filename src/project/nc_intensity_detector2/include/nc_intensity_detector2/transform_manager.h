#ifndef NC_INTENSITY_DETECTOR2_TRANSFORM_MANAGER_H
#define NC_INTENSITY_DETECTOR2_TRANSFORM_MANAGER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

namespace nc_intensity_detector2 {

class TransformManager {
public:
  TransformManager(double camera_tf_x, double camera_tf_y, double camera_tf_z, double camera_tf_yaw);
  
  // Transform point cloud from base_link to camera frame
  void transformPointCloudToCamera(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_base,
      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_camera);
  
  // Transform pose from camera frame to base_link
  void transformPoseToBase(
      const geometry_msgs::PoseStamped& pose_camera,
      geometry_msgs::PoseStamped& pose_base);

private:
  double camera_tf_x_;
  double camera_tf_y_;
  double camera_tf_z_;
  double camera_tf_yaw_;
};

} // namespace nc_intensity_detector2

#endif // NC_INTENSITY_DETECTOR2_TRANSFORM_MANAGER_H

