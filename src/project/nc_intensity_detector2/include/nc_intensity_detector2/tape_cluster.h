#ifndef NC_INTENSITY_DETECTOR2_TAPE_CLUSTER_H
#define NC_INTENSITY_DETECTOR2_TAPE_CLUSTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>

namespace nc_intensity_detector2 {

struct TapeCluster {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud;  // 평면 찾기에 사용된 포인트 (RANSAC inliers)
  pcl::PointXYZ center;
  double width;   // 가로 (60cm)
  double height;  // 세로 (40cm)
  double distance;  // 거리 (x)
  double y;  // 횡 위치
  double yaw;  // yaw 각도
  double distance_from_center;  // 중앙으로부터의 거리
};

} // namespace nc_intensity_detector2

#endif // NC_INTENSITY_DETECTOR2_TAPE_CLUSTER_H

