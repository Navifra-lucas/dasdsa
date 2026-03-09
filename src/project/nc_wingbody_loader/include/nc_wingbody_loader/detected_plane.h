#ifndef NC_WINGBODY_LOADER_DETECTED_PLANE_H
#define NC_WINGBODY_LOADER_DETECTED_PLANE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <string>

namespace nc_wingbody_loader {

enum PlaneType {
  UNKNOWN = 0,
  DOOR,
  HEAD_WALL,
  SIDE_WALL_LEFT,
  SIDE_WALL_RIGHT,
  FLOOR,
  EXISTING_PALLET
};

struct DetectedPlane {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  // inlier points
  Eigen::Vector3f normal;                      // unit normal vector
  Eigen::Vector3f centroid;                    // center point
  Eigen::Vector4f coefficients;                // ax+by+cz+d=0
  float width = 0.0f;                         // bounding box width (X span)
  float height = 0.0f;                        // bounding box height (Y span)
  float depth = 0.0f;                         // bounding box depth (Z span)
  float distance = 0.0f;                      // distance from camera (Z of centroid)
  int inlier_count = 0;                       // number of inlier points
  PlaneType type = UNKNOWN;                   // classification type
  pcl::PointXYZ bb_min;                       // bounding box min corner
  pcl::PointXYZ bb_max;                       // bounding box max corner

  DetectedPlane() : cloud(new pcl::PointCloud<pcl::PointXYZ>),
                    normal(Eigen::Vector3f::Zero()),
                    centroid(Eigen::Vector3f::Zero()),
                    coefficients(Eigen::Vector4f::Zero()) {
    bb_min.x = bb_min.y = bb_min.z = 0.0f;
    bb_max.x = bb_max.y = bb_max.z = 0.0f;
  }
};

struct PlacementResult {
  bool success = false;
  double x = 0.0;    // base_link X (forward)
  double y = 0.0;    // base_link Y (lateral)
  double z = 0.0;    // base_link Z (height)
  double yaw = 0.0;  // base_link yaw (orientation)
  std::string error_msg;
};

} // namespace nc_wingbody_loader

#endif // NC_WINGBODY_LOADER_DETECTED_PLANE_H
