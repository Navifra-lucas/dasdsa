#ifndef NC_WINGBODY_LOADER_PLANE_DETECTOR_H
#define NC_WINGBODY_LOADER_PLANE_DETECTOR_H

#include "nc_wingbody_loader/detected_plane.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace nc_wingbody_loader {

class PlaneDetector {
public:
  PlaneDetector(int max_planes, double ransac_distance_threshold,
                int ransac_max_iterations, int min_plane_points);

  std::vector<DetectedPlane> detectPlanes(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

private:
  int max_planes_;
  double ransac_distance_threshold_;
  int ransac_max_iterations_;
  int min_plane_points_;
};

} // namespace nc_wingbody_loader

#endif // NC_WINGBODY_LOADER_PLANE_DETECTOR_H
