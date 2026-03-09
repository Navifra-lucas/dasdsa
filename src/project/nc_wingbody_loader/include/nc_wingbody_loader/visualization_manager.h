#ifndef NC_WINGBODY_LOADER_VISUALIZATION_MANAGER_H
#define NC_WINGBODY_LOADER_VISUALIZATION_MANAGER_H

#include "nc_wingbody_loader/detected_plane.h"
#include "nc_wingbody_loader/transform_manager.h"
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>

namespace nc_wingbody_loader {

class VisualizationManager {
public:
  VisualizationManager(ros::Publisher& marker_array_pub,
                       ros::Publisher& classified_pc_pub,
                       ros::Publisher& filtered_pc_pub,
                       ros::Publisher& target_marker_pub,
                       const std::string& base_frame,
                       TransformManager* transform_manager);

  void publishPlaneMarkers(const std::vector<DetectedPlane>& planes,
                           const ros::Time& stamp);

  void publishClassifiedPointCloud(const std::vector<DetectedPlane>& planes,
                                   const ros::Time& stamp);

  void publishFilteredPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                 const ros::Time& stamp);

  void publishTargetMarker(const PlacementResult& result,
                           const ros::Time& stamp,
                           double pallet_depth,
                           double pallet_width = 1.2,
                           double pallet_height = 0.15);

  void clearMarkers();

private:
  void getColorForType(PlaneType type, float& r, float& g, float& b);

  ros::Publisher& marker_array_pub_;
  ros::Publisher& classified_pc_pub_;
  ros::Publisher& filtered_pc_pub_;
  ros::Publisher& target_marker_pub_;
  std::string base_frame_;
  TransformManager* transform_manager_;
};

} // namespace nc_wingbody_loader

#endif // NC_WINGBODY_LOADER_VISUALIZATION_MANAGER_H
