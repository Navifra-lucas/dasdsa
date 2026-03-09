#include "nc_wingbody_loader/plane_detector.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <util/logger.hpp>
#include <cmath>

namespace nc_wingbody_loader {

PlaneDetector::PlaneDetector(int max_planes, double ransac_distance_threshold,
                             int ransac_max_iterations, int min_plane_points)
  : max_planes_(max_planes),
    ransac_distance_threshold_(ransac_distance_threshold),
    ransac_max_iterations_(ransac_max_iterations),
    min_plane_points_(min_plane_points) {
}

std::vector<DetectedPlane> PlaneDetector::detectPlanes(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {

  std::vector<DetectedPlane> planes;

  if (!cloud || cloud->size() < static_cast<size_t>(min_plane_points_)) {
    LOG_WARNING("PlaneDetector: Input cloud too small (%zu points, need %d)",
                cloud ? cloud->size() : 0, min_plane_points_);
    return planes;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>(*cloud));

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(ransac_max_iterations_);
  seg.setDistanceThreshold(ransac_distance_threshold_);
  seg.setOptimizeCoefficients(true);
  seg.setProbability(0.99);

  pcl::ExtractIndices<pcl::PointXYZ> extract;

  for (int i = 0; i < max_planes_; ++i) {
    if (remaining->size() < static_cast<size_t>(min_plane_points_)) {
      break;
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setInputCloud(remaining);
    seg.segment(*inliers, *coefficients);

    if (static_cast<int>(inliers->indices.size()) < min_plane_points_) {
      LOG_DEBUG("PlaneDetector: Plane %d has too few inliers (%zu), stopping",
                i, inliers->indices.size());
      break;
    }

    // Build DetectedPlane
    DetectedPlane plane;
    plane.inlier_count = inliers->indices.size();

    // Extract inlier points
    extract.setInputCloud(remaining);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane.cloud);

    // Store coefficients: ax+by+cz+d=0
    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];
    plane.coefficients = Eigen::Vector4f(a, b, c, d);

    // Normalize normal vector
    float norm = std::sqrt(a * a + b * b + c * c);
    if (norm < 1e-6f) {
      // Remove inliers and continue
      extract.setNegative(true);
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
      extract.filter(*temp);
      remaining = temp;
      continue;
    }
    plane.normal = Eigen::Vector3f(a / norm, b / norm, c / norm);

    // Compute centroid
    Eigen::Vector4f centroid4;
    pcl::compute3DCentroid(*plane.cloud, centroid4);
    plane.centroid = centroid4.head<3>();
    plane.distance = std::abs(plane.centroid.z());

    // Compute bounding box
    pcl::getMinMax3D(*plane.cloud, plane.bb_min, plane.bb_max);
    plane.width = plane.bb_max.x - plane.bb_min.x;
    plane.height = plane.bb_max.y - plane.bb_min.y;
    plane.depth = plane.bb_max.z - plane.bb_min.z;

    planes.push_back(plane);

    LOG_DEBUG("PlaneDetector: Plane %d - inliers=%d, normal=(%.2f,%.2f,%.2f), centroid=(%.2f,%.2f,%.2f), size=%.2fx%.2f, dist=%.2f",
              i, plane.inlier_count,
              plane.normal.x(), plane.normal.y(), plane.normal.z(),
              plane.centroid.x(), plane.centroid.y(), plane.centroid.z(),
              plane.width, plane.height, plane.distance);

    // Remove inliers from remaining cloud
    extract.setInputCloud(remaining);
    extract.setIndices(inliers);
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*temp);
    remaining = temp;
  }

  LOG_INFO("PlaneDetector: Detected %zu planes from %zu input points",
           planes.size(), cloud->size());
  return planes;
}

} // namespace nc_wingbody_loader
