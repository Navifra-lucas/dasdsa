#pragma once
#include <sophus/se2.h>
#include <sophus/se3.h>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>

#define STRINGFY(T) (#T)
namespace ANSWER {
using Pose3D = Sophus::SE3d;
using Point3D = Eigen::Vector3d;
using PointCloud3D = std::vector<Point3D>;
using ClusterIndices = std::vector<int>;
using Point2D = Eigen::Vector2f;
using PointCloud2D = std::vector<Point2D, Eigen::aligned_allocator<Point2D>>;
}  // namespace ANSWER