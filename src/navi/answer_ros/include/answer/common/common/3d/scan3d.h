/**
 * @class scan 3d
 * @brief 3d lidar structure
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */
#pragma once
// #include "common/types.h"
#include "glog/logging.h"
#include "nanoflann/nanoflann.h"

#include <sophus/se2.h>
#include <sophus/se3.h>

using namespace Eigen;
using namespace std;

namespace ANSWER {

class Scan3D {
private:
    /* data */
    double timestamp;
    // homogeneous coordinates
    std::vector<Eigen::Vector3d> point_cloud;
    std::vector<Eigen::Vector4d> point_cloud_hom;
    std::vector<Eigen::Vector3d> normals;
    // std::unique_ptr<KDTree3d> kdtree_3d_;
    Sophus::SE3d origin;
    Sophus::SE3d robot_to_lidar;
    std::vector<double> timestamps;
    std::vector<double> intensities;

public:
    Scan3D(/* args */);
    ~Scan3D();

    template <typename T>
    Scan3D(const std::vector<Eigen::Matrix<T, 2, 1>> &scan2d)
    {
        point_cloud.clear();
        point_cloud.reserve(scan2d.size());
        for (const auto &point : scan2d) {
            double x = static_cast<double>(point.x());
            double y = static_cast<double>(point.y());
            point_cloud.emplace_back(Eigen::Vector3d(x, y, 0));
        }
    }
    // pointcloud
    // Pose2D origin;
    // vector<Pose2D> point_cloud;
    const std::vector<Eigen::Vector3d> &GetPointCloud() const;
    const std::vector<Eigen::Vector4d> &GetPointCloudHomogeneous() const
    {
        return point_cloud_hom;
    }
    const Sophus::SE3d &GetOrigin() const;
    const void SetPointCloud(const std::vector<Eigen::Vector3d> &point_cloud)
    {
        this->point_cloud = point_cloud;
    }
    const void SetOrigin(const Sophus::SE3d &origin) { this->origin = origin; }
    const void SetNormals(const std::vector<Eigen::Vector3d> &normals)
    {
        this->normals = normals;
    }
    std::vector<Eigen::Vector3d> &MutablePointCloud() { return point_cloud; }
    std::vector<Eigen::Vector4d> &MutablePointCloudHomogeneous()
    {
        return point_cloud_hom;
    }
    std::vector<Eigen::Vector3d> &MutableNormals() { return normals; }
    const std::vector<Eigen::Vector3d> &GetNormals() const { return normals; }

    std::vector<double> &MutableTimestamps() { return timestamps; }
    const std::vector<double> &GetTimestamps() const { return timestamps; }
    const void SetTimestamps(const std::vector<double> &timestamps)
    {
        this->timestamps = timestamps;
    }

    void SetExtrinsicParameter(
        const double &x, const double &y, const double &z, const double &roll,
        const double &pitch, const double &yaw);
    void TransformToRobotFrame();
    const double &GetTimestamp() const { return timestamp; }
    void SetTimestamp(const double &timestamp) { this->timestamp = timestamp; }
    size_t Size() const { return point_cloud.size(); }
};
}  // namespace ANSWER
