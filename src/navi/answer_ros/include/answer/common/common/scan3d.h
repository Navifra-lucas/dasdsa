/**
 * @class scan 3d
 * @brief 3d lidar structure
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */
#pragma once
// #include "common/3d/kdtree_wrapper_3d.h"
#include "common/pch.h"
//#include "common/pose3d.h"
#include "glog/logging.h"
#include "nanoflann/nanoflann.h"

using namespace Eigen;
using namespace std;
using Point3D = Eigen::Vector3d;
using PointCloud3D = vector<Point3D>;

namespace ANSWER {

class Scan3D {
private:
    /* data */
    PointCloud3D point_cloud;
    PointCloud3D normals;
    // std::unique_ptr<KDTree3d> kdtree_3d_;
    Pose3D origin;
    Pose3D robot_to_lidar;
    std::vector<double> timestamps;

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
            point_cloud.emplace_back(Point3D(x, y, 0));
        }
    }
    // pointcloud
    // Pose2D origin;
    // vector<Pose2D> point_cloud;
    const PointCloud3D &GetPointCloud() const;
    const Pose3D &GetOrigin() const;
    const void SetPointCloud(const PointCloud3D &point_cloud)
    {
        this->point_cloud = point_cloud;
    }
    const void SetOrigin(const Pose3D &origin) { this->origin = origin; }
    const void SetNormals(const PointCloud3D &normals)
    {
        this->normals = normals;
    }
    PointCloud3D &MutablePointCloud() { return point_cloud; }
    PointCloud3D &MutableNormals() { return normals; }
    const PointCloud3D &GetNormals() const { return normals; }

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
};

}  // namespace ANSWER
