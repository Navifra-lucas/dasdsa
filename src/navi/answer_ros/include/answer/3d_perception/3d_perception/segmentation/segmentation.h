#pragma once
//#include "common/3d/pose3d.h"
#include "common/3d/scan3d.h"
#include "common/answer_utils.h"
#include "common/callbacks.h"
#include "common/pch.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/label.h>
#include "common/3d/point_cloud_utils.h"

#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>
#include <vector>

namespace ANSWER {
namespace PERCEPTION {
struct PlaneRansacParameters {
    double point_to_plane_dist_threshold = 0.05;
    bool use_orthogonal_check = false;
    bool use_parallel_check = false;
    bool ground_removal = false;
    double ground_point_max_height = 0.2;
    int ransac_iter = 1000;
    int min_support = 50;
    Eigen::Vector3d reference_orthogonal_normal =
        Eigen::Vector3d(0.0, 0.0, 1.0);
    Eigen::Vector3d reference_parallel_normal = Eigen::Vector3d(0.0, 0.0, 1.0);
    double parallel_cos_threshold =
        0.95;  // n_candidate 이 n3 와 18° 이내로 평행
    double orthogonal_cos_threshold =
        0.2;  // n_candidate 이 n3 와 78° 이내로 직교
};

struct LineRansacParameters {
    double point_to_line_dist_threshold = 0.05;

    bool use_orthogonal_check = false;
    bool use_parallel_check = false;
    bool use_local_growth = false;
    int ransac_iter = 1000;
    int min_support = 50;

    Eigen::Vector3d reference_orthogonal_vector =
        Eigen::Vector3d(0.0, 0.0, 1.0);
    Eigen::Vector3d reference_parallel_vector = Eigen::Vector3d(0.0, 0.0, 1.0);
    double parallel_cos_threshold =
        0.95;  // n_candidate 이 n3 와 18° 이내로 평행
    double orthogonal_cos_threshold =
        0.2;  // n_candidate 이 n3 와 78° 이내로 직교
    double knn_radius = 0.3;
};
struct Point3N {
    Eigen::Vector3d p;
    Eigen::Vector3d n;
    bool used = false;
};
struct Plane {
    Eigen::Vector3d n;
    double d;
};
struct Cuboid {
    Eigen::Vector3d center;  // center point
    Eigen::Vector3d extents;  // half-lengths along each axis
    Eigen::Matrix3d axes;  // columns are the direction vectors
};
struct Line {
    Eigen::Vector3d point;
    Eigen::Vector3d direction;
};

class Segmentation {
private:
    //
    Plane FitPlaneTLS(const PointCloud3D &pts, const std::vector<int> &idxs);
    Plane FitPlaneFrom3Pts(
        const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
        const Eigen::Vector3d &p2);
    double PointPlaneDistance(const Plane &pl, const Eigen::Vector3d &pt);
    double PointLineDistance(
        const Eigen::Vector3d &p, const Eigen::Vector3d &p0,
        const Eigen::Vector3d &dir);
    double NormalCosAngle(const Eigen::Vector3d &a, const Eigen::Vector3d &b);
    std::vector<std::vector<int>> SplitParallelByDistance(
        const PointCloud3D &pts, const Plane &pl,
        const std::vector<int> &inliers, double gap_thresh, int min_layer_size);
    bool RefineCuboid(
        const PointCloud3D &pts, const std::vector<int> &inliers,
        const Cuboid &initial_cuboid, Cuboid &refined_cuboid);

    double dist_thresh_;  // meters
    double cos_thresh_;  // normal angle <= 10 deg
    double knn_radius_;  // for optional local growth
    int min_support_;
    int ransac_iters_;

    double split_gap_;
    int min_support_per_layer_;
    bool use_local_growth_;
    bool use_orthogonal_check_;
    double orthogonal_cos_threshold_;
    std::mt19937 rng_;
    PlaneRansacParameters plane_ransac_params_;
    LineRansacParameters line_ransac_params_;

    bool SegmentPlane(
        const Scan3D &frame, Plane &out_plane, std::vector<int> &out_inliers,
        const PlaneRansacParameters &plane_ransac_params);
    bool SegmentLine(
        const Scan3D &frame, Line &out_line, std::vector<int> &out_inliers,
        const LineRansacParameters &line_ransac_params);

public:
    Segmentation();
    ~Segmentation();

    std::vector<std::vector<int>> SegmentMultiplePlane(
        const Scan3D &frame, KDTree3d *kdtree_3d);

    bool GetOrthogonalPlane(
        const Scan3D &input_scan, Eigen::Vector3d &target_normal,
        std::vector<int> &out_inliers);

    bool GetParallelPlane(
        const Scan3D &input_scan, Eigen::Vector3d &target_normal,
        std::vector<int> &out_inliers);
    bool GetGroundParallelLine(
        const Scan3D &input_scan, std::vector<int> &out_inliers);

    bool GetGroundOrthogonalLine(
        const Scan3D &input_scan, std::vector<int> &out_inliers);

    void MakeDepthImage(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, const int width,
        const int height, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud);
    void SaveDepthImage(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &depth_image,
        const std::string &file_path);
    void UpdateParameters();
    bool FitCuboid(
        const Scan3D &frame, KDTree3d *kdtree_3d, Cuboid &res,
        PointCloud3D &cuboid_corners, const bool visualize_planes = false);
    bool GroundRemoval(const Scan3D &input_scan, std::vector<int> &out_inliers);
};

}  // namespace PERCEPTION
}  // namespace ANSWER