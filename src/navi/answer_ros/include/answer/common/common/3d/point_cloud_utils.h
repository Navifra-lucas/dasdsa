#pragma once
//#include "common/3d/pose3d.h"
#include "common/3d/scan3d.h"
#include "common/pch.h"
#include "common/types.h"
#include "nanoflann/nanoflann.h"

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vector>

#include "Eigen/Core"

namespace ANSWER {
struct PointCloud3DAdaptor {
    const PointCloud3D &obj;

    PointCloud3DAdaptor(const PointCloud3D &obj_)
        : obj(obj_)
    {
    }

    inline size_t kdtree_get_point_count() const { return obj.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return obj[idx].x();
        else if (dim == 1)
            return obj[idx].y();
        else
            return obj[idx].z();
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const
    {
        return false;
    }
};
typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud3DAdaptor>,
    PointCloud3DAdaptor, 3 /* dim */>
    KDTree3d;
namespace KDTreeWrapper3d {
static std::unique_ptr<KDTree3d> GetKDTree3dPtr(
    const PointCloud3D &point_cloud, const PointCloud3DAdaptor &adaptor)
{
    auto kdtree = std::make_unique<KDTree3d>(
        3, adaptor,
        nanoflann::KDTreeSingleIndexAdaptorParams(
            10, nanoflann::KDTreeSingleIndexAdaptorFlags::None, 0));
    return kdtree;
}
}  // namespace KDTreeWrapper3d
namespace PointCloudUtils {
static void ComputeNormals(
    Scan3D &frame, KDTree3d *kdtree, const bool use_radius_search = false,
    const double &radius = 0.5, const bool view_point_arrange = false)
{
    LOG_INFO(
        "Computing normals use radius search: {}, radius: {}",
        use_radius_search, radius);
    const size_t target_size = frame.GetPointCloud().size();
    PointCloud3D normals(target_size, Point3D());
    int K = 9;
    double search_radius = radius;
    std::vector<size_t> indices(target_size);
    std::iota(indices.begin(), indices.end(), 0);
    std::for_each(
        std::execution::par_unseq, indices.begin(), indices.end(),
        [&](size_t idx) {
            const auto &point = frame.GetPointCloud()[idx];
            std::vector<size_t> ret_indexes(K);
            std::vector<double> out_dists_sqr(K);
            std::vector<double> query_pt = {point.x(), point.y(), point.z()};
            if (!use_radius_search) {
                nanoflann::KNNResultSet<double> resultSet(K);
                resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
                kdtree->findNeighbors(
                    resultSet, &query_pt[0], nanoflann::SearchParameters(10));
                Eigen::MatrixXd neighbors = Eigen::MatrixXd::Zero(K, 3);
                // distance threshold can be added here
                for (size_t i = 0; i < resultSet.size(); i++) {
                    neighbors(i, 0) = frame.GetPointCloud()[ret_indexes[i]].x();
                    neighbors(i, 1) = frame.GetPointCloud()[ret_indexes[i]].y();
                    neighbors(i, 2) = frame.GetPointCloud()[ret_indexes[i]].z();
                }

                Eigen::RowVector3d centroid = neighbors.colwise().mean();

                Eigen::MatrixXd centered = neighbors.rowwise() - centroid;

                Eigen::Matrix3d cov =
                    (centered.transpose() * centered) / double(K);
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);

                Eigen::Vector3d normal = solver.eigenvectors().col(
                    0);  // The smallest singular value corresponds to column 1

                // flip normal toward viewpoint
                if (view_point_arrange) {
                    Eigen::Vector3d view_point(0.0, 0.0, 0.0);
                    Eigen::Vector3d point_to_view = view_point - point;
                    if (normal.dot(point_to_view) < 0.0) {
                        normal = -normal;
                    }
                }
                normals[idx] = Point3D(normal.x(), normal.y(), normal.z());
            }
            else {
                std::vector<nanoflann::ResultItem<uint32_t, double>>
                    ret_matches;
                size_t match_num = kdtree->radiusSearch(
                    &query_pt[0], search_radius, ret_matches);
                // NLOG(info)<< "match num : "<<match_num;
                if (match_num < 3) {
                    normals[idx] = Point3D(0.0, 0.0, 0.0);
                    return;
                }

                Eigen::MatrixXd neighbors = Eigen::MatrixXd::Zero(match_num, 3);
                for (size_t i = 0; i < match_num; i++) {
                    neighbors(i, 0) =
                        frame.GetPointCloud()[ret_matches[i].first].x();
                    neighbors(i, 1) =
                        frame.GetPointCloud()[ret_matches[i].first].y();
                    neighbors(i, 2) =
                        frame.GetPointCloud()[ret_matches[i].first].z();
                }
                Eigen::RowVector3d centroid = neighbors.colwise().mean();

                Eigen::MatrixXd centered = neighbors.rowwise() - centroid;

                Eigen::Matrix3d cov =
                    (centered.transpose() * centered) / double(match_num);
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);

                Eigen::Vector3d normal = solver.eigenvectors().col(
                    0);  // The smallest singular value corresponds to column 1
                if (normal.hasNaN()) {
                    normals[idx] = Point3D(0.0, 0.0, 0.0);
                    return;
                }
                // flip normal toward viewpoint
                if (view_point_arrange) {
                    Eigen::Vector3d view_point(0.0, 0.0, 0.0);
                    Eigen::Vector3d point_to_view = view_point - point;
                    if (normal.dot(point_to_view) < 0.0) {
                        normal = -normal;
                    }
                }
                normals[idx] = Point3D(normal.x(), normal.y(), normal.z());
            }
        });
    frame.SetNormals(normals);
}
static std::vector<std::vector<int>> EuclideanDistanceClustering(
    const Scan3D &frame, const KDTree3d *kdtree, const float cluster_tolerance,
    const int cluster_size_min, const int cluster_size_max,
    const bool consider_normals = false,
    const float normal_angle_threshold = 5.0)
{
    // LOG_INFO("normal_angle_threshold {}", normal_angle_threshold);
    std::vector<std::vector<int>> clusters;
    if (frame.GetPointCloud().empty()) {
        LOG_WARN("Empty point cloud received for clustering.");
        return clusters;
    }
    std::vector<bool> processed(frame.GetPointCloud().size(), false);
    for (size_t i = 0; i < frame.GetPointCloud().size(); ++i) {
        if (processed[i])
            continue;
        std::vector<int> cluster;
        std::queue<int> seed_queue;
        // if (frame.GetPointCloud()[i].norm() < DBL_EPSILON) {
        //     processed[i] = true;
        //     continue;
        // }
        seed_queue.push(i);
        processed[i] = true;
        size_t queue_index = 0;
        while (!seed_queue.empty()) {
            int current_index = seed_queue.front();
            seed_queue.pop();
            const auto &point = frame.GetPointCloud()[current_index];
            cluster.push_back(current_index);
            std::vector<nanoflann::ResultItem<uint32_t, double>> ret_indexes;
            const auto &normal = frame.GetNormals()[current_index];
            std::vector<double> query_pt = {point.x(), point.y(), point.z()};
            size_t match_num = kdtree->radiusSearch(
                &query_pt[0], cluster_tolerance, ret_indexes);
            for (size_t j = 0; j < match_num; ++j) {
                int neighbor_index = static_cast<int>(ret_indexes[j].first);
                if (consider_normals) {
                    const auto &neighbor_normal =
                        frame.GetNormals()[neighbor_index];
                    if (std::fabs(neighbor_normal.norm()) < DBL_EPSILON ||
                        std::fabs(normal.norm()) < DBL_EPSILON) {
                        continue;
                    }
                    double cos_angle = std::acos(
                        normal.dot(neighbor_normal) /
                        (normal.norm() * neighbor_normal.norm()));
                    double deg = cos_angle * 180.0 / M_PI;
                    // LOG_INFO("deg {}", deg);
                    if (deg > normal_angle_threshold || std::isnan(cos_angle))
                        continue;

                    // LOG_INFO("angle : {}", angle);
                }
                if (!processed[neighbor_index]) {
                    seed_queue.push(neighbor_index);
                    processed[neighbor_index] = true;
                }
            }
        }
        if (cluster.size() >= static_cast<size_t>(cluster_size_min) &&
            cluster.size() <= static_cast<size_t>(cluster_size_max)) {
            clusters.push_back(cluster);
        }
    }
    return clusters;
}

static Eigen::Vector3f GetColorCode(const int id, const int max_id)
{
    auto HSVtoRGB = [](float h, float s, float v) {
        float c = v * s;
        float x = c * (1 - fabs(fmod(h * 6.0f, 2.0f) - 1));
        float m = v - c;

        float r, g, b;

        if (h < 1.0f / 6.0f) {
            r = c;
            g = x;
            b = 0;
        }
        else if (h < 2.0f / 6.0f) {
            r = x;
            g = c;
            b = 0;
        }
        else if (h < 3.0f / 6.0f) {
            r = 0;
            g = c;
            b = x;
        }
        else if (h < 4.0f / 6.0f) {
            r = 0;
            g = x;
            b = c;
        }
        else if (h < 5.0f / 6.0f) {
            r = x;
            g = 0;
            b = c;
        }
        else {
            r = c;
            g = 0;
            b = x;
        }

        return Eigen::Vector3f((r + m), (g + m), (b + m));
    };

    auto frac = [](float x) {
        return x - floorf(x);  // 0~1 사이 소수 부분만 사용
    };

    // float hue = (float)id / (float)max_id;  // 0~1 사이
    const float golden = 0.61803398875f;
    // float h = fmodf(id * golden, 1.0f);  // 0~1에서 pseudo-random하게 흩어짐
    float h = frac(id * golden);  // 0~1, pseudo-random hue
    float s = 0.6f + 0.4f * frac(id * 0.37f);  // 0.6~1.0
    float v = 0.7f + 0.3f * frac(id * 0.53f);  // 0.7~1.0

    return HSVtoRGB(h, s, v);  // 강한 채도/밝기
}
static size_t ConcatenatePointCloud(
    const Scan3D &input_cloud, Scan3D &output_cloud)
{
    size_t total_points = output_cloud.GetPointCloud().size() +
        input_cloud.GetPointCloud().size();
    for (size_t i = 0; i < input_cloud.GetPointCloud().size(); ++i) {
        output_cloud.MutablePointCloud().push_back(
            input_cloud.GetPointCloud()[i]);
        if (input_cloud.GetNormals().empty() == false) {
            output_cloud.MutableNormals().push_back(
                input_cloud.GetNormals()[i]);
        }
    }
    return total_points;
}
static std::pair<Scan3D, Scan3D> SubstractPointCloud(
    const Scan3D &input_cloud, const std::vector<int> &indices_to_remove)
{
    Scan3D removed_cloud;
    Scan3D remaining_cloud;
    std::unordered_set<int> indices_set(
        indices_to_remove.begin(), indices_to_remove.end());
    for (size_t i = 0; i < input_cloud.GetPointCloud().size(); ++i) {
        if (indices_set.find(static_cast<int>(i)) ==
            indices_set.end()) {  // not found in remove set
            remaining_cloud.MutablePointCloud().push_back(
                input_cloud.GetPointCloud()[i]);
            if (input_cloud.GetNormals().empty() == false) {
                remaining_cloud.MutableNormals().push_back(
                    input_cloud.GetNormals()[i]);
            }
        }
        else {
            removed_cloud.MutablePointCloud().push_back(
                input_cloud.GetPointCloud()[i]);
            if (input_cloud.GetNormals().empty() == false) {
                removed_cloud.MutableNormals().push_back(
                    input_cloud.GetNormals()[i]);
            }
        }
    }
    // LOG_INFO(
    //     "SubstractPointCloud: remaining {} points, removed {} points",
    //     remaining_cloud.GetPointCloud().size(),
    //     removed_cloud.GetPointCloud().size());
    return std::make_pair(remaining_cloud, removed_cloud);
}
static std::pair<Point3D, Eigen::Matrix3d> ComputeCentroidAndCovariance(
    const Scan3D &frame,
    const int min_height = std::numeric_limits<int>::lowest(),
    const int max_height = std::numeric_limits<int>::max())
{
    Point3D centroid(0.0, 0.0, 0.0);
    int count = 0;
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    if (frame.GetPointCloud().empty()) {
        return std::make_pair(centroid, covariance);
    }
    for (const auto &point : frame.GetPointCloud()) {
        if (point.z() < min_height || point.z() > max_height ||
            point.hasNaN()) {
            continue;
        }
        centroid += point;
        count++;
    }
    centroid /= static_cast<double>(count);

    for (const auto &point : frame.GetPointCloud()) {
        if (point.z() < min_height || point.z() > max_height ||
            point.hasNaN()) {
            continue;
        }
        Eigen::Vector3d centered_point = (point - centroid);
        covariance += centered_point * centered_point.transpose();
    }

    if (count == 0) {
        return std::make_pair(centroid, covariance);
    }
    covariance /= static_cast<double>(count);

    return std::make_pair(centroid, covariance);
}
static std::vector<double> ComputeKNNMeanDistances(
    const Scan3D &frame, KDTree3d *kdtree, const int K)
{
    std::vector<double> knn_mean_distances(frame.GetPointCloud().size(), 0.0);
    // knn_mean_distances.resize(frame.GetPointCloud().size());

    std::vector<size_t> indices(frame.GetPointCloud().size(), 0);
    std::iota(indices.begin(), indices.end(), 0);
    std::for_each(
        std::execution::par_unseq, indices.begin(), indices.end(),
        [&](size_t idx) {
            const auto &point = frame.GetPointCloud()[idx];
            std::vector<size_t> ret_indexes(K);
            std::vector<double> out_dists_sqr(K);
            std::vector<double> query_pt = {point.x(), point.y(), point.z()};

            nanoflann::KNNResultSet<double> resultSet(K);
            resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
            kdtree->findNeighbors(
                resultSet, &query_pt[0], nanoflann::SearchParameters(10));

            double sum_dist = 0.0;
            for (size_t j = 0; j < resultSet.size(); j++) {
                sum_dist += std::sqrt(out_dists_sqr[j]);
            }
            double mean_dist = sum_dist / static_cast<double>(resultSet.size());
            knn_mean_distances[idx] = mean_dist;
        });

    return knn_mean_distances;
}
}  // namespace PointCloudUtils
}  // namespace ANSWER