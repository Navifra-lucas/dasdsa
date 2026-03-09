#include "common/pch.h"

#include <opencv2/opencv.hpp>

#include "3d_perception/segmentation/segmentation.h"

namespace ANSWER {
namespace PERCEPTION {

Segmentation::Segmentation()
    : dist_thresh_(0.02)
    , cos_thresh_(
          std::cos(10.0 * double(M_PI / 180.0)))  // normal angle <= 10 deg
    , min_support_(500)
    , ransac_iters_(1000)
    , knn_radius_(0.3)  // for optional local growth
    , use_local_growth_(true)
    , rng_(std::chrono::steady_clock::now().time_since_epoch().count())
{
    UpdateParameters();
}
Segmentation::~Segmentation()
{
}
std::vector<std::vector<int>> Segmentation::SegmentMultiplePlane(
    const Scan3D &frame, KDTree3d *kdtree_3d)
{
    std::vector<bool> used(frame.GetPointCloud().size(), false);
    std::vector<bool> used_pts(frame.GetPointCloud().size(), false);
    std::vector<int> pool(frame.GetPointCloud().size(), 0);
    std::iota(pool.begin(), pool.end(), 0);
    std::vector<std::vector<int>> clusters;
    LOG_INFO("Ongoing multi plane segmentation...");
    while (pool.size() >= static_cast<size_t>(min_support_)) {
        std::vector<int> best_inliers;
        Plane best_plane;
        int best_score = -1;
        for (int iter = 0; iter < ransac_iters_; ++iter) {
            int ri0 = std::uniform_int_distribution<int>(
                0, (int)pool.size() - 1)(rng_);
            int ri1 = std::uniform_int_distribution<int>(
                0, (int)pool.size() - 1)(rng_);
            int ri2 = std::uniform_int_distribution<int>(
                0, (int)pool.size() - 1)(rng_);
            if (ri0 == ri1 || ri1 == ri2 || ri0 == ri2) {
                --iter;
                continue;
            }
            int i0 = pool[ri0];
            int i1 = pool[ri1];
            int i2 = pool[ri2];
            Plane pl = FitPlaneFrom3Pts(
                frame.GetPointCloud()[i0], frame.GetPointCloud()[i1],
                frame.GetPointCloud()[i2]);
            if (!std::isfinite(pl.d)) {
                --iter;
                continue;
            }
            Eigen::Vector3d n_hint =
                (frame.GetNormals()[i0] + frame.GetNormals()[i1] +
                 frame.GetNormals()[i2])
                    .normalized();
            if (pl.n.dot(n_hint) < 0) {
                pl.n = -pl.n;
                pl.d = -pl.d;
            }
            int score = 0;
            for (int idx : pool) {
                if (PointPlaneDistance(pl, frame.GetPointCloud()[idx]) <=
                        dist_thresh_ &&
                    NormalCosAngle(frame.GetNormals()[idx], pl.n) >=
                        cos_thresh_)
                    ++score;
            }
            if (score > best_score) {
                best_score = score;
                best_plane = pl;
            }
        }
        if (best_score < min_support_)
            break;
        std::vector<int> inliers;
        inliers.reserve(best_score);
        for (int idx : pool) {
            if (used_pts[idx] == false &&
                PointPlaneDistance(best_plane, frame.GetPointCloud()[idx]) <=
                    dist_thresh_ &&
                NormalCosAngle(frame.GetNormals()[idx], best_plane.n) >=
                    cos_thresh_) {
                inliers.push_back(idx);
                used_pts[idx] = true;
            }
        }

        Plane refined = FitPlaneTLS(frame.GetPointCloud(), inliers);

        if (use_local_growth_) {
            std::vector<bool> is_inlier(frame.GetPointCloud().size(), false);
            for (int i : inliers)
                is_inlier[i] = true;
            std::queue<int> q;
            for (int i : inliers)
                q.push(i);

            while (!q.empty()) {
                int cur = q.front();
                q.pop();

                std::vector<double> query_pt = {
                    frame.GetPointCloud()[cur].x(),
                    frame.GetPointCloud()[cur].y(),
                    frame.GetPointCloud()[cur].z()};

                std::vector<nanoflann::ResultItem<uint32_t, double>>
                    ret_matches;
                size_t match_num = kdtree_3d->radiusSearch(
                    &query_pt[0], knn_radius_, ret_matches);
                for (size_t i = 0; i < match_num; i++) {
                    int j = ret_matches[i].first;
                    if (is_inlier[j])
                        continue;

                    if (PointPlaneDistance(refined, frame.GetPointCloud()[j]) <=
                            dist_thresh_ &&
                        NormalCosAngle(frame.GetNormals()[j], refined.n) >=
                            cos_thresh_) {
                        is_inlier[j] = true;
                        q.push(j);
                    }
                }
            }
            inliers.clear();
            for (int idx : pool)
                if (is_inlier[idx])
                    inliers.push_back(idx);
            if (!inliers.empty())
                refined = FitPlaneTLS(frame.GetPointCloud(), inliers);
        }
        if ((int)inliers.size() >= min_support_) {
            clusters.push_back(inliers);
            for (int i : inliers) {
                used[i] = true;
            }
            std::vector<int> new_pool;
            new_pool.reserve(pool.size() - inliers.size());
            for (int idx : pool)
                if (!used[idx])
                    new_pool.push_back(idx);
            pool.swap(new_pool);
        }
        else {
            break;
        }
    }
    LOG_INFO("Plane segmentation finished, {} planes found", clusters.size());
    return clusters;
}

bool Segmentation::SegmentPlane(
    const Scan3D &frame, Plane &out_plane, std::vector<int> &out_inliers,
    const PlaneRansacParameters &plane_ransac_params)
{
    if (frame.GetPointCloud().size() < 3) {
        LOG_WARN("Plane segmentation failed: not enough points");
        return false;
    }

    // 바닥 제거 시 고정 시드 사용 → 동일 입력에 대해 항상 동일 평면 선택, 감지 안정화
    static constexpr unsigned int kGroundRansacSeed = 42u;
    std::mt19937 local_ground_rng(kGroundRansacSeed);
    std::mt19937 &use_rng = plane_ransac_params.ground_removal ? local_ground_rng : rng_;

    // ground_removal 시 높이 조건을 만족하는 포인트 인덱스를 미리 수집
    std::vector<int> ground_candidate_indices;
    if (plane_ransac_params.ground_removal) {
        ground_candidate_indices.reserve(frame.GetPointCloud().size());
        for (int i = 0; i < (int)frame.GetPointCloud().size(); ++i) {
            if (std::abs(frame.GetPointCloud()[i].z()) <=
                plane_ransac_params.ground_point_max_height) {
                ground_candidate_indices.push_back(i);
            }
        }
        if (ground_candidate_indices.size() < 3) {
            LOG_WARN("Not enough ground candidate points: {}",
                     ground_candidate_indices.size());
            return false;
        }
    }

    int bestInlierCount = -1;
    Plane bestPlane;

    for (int iter = 0; iter < plane_ransac_params.ransac_iter; ++iter) {
        int i0, i1, i2;
        if (plane_ransac_params.ground_removal) {
            // 미리 수집한 후보에서만 샘플링 → --iter 재시도 없이 O(1) 샘플링
            int ri0 = std::uniform_int_distribution<int>(
                0, (int)ground_candidate_indices.size() - 1)(use_rng);
            int ri1 = std::uniform_int_distribution<int>(
                0, (int)ground_candidate_indices.size() - 1)(use_rng);
            int ri2 = std::uniform_int_distribution<int>(
                0, (int)ground_candidate_indices.size() - 1)(use_rng);
            if (ri0 == ri1 || ri1 == ri2 || ri0 == ri2) {
                --iter;
                continue;
            }
            i0 = ground_candidate_indices[ri0];
            i1 = ground_candidate_indices[ri1];
            i2 = ground_candidate_indices[ri2];
        }
        else {
            i0 = std::uniform_int_distribution<int>(
                0, (int)frame.GetPointCloud().size() - 1)(use_rng);
            i1 = std::uniform_int_distribution<int>(
                0, (int)frame.GetPointCloud().size() - 1)(use_rng);
            i2 = std::uniform_int_distribution<int>(
                0, (int)frame.GetPointCloud().size() - 1)(use_rng);
            if (i0 == i1 || i0 == i2 || i1 == i2) {
                --iter;
                continue;
            }
        }

        Plane candidate = FitPlaneFrom3Pts(
            frame.GetPointCloud()[i0], frame.GetPointCloud()[i1],
            frame.GetPointCloud()[i2]);

        if (!std::isfinite(candidate.d)) {
            --iter;
            continue;
        }

        if (plane_ransac_params.use_orthogonal_check) {
            double dot = std::abs(candidate.n.dot(
                plane_ransac_params.reference_orthogonal_normal));
            if (dot > plane_ransac_params.orthogonal_cos_threshold)
                continue;
        }
        if (plane_ransac_params.use_parallel_check) {
            double dot = std::abs(
                candidate.n.dot(plane_ransac_params.reference_parallel_normal));
            if (dot < plane_ransac_params.parallel_cos_threshold) {
                continue;
            }
        }
        if (plane_ransac_params.ground_removal) {
            double dot = std::abs(
                candidate.n.dot(plane_ransac_params.reference_parallel_normal));
            if (dot < plane_ransac_params.parallel_cos_threshold) {
                continue;
            }
        }

        // 먼저 카운트만 세서 최고 기록인지 확인 (벡터 할당 비용 절약)
        int count = 0;
        const int n_pts = (int)frame.GetPointCloud().size();
        const double dist_th = plane_ransac_params.point_to_plane_dist_threshold;
        const auto &pts = frame.GetPointCloud();
        const auto &plane_n = candidate.n;
        const double plane_d = candidate.d;
        for (int idx = 0; idx < n_pts; ++idx) {
            double d = std::abs(plane_n.dot(pts[idx]) + plane_d);
            if (d < dist_th) {
                ++count;
            }
        }
        if (count > bestInlierCount) {
            bestInlierCount = count;
            bestPlane = candidate;
            // 최고 기록일 때만 인라이어 리스트 구축
            out_inliers.clear();
            out_inliers.reserve(count);
            for (int idx = 0; idx < n_pts; ++idx) {
                double d = std::abs(plane_n.dot(pts[idx]) + plane_d);
                if (d < dist_th) {
                    out_inliers.push_back(idx);
                }
            }
            // ground removal은 바닥이 전체 포인트의 다수를 차지하므로 절반 이상이면 조기 종료
            // (기존 n_pts/5는 너무 낮아서 틸트된 평면이나 엉뚱한 평면도 통과함)
            if (plane_ransac_params.ground_removal &&
                bestInlierCount > n_pts / 2) {
                break;
            }
        }
    }

    if (bestInlierCount < plane_ransac_params.min_support) {
        LOG_WARN(
            "Plane segmentation failed: no plane found with enough inliers");
        return false;
    }

    Plane refined = FitPlaneTLS(frame.GetPointCloud(), out_inliers);

    // ground removal은 RANSAC 결과로 충분하므로 local_growth 스킵 (KD-tree+BFS 비용 절약)
    if (use_local_growth_ && !plane_ransac_params.ground_removal) {
        PointCloud3DAdaptor adaptor =
            PointCloud3DAdaptor(frame.GetPointCloud());
        auto kdtree_3d =
            KDTreeWrapper3d::GetKDTree3dPtr(frame.GetPointCloud(), adaptor);
        if (kdtree_3d == nullptr) {
            LOG_ERROR("Failed to build KD-Tree");
            return false;
        }
        std::vector<bool> is_inlier(frame.GetPointCloud().size(), false);
        for (int i : out_inliers)
            is_inlier[i] = true;
        std::queue<int> q;
        for (int i : out_inliers)
            q.push(i);

        while (!q.empty()) {
            int cur = q.front();
            q.pop();

            std::vector<double> query_pt = {
                frame.GetPointCloud()[cur].x(), frame.GetPointCloud()[cur].y(),
                frame.GetPointCloud()[cur].z()};

            std::vector<nanoflann::ResultItem<uint32_t, double>> ret_matches;
            size_t match_num =
                kdtree_3d->radiusSearch(&query_pt[0], knn_radius_, ret_matches);
            for (size_t i = 0; i < match_num; i++) {
                int j = ret_matches[i].first;
                if (is_inlier[j])
                    continue;

                if (PointPlaneDistance(refined, frame.GetPointCloud()[j]) <=
                    plane_ransac_params.point_to_plane_dist_threshold) {
                    is_inlier[j] = true;
                    q.push(j);
                }
            }
        }
        out_inliers.clear();
        for (int idx = 0; idx < (int)frame.GetPointCloud().size(); ++idx)
            if (is_inlier[idx])
                out_inliers.push_back(idx);
        if (!out_inliers.empty())
            refined = FitPlaneTLS(frame.GetPointCloud(), out_inliers);
    }

    if (refined.n.dot(bestPlane.n) < 0.0)
        refined.n = -refined.n;

    bestPlane.n = refined.n.normalized();
    bestPlane.d = refined.d;
    out_plane = bestPlane;

    return true;
}

void Segmentation::MakeDepthImage(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, const int width,
    const int height, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud)
{
    float h_fov = 90.0f * (M_PI / 180.0f);
    float v_fov = 60.0f * (M_PI / 180.0f);
    output_cloud->points.resize(
        width * height,
        pcl::PointXYZ(
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN()));
    output_cloud->width = width;
    output_cloud->height = height;
    output_cloud->is_dense = false;
    for (const auto &pt : input_cloud->points) {
        float az = std::atan2(pt.y, pt.x);
        float el = std::atan2(pt.z, std::sqrt(pt.x * pt.x + pt.y * pt.y));

        int u = static_cast<int>((az + h_fov / 2) / h_fov * width);
        int v = static_cast<int>((el + v_fov / 2) / v_fov * height);

        if (u >= 0 && u < width && v >= 0 && v < height) {
            LOG_INFO(
                "u: {}, v: {}, x: {}, y: {}, z: {}", u, v, pt.x, pt.y, pt.z);
            output_cloud->points[v * width + u] = pt;
        }
    }
}

void Segmentation::SaveDepthImage(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &depth_image,
    const std::string &file_path)
{
    int width = depth_image->width;
    int height = depth_image->height;
    cv::Mat depth_mat(height, width, CV_32FC1);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const auto &pt = depth_image->at(x, y);
            if (std::isfinite(pt.z)) {
                depth_mat.at<float>(y, x) = pt.z;
            }
            else {
                depth_mat.at<float>(y, x) = 0.0f;
            }
        }
    }
    cv::Mat depth_16u;
    depth_mat.convertTo(
        depth_16u, CV_16UC1, 65535.0 / 10.0);
    cv::imwrite(file_path, depth_16u);
}

Plane Segmentation::FitPlaneTLS(
    const PointCloud3D &pts, const std::vector<int> &idxs)
{
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (int i : idxs)
        mean += pts[i].head<3>();
    mean /= double(idxs.size());

    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
    for (int i : idxs) {
        Eigen::Vector3d q = pts[i].head<3>() - mean;
        C += q * q.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
    Eigen::Vector3d n =
        es.eigenvectors().col(0).normalized();
    double d = -n.dot(mean);
    return {n, d};
}
Plane Segmentation::FitPlaneFrom3Pts(
    const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
    const Eigen::Vector3d &p2)
{
    Eigen::Vector3d n = (p1 - p0).cross(p2 - p0);
    double nn = n.norm();
    if (nn < DBL_EPSILON)
        return {
            Eigen::Vector3d(0, 0, 1),
            std::numeric_limits<double>::infinity()};
    n /= nn;
    double d = -n.dot(p0);
    return {n, d};
}
double Segmentation::PointPlaneDistance(
    const Plane &pl, const Eigen::Vector3d &pt)
{
    return std::abs(pl.n.dot(pt) + pl.d);
}

double Segmentation::PointLineDistance(
    const Eigen::Vector3d &p, const Eigen::Vector3d &p0,
    const Eigen::Vector3d &dir)
{
    Eigen::Vector3d v = p - p0;
    Eigen::Vector3d cp = v.cross(dir);
    double num = cp.norm();
    double den = dir.norm();
    if (den < DBL_EPSILON)
        return std::numeric_limits<double>::max();
    return num / den;
}

double Segmentation::NormalCosAngle(
    const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
    return a.dot(b);
}

std::vector<std::vector<int>> Segmentation::SplitParallelByDistance(
    const PointCloud3D &pts, const Plane &pl, const std::vector<int> &inliers,
    double gap_thresh, int min_layer_size)
{
    if ((int)inliers.size() < min_layer_size)
        return {};

    std::vector<std::pair<double, int>> depths;
    depths.reserve(inliers.size());
    for (int idx : inliers) {
        double s = pl.n.dot(pts[idx].head<3>());
        depths.emplace_back(s, idx);
    }
    std::sort(depths.begin(), depths.end(), [](const auto &a, const auto &b) {
        return a.first < b.first;
    });

    std::vector<std::vector<int>> layers;
    std::vector<int> cur;
    cur.reserve(inliers.size());
    for (size_t i = 0; i < depths.size(); ++i) {
        if (!cur.empty()) {
            double gap = depths[i].first - depths[i - 1].first;
            if (gap > gap_thresh) {
                if ((int)cur.size() >= min_layer_size)
                    layers.push_back(cur);
                cur.clear();
            }
        }
        cur.push_back(depths[i].second);
    }
    if ((int)cur.size() >= min_layer_size)
        layers.push_back(cur);

    return layers;
}

bool Segmentation::FitCuboid(
    const Scan3D &frame, KDTree3d *kdtree_3d, Cuboid &res,
    PointCloud3D &cuboid_corners, const bool visualize_planes)
{
    Scan3D working_frame = frame;
    Pose3D dummy_pose;
    Plane plane1, plane2, plane3;

    float scale = 1.0f;
    std::vector<int> inliers1, inliers2, inliers3;
    plane_ransac_params_.use_orthogonal_check = false;
    plane_ransac_params_.use_parallel_check = false;
    plane_ransac_params_.ground_removal = false;
    if (!SegmentPlane(working_frame, plane1, inliers1, plane_ransac_params_)) {
        LOG_WARN("Cuboid fitting failed: could not find first plane");
        return false;
    }
    auto [remaining_frame, inlier_frame] =
        PointCloudUtils::SubstractPointCloud(working_frame, inliers1);
    if (visualize_planes) {
        VisualizerCallback::point_cloud_3d_callback(
            inlier_frame, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose), "inlier1",
            "red", scale);
    }

    if (remaining_frame.GetPointCloud().size() <
        static_cast<size_t>(plane_ransac_params_.min_support)) {
        LOG_WARN(
            "Cuboid fitting failed: not enough points after first plane {}",
            remaining_frame.GetPointCloud().size());
        return false;
    }

    plane_ransac_params_.use_orthogonal_check = true;
    plane_ransac_params_.reference_orthogonal_normal = plane1.n;
    plane_ransac_params_.ground_removal = false;
    if (!SegmentPlane(
            remaining_frame, plane2, inliers2, plane_ransac_params_)) {
        LOG_WARN("Cuboid fitting failed: could not find second plane");
        return false;
    }
    auto [remaining_frame2, inlier_frame2] =
        PointCloudUtils::SubstractPointCloud(remaining_frame, inliers2);

    if (visualize_planes) {
        VisualizerCallback::point_cloud_3d_callback(
            inlier_frame2, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose), "inlier2",
            "blue", scale);
    }

    Eigen::Vector3d n3 = plane1.n.cross(plane2.n).normalized();

    if (n3.norm() < DBL_EPSILON) {
        LOG_WARN(
            "Cuboid fitting failed: first two planes are parallel {} {} {}",
            n3.x(), n3.y(), n3.z());
        return false;
    }

    plane_ransac_params_.use_orthogonal_check = false;
    plane_ransac_params_.use_parallel_check = true;
    plane_ransac_params_.reference_parallel_normal = plane2.n;
    plane_ransac_params_.ground_removal = false;
    if (!SegmentPlane(
            remaining_frame2, plane3, inliers3, plane_ransac_params_)) {
        LOG_WARN("Cuboid fitting failed: could not find third plane");
        return false;
    }
    auto [remaining_frame3, inlier_frame3] =
        PointCloudUtils::SubstractPointCloud(remaining_frame2, inliers3);

    if (visualize_planes) {
        VisualizerCallback::point_cloud_3d_callback(
            inlier_frame3, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose), "inlier3",
            "black", scale);
    }

    plane2.n = (n3.cross(plane1.n)).normalized();
    plane1.n = plane1.n.normalized();
    Eigen::Matrix3d axes;
    axes.col(0) = plane1.n;
    axes.col(1) = plane2.n;
    axes.col(2) = n3;

    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double minZ = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();
    double maxZ = -std::numeric_limits<double>::max();

    for (const auto &p : frame.GetPointCloud()) {
        double x = p.dot(plane1.n);
        double y = p.dot(plane2.n);
        double z = p.dot(n3);
        minX = std::min(minX, x);
        maxX = std::max(maxX, x);
        minY = std::min(minY, y);
        maxY = std::max(maxY, y);
        minZ = std::min(minZ, z);
        maxZ = std::max(maxZ, z);
    }

    Eigen::Vector3d extents;
    extents.x() = 0.5 * (maxX - minX);
    extents.y() = 0.5 * (maxY - minY);
    extents.z() = 0.5 * (maxZ - minZ);

    if (extents.minCoeff() <= 0.0) {
        LOG_WARN("Cuboid fitting failed: invalid extents");
        return false;
    }

    Eigen::Vector3d centerLocal;
    centerLocal.x() = 0.5 * (maxX + minX);
    centerLocal.y() = 0.5 * (maxY + minY);
    centerLocal.z() = 0.5 * (maxZ + minZ);

    Eigen::Vector3d centerWorld = axes * centerLocal;

    res.center = centerWorld;
    res.axes = axes;
    res.extents = extents;
    auto get_corner = [&](const Cuboid cuboid) {
        PointCloud3D corners;

        Eigen::Vector3d ax = cuboid.axes.col(0);
        Eigen::Vector3d ay = cuboid.axes.col(1);
        Eigen::Vector3d az = cuboid.axes.col(2);

        for (int sx = -1; sx <= 1; sx += 2) {
            for (int sy = -1; sy <= 1; sy += 2) {
                for (int sz = -1; sz <= 1; sz += 2) {
                    Eigen::Vector3d c = cuboid.center +
                        sx * cuboid.extents.x() * ax +
                        sy * cuboid.extents.y() * ay +
                        sz * cuboid.extents.z() * az;
                    corners.push_back(c);
                }
            }
        }
        return corners;
    };

    std::vector<int> inliers_union = inliers1;
    inliers_union.insert(inliers_union.end(), inliers2.begin(), inliers2.end());
    inliers_union.insert(inliers_union.end(), inliers3.begin(), inliers3.end());
    cuboid_corners = get_corner(res);

    Cuboid refined;
    if (RefineCuboid(frame.GetPointCloud(), inliers_union, res, refined)) {
        res = refined;
        cuboid_corners = get_corner(res);
        LOG_INFO("Cuboid refined successfully");
    }

    return true;
}

bool Segmentation::RefineCuboid(
    const PointCloud3D &pts, const std::vector<int> &inliers,
    const Cuboid &initial_cuboid, Cuboid &refined_cuboid)
{
    if (inliers.size() < 20)
        return false;

    double bandRatio = 0.1;
    const Eigen::Matrix3d &axes = initial_cuboid.axes;
    const Eigen::Vector3d &ext0 = initial_cuboid.extents;
    const Eigen::Vector3d &c0 = initial_cuboid.center;

    Eigen::Vector3d centerLocal0;
    centerLocal0.x() = axes.col(0).dot(c0);
    centerLocal0.y() = axes.col(1).dot(c0);
    centerLocal0.z() = axes.col(2).dot(c0);

    Eigen::Vector3d min0 = centerLocal0 - ext0;
    Eigen::Vector3d max0 = centerLocal0 + ext0;

    Eigen::Vector3d minRefined = min0;
    Eigen::Vector3d maxRefined = max0;

    for (int axisIdx = 0; axisIdx < 3; ++axisIdx) {
        Eigen::Vector3d n = axes.col(axisIdx);

        double band = bandRatio * ext0[axisIdx];
        if (band < 1e-6)
            band = 1e-6;

        std::vector<double> nearMin;
        std::vector<double> nearMax;
        nearMin.reserve(inliers.size());
        nearMax.reserve(inliers.size());

        for (int idx : inliers) {
            const Eigen::Vector3d &p = pts[idx].head<3>();
            double t = n.dot(p);

            if (std::fabs(t - min0[axisIdx]) < band)
                nearMin.push_back(t);
            if (std::fabs(t - max0[axisIdx]) < band)
                nearMax.push_back(t);
        }

        if (!nearMin.empty()) {
            double sum = 0.0;
            for (double v : nearMin)
                sum += v;
            minRefined[axisIdx] = sum / (double)nearMin.size();
        }

        if (!nearMax.empty()) {
            double sum = 0.0;
            for (double v : nearMax)
                sum += v;
            maxRefined[axisIdx] = sum / (double)nearMax.size();
        }
    }

    Eigen::Vector3d centerLocal_new = 0.5 * (minRefined + maxRefined);
    Eigen::Vector3d ext_new = 0.5 * (maxRefined - minRefined);

    for (int i = 0; i < 3; ++i)
        ext_new[i] = std::max(ext_new[i], 1e-6);

    Eigen::Vector3d centerWorld_new = axes.col(0) * centerLocal_new.x() +
        axes.col(1) * centerLocal_new.y() + axes.col(2) * centerLocal_new.z();

    refined_cuboid.axes = axes;
    refined_cuboid.center = centerWorld_new;
    refined_cuboid.extents = ext_new;

    return true;
}

bool Segmentation::GroundRemoval(
    const Scan3D &input_scan, std::vector<int> &out_inliers)
{
    plane_ransac_params_.use_orthogonal_check = false;
    plane_ransac_params_.use_parallel_check = false;
    plane_ransac_params_.ground_removal = true;
    plane_ransac_params_.reference_parallel_normal = Eigen::Vector3d(0, 0, 1);
    Plane ground_plane;
    std::vector<int> plane_idx;
    if (!SegmentPlane(
            input_scan, ground_plane, plane_idx, plane_ransac_params_)) {
        LOG_WARN("Ground removal failed: could not find ground plane");
        return false;
    }
    // 법선을 항상 위쪽(+Z)으로 통일 → "아래" 판단이 프레임마다 뒤바뀌는 현상 방지
    if (ground_plane.n.z() < 0.0) {
        ground_plane.n = -ground_plane.n;
        ground_plane.d = -ground_plane.d;
    }
    // 바닥면 + 바닥면 아래 포인트 모두 제거 (signed distance <= margin 인 점 제거)
    const double kBelowMargin = 0.01;  // [m] 바닥면 위 3cm 이내 포인트까지 포함하여 제거
    out_inliers.clear();
    out_inliers.reserve(input_scan.GetPointCloud().size());
    for (size_t i = 0; i < input_scan.GetPointCloud().size(); ++i) {
        double signed_dist =
            ground_plane.n.dot(input_scan.GetPointCloud()[i]) + ground_plane.d;
        if (signed_dist <= kBelowMargin)
            out_inliers.push_back(static_cast<int>(i));
    }
    LOG_INFO(
        "Ground removal success: {} points removed (floor + below)",
        out_inliers.size());
    return true;
}
bool Segmentation::GetParallelPlane(
    const Scan3D &input_scan, Eigen::Vector3d &target_normal,
    std::vector<int> &out_inliers)
{
    plane_ransac_params_.use_orthogonal_check = false;
    plane_ransac_params_.use_parallel_check = true;
    plane_ransac_params_.ground_removal = false;
    plane_ransac_params_.reference_parallel_normal = target_normal;
    Plane detected_plane;
    std::vector<int> plane_idx;
    if (!SegmentPlane(
            input_scan, detected_plane, plane_idx, plane_ransac_params_)) {
        LOG_WARN(
            "Parallel plane segmentation failed: could not find parallel plane");
        return false;
    }
    LOG_INFO(
        "Parallel plane segmentation success: {} points found",
        plane_idx.size());
    out_inliers = plane_idx;
    target_normal = detected_plane.n;
    return true;
}
bool Segmentation::GetGroundParallelLine(
    const Scan3D &input_scan, std::vector<int> &out_inliers)
{
    Line detected_line;
    std::vector<int> line_idx;
    line_ransac_params_.use_parallel_check = true;
    line_ransac_params_.use_orthogonal_check = false;
    line_ransac_params_.reference_orthogonal_vector = Eigen::Vector3d(0, 0, 0);
    line_ransac_params_.reference_parallel_vector = Eigen::Vector3d(0, 0, 1);

    if (!SegmentLine(
            input_scan, detected_line, line_idx, line_ransac_params_)) {
        LOG_WARN("Line segmentation failed: could not find line");
        return false;
    }
    LOG_INFO("Line segmentation success: {} points found", line_idx.size());
    out_inliers = line_idx;
    return true;
}
bool Segmentation::GetGroundOrthogonalLine(
    const Scan3D &input_scan, std::vector<int> &out_inliers)
{
    Line detected_line;
    std::vector<int> line_idx;
    line_ransac_params_.use_parallel_check = false;
    line_ransac_params_.use_orthogonal_check = true;
    line_ransac_params_.reference_orthogonal_vector = Eigen::Vector3d(0, 0, 1);
    line_ransac_params_.reference_parallel_vector = Eigen::Vector3d(0, 0, 0);

    if (!SegmentLine(
            input_scan, detected_line, line_idx, line_ransac_params_)) {
        LOG_WARN("Line segmentation failed: could not find line");
        return false;
    }
    LOG_INFO("Line segmentation success: {} points found", line_idx.size());
    out_inliers = line_idx;
    return true;
}
bool Segmentation::SegmentLine(
    const Scan3D &frame, Line &out_line, std::vector<int> &out_inliers,
    const LineRansacParameters &line_ransac_params)
{
    Line best_model;
    size_t best_inlier_count = 0;

    if (frame.GetPointCloud().size() < 2) {
        LOG_WARN("Not enough points!");
        return false;
    }

    for (int iter = 0; iter < line_ransac_params.ransac_iter; ++iter) {
        int i1 = std::uniform_int_distribution<int>(
            0, (int)frame.GetPointCloud().size() - 1)(rng_);
        int i2 = std::uniform_int_distribution<int>(
            0, (int)frame.GetPointCloud().size() - 1)(rng_);
        if (i1 == i2) {
            --iter;
            continue;
        }

        const Eigen::Vector3d &p1 = frame.GetPointCloud()[i1].head<3>();
        const Eigen::Vector3d &p2 = frame.GetPointCloud()[i2].head<3>();

        Eigen::Vector3d dir = p2 - p1;
        if (dir.norm() < DBL_EPSILON) {
            --iter;
            continue;
        }
        dir.normalize();
        if (line_ransac_params.use_orthogonal_check) {
            double dot = std::abs(
                dir.dot(line_ransac_params.reference_orthogonal_vector));
            if (dot > line_ransac_params.orthogonal_cos_threshold)
                continue;
        }
        if (line_ransac_params.use_parallel_check) {
            double dot =
                std::abs(dir.dot(line_ransac_params.reference_parallel_vector));
            if (dot < line_ransac_params.parallel_cos_threshold) {
                --iter;
                continue;
            }
        }

        std::vector<int> inliers;
        for (size_t idx = 0; idx < frame.GetPointCloud().size(); ++idx) {
            double d = PointLineDistance(frame.GetPointCloud()[idx], p1, dir);
            if (d < line_ransac_params.point_to_line_dist_threshold) {
                inliers.push_back(static_cast<int>(idx));
            }
        }

        if (inliers.size() > best_inlier_count &&
            static_cast<int>(inliers.size()) >=
                line_ransac_params.min_support) {
            best_inlier_count = inliers.size();
            best_model.point = p1;
            best_model.direction = dir.normalized();
            out_inliers = inliers;
        }
    }

    if (out_inliers.empty()) {
        return false;
    }
    if (line_ransac_params.use_local_growth) {
        PointCloud3DAdaptor adaptor =
            PointCloud3DAdaptor(frame.GetPointCloud());
        auto kdtree_3d =
            KDTreeWrapper3d::GetKDTree3dPtr(frame.GetPointCloud(), adaptor);
        if (kdtree_3d == nullptr) {
            LOG_ERROR("Failed to build KD-Tree");
            return false;
        }
        std::vector<bool> is_inlier(frame.GetPointCloud().size(), false);
        for (int i : out_inliers)
            is_inlier[i] = true;
        std::queue<int> q;
        for (int i : out_inliers)
            q.push(i);

        while (!q.empty()) {
            int cur = q.front();
            q.pop();

            std::vector<double> query_pt = {
                frame.GetPointCloud()[cur].x(), frame.GetPointCloud()[cur].y(),
                frame.GetPointCloud()[cur].z()};

            std::vector<nanoflann::ResultItem<uint32_t, double>> ret_matches;
            size_t match_num = kdtree_3d->radiusSearch(
                &query_pt[0], line_ransac_params_.knn_radius, ret_matches);
            for (size_t i = 0; i < match_num; i++) {
                int j = ret_matches[i].first;
                if (is_inlier[j])
                    continue;

                if (PointLineDistance(
                        frame.GetPointCloud()[j], best_model.point,
                        best_model.direction) <=
                    line_ransac_params.point_to_line_dist_threshold) {
                    is_inlier[j] = true;
                    q.push(j);
                }
            }
        }
        out_inliers.clear();
        for (int idx = 0; idx < (int)frame.GetPointCloud().size(); ++idx)
            if (is_inlier[idx])
                out_inliers.push_back(idx);
    }

    return true;
}

bool Segmentation::GetOrthogonalPlane(
    const Scan3D &input_scan, Eigen::Vector3d &target_normal,
    std::vector<int> &out_inliers)
{
    plane_ransac_params_.use_orthogonal_check = true;
    plane_ransac_params_.use_parallel_check = false;
    plane_ransac_params_.ground_removal = false;
    plane_ransac_params_.reference_orthogonal_normal = target_normal;
    Plane detected_plane;
    std::vector<int> plane_idx;
    if (!SegmentPlane(
            input_scan, detected_plane, plane_idx, plane_ransac_params_)) {
        LOG_WARN(
            "Orthogonal plane segmentation failed: could not find orthogonal plane");
        return false;
    }
    LOG_INFO(
        "Orthogonal plane segmentation success: {} points found",
        plane_idx.size());
    out_inliers = plane_idx;

    target_normal = detected_plane.n;
    return true;
}

void Segmentation::UpdateParameters()
{
    dist_thresh_ =
        Configurator::GetInstance()
            .GetParamValue(
                "perception", "multi_plane_segmentation", "dist_threshold")
            .convert<double>();
    cos_thresh_ =
        Configurator::GetInstance()
            .GetParamValue(
                "perception", "multi_plane_segmentation", "cos_threshold")
            .convert<double>();
    min_support_ =
        Configurator::GetInstance()
            .GetParamValue(
                "perception", "multi_plane_segmentation", "min_support")
            .convert<int>();
    ransac_iters_ =
        Configurator::GetInstance()
            .GetParamValue(
                "perception", "multi_plane_segmentation", "ransac_iter")
            .convert<int>();
    knn_radius_ =
        Configurator::GetInstance()
            .GetParamValue(
                "perception", "multi_plane_segmentation", "knn_radius")
            .convert<double>();
    use_local_growth_ =
        Configurator::GetInstance()
            .GetParamValue(
                "perception", "multi_plane_segmentation", "use_local_growth")
            .convert<bool>();

    split_gap_ = Configurator::GetInstance()
                     .GetParamValue(
                         "perception", "multi_plane_segmentation", "split_gap")
                     .convert<double>();
    min_support_per_layer_ = Configurator::GetInstance()
                                 .GetParamValue(
                                     "perception", "multi_plane_segmentation",
                                     "min_support_per_layer")
                                 .convert<int>();

    plane_ransac_params_.point_to_plane_dist_threshold =
        Configurator::GetInstance()
            .GetParamValue(
                "perception", "single_plane_segmentation",
                "point_to_plane_dist_threshold")
            .convert<double>();
    plane_ransac_params_.min_support =
        Configurator::GetInstance()
            .GetParamValue(
                "perception", "single_plane_segmentation", "min_support")
            .convert<int>();
    plane_ransac_params_.ransac_iter =
        Configurator::GetInstance()
            .GetParamValue(
                "perception", "single_plane_segmentation", "ransac_iter")
            .convert<int>();
    plane_ransac_params_.parallel_cos_threshold =
        Configurator::GetInstance()
            .GetParamValue(
                "perception", "single_plane_segmentation",
                "parallel_cos_threshold")
            .convert<double>();
    plane_ransac_params_.orthogonal_cos_threshold =
        Configurator::GetInstance()
            .GetParamValue(
                "perception", "single_plane_segmentation",
                "orthogonal_cos_threshold")
            .convert<double>();
    plane_ransac_params_.ground_point_max_height =
        Configurator::GetInstance()
            .GetParamValue(
                "perception", "single_plane_segmentation",
                "ground_point_max_height")
            .convert<double>();
    line_ransac_params_.point_to_line_dist_threshold =
        Configurator::GetInstance()
            .GetParamValue(
                "perception", "line_segmentation",
                "point_to_line_dist_threshold")
            .convert<double>();
    line_ransac_params_.min_support =
        Configurator::GetInstance()
            .GetParamValue("perception", "line_segmentation", "min_support")
            .convert<int>();
    line_ransac_params_.ransac_iter =
        Configurator::GetInstance()
            .GetParamValue("perception", "line_segmentation", "ransac_iter")
            .convert<int>();
    line_ransac_params_.orthogonal_cos_threshold =
        Configurator::GetInstance()
            .GetParamValue(
                "perception", "line_segmentation", "orthogonal_cos_threshold")
            .convert<double>();
    line_ransac_params_.parallel_cos_threshold =
        Configurator::GetInstance()
            .GetParamValue(
                "perception", "line_segmentation", "parallel_cos_threshold")
            .convert<double>();
    line_ransac_params_.use_local_growth =
        Configurator::GetInstance()
            .GetParamValue(
                "perception", "line_segmentation", "use_local_growth")
            .convert<bool>();
    line_ransac_params_.knn_radius =
        Configurator::GetInstance()
            .GetParamValue("perception", "line_segmentation", "knn_radius")
            .convert<double>();
    LOG_INFO(
        "plane ransac prameters {}, {}, {}, {} {}",
        plane_ransac_params_.point_to_plane_dist_threshold,
        plane_ransac_params_.min_support, plane_ransac_params_.ransac_iter,
        plane_ransac_params_.parallel_cos_threshold,
        plane_ransac_params_.orthogonal_cos_threshold);
    LOG_INFO("Segmentation parameters updated");
}
}  // namespace PERCEPTION
}  // namespace ANSWER
