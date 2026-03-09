#include "common/pch.h"

#include "3d_perception/pallet_detector.h"

namespace ANSWER {
namespace PERCEPTION {
PalletDetector::PalletDetector()
{
    UpdateParameters();
}
PalletDetector::~PalletDetector()
{
}
void PalletDetector::DetectPallets(const Scan3D &frame, const bool is_unloading)
{
    Pose3D dummy_pose;

    float scale = 1.0f;

    // step 1 : Preprocessing
    // step 1-1 : Range Filter
    TimeChecker tc;
    Scan3D inliers = frame;
    if (use_range_filter_) {
        tc.MeasureStart();
        LOG_INFO(
            "Before Range Filter PointCloud Size: {}",
            frame.GetPointCloud().size());
        inliers = preprocessor_.FilterByRange(frame, min_range_, max_range_);
        LOG_INFO(
            "After Range Filter PointCloud Size: {}",
            inliers.GetPointCloud().size());
        tc.MeasureEndMicro("range filtering");
    }
    // max_range_);
    if (use_voxel_filter_) {
        tc.MeasureStart();
        LOG_INFO(
            "Before Voxel Filter PointCloud Size: {}",
            inliers.GetPointCloud().size());
        inliers = preprocessor_.FilterByVoxel(inliers, voxel_leaf_size_);
        LOG_INFO(
            "After Voxel Filter PointCloud Size: {}",
            inliers.GetPointCloud().size());
        tc.MeasureEndMicro("voxel filtering");
    }

    if (use_sor_filter_) {
        PointCloud3DAdaptor adaptor =
            PointCloud3DAdaptor(inliers.GetPointCloud());
        kdtree_3d_ =
            KDTreeWrapper3d::GetKDTree3dPtr(inliers.GetPointCloud(), adaptor);
        if (kdtree_3d_ == nullptr) {
            LOG_ERROR("Failed to build KD-Tree");
            return;
        }
        tc.MeasureStart();
        auto result = preprocessor_.RemoveStatisticalOutliers(
            inliers, kdtree_3d_.get(), sor_mean_k_, sor_std_dev_mul_thresh_);
        inliers = result;
        LOG_INFO(
            "After SOR Filter PointCloud Size: {}",
            inliers.GetPointCloud().size());
        tc.MeasureEndMicro("sor filtering");
    }

#if 1
    VisualizerCallback::point_cloud_3d_callback(
        inliers, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose), "inliers", "blue",
        scale);
#endif

    // step 1-2 : Ground Removal
    std::vector<int> ground_removed_inliers;
    if (ground_removal_) {
        tc.MeasureStart();
        LOG_INFO(
            "Before Ground Removal PointCloud Size: {}",
            inliers.GetPointCloud().size());
        if (segmentation_.GroundRemoval(inliers, ground_removed_inliers)) {
            auto [remaining_frame, inlier_frame] =
                PointCloudUtils::SubstractPointCloud(
                    inliers, ground_removed_inliers);
            inliers = remaining_frame;
        }

        LOG_INFO(
            "After Ground Removal PointCloud Size: {}",
            inliers.GetPointCloud().size());
        tc.MeasureEndMicro("ground removal");
    }

    // step 1-3 : ROI Filtering
    if (roi_filter_) {
        tc.MeasureStart();
        inliers = preprocessor_.FilterByROI(
            inliers, horizontal_angle_range_, max_height_, min_height_,
            max_width_);
        tc.MeasureEndMicro("roi filtering");
    }

    // step 2 : Build KD-Tree
    tc.MeasureStart();
    PointCloud3DAdaptor adaptor = PointCloud3DAdaptor(inliers.GetPointCloud());
    kdtree_3d_ =
        KDTreeWrapper3d::GetKDTree3dPtr(inliers.GetPointCloud(), adaptor);
    if (kdtree_3d_ == nullptr) {
        LOG_ERROR("Failed to build KD-Tree");
        return;
    }
    tc.MeasureEndMicro("kd tree building");
    // step 3 : Compute Normals
    tc.MeasureStart();
    PointCloudUtils::ComputeNormals(
        inliers, kdtree_3d_.get(), use_radius_search_, search_radius_);
    tc.MeasureEndMicro("normal computation");

    Scan3D segmented_planes = inliers;

    // step 6 : point cloud clustering
    tc.MeasureStart();
    auto clusters = PointCloudUtils::EuclideanDistanceClustering(
        segmented_planes, kdtree_3d_.get(), cluster_tolerance_,
        min_cluster_size_, max_cluster_size_, use_normals_,
        normal_angle_threshold_);

    tc.MeasureEndMicro("point cloud clustering");
    LOG_INFO(
        "Point cloud clustering finished, {} clusters found", clusters.size());

    // step 7 : calculate centroid for each cluster
    int cluster_id = 0;
    for (const auto &cluster : clusters) {
        Scan3D clustered_pc;
        Eigen::Vector3d cluster_centroid(0.0, 0.0, 0.0);
        for (const auto &idx : cluster) {
            clustered_pc.MutablePointCloud().emplace_back(
                segmented_planes.GetPointCloud()[idx]);
            clustered_pc.MutableNormals().emplace_back(
                segmented_planes.GetNormals()[idx]);
            cluster_centroid += segmented_planes.GetPointCloud()[idx].head<3>();
        }
        cluster_centroid /= static_cast<double>(cluster.size());
        double robot_to_cluster_angle = std::atan2(
            cluster_centroid.y(),
            cluster_centroid.x());

        if (std::fabs(robot_to_cluster_angle * 180 / M_PI) > 10.0) {
            LOG_INFO(
                "Cluster {} ignored due to angle {} > 10 degrees", cluster_id,
                robot_to_cluster_angle * 180 / M_PI);
            cluster_id++;
            continue;
        }
        LOG_INFO(
            "center at {} {} {}", cluster_centroid.x(), cluster_centroid.y(),
            cluster_centroid.z());

        std::vector<int> temp_inliers;
        std::vector<Scan3D> line_inlier_frames;
        while (
            segmentation_.GetGroundParallelLine(clustered_pc, temp_inliers)) {
            auto [remaining_frame, inlier_frame] =
                PointCloudUtils::SubstractPointCloud(
                    clustered_pc, temp_inliers);

            line_inlier_frames.push_back(inlier_frame);
            clustered_pc = remaining_frame;
        }
        std::vector<Point3D> centroid_of_lines;
        if (line_inlier_frames.size() > 1) {
            for (size_t i = 0; i < line_inlier_frames.size(); ++i) {
                auto [centroid, covariance] =
                    PointCloudUtils::ComputeCentroidAndCovariance(
                        line_inlier_frames[i], pallet_unloading_height_min_,
                        pallet_unloading_height_max_);
                if (centroid.hasNaN()) {
                    LOG_WARN("Centroid has NaN values, skipping visualization");
                    continue;
                }

                centroid_of_lines.emplace_back(centroid);
                LOG_INFO(
                    "line {} centroid at {} {} {}", i, centroid.x(),
                    centroid.y(), centroid.z());
            }
            std::pair<Point3D, Point3D> pallet_line_endpoints;
            Scan3D pallet_center;
            if (centroid_of_lines.empty()) {
                LOG_INFO(
                    "No centroids of lines found, skipping pallet detection");
                cluster_id++;
                continue;
            }

            for (size_t i = 0; i < centroid_of_lines.size(); ++i) {
                for (size_t j = i + 1; j < centroid_of_lines.size(); ++j) {
                    double distance =
                        (centroid_of_lines[i] - centroid_of_lines[j]).norm();
                    if (distance > 1.5 && distance < 2.0 &&
                        std::fabs(
                            centroid_of_lines[i].x() -
                            centroid_of_lines[j].x()) < 0.1) {
                        pallet_center.MutablePointCloud().push_back(
                            (centroid_of_lines[i] + centroid_of_lines[j]) /
                            2.0);
                        pallet_line_endpoints = std::make_pair(
                            centroid_of_lines[i], centroid_of_lines[j]);
                        LOG_INFO(
                            "Pallet detected based on line centroids distance {} ",
                            pallet_center.GetPointCloud()
                                .back()
                                .head<2>()
                                .norm());
                        float center_scale = 3.0f;

                        pallet_center.MutablePointCloud().back().z() = 1.5f;

                        VisualizerCallback::point_cloud_3d_callback(
                            pallet_center,
                            ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
                            "pallet_center", "black", center_scale);
                        Scan3D line_endpoints_pc;
                        line_endpoints_pc.MutablePointCloud().insert(
                            line_endpoints_pc.MutablePointCloud().end(),
                            line_inlier_frames[i].GetPointCloud().begin(),
                            line_inlier_frames[i].GetPointCloud().end());

                        line_endpoints_pc.MutablePointCloud().insert(
                            line_endpoints_pc.MutablePointCloud().end(),
                            line_inlier_frames[j].GetPointCloud().begin(),
                            line_inlier_frames[j].GetPointCloud().end());
                        std::string id = "final_lines";
                        VisualizerCallback::point_cloud_3d_callback(
                            line_endpoints_pc,
                            ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose), id,
                            "red", scale);
                    }
                }
            }
        }
        cluster_id++;
    }
}
void PalletDetector::UpdateParameters()
{
    LOG_INFO("PalletDetector parameters updated");
    min_range_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "preprocessor", "min_range")
            .convert<float>();
    max_range_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "preprocessor", "max_range")
            .convert<float>();

    min_height_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "preprocessor", "min_height")
            .convert<float>();

    voxel_leaf_size_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "preprocessor", "voxel_leaf_size")
            .convert<float>();
    ground_removal_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "preprocessor", "ground_removal")
            .convert<bool>();
    roi_filter_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "preprocessor", "roi_filter")
            .convert<bool>();
    horizontal_angle_range_ =
        Configurator::GetInstance()
            .GetParamValue(
                "pallet_detector", "preprocessor", "horizontal_angle_range")
            .convert<float>();
    max_height_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "preprocessor", "max_height")
            .convert<float>();
    max_width_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "preprocessor", "max_width")
            .convert<float>();
    use_radius_search_ =
        Configurator::GetInstance()
            .GetParamValue(
                "pallet_detector", "preprocessor", "use_radius_search")
            .convert<bool>();
    search_radius_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "preprocessor", "search_radius")
            .convert<float>();

    cluster_tolerance_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "clustering", "cluster_tolerance")
            .convert<float>();
    min_cluster_size_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "clustering", "min_cluster_size")
            .convert<int>();
    max_cluster_size_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "clustering", "max_cluster_size")
            .convert<int>();
    use_normals_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "clustering", "use_normals")
            .convert<bool>();
    normal_angle_threshold_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "clustering", "angle_threshold")
            .convert<float>();

    use_voxel_filter_ =
        Configurator::GetInstance()
            .GetParamValue(
                "pallet_detector", "preprocessor", "use_voxel_filter")
            .convert<bool>();
    use_range_filter_ =
        Configurator::GetInstance()
            .GetParamValue(
                "pallet_detector", "preprocessor", "use_range_filter")
            .convert<bool>();
    pallet_unloading_height_min_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "unloading", "height_min")
            .convert<double>();
    pallet_unloading_height_max_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "unloading", "height_max")
            .convert<double>();
    use_sor_filter_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "preprocessor", "use_sor_filter")
            .convert<bool>();
    sor_mean_k_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "preprocessor", "sor_mean_k")
            .convert<int>();
    sor_std_dev_mul_thresh_ =
        Configurator::GetInstance()
            .GetParamValue(
                "pallet_detector", "preprocessor", "sor_std_dev_mul_thresh")
            .convert<double>();
}
}  // namespace PERCEPTION
}  // namespace ANSWER
