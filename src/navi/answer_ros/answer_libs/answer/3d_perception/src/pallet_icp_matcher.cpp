#include "common/pch.h"

#include "3d_perception/pallet_icp_matcher.h"

#include <chrono>

namespace ANSWER {
namespace PERCEPTION {
PalletICPMatcher::PalletICPMatcher()
    : b_detecting_(false)
{
    UpdateParameters();
    icp3d_ = std::make_unique<MATCHER::icp3d>();

    std::string model_pcd_file =
        Path::GetInstance().GetROSInstallPath() + "/template/";

    for (auto i = 0; i < tmpl_lists_->size(); ++i) {
        auto tmpl = tmpl_lists_->get(i).extract<Poco::JSON::Object::Ptr>();

        std::string tmpl_name = tmpl->getValue<std::string>("file_name");
        LOG_INFO("tmpl name : {}", tmpl_name);

        std::string tmpl_path = model_pcd_file;
        LOG_INFO("tmpl path : {}", tmpl_path);

        bool center_template = true;
        if (tmpl->has("center_template")) {
            center_template = tmpl->getValue<bool>("center_template");
        }

        Scan3D model_scan;
        if (LoadTemplateFile(tmpl_path, tmpl_name, model_scan, center_template)) {
            double width = tmpl->getValue<double>("width");
            double height = tmpl->getValue<double>("height");

            model_widths_.emplace_back(width);
            model_heights_.emplace_back(height);

            bool ground_pallet = false;
            if (tmpl->has("ground_pallet")) {
                ground_pallet = tmpl->getValue<bool>("ground_pallet");
            }
            model_ground_pallet_.emplace_back(ground_pallet);

            bool use_pillar = false;
            if (tmpl->has("use_pillar_detection")) {
                use_pillar = tmpl->getValue<bool>("use_pillar_detection");
            }
            model_use_pillar_detection_.emplace_back(use_pillar);

            bool tmpl_ground_removal = false;
            if (tmpl->has("ground_removal")) {
                tmpl_ground_removal = tmpl->getValue<bool>("ground_removal");
            }
            model_ground_removal_.emplace_back(tmpl_ground_removal);

            model_center_template_.emplace_back(center_template);
            model_scans_.emplace_back(model_scan);
        }
    }

    delta_transform_ = Pose3D();
    robot_to_pallet_center_ = Pose3D();
    odom_delta_ = Pose3D();
}
PalletICPMatcher::~PalletICPMatcher()
{
    b_detecting_ = false;
    if (pallet_icp_thread_.joinable()) {
        pallet_icp_thread_.join();
    }
}

void PalletICPMatcher::DetectThread()
{
}

bool PalletICPMatcher::LoadTemplateFile(
    const std::string &model_path, const std::string &file_name, Scan3D &scan,
    bool center_template)
{
    TimeChecker tc;

    // Clear output vector
    scan.MutablePointCloud().clear();
    scan.MutableNormals().clear();

    const std::string &file_path = model_path + file_name;

    // Load PCD file using PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    if (file_name.find(".pcd") != std::string::npos) {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *pcl_cloud) == -1) {
            LOG_ERROR("Failed to load PCD file: {}", file_path);
            return false;
        }
    }
    else {
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(file_path, *pcl_cloud) == -1) {
            LOG_ERROR("Failed to load PLY file: {}", file_path);
            return false;
        }
    }

    LOG_INFO(
        "Successfully loaded PCD file: {}, {} points", file_path,
        pcl_cloud->points.size());

    // Voxel filtering using PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    if (use_voxel_filter_) {
        tc.MeasureStart();
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(pcl_cloud);
        voxel_filter.setLeafSize(
            voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel_filter.filter(*filtered_cloud);
        tc.MeasureEndMicro("voxel filtering (PCL)");
        LOG_INFO(
            "Voxel filtered: {} -> {} points", pcl_cloud->points.size(),
            filtered_cloud->points.size());
    }
    else {
        filtered_cloud = pcl_cloud;
    }

    if (use_template_range_filter_) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr range_filtered_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);

        tc.MeasureStart();
        for (const auto &point : filtered_cloud->points) {
            if ((point.x) >= 0.6f) {
                range_filtered_cloud->points.push_back(point);
            }
        }
        range_filtered_cloud->width = range_filtered_cloud->points.size();
        range_filtered_cloud->height = 1;
        range_filtered_cloud->is_dense = true;
        tc.MeasureEndMicro("range filtering (PCL)");

        LOG_INFO(
            "Range filtered (x >= 0.6m): {} -> {} points",
            filtered_cloud->points.size(), range_filtered_cloud->points.size());

        filtered_cloud = range_filtered_cloud;
    }

    // Convert to Scan3D format (single conversion)
    scan.MutablePointCloud().reserve(filtered_cloud->points.size());

    for (size_t i = 0; i < filtered_cloud->points.size(); ++i) {
        const auto &point = filtered_cloud->points[i];

        // Skip invalid points
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            continue;
        }

        scan.MutablePointCloud().emplace_back(
            Eigen::Vector3d(point.x, point.y, point.z));
    }

    // 템플릿을 원점 중심으로 정규화 (KD-tree/normal 계산 전에 수행)
    // PLY 파일이 원점 중심이 아닌 경우 initial_guess 위치에 템플릿이 제대로 배치되지 않아
    // 대응점을 찾지 못함 (overlap 0%)
    if (center_template && !scan.GetPointCloud().empty()) {
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        for (const auto &p : scan.GetPointCloud())
            center += p;
        center /= static_cast<double>(scan.GetPointCloud().size());
        LOG_INFO(
            "Template center (before centering): x={:.3f}, y={:.3f}, z={:.3f} [m]",
            center.x(), center.y(), center.z());

        if (center.norm() > 0.01) {
            LOG_INFO(
                "Centering template: shifting by ({:.3f}, {:.3f}, {:.3f})",
                -center.x(), -center.y(), -center.z());
            for (auto &p : scan.MutablePointCloud()) {
                p -= center;
            }
        }
    }
    else if (!center_template) {
        LOG_INFO("Template centering SKIPPED (center_template=false) for {}",
                 file_name);
    }

    tc.MeasureStart();
    PointCloud3DAdaptor adaptor = PointCloud3DAdaptor(scan.GetPointCloud());
    auto kdtree_3d =
        KDTreeWrapper3d::GetKDTree3dPtr(scan.GetPointCloud(), adaptor);
    if (kdtree_3d == nullptr) {
        LOG_ERROR("Failed to build KD-Tree");
        return false;
    }
    tc.MeasureEndMicro("kd tree building");

    // Compute Normals (센터링된 포인트 기준으로 계산)
    tc.MeasureStart();
    PointCloudUtils::ComputeNormals(
        scan, kdtree_3d.get(), use_radius_search_, search_radius_,
        view_point_arrange_);
    tc.MeasureEndMicro("normal computation");

    LOG_INFO(
        "Template loaded: {} points with normals (centered at origin)",
        scan.GetPointCloud().size());

    return true;
}

Scan3D PalletICPMatcher::TransformTemplate(
    const int &pallet_type, const Eigen::Matrix3d &rotation,
    const Eigen::Vector3d &translation) const
{
    Scan3D transformed_scan;
    const auto &template_points = model_scans_[pallet_type].GetPointCloud();
    const auto &template_normals = model_scans_[pallet_type].GetNormals();

    transformed_scan.MutablePointCloud().reserve(template_points.size());
    transformed_scan.MutableNormals().reserve(template_normals.size());

    // Transform points: p' = R * p + t
    for (const auto &point : template_points) {
        Eigen::Vector3d transformed_point =
            rotation * point.head<3>() + translation;
        transformed_scan.MutablePointCloud().emplace_back(transformed_point);
    }

    // Transform normals: n' = R * n (normals only need rotation)
    for (const auto &normal : template_normals) {
        Eigen::Vector3d transformed_normal = rotation * normal.head<3>();
        // Normalize to ensure unit length after rotation
        if (transformed_normal.norm() > 1e-6) {
            transformed_normal.normalize();
        }
        transformed_scan.MutableNormals().emplace_back(transformed_normal);
    }

    // LOG_INFO(
    //     "Transformed template: {} points, {} normals",
    //     transformed_scan.GetPointCloud().size(),
    //     transformed_scan.GetNormals().size());

    return transformed_scan;
}

Scan3D PalletICPMatcher::TransformTemplate(
    const int &pallet_type, const Eigen::Matrix4d &transformation) const
{
    // Extract rotation and translation from 4x4 transformation matrix
    Eigen::Matrix3d rotation = transformation.block<3, 3>(0, 0);
    Eigen::Vector3d translation = transformation.block<3, 1>(0, 3);

    return TransformTemplate(pallet_type, rotation, translation);
}

void PalletICPMatcher::FindCorrespondences(
    const Scan3D &source, const Scan3D &target, KDTree3d *target_kdtree, int K,
    double max_distance, std::vector<int> &source_indices,
    std::vector<int> &target_indices)
{
    source_indices.clear();
    target_indices.clear();

    const auto &source_points = source.GetPointCloud();
    const double max_dist_sq = max_distance * max_distance;

    for (size_t i = 0; i < source_points.size(); ++i) {
        const auto &point = source_points[i].head<3>();

        // Find nearest neighbor in target
        std::vector<size_t> ret_indices(K);
        std::vector<double> out_dists_sqr(K);

        nanoflann::KNNResultSet<double> result_set(K);
        result_set.init(&ret_indices[0], &out_dists_sqr[0]);

        double query_pt[3] = {point.x(), point.y(), point.z()};
        target_kdtree->findNeighbors(
            result_set, query_pt, nanoflann::SearchParameters());

        // Check if correspondence is valid (within max distance)
        if (out_dists_sqr[0] < max_dist_sq) {
            source_indices.push_back(static_cast<int>(i));
            target_indices.push_back(static_cast<int>(ret_indices[0]));
        }
    }
}

Eigen::Matrix4d PalletICPMatcher::ComputeTransformation(
    const Scan3D &source, const Scan3D &target,
    const std::vector<int> &source_indices,
    const std::vector<int> &target_indices)
{
    const auto &source_points = source.GetPointCloud();
    const auto &target_points = target.GetPointCloud();
    const auto &target_normals = target.GetNormals();

    Eigen::MatrixXd A(source_indices.size(), 6);
    Eigen::VectorXd b(source_indices.size());

    for (size_t i = 0; i < source_indices.size(); ++i) {
        const Eigen::Vector3d &p_source = source_points[source_indices[i]];
        const Eigen::Vector3d &p_target = target_points[target_indices[i]];
        const Eigen::Vector3d &n_target = target_normals[target_indices[i]];

        if (n_target.norm() < 1e-6) {
            A.row(i).setZero();
            b(i) = 0.0;
            continue;
        }

        Eigen::Vector3d diff = p_source - p_target;
        Eigen::Vector3d cross = p_source.cross(n_target);

        A.row(i) << cross.x(), cross.y(), cross.z(), n_target.x(), n_target.y(),
            n_target.z();
        b(i) = -diff.dot(n_target);
    }

    Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

    Eigen::Vector3d omega(x(0), x(1), x(2));
    Eigen::Vector3d trans(x(3), x(4), x(5));

    double theta = omega.norm();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    if (theta > 1e-6) {
        Eigen::Vector3d axis = omega / theta;
        R = Eigen::AngleAxisd(theta, axis).toRotationMatrix();
    }

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = R;
    transformation.block<3, 1>(0, 3) = trans;

    return transformation;
}

PalletICPMatcher::ICPResult PalletICPMatcher::AlignICP(
    const int &pallet_type, const Scan3D &target_cloud,
    const Eigen::Matrix4d &initial_guess, int max_iterations,
    double transformation_epsilon, double max_correspondence_distance)
{
    ICPResult result;
    result.transformation = Eigen::Matrix4d::Identity();
    result.converged = false;
    result.num_iterations = 0;
    result.fitness_score = std::numeric_limits<double>::max();

    LOG_INFO("Starting Point-to-Plane ICP alignment (PCL)...");
    LOG_INFO(
        "  Max iterations: {}, transformation epsilon: {}, max correspondence "
        "distance: {}",
        max_iterations, transformation_epsilon, max_correspondence_distance);

    TimeChecker tc;

    tc.MeasureStart();

    // Source: pallet template (with normals)
    pcl::PointCloud<pcl::PointNormal>::Ptr source_pcl(
        new pcl::PointCloud<pcl::PointNormal>);
    const auto &template_points = model_scans_[pallet_type].GetPointCloud();
    const auto &template_normals = model_scans_[pallet_type].GetNormals();
    source_pcl->points.reserve(template_points.size());

    for (size_t i = 0; i < template_points.size(); ++i) {
        const auto &point = template_points[i];
        const auto &normal = template_normals[i];

        pcl::PointNormal pcl_point;
        pcl_point.x = point.x();
        pcl_point.y = point.y();
        pcl_point.z = point.z();
        pcl_point.normal_x = normal.x();
        pcl_point.normal_y = normal.y();
        pcl_point.normal_z = normal.z();
        source_pcl->points.emplace_back(pcl_point);
    }
    source_pcl->width = source_pcl->points.size();
    source_pcl->height = 1;
    source_pcl->is_dense = true;

    // Target: sensor data
    pcl::PointCloud<pcl::PointNormal>::Ptr target_pcl(
        new pcl::PointCloud<pcl::PointNormal>);
    const auto &target_points = target_cloud.GetPointCloud();

    if (target_cloud.GetNormals().size() == target_points.size()) {
        const auto &target_normals = target_cloud.GetNormals();
        target_pcl->points.reserve(target_points.size());

        for (size_t i = 0; i < target_points.size(); ++i) {
            const auto &point = target_points[i];
            const auto &normal = target_normals[i];

            pcl::PointNormal pcl_point;
            pcl_point.x = point.x();
            pcl_point.y = point.y();
            pcl_point.z = point.z();
            pcl_point.normal_x = normal.x();
            pcl_point.normal_y = normal.y();
            pcl_point.normal_z = normal.z();
            target_pcl->points.emplace_back(pcl_point);
        }
        target_pcl->width = target_pcl->points.size();
        target_pcl->height = 1;
        target_pcl->is_dense = true;
    }
    else {
        LOG_ERROR(
            "Target normals size ({}) does not match points size ({}), "
            "cannot run Point-to-Plane ICP",
            target_cloud.GetNormals().size(), target_points.size());
        return result;
    }

    tc.MeasureEndMicro("PCL point cloud with normals conversion");

    LOG_INFO(
        "Converted to PCL: source = {} points, target = {} points",
        source_pcl->points.size(), target_pcl->points.size());

    tc.MeasureStart();
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>
        icp;

    icp.setInputSource(source_pcl);
    icp.setInputTarget(target_pcl);

    icp.setMaximumIterations(max_iterations);
    icp.setTransformationEpsilon(transformation_epsilon);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance);

    tc.MeasureEndMicro("Point-to-Plane ICP setup");

    tc.MeasureStart();
    pcl::PointCloud<pcl::PointNormal>::Ptr aligned_source(
        new pcl::PointCloud<pcl::PointNormal>);

    Eigen::Matrix4f initial_guess_f = initial_guess.cast<float>();

    icp.align(*aligned_source, initial_guess_f);
    tc.MeasureEndMicro("Point-to-Plane ICP alignment");

    result.converged = icp.hasConverged();
    result.fitness_score = icp.getFitnessScore();
    result.num_iterations = icp.nr_iterations_;

    Eigen::Matrix4f final_transform_f = icp.getFinalTransformation();
    result.transformation = final_transform_f.cast<double>();

    if (result.converged) {
        LOG_INFO("Point-to-Plane ICP converged successfully");
    }
    else {
        LOG_WARN("Point-to-Plane ICP did not converge");
    }

    LOG_INFO(
        "Point-to-Plane ICP finished: converged = {}, iterations = {}, fitness score = {:.6f}",
        result.converged, result.num_iterations, result.fitness_score);

    return result;
}

PalletICPMatcher::ICPResult PalletICPMatcher::AlignGICP(
    const int &pallet_type, const Scan3D &target_cloud,
    const Eigen::Matrix4d &initial_guess, int max_iterations,
    double transformation_epsilon, double max_correspondence_distance)
{
    ICPResult result;
    result.transformation = Eigen::Matrix4d::Identity();
    result.converged = false;
    result.num_iterations = 0;
    result.fitness_score = std::numeric_limits<double>::max();

    LOG_INFO("Starting GICP alignment...");
    LOG_INFO(
        "  Max iterations: {}, transformation epsilon: {}, max correspondence "
        "distance: {}",
        max_iterations, transformation_epsilon, max_correspondence_distance);

    TimeChecker tc;

    tc.MeasureStart();

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_pcl(
        new pcl::PointCloud<pcl::PointXYZ>);
    const auto &template_points = model_scans_[pallet_type].GetPointCloud();
    source_pcl->points.reserve(template_points.size());

    for (const auto &point : template_points) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x();
        pcl_point.y = point.y();
        pcl_point.z = point.z();
        source_pcl->points.emplace_back(pcl_point);
    }
    source_pcl->width = source_pcl->points.size();
    source_pcl->height = 1;
    source_pcl->is_dense = true;

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_pcl(
        new pcl::PointCloud<pcl::PointXYZ>);
    const auto &target_points = target_cloud.GetPointCloud();
    target_pcl->points.reserve(target_points.size());

    for (const auto &point : target_points) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x();
        pcl_point.y = point.y();
        pcl_point.z = point.z();
        target_pcl->points.emplace_back(pcl_point);
    }
    target_pcl->width = target_pcl->points.size();
    target_pcl->height = 1;
    target_pcl->is_dense = true;

    tc.MeasureEndMicro("PCL point cloud conversion");

    LOG_INFO(
        "Converted to PCL: source = {} points, target = {} points",
        source_pcl->points.size(), target_pcl->points.size());

    tc.MeasureStart();
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

    gicp.setInputSource(source_pcl);
    gicp.setInputTarget(target_pcl);

    gicp.setMaximumIterations(max_iterations);
    gicp.setTransformationEpsilon(transformation_epsilon);
    gicp.setMaxCorrespondenceDistance(max_correspondence_distance);
    gicp.setRotationEpsilon(transformation_epsilon);

    tc.MeasureEndMicro("GICP setup");

    tc.MeasureStart();
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_source(
        new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Matrix4f initial_guess_f = initial_guess.cast<float>();

    gicp.align(*aligned_source, initial_guess_f);
    tc.MeasureEndMicro("GICP alignment");

    result.converged = gicp.hasConverged();
    result.fitness_score = gicp.getFitnessScore();
    result.num_iterations = gicp.nr_iterations_;

    Eigen::Matrix4f final_transform_f = gicp.getFinalTransformation();
    result.transformation = final_transform_f.cast<double>();

    if (result.converged) {
        LOG_INFO("GICP converged successfully");
    }
    else {
        LOG_WARN("GICP did not converge");
    }

    LOG_INFO(
        "GICP finished: converged = {}, iterations = {}, fitness score = {:.6f}",
        result.converged, result.num_iterations, result.fitness_score);

    return result;
}

void PalletICPMatcher::DetectPalletsThread(
    const Scan3D &frame, const int &pallet_type, const Pose3D &guess,
    const Pose3D &odom_delta, const Pose3D &cur_robot_pose,
    const long int &time_stamp)
{
    bool b_detecting = false;
    {
        std::lock_guard<std::mutex> lock(do_detecting_mtx_);
        b_detecting = b_detecting_;
    }
    LOG_DEBUG("DetectPalletsThread: b_detecting={}", b_detecting);
    if (b_detecting == false) {
        if (pallet_icp_thread_.joinable()) {
            pallet_icp_thread_.join();
        }
        {
            std::lock_guard<std::mutex> lock(delta_mtx_);
            odom_delta_ = odom_delta_ * odom_delta;
            robot_to_pallet_center_ = robot_to_pallet_center_ * odom_delta_;
        }
        Pose3D launch_pose;
        {
            std::lock_guard<std::mutex> lock(delta_mtx_);
            launch_pose = robot_to_pallet_center_;
            odom_delta_ = Pose3D();
        }
        LOG_INFO("DetectPalletsThread: launching (pallet_type={})", pallet_type);
        pallet_icp_thread_ = std::thread(
            &PalletICPMatcher::DetectPallets, this, frame, pallet_type,
            launch_pose, cur_robot_pose, time_stamp);
    }
    else {
        std::lock_guard<std::mutex> lock(delta_mtx_);
        odom_delta_ = odom_delta_ * odom_delta;
    }
}

void PalletICPMatcher::DetectPallets(
    const Scan3D &frame, const int &pallet_type, const Pose3D &guess,
    const Pose3D &cur_robot_pose, const long int &time_stamp)
{
    const auto &template_points = model_scans_[pallet_type].GetPointCloud();

    if (template_points.empty()) {
        LOG_ERROR("Pallet template point cloud is empty!");
        return;
    }

    if (frame.GetPointCloud().empty()) {
        LOG_ERROR("Input sensor point cloud is empty!");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(do_detecting_mtx_);
        b_detecting_ = true;
    }

    if (pallet_type < static_cast<int>(model_use_pillar_detection_.size()) &&
        model_use_pillar_detection_[pallet_type]) {
        if (DetectPalletsByPillars(
                frame, pallet_type, guess, cur_robot_pose, time_stamp)) {
            std::lock_guard<std::mutex> lock(do_detecting_mtx_);
            b_detecting_ = false;
            return;
        }
        LOG_WARN("[PillarDetect] pillar detection failed, falling back to ICP");
    }

    Pose3D dummy_pose;
    Pose3D result;

    using Clock = std::chrono::steady_clock;
    const auto t_cycle_start = Clock::now();

    TimeChecker tc;
    Scan3D inliers = frame;
    if (use_range_filter_) {
        const auto t0 = Clock::now();
        inliers = preprocessor_.FilterByRange(frame, min_range_, max_range_);
        const auto ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        LOG_INFO("[DetectPallets timing] range_filter: {:.1f} ms", ms);
    }

    if (use_voxel_filter_) {
        const auto t0 = Clock::now();
        inliers = preprocessor_.FilterByVoxel(inliers, 0.02f);
        const auto ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        LOG_INFO("[DetectPallets timing] voxel_downsample: {:.1f} ms", ms);
    }

    // Per-template ground removal: type1/type2는 false, ground pallet(jong/plastic/wood)는 true
    bool do_ground_removal = false;
    if (pallet_type >= 0 &&
        pallet_type < static_cast<int>(model_ground_removal_.size())) {
        do_ground_removal = model_ground_removal_[pallet_type];
    }
    if (do_ground_removal) {
        const auto t0 = Clock::now();
        std::vector<int> ground_inliers;
        if (segmentation_.GroundRemoval(inliers, ground_inliers)) {
            auto [remaining_frame, inlier_frame] =
                PointCloudUtils::SubstractPointCloud(inliers, ground_inliers);
            const size_t n_remaining = remaining_frame.GetPointCloud().size();
            const size_t n_template = template_points.size();
            if (n_remaining >= n_template) {
                inliers = remaining_frame;
            }
            else {
                LOG_WARN(
                    "Ground removal too aggressive: {} pts remain < template "
                    "{} pts, skipping ground removal",
                    n_remaining, n_template);
            }
        }
        const auto ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        LOG_INFO("[DetectPallets timing] ground_removal: {:.1f} ms", ms);
        LOG_INFO("After ground removal PointCloud Size: {}",
                 inliers.GetPointCloud().size());
    }
    if (use_sor_filter_) {
        const auto t0_kdt = Clock::now();
        PointCloud3DAdaptor adaptor =
            PointCloud3DAdaptor(inliers.GetPointCloud());
        kdtree_3d_ =
            KDTreeWrapper3d::GetKDTree3dPtr(inliers.GetPointCloud(), adaptor);
        if (kdtree_3d_ == nullptr) {
            LOG_ERROR("Failed to build KD-Tree");
            return;
        }
        const auto ms_kdt = std::chrono::duration<double, std::milli>(Clock::now() - t0_kdt).count();
        LOG_INFO("[DetectPallets timing] kdtree_for_sor: {:.1f} ms", ms_kdt);
        const auto t0_sor = Clock::now();
        auto result_sor = preprocessor_.RemoveStatisticalOutliers(
            inliers, kdtree_3d_.get(), sor_mean_k_, sor_std_dev_mul_thresh_);
        inliers = result_sor;
        const auto ms_sor = std::chrono::duration<double, std::milli>(Clock::now() - t0_sor).count();
        LOG_INFO("[DetectPallets timing] sor_filter: {:.1f} ms", ms_sor);
        LOG_INFO(
            "After SOR Filter PointCloud Size: {}",
            inliers.GetPointCloud().size());
    }

    Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();
    initial_guess.block<3, 3>(0, 0) = guess.rotationMatrix();
    initial_guess.block<3, 1>(0, 3) = guess.translation();

    // RViz 디버그 발행은 ICP 완료 후에만 수행 (ACCEPTED/REJECTED 브랜치에서 sensor_matched, template_initial, template_aligned 발행)
    // template_initial은 perception_ros1에서 매 프레임 이미 발행됨

    // NOTE: adaptor는 kdtree_3d_보다 오래 살아야 함 (nanoflann이 참조를 보유)
    PointCloud3DAdaptor adaptor_for_icp = PointCloud3DAdaptor(inliers.GetPointCloud());
    {
        const auto t0 = Clock::now();
        kdtree_3d_ =
            KDTreeWrapper3d::GetKDTree3dPtr(inliers.GetPointCloud(), adaptor_for_icp);
        if (kdtree_3d_ == nullptr) {
            LOG_ERROR("Failed to build KD-Tree");
            return;
        }
        const auto ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        LOG_INFO("[DetectPallets timing] kdtree_for_normals: {:.1f} ms", ms);
    }

    {
        const auto t0 = Clock::now();
        PointCloudUtils::ComputeNormals(
            inliers, kdtree_3d_.get(), use_radius_search_, search_radius_,
            view_point_arrange_);
        const auto ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        LOG_INFO("[DetectPallets timing] normal_computation: {:.1f} ms", ms);
    }

    LOG_INFO(
        "Preprocessed sensor data: {} points with normals",
        inliers.GetPointCloud().size());

    // 전처리 후 빈 포인트 클라우드 또는 normals 불일치 방어
    if (inliers.GetPointCloud().empty()) {
        LOG_ERROR("Preprocessed sensor data is empty, skipping ICP");
        std::lock_guard<std::mutex> lock(do_detecting_mtx_);
        b_detecting_ = false;
        return;
    }
    if (inliers.GetNormals().size() != inliers.GetPointCloud().size()) {
        LOG_ERROR(
            "Sensor normals size ({}) != points size ({}), skipping ICP",
            inliers.GetNormals().size(), inliers.GetPointCloud().size());
        std::lock_guard<std::mutex> lock(do_detecting_mtx_);
        b_detecting_ = false;
        return;
    }
    if (pallet_type < 0 || pallet_type >= static_cast<int>(model_scans_.size())) {
        LOG_ERROR("Invalid pallet_type: {} (model_scans size: {})",
                  pallet_type, model_scans_.size());
        std::lock_guard<std::mutex> lock(do_detecting_mtx_);
        b_detecting_ = false;
        return;
    }
    if (model_scans_[pallet_type].GetNormals().size() !=
        model_scans_[pallet_type].GetPointCloud().size()) {
        LOG_ERROR(
            "Template normals size ({}) != points size ({}), skipping ICP",
            model_scans_[pallet_type].GetNormals().size(),
            model_scans_[pallet_type].GetPointCloud().size());
        std::lock_guard<std::mutex> lock(do_detecting_mtx_);
        b_detecting_ = false;
        return;
    }

    static TimeChecker time_recorder;
    if (use_pcl_icp_) {
        const auto t_icp_start = Clock::now();
        time_recorder.RecordStart("pcl_icp");
        auto icp_result = AlignICP(
            pallet_type, inliers, initial_guess,
            max_iteration_, transformation_epsilon_,
            max_correspondence_distance_);
        time_recorder.RecordEnd("pcl_icp");
        const auto ms_icp = std::chrono::duration<double, std::milli>(Clock::now() - t_icp_start).count();
        LOG_INFO("[DetectPallets timing] icp_alignment: {:.1f} ms", ms_icp);
        if (icp_result.converged && icp_result.fitness_score < 0.1) {
            LOG_INFO("PCL ICP ACCEPTED: fitness={:.6f} < 0.1", icp_result.fitness_score);
            LOG_INFO("  Fitness score: {:.6f}", icp_result.fitness_score);
            LOG_INFO("  Iterations: {}", icp_result.num_iterations);

            Eigen::Matrix3d R = icp_result.transformation.block<3, 3>(0, 0);
            Eigen::Vector3d t = icp_result.transformation.block<3, 1>(0, 3);
            Sophus::SO3d so3 = Sophus::SO3d::fitToSO3(R);
            result = Pose3D(so3, t);

            // Constrain z for ground pallets
            if (pallet_type < static_cast<int>(model_ground_pallet_.size()) &&
                model_ground_pallet_[pallet_type]) {
                LOG_INFO("Ground pallet z constrained: {:.3f} -> {:.3f}",
                         result.translation().z(), guess.translation().z());
                result.translation().z() = guess.translation().z();
            }

            Eigen::Vector3d euler = R.eulerAngles(2, 1, 0);
            double yaw = euler(0) * 180.0 / M_PI;

            LOG_INFO(
                "  Position: x={:.3f}, y={:.3f}, z={:.3f}", t.x(), t.y(),
                t.z());
            LOG_INFO("  Yaw angle: {:.2f} degrees", yaw);

            Scan3D aligned_template =
                TransformTemplate(pallet_type, icp_result.transformation);
            float scale = 1.0f;

            VisualizerCallback::point_cloud_3d_callback(
                inliers, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
                "sensor_data", "blue", scale);

            VisualizerCallback::point_cloud_3d_callback(
                aligned_template, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
                "aligned_pallet", "green", scale);

            Scan3D initial_template =
                TransformTemplate(pallet_type, initial_guess);
            VisualizerCallback::point_cloud_3d_callback(
                initial_template, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
                "initial_guess", "red", scale);

            PerceptionResultCallback::pallet_pose_callback(
                result, cur_robot_pose, time_stamp);
            {
                std::lock_guard<std::mutex> lock(delta_mtx_);
                delta_transform_ = guess.inverse() * result;
                robot_to_pallet_center_ = result;
            }
            LOG_INFO(
                "  Delta Position: x={:.3f}, y={:.3f}, z={:.3f}",
                delta_transform_.translation().x(),
                delta_transform_.translation().y(),
                delta_transform_.translation().z());
        }
        else {
            LOG_WARN("PCL ICP REJECTED: converged={}, fitness={:.6f} (threshold=0.1)",
                     icp_result.converged, icp_result.fitness_score);
            LOG_WARN("  Converged: {}", icp_result.converged);
            LOG_WARN("  Fitness score: {:.6f}", icp_result.fitness_score);
            LOG_WARN("  Iterations: {}", icp_result.num_iterations);

            Scan3D aligned_template =
                TransformTemplate(pallet_type, icp_result.transformation);
            float scale = 1.0f;

            VisualizerCallback::point_cloud_3d_callback(
                inliers, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
                "sensor_data", "blue", scale);

            VisualizerCallback::point_cloud_3d_callback(
                aligned_template, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
                "aligned_pallet", "green", scale);

            Scan3D initial_template =
                TransformTemplate(pallet_type, initial_guess);
            VisualizerCallback::point_cloud_3d_callback(
                initial_template, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
                "initial_guess", "red", scale);
        }
    }
    else {
        const size_t n_tmpl = model_scans_[pallet_type].GetPointCloud().size();
        const size_t n_sensor = inliers.GetPointCloud().size();
        LOG_INFO(
            "[ICP debug] before align: template_pts={}, sensor_pts={}, "
            "initial_guess=({:.3f}, {:.3f}, {:.3f}), max_corr_dist={:.3f} [m]",
            n_tmpl, n_sensor, guess.translation().x(), guess.translation().y(),
            guess.translation().z(),
            static_cast<double>(max_correspondence_distance_));

        // 초기 추정 위치에서 템플릿-센서 겹침 정도 진단 (max_correspondence_distance 이내 대응점 개수)
        Scan3D tmpl_at_guess = TransformTemplate(pallet_type, guess.matrix());
        std::vector<int> src_idx, tgt_idx;
        FindCorrespondences(tmpl_at_guess, inliers, kdtree_3d_.get(), 1,
                           static_cast<double>(max_correspondence_distance_),
                           src_idx, tgt_idx);
        const int n_initial_corr = static_cast<int>(src_idx.size());
        const double overlap_ratio =
            (n_tmpl > 0)
                ? (100.0 * static_cast<double>(n_initial_corr) /
                   static_cast<double>(n_tmpl))
                : 0.0;
        LOG_INFO(
            "[ICP debug] initial overlap: {} / {} template points have a "
            "sensor point within {:.3f}m ({:.1f}%)",
            n_initial_corr, n_tmpl,
            static_cast<double>(max_correspondence_distance_), overlap_ratio);

        LOG_INFO(
            "initial guess: {}", initial_guess.block<3, 1>(0, 3).transpose());
        const auto t_icp_start = Clock::now();
        time_recorder.RecordStart("my_icp");
        auto res = icp3d_->AlignPointToPlane(
            model_scans_[pallet_type], inliers, kdtree_3d_.get(), guess);

        time_recorder.RecordEnd("my_icp");
        const auto ms_icp = std::chrono::duration<double, std::milli>(Clock::now() - t_icp_start).count();
        LOG_INFO("[DetectPallets timing] icp_alignment: {:.1f} ms", ms_icp);
        LOG_INFO("result {}", res.GetCorrection().translation().transpose());
        LOG_INFO(
            "rmse {}, confidence {} iter {}", res.GetRMSE(),
            res.GetMatchRatio(), res.GetIteration());

        if (res.GetRMSE() < rmse_threshold_ &&
            res.GetMatchRatio() > confidence_threshold_) {
            LOG_INFO("Custom ICP ACCEPTED: rmse={:.4f} < {:.4f}, confidence={:.1f} > {:.1f}",
                     res.GetRMSE(), rmse_threshold_, res.GetMatchRatio(), confidence_threshold_);
            result = res.GetCorrection();

            // Constrain z for ground pallets
            if (pallet_type < static_cast<int>(model_ground_pallet_.size()) &&
                model_ground_pallet_[pallet_type]) {
                LOG_INFO("Ground pallet z constrained: {:.3f} -> {:.3f}",
                         result.translation().z(), guess.translation().z());
                result.translation().z() = guess.translation().z();
            }

            Scan3D aligned_template =
                TransformTemplate(pallet_type, result.matrix());
            float scale = 1.0f;

            VisualizerCallback::point_cloud_3d_callback(
                inliers, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
                "sensor_data", "blue", scale);

            VisualizerCallback::point_cloud_3d_callback(
                aligned_template, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
                "aligned_pallet", "green", scale);

            Scan3D initial_template =
                TransformTemplate(pallet_type, initial_guess);
            VisualizerCallback::point_cloud_3d_callback(
                initial_template, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
                "initial_guess", "red", scale);

            PerceptionResultCallback::pallet_pose_callback(
                result, cur_robot_pose, time_stamp);
            {
                std::lock_guard<std::mutex> lock(delta_mtx_);
                delta_transform_ = guess.inverse() * result;
                robot_to_pallet_center_ = result;
            }
            LOG_INFO(
                "  Delta Position: x={:.3f}, y={:.3f}, z={:.3f}",
                delta_transform_.translation().x(),
                delta_transform_.translation().y(),
                delta_transform_.translation().z());
        }
        else {
            LOG_WARN(
                "Custom ICP REJECTED: rmse={:.4f} (th={:.4f}), "
                "confidence={:.1f} (th={:.1f})",
                res.GetRMSE(), rmse_threshold_, res.GetMatchRatio(),
                confidence_threshold_);
            LOG_WARN(
                "[ICP debug] REJECTED hint: if confidence=0, check "
                "max_correspondence_distance (current={:.3f}m). "
                "Increase it if initial_overlap was 0%% or very low.",
                static_cast<double>(max_correspondence_distance_));

            Scan3D aligned_template =
                TransformTemplate(pallet_type, res.GetCorrection().matrix());
            float scale = 1.0f;

            VisualizerCallback::point_cloud_3d_callback(
                inliers, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
                "sensor_data", "blue", scale);

            VisualizerCallback::point_cloud_3d_callback(
                aligned_template, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
                "aligned_pallet", "green", scale);

            Scan3D initial_template =
                TransformTemplate(pallet_type, initial_guess);
            VisualizerCallback::point_cloud_3d_callback(
                initial_template, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
                "initial_guess", "red", scale);
        }
    }

    const auto total_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - t_cycle_start)
            .count();
    LOG_INFO("[DetectPallets timing] ** total_cycle: {:.1f} ms **", total_ms);

    {
        std::lock_guard<std::mutex> lock(do_detecting_mtx_);
        b_detecting_ = false;
    }
}

bool PalletICPMatcher::DetectPalletsByPillars(
    const Scan3D &frame, const int &pallet_type,
    const Pose3D &guess, const Pose3D &cur_robot_pose,
    const long int &time_stamp)
{
    using Clock = std::chrono::steady_clock;
    const auto t_start = Clock::now();

    LOG_INFO("[PillarDetect] start (pallet_type={})", pallet_type);

    const double template_width =
        (pallet_type < static_cast<int>(model_widths_.size()))
            ? model_widths_[pallet_type]
            : 1.0;
    const Eigen::Vector3d guess_pos = guess.translation();

    // --- 1. Preprocessing (always include ground removal) ---
    Scan3D inliers = frame;
    if (use_range_filter_) {
        inliers = preprocessor_.FilterByRange(frame, min_range_, max_range_);
    }
    if (use_voxel_filter_) {
        inliers = preprocessor_.FilterByVoxel(inliers, 0.02f);
    }

    LOG_INFO("[PillarDetect] after range+voxel: {} points",
             inliers.GetPointCloud().size());

    // Ground removal is critical for pillar isolation (floor connects pillars)
    if (ground_removal_) {
        std::vector<int> ground_inliers;
        if (segmentation_.GroundRemoval(inliers, ground_inliers)) {
            auto [remaining, removed] =
                PointCloudUtils::SubstractPointCloud(inliers, ground_inliers);
            inliers = remaining;
            LOG_INFO("[PillarDetect] after ground removal: {} points",
                     inliers.GetPointCloud().size());
        }
    }

    if (use_sor_filter_) {
        PointCloud3DAdaptor adaptor_sor(inliers.GetPointCloud());
        auto kdtree_sor = KDTreeWrapper3d::GetKDTree3dPtr(
            inliers.GetPointCloud(), adaptor_sor);
        if (kdtree_sor) {
            inliers = preprocessor_.RemoveStatisticalOutliers(
                inliers, kdtree_sor.get(), sor_mean_k_, sor_std_dev_mul_thresh_);
        }
    }

    // Build KD-tree and compute normals
    PointCloud3DAdaptor adaptor_main(inliers.GetPointCloud());
    auto kdtree_main = KDTreeWrapper3d::GetKDTree3dPtr(
        inliers.GetPointCloud(), adaptor_main);
    if (!kdtree_main) {
        LOG_WARN("[PillarDetect] failed to build KD-tree");
        return false;
    }
    PointCloudUtils::ComputeNormals(
        inliers, kdtree_main.get(), use_radius_search_, search_radius_,
        view_point_arrange_);

    if (inliers.GetPointCloud().size() < 15) {
        LOG_WARN("[PillarDetect] too few points after preprocessing: {}",
                 inliers.GetPointCloud().size());
        return false;
    }

    LOG_INFO("[PillarDetect] preprocessed: {} points",
             inliers.GetPointCloud().size());

    // --- 2. ROI extraction around guess position ---
    // Use a bounding box centered on guess: ±roi_margin in X and Y
    const double roi_margin = template_width * 0.8;
    Scan3D roi_cloud;
    for (size_t i = 0; i < inliers.GetPointCloud().size(); ++i) {
        const auto &p = inliers.GetPointCloud()[i];
        if (std::fabs(p.x() - guess_pos.x()) < roi_margin &&
            std::fabs(p.y() - guess_pos.y()) < roi_margin) {
            roi_cloud.MutablePointCloud().push_back(p);
            if (!inliers.GetNormals().empty()) {
                roi_cloud.MutableNormals().push_back(inliers.GetNormals()[i]);
            }
        }
    }

    LOG_INFO("[PillarDetect] ROI around guess ({:.2f},{:.2f}): {} points "
             "(margin={:.2f}m)",
             guess_pos.x(), guess_pos.y(), roi_cloud.GetPointCloud().size(),
             roi_margin);

    if (roi_cloud.GetPointCloud().size() < 15) {
        LOG_WARN("[PillarDetect] too few points in ROI: {}",
                 roi_cloud.GetPointCloud().size());
        return false;
    }

    // --- RViz: publish ROI candidate cloud (yellow) ---
    Pose3D dummy_pose;
    float scale = 1.0f;
    VisualizerCallback::point_cloud_3d_callback(
        roi_cloud, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
        "pillar_candidate", "yellow", scale);

    // --- 3. Sub-clustering within ROI to isolate pillars ---
    PointCloud3DAdaptor adaptor_roi(roi_cloud.GetPointCloud());
    auto kdtree_roi = KDTreeWrapper3d::GetKDTree3dPtr(
        roi_cloud.GetPointCloud(), adaptor_roi);
    if (!kdtree_roi) {
        LOG_WARN("[PillarDetect] failed to build ROI KD-tree");
        return false;
    }

    auto sub_clusters = PointCloudUtils::EuclideanDistanceClustering(
        roi_cloud, kdtree_roi.get(), pillar_cluster_tolerance_,
        pillar_min_cluster_size_, pillar_max_cluster_size_);

    LOG_INFO("[PillarDetect] sub-clusters in ROI: {}", sub_clusters.size());

    // --- RViz: publish each sub-cluster with distinct color ---
    Scan3D all_clusters_vis;
    for (size_t ci = 0; ci < sub_clusters.size(); ++ci) {
        Scan3D cluster_vis;
        for (int idx : sub_clusters[ci]) {
            cluster_vis.MutablePointCloud().push_back(
                roi_cloud.GetPointCloud()[idx]);
        }
        Eigen::Vector3f color = PointCloudUtils::GetColorCode(
            static_cast<int>(ci), static_cast<int>(sub_clusters.size()));
        VisualizerCallback::point_cloud_3d_with_color_code_callback(
            cluster_vis,
            ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
            "pillar_cluster_" + std::to_string(ci), color);
    }

    // --- 4. Filter by normal: keep only vertical-face clusters (|nz| < threshold) ---
    struct PillarCandidate {
        Eigen::Vector3d centroid;
        Eigen::Vector3d mean_normal;
        int point_count;
    };
    std::vector<PillarCandidate> pillars;

    for (const auto &sc : sub_clusters) {
        Eigen::Vector3d cent = Eigen::Vector3d::Zero();
        Eigen::Vector3d mean_n = Eigen::Vector3d::Zero();
        for (int idx : sc) {
            cent += roi_cloud.GetPointCloud()[idx];
            if (!roi_cloud.GetNormals().empty()) {
                mean_n += roi_cloud.GetNormals()[idx];
            }
        }
        cent /= static_cast<double>(sc.size());
        if (mean_n.norm() > 1e-6) {
            mean_n.normalize();
        }

        LOG_INFO(
            "[PillarDetect] sub-cluster: centroid=({:.3f},{:.3f},{:.3f}), "
            "normal=({:.2f},{:.2f},{:.2f}), |nz|={:.2f}, pts={}",
            cent.x(), cent.y(), cent.z(), mean_n.x(), mean_n.y(),
            mean_n.z(), std::fabs(mean_n.z()), sc.size());

        // Check vertical face: |nz| should be small
        if (std::fabs(mean_n.z()) < pillar_normal_z_max_) {
            pillars.push_back({cent, mean_n, static_cast<int>(sc.size())});
        }
    }

    LOG_INFO("[PillarDetect] vertical-face pillars: {}", pillars.size());

    if (pillars.size() < 3) {
        LOG_WARN("[PillarDetect] need >= 3 pillar faces, found {}",
                 pillars.size());
        return false;
    }

    // --- 5. Find best triplet: 3 pillars with aligned X and equal Y-spacing ---
    // Sort pillars by Y coordinate
    std::sort(pillars.begin(), pillars.end(),
              [](const PillarCandidate &a, const PillarCandidate &b) {
                  return a.centroid.y() < b.centroid.y();
              });

    bool triplet_found = false;
    int idx_left = -1, idx_mid = -1, idx_right = -1;

    for (size_t i = 0; i < pillars.size() && !triplet_found; ++i) {
        for (size_t j = i + 1; j < pillars.size() && !triplet_found; ++j) {
            for (size_t k = j + 1; k < pillars.size() && !triplet_found; ++k) {
                const auto &p0 = pillars[i].centroid;
                const auto &p1 = pillars[j].centroid;
                const auto &p2 = pillars[k].centroid;

                // Check X-alignment: all three should have similar X
                double mean_x = (p0.x() + p1.x() + p2.x()) / 3.0;
                if (std::fabs(p0.x() - mean_x) > pillar_x_distance_tolerance_ ||
                    std::fabs(p1.x() - mean_x) > pillar_x_distance_tolerance_ ||
                    std::fabs(p2.x() - mean_x) > pillar_x_distance_tolerance_) {
                    continue;
                }

                // Check Y-spacing: gap01 ≈ gap12 ≈ width/2
                double gap01 = p1.y() - p0.y();
                double gap12 = p2.y() - p1.y();
                double expected_gap = template_width / 2.0;

                if (std::fabs(gap01 - expected_gap) < pillar_spacing_tolerance_ &&
                    std::fabs(gap12 - expected_gap) < pillar_spacing_tolerance_) {
                    triplet_found = true;
                    idx_left = static_cast<int>(i);
                    idx_mid = static_cast<int>(j);
                    idx_right = static_cast<int>(k);
                }
            }
        }
    }

    if (!triplet_found) {
        LOG_WARN("[PillarDetect] no valid 3-pillar triplet found "
                 "(expected_gap={:.3f}m, spacing_tol={:.3f}m, x_tol={:.3f}m)",
                 template_width / 2.0, pillar_spacing_tolerance_,
                 pillar_x_distance_tolerance_);
        // Log all pillar positions for debugging
        for (size_t i = 0; i < pillars.size(); ++i) {
            LOG_WARN("[PillarDetect]   pillar[{}]: ({:.3f},{:.3f},{:.3f})",
                     i, pillars[i].centroid.x(), pillars[i].centroid.y(),
                     pillars[i].centroid.z());
        }
        return false;
    }

    LOG_INFO("[PillarDetect] triplet found: L={}, M={}, R={}", idx_left,
             idx_mid, idx_right);

    // --- 6. Compute pose ---
    const Eigen::Vector3d &pl = pillars[idx_left].centroid;
    const Eigen::Vector3d &pm = pillars[idx_mid].centroid;
    const Eigen::Vector3d &pr = pillars[idx_right].centroid;

    Eigen::Vector3d center = (pl + pm + pr) / 3.0;

    // Yaw: direction perpendicular to left→right line
    Eigen::Vector3d lr_dir = (pr - pl).normalized();
    // Perpendicular in XY plane (pointing toward the robot, i.e., -X direction)
    double yaw = std::atan2(-lr_dir.x(), lr_dir.y());

    // 180° ambiguity: check if pillar face normals point toward robot (-X direction)
    Eigen::Vector3d avg_normal = (pillars[idx_left].mean_normal +
                                  pillars[idx_mid].mean_normal +
                                  pillars[idx_right].mean_normal);
    if (avg_normal.norm() > 1e-6) avg_normal.normalize();
    Eigen::Vector3d forward_dir(std::cos(yaw), std::sin(yaw), 0.0);
    if (avg_normal.dot(forward_dir) < 0.0) {
        yaw += M_PI;
        // Normalize to [-pi, pi]
        if (yaw > M_PI) yaw -= 2.0 * M_PI;
    }

    LOG_INFO(
        "[PillarDetect] ACCEPTED: center=({:.3f},{:.3f},{:.3f}), "
        "yaw={:.1f}deg",
        center.x(), center.y(), center.z(), yaw * 180.0 / M_PI);

    // Build result pose
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    Sophus::SO3d so3(R);
    Pose3D result(so3, center);

    // Constrain z for ground pallets
    if (pallet_type < static_cast<int>(model_ground_pallet_.size()) &&
        model_ground_pallet_[pallet_type]) {
        result.translation().z() = guess.translation().z();
    }

    // --- 7. Publish results ---
    VisualizerCallback::point_cloud_3d_callback(
        inliers, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
        "sensor_data", "blue", scale);

    // Visualize pillar centroids as aligned_pallet (green)
    Scan3D pillar_vis;
    pillar_vis.MutablePointCloud().push_back(pl);
    pillar_vis.MutablePointCloud().push_back(pm);
    pillar_vis.MutablePointCloud().push_back(pr);
    VisualizerCallback::point_cloud_3d_callback(
        pillar_vis,
        ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
        "aligned_pallet", "green", scale);

    PerceptionResultCallback::pallet_pose_callback(
        result, cur_robot_pose, time_stamp);
    {
        std::lock_guard<std::mutex> lock(delta_mtx_);
        delta_transform_ = guess.inverse() * result;
        robot_to_pallet_center_ = result;
    }

    const auto total_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - t_start)
            .count();
    LOG_INFO("[PillarDetect] total: {:.1f} ms", total_ms);

    return true;
}

void PalletICPMatcher::UpdateParameters()
{
    LOG_INFO("PalletICPMatcher parameters updated");
    min_range_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "preprocessor", "min_range")
            .convert<float>();
    max_range_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "preprocessor", "max_range")
            .convert<float>();

    voxel_leaf_size_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "preprocessor", "voxel_leaf_size")
            .convert<float>();
    roi_filter_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "preprocessor", "roi_filter")
            .convert<bool>();
    horizontal_angle_range_ =
        Configurator::GetInstance()
            .GetParamValue(
                "pallet_detector", "preprocessor", "horizontal_angle_range")
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
            .convert<float>();

    use_template_range_filter_ =
        Configurator::GetInstance()
            .GetParamValue(
                "pallet_detector", "preprocessor", "use_template_range_filter")
            .convert<bool>();

    max_iteration_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "icp", "max_iteration")
            .convert<int>();

    transformation_epsilon_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "icp", "transformation_epsilon")
            .convert<float>();

    max_correspondence_distance_ =
        Configurator::GetInstance()
            .GetParamValue(
                "pallet_detector", "icp", "max_correspondence_distance")
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
    view_point_arrange_ =
        Configurator::GetInstance()
            .GetParamValue(
                "pallet_detector", "preprocessor", "view_point_arrange")
            .convert<bool>();

    tmpl_lists_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "template", "template_lists")
            .extract<Poco::JSON::Array::Ptr>();
    use_pcl_icp_ = Configurator::GetInstance()
                       .GetParamValue("pallet_detector", "icp", "use_pcl_icp")
                       .convert<bool>();
    rmse_threshold_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "icp", "rmse_threshold")
            .convert<float>();
    confidence_threshold_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "icp", "confidence_threshold")
            .convert<float>();

    ground_removal_ =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "preprocessor", "ground_removal")
            .convert<bool>();

    ground_removal_distance_threshold_ =
        Configurator::GetInstance()
            .GetParamValue(
                "pallet_detector", "preprocessor",
                "ground_removal_distance_threshold")
            .convert<float>();

    ground_removal_max_iterations_ = Configurator::GetInstance()
                                         .GetParamValue(
                                             "pallet_detector", "preprocessor",
                                             "ground_removal_max_iterations")
                                         .convert<int>();

    pillar_cluster_tolerance_ =
        Configurator::GetInstance()
            .GetParamValue(
                "pallet_detector", "pillar_detection",
                "pillar_cluster_tolerance")
            .convert<float>();
    pillar_min_cluster_size_ =
        Configurator::GetInstance()
            .GetParamValue(
                "pallet_detector", "pillar_detection",
                "pillar_min_cluster_size")
            .convert<int>();
    pillar_max_cluster_size_ =
        Configurator::GetInstance()
            .GetParamValue(
                "pallet_detector", "pillar_detection",
                "pillar_max_cluster_size")
            .convert<int>();
    pillar_normal_z_max_ =
        Configurator::GetInstance()
            .GetParamValue(
                "pallet_detector", "pillar_detection", "pillar_normal_z_max")
            .convert<float>();
    pillar_spacing_tolerance_ =
        Configurator::GetInstance()
            .GetParamValue(
                "pallet_detector", "pillar_detection",
                "pillar_spacing_tolerance")
            .convert<float>();
    pillar_x_distance_tolerance_ =
        Configurator::GetInstance()
            .GetParamValue(
                "pallet_detector", "pillar_detection",
                "pillar_x_distance_tolerance")
            .convert<float>();
    pillar_skip_ground_removal_ =
        Configurator::GetInstance()
            .GetParamValue(
                "pallet_detector", "pillar_detection", "skip_ground_removal")
            .convert<bool>();
}
}  // namespace PERCEPTION
}  // namespace ANSWER
