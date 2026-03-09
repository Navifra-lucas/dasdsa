#include "common/pch.h"

#include "3d_perception/wingbody_detector.h"

namespace ANSWER {
namespace PERCEPTION {
WingbodyDetector::WingbodyDetector(/* args */)
    : b_detecting_(false)
{
    LOG_INFO("WingbodyDetector initialized");
    UpdateParameters();
    icp3d_ = std::make_unique<MATCHER::icp3d>();

    std::string model_pcd_file =
        Path::GetInstance().GetROSInstallPath() + "/template/";

    if (tmpl_lists_ && tmpl_lists_->size() > 0) {
        for (auto i = 0; i < tmpl_lists_->size(); ++i) {
            auto tmpl = tmpl_lists_->get(i).extract<Poco::JSON::Object::Ptr>();

            std::string tmpl_name = tmpl->getValue<std::string>("file_name");
            LOG_INFO("wingbody tmpl name : {}", tmpl_name);

            std::string tmpl_path = model_pcd_file;

            Scan3D model_scan;
            if (LoadTemplateFile(tmpl_path, tmpl_name, model_scan)) {
                double width = tmpl->getValue<double>("width");
                double height = tmpl->getValue<double>("height");

                model_widths_.emplace_back(width);
                model_heights_.emplace_back(height);

                model_scans_.emplace_back(model_scan);
            }
        }
    }

    delta_transform_ = Pose3D();
    robot_to_pallet_center_ = Pose3D();
    odom_delta_ = Pose3D();

    ServiceCallback::update_parameters_callback.add(
        [this]() { UpdateParameters(); });
}

WingbodyDetector::~WingbodyDetector()
{
    b_detecting_ = false;
    if (wingbody_icp_thread_.joinable()) {
        wingbody_icp_thread_.join();
    }
}

void WingbodyDetector::ProcessPointCloud(const Scan3D &pcd)
{
    Pose3D dummy_pose;

    // step 1 : Preprocessing
    // step 1-1 : Range Filter
    TimeChecker tc;
    tc.MeasureStart();
    auto inliers = preprocessor_.FilterByRange(pcd, min_range_, max_range_);
    tc.MeasureEndMicro("range filtering");

    // step 1-2 : Ground Removal
    tc.MeasureStart();
    if (ground_removal_) {
        inliers = preprocessor_.RemoveGround(
            inliers, ground_removal_distance_threshold_,
            ground_removal_max_iterations_);
    }
    tc.MeasureEndMicro("ground removal");

    // step 1-3 : ROI Filtering
    tc.MeasureStart();
    if (roi_filter_) {
        inliers = preprocessor_.FilterByROI(
            inliers, horizontal_angle_range_, max_height_, min_height_);
    }
    tc.MeasureEndMicro("roi filtering");

    PointCloud3DAdaptor adaptor = PointCloud3DAdaptor(inliers.GetPointCloud());
    kdtree_3d_ =
        KDTreeWrapper3d::GetKDTree3dPtr(inliers.GetPointCloud(), adaptor);
    if (kdtree_3d_ == nullptr) {
        LOG_ERROR("Failed to build KD-Tree");
        return;
    }
    if (use_sor_filter_) {
        tc.MeasureStart();
        auto result = preprocessor_.RemoveStatisticalOutliers(
            inliers, kdtree_3d_.get(), sor_mean_k_, sor_std_dev_mul_thresh_);
        inliers = result;
        LOG_INFO(
            "After SOR Filter PointCloud Size: {}",
            inliers.GetPointCloud().size());
        tc.MeasureEndMicro("sor filtering");

        // SOR 필터로 포인트가 변경되었으므로 KD-tree 재구축
        PointCloud3DAdaptor adaptor_rebuilt =
            PointCloud3DAdaptor(inliers.GetPointCloud());
        kdtree_3d_ =
            KDTreeWrapper3d::GetKDTree3dPtr(inliers.GetPointCloud(), adaptor_rebuilt);
        if (kdtree_3d_ == nullptr) {
            LOG_ERROR("Failed to rebuild KD-Tree after SOR filter");
            return;
        }
    }

    // step 3 : Compute Normals
    tc.MeasureStart();
    PointCloudUtils::ComputeNormals(
        inliers, kdtree_3d_.get(), use_radius_search_, search_radius_,
        view_point_arrange_);
    tc.MeasureEndMicro("normal computation");

    // step 4 : Clustering
    auto clusters = PointCloudUtils::EuclideanDistanceClustering(
        inliers, kdtree_3d_.get(), cluster_tolerance_, min_cluster_size_,
        max_cluster_size_, use_normals_, normal_angle_threshold_);
    LOG_INFO("{} clusters found", clusters.size());

    int id = 0;
    for (auto cluster : clusters) {
        Scan3D clustered_pc;
        for (auto idx : cluster) {
            clustered_pc.MutablePointCloud().emplace_back(
                inliers.GetPointCloud()[idx]);
        }
        std::string cluster_name = "cluster_" + std::to_string(id++);
        auto color_code = PointCloudUtils::GetColorCode(id, clusters.size());
        VisualizerCallback::point_cloud_3d_with_color_code_callback(
            clustered_pc, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
            cluster_name, color_code);
    }
}

bool WingbodyDetector::LoadTemplateFile(
    const std::string &model_path, const std::string &file_name, Scan3D &scan)
{
    TimeChecker tc;

    scan.MutablePointCloud().clear();
    scan.MutableNormals().clear();

    const std::string &file_path = model_path + file_name;

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
        "Successfully loaded template file: {}, {} points", file_path,
        pcl_cloud->points.size());

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
    }
    else {
        filtered_cloud = pcl_cloud;
    }

    scan.MutablePointCloud().reserve(filtered_cloud->points.size());

    for (size_t i = 0; i < filtered_cloud->points.size(); ++i) {
        const auto &point = filtered_cloud->points[i];

        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            continue;
        }

        scan.MutablePointCloud().emplace_back(
            Eigen::Vector3d(point.x, point.y, point.z));
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

    tc.MeasureStart();
    PointCloudUtils::ComputeNormals(
        scan, kdtree_3d.get(), use_radius_search_, search_radius_,
        view_point_arrange_);
    tc.MeasureEndMicro("normal computation");

    LOG_INFO(
        "Wingbody template loaded: {} points with normals",
        scan.GetPointCloud().size());

    return true;
}

Scan3D WingbodyDetector::TransformTemplate(
    const int &pallet_type, const Eigen::Matrix3d &rotation,
    const Eigen::Vector3d &translation) const
{
    Scan3D transformed_scan;
    if (pallet_type < 0 || pallet_type >= static_cast<int>(model_scans_.size())) {
        LOG_ERROR("Invalid template type: {}", pallet_type);
        return transformed_scan;
    }

    const auto &template_points = model_scans_[pallet_type].GetPointCloud();
    const auto &template_normals = model_scans_[pallet_type].GetNormals();

    transformed_scan.MutablePointCloud().reserve(template_points.size());
    transformed_scan.MutableNormals().reserve(template_normals.size());

    for (const auto &point : template_points) {
        Eigen::Vector3d transformed_point =
            rotation * point.head<3>() + translation;
        transformed_scan.MutablePointCloud().emplace_back(transformed_point);
    }

    for (const auto &normal : template_normals) {
        Eigen::Vector3d transformed_normal = rotation * normal.head<3>();
        if (transformed_normal.norm() > 1e-6) {
            transformed_normal.normalize();
        }
        transformed_scan.MutableNormals().emplace_back(transformed_normal);
    }

    return transformed_scan;
}

Scan3D WingbodyDetector::TransformTemplate(
    const int &pallet_type, const Eigen::Matrix4d &transformation) const
{
    Eigen::Matrix3d rotation = transformation.block<3, 3>(0, 0);
    Eigen::Vector3d translation = transformation.block<3, 1>(0, 3);
    return TransformTemplate(pallet_type, rotation, translation);
}

void WingbodyDetector::DetectWingbodyThread(
    const Scan3D &frame, const Pose3D &guess, const Pose3D &odom_delta,
    const Pose3D &cur_robot_pose, const int time_stamp)
{
    bool b_detecting = false;
    {
        std::lock_guard<std::mutex> lock(do_detecting_mtx_);
        b_detecting = b_detecting_;
    }
    if (b_detecting == false) {
        if (wingbody_icp_thread_.joinable()) {
            wingbody_icp_thread_.join();
        }
        Pose3D launch_pose;
        {
            std::lock_guard<std::mutex> lock(delta_mtx_);
            odom_delta_ = odom_delta_ * odom_delta;
            robot_to_pallet_center_ = robot_to_pallet_center_ * odom_delta_;
            launch_pose = robot_to_pallet_center_;
            odom_delta_ = Pose3D();
        }
        wingbody_icp_thread_ = std::thread(
            &WingbodyDetector::DetectWingbody, this, frame,
            launch_pose, cur_robot_pose, time_stamp);
    }
    else {
        std::lock_guard<std::mutex> lock(delta_mtx_);
        odom_delta_ = odom_delta_ * odom_delta;
    }
}

void WingbodyDetector::DetectWingbody(
    const Scan3D &frame, const Pose3D &guess, const Pose3D &cur_robot_pose,
    const int timestamp)
{
    if (frame.GetPointCloud().empty()) {
        LOG_ERROR("Input sensor point cloud is empty!");
        return;
    }

    if (model_scans_.empty()) {
        LOG_WARN("No wingbody templates loaded, falling back to ProcessPointCloud");
        ProcessPointCloud(frame);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(do_detecting_mtx_);
        b_detecting_ = true;
    }

    Pose3D dummy_pose;
    Pose3D result;
    TimeChecker tc;

    // Preprocessing
    Scan3D inliers = frame;
    tc.MeasureStart();
    inliers = preprocessor_.FilterByRange(frame, min_range_, max_range_);
    tc.MeasureEndMicro("range filtering");

    if (use_voxel_filter_) {
        tc.MeasureStart();
        inliers = preprocessor_.FilterByVoxel(inliers, voxel_leaf_size_);
        tc.MeasureEndMicro("voxel filtering");
    }

    if (ground_removal_) {
        tc.MeasureStart();
        inliers = preprocessor_.RemoveGround(
            inliers, ground_removal_distance_threshold_,
            ground_removal_max_iterations_);
        tc.MeasureEndMicro("ground removal");
    }

    if (roi_filter_) {
        tc.MeasureStart();
        inliers = preprocessor_.FilterByROI(
            inliers, horizontal_angle_range_, max_height_, min_height_);
        tc.MeasureEndMicro("roi filtering");
    }

    if (use_sor_filter_) {
        PointCloud3DAdaptor adaptor_sor =
            PointCloud3DAdaptor(inliers.GetPointCloud());
        kdtree_3d_ =
            KDTreeWrapper3d::GetKDTree3dPtr(inliers.GetPointCloud(), adaptor_sor);
        if (kdtree_3d_ != nullptr) {
            tc.MeasureStart();
            auto sor_result = preprocessor_.RemoveStatisticalOutliers(
                inliers, kdtree_3d_.get(), sor_mean_k_, sor_std_dev_mul_thresh_);
            inliers = sor_result;
            tc.MeasureEndMicro("sor filtering");
        }
    }

    // Build KD-Tree
    tc.MeasureStart();
    PointCloud3DAdaptor adaptor = PointCloud3DAdaptor(inliers.GetPointCloud());
    kdtree_3d_ =
        KDTreeWrapper3d::GetKDTree3dPtr(inliers.GetPointCloud(), adaptor);
    if (kdtree_3d_ == nullptr) {
        LOG_ERROR("Failed to build KD-Tree");
        std::lock_guard<std::mutex> lock(do_detecting_mtx_);
        b_detecting_ = false;
        return;
    }
    tc.MeasureEndMicro("kd tree building");

    // Compute Normals
    tc.MeasureStart();
    PointCloudUtils::ComputeNormals(
        inliers, kdtree_3d_.get(), use_radius_search_, search_radius_,
        view_point_arrange_);
    tc.MeasureEndMicro("normal computation");

    // ICP alignment with template (type 0 by default)
    int wingbody_type = 0;
    Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();
    initial_guess.block<3, 3>(0, 0) = guess.rotationMatrix();
    initial_guess.block<3, 1>(0, 3) = guess.translation();

    tc.MeasureStart();
    auto res = icp3d_->AlignPointToPlane(
        model_scans_[wingbody_type], inliers, kdtree_3d_.get(), guess);
    tc.MeasureEndMicro("wingbody icp alignment");

    LOG_INFO(
        "Wingbody ICP: rmse={}, confidence={}, iter={}",
        res.GetRMSE(), res.GetMatchRatio(), res.GetIteration());

    if (res.GetRMSE() < rmse_threshold_ &&
        res.GetMatchRatio() > confidence_threshold_) {
        result = res.GetCorrection();

        Scan3D aligned_template =
            TransformTemplate(wingbody_type, res.GetCorrection().matrix());
        float scale = 1.0f;

        VisualizerCallback::point_cloud_3d_callback(
            inliers, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
            "wingbody_sensor_data", "blue", scale);

        VisualizerCallback::point_cloud_3d_callback(
            aligned_template, ANSWER::answer_utils::ConvertSE3fToIsometry3d(dummy_pose),
            "wingbody_aligned", "green", scale);

        PerceptionResultCallback::pallet_pose_callback(
            result, cur_robot_pose, timestamp);
        {
            std::lock_guard<std::mutex> lock(delta_mtx_);
            delta_transform_ = guess.inverse() * result;
            robot_to_pallet_center_ = result;
        }
    }
    else {
        LOG_WARN("Wingbody detection failed!");
    }

    {
        std::lock_guard<std::mutex> lock(do_detecting_mtx_);
        b_detecting_ = false;
    }
}

void WingbodyDetector::BuildKDTree(
    Scan3D &inliers, const PointCloud3DAdaptor &adaptor)
{
    kdtree_3d_ =
        KDTreeWrapper3d::GetKDTree3dPtr(inliers.GetPointCloud(), adaptor);
    if (kdtree_3d_ == nullptr) {
        LOG_ERROR("Failed to build KD-Tree");
        return;
    }
}

void WingbodyDetector::UpdateParameters()
{
    LOG_INFO("WingbodyDetector parameters updated");
    min_range_ =
        Configurator::GetInstance()
            .GetParamValue("wingbody_detector", "preprocessor", "min_range")
            .convert<float>();
    max_range_ =
        Configurator::GetInstance()
            .GetParamValue("wingbody_detector", "preprocessor", "max_range")
            .convert<float>();

    ground_removal_ =
        Configurator::GetInstance()
            .GetParamValue(
                "wingbody_detector", "preprocessor", "ground_removal")
            .convert<bool>();
    ground_removal_distance_threshold_ =
        Configurator::GetInstance()
            .GetParamValue(
                "wingbody_detector", "preprocessor",
                "ground_removal_distance_threshold")
            .convert<float>();
    ground_removal_max_iterations_ =
        Configurator::GetInstance()
            .GetParamValue(
                "wingbody_detector", "preprocessor",
                "ground_removal_max_iterations")
            .convert<int>();
    roi_filter_ =
        Configurator::GetInstance()
            .GetParamValue("wingbody_detector", "preprocessor", "roi_filter")
            .convert<bool>();
    horizontal_angle_range_ =
        Configurator::GetInstance()
            .GetParamValue(
                "wingbody_detector", "preprocessor", "horizontal_angle_range")
            .convert<float>();
    max_height_ =
        Configurator::GetInstance()
            .GetParamValue("wingbody_detector", "preprocessor", "max_height")
            .convert<float>();
    min_height_ =
        Configurator::GetInstance()
            .GetParamValue("wingbody_detector", "preprocessor", "min_height")
            .convert<float>();
    use_sor_filter_ =
        Configurator::GetInstance()
            .GetParamValue(
                "wingbody_detector", "preprocessor", "use_sor_filter")
            .convert<bool>();
    sor_mean_k_ =
        Configurator::GetInstance()
            .GetParamValue("wingbody_detector", "preprocessor", "sor_mean_k")
            .convert<float>();
    sor_std_dev_mul_thresh_ =
        Configurator::GetInstance()
            .GetParamValue(
                "wingbody_detector", "preprocessor", "sor_std_dev_mul_thresh")
            .convert<float>();
    use_radius_search_ =
        Configurator::GetInstance()
            .GetParamValue(
                "wingbody_detector", "preprocessor", "use_radius_search")
            .convert<bool>();
    search_radius_ =
        Configurator::GetInstance()
            .GetParamValue("wingbody_detector", "preprocessor", "search_radius")
            .convert<float>();
    view_point_arrange_ =
        Configurator::GetInstance()
            .GetParamValue(
                "wingbody_detector", "preprocessor", "view_point_arrange")
            .convert<bool>();

    cluster_tolerance_ =
        Configurator::GetInstance()
            .GetParamValue(
                "wingbody_detector", "clustering", "cluster_tolerance")
            .convert<float>();
    min_cluster_size_ =
        Configurator::GetInstance()
            .GetParamValue(
                "wingbody_detector", "clustering", "min_cluster_size")
            .convert<int>();
    max_cluster_size_ =
        Configurator::GetInstance()
            .GetParamValue(
                "wingbody_detector", "clustering", "max_cluster_size")
            .convert<int>();
    use_normals_ =
        Configurator::GetInstance()
            .GetParamValue("wingbody_detector", "clustering", "use_normals")
            .convert<bool>();
    normal_angle_threshold_ =
        Configurator::GetInstance()
            .GetParamValue("wingbody_detector", "clustering", "angle_threshold")
            .convert<float>();

    use_voxel_filter_ =
        Configurator::GetInstance()
            .GetParamValue(
                "wingbody_detector", "preprocessor", "use_voxel_filter")
            .convert<bool>();
    rmse_threshold_ =
        Configurator::GetInstance()
            .GetParamValue("wingbody_detector", "icp", "rmse_threshold")
            .convert<float>();
    confidence_threshold_ =
        Configurator::GetInstance()
            .GetParamValue("wingbody_detector", "icp", "confidence_threshold")
            .convert<float>();

    // Load template lists if available
    try {
        tmpl_lists_ =
            Configurator::GetInstance()
                .GetParamValue(
                    "wingbody_detector", "template", "template_lists")
                .extract<Poco::JSON::Array::Ptr>();
    }
    catch (...) {
        tmpl_lists_ = nullptr;
    }
}

}  // namespace PERCEPTION
}  // namespace ANSWER
