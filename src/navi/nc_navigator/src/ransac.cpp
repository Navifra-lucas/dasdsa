#include "nc_navigator/ransac.hpp"

namespace navi_ransac {

RANSAC::RANSAC() : nh_()
{
    RegistListener();
}

RANSAC::~RANSAC(){ }

bool RANSAC::SetParam(std::shared_ptr<Parameters> st_parameters)
{
    st_parameters_ = st_parameters;

    return true;
}

void RANSAC::RegistListener()
{
    // hole_state_sub_ = nh_.subscribe<std_msgs::Bool>("/hole_detector/hole_mode", 10, boost::bind(&RANSAC::HoleStateCallback, this, _1));
}

// void RANSAC::HoleStateCallback(const std_msgs::Bool::ConstPtr& msg)
// {

// }

bool RANSAC::Notify(const std::string &str_cbf_name, const boost::any &any_type_var)
{
    if (map_callback_pt_.find(str_cbf_name) == map_callback_pt_.end())
    {
        return false;
    }
    else
    {
        map_callback_pt_[str_cbf_name](any_type_var);
        return true;
    }
}

bool RANSAC::RegistCallbackFunc(const std::string &str_cbf_name, const std::function<void(const boost::any &)> &pt_func)
{
    if (map_callback_pt_.find(str_cbf_name) == map_callback_pt_.end())
    {
        map_callback_pt_[str_cbf_name] = pt_func;
        return true;
    }
    else
    {
        return false;
    }
}

std::vector<pPointCloudXYZ> RANSAC::RansacProcess(const pPointCloudXYZ& input_cloud_ptr)
{
    // NLOG(info) << "dan_Hole Detector State : " << b_hole_state_;
    // NLOG(info) << "dan_Hole Detector State : " << b_hole_state_;
    // NLOG(info) << "dan_Hole Detector State : " << b_hole_state_;
    // NLOG(info) << "dan_distance_m : " << st_parameters_->ransac_param.d_distance_m;
    // NLOG(info) << "dan_distance_m : " << st_parameters_->ransac_param.d_distance_m;
    // NLOG(info) << "dan_distance_m : " << st_parameters_->ransac_param.d_distance_m;
    // NLOG(info) << "dan_ground_limit_m : " <<st_parameters_->ransac_param.d_ground_limit_m;
    // NLOG(info) << "dan_ground_limit_m : " <<st_parameters_->ransac_param.d_ground_limit_m;
    // NLOG(info) << "dan_ground_limit_m : " <<st_parameters_->ransac_param.d_ground_limit_m;
    
    pPointCloudXYZ pass_through_cloud_ptr (new PointCloudXYZ);
    pPointCloudXYZ downsampled_cloud_ptr (new PointCloudXYZ);

    std::chrono::duration<double> eclipse;
    std::chrono::system_clock::time_point timer = std::chrono::system_clock::now();
    PassThrough(input_cloud_ptr, pass_through_cloud_ptr);
    eclipse = std::chrono::system_clock::now() - timer;
    NLOG(debug) << "PassThrough eclipse: " << eclipse.count() * 1000;

    timer = std::chrono::system_clock::now();
    VoxelGridFilter(pass_through_cloud_ptr, downsampled_cloud_ptr);
    eclipse = std::chrono::system_clock::now() - timer;
    NLOG(debug) << "VoxelGridFilter eclipse: " << eclipse.count() * 1000;

    timer = std::chrono::system_clock::now();
    auto ransac_result = FilterGround(downsampled_cloud_ptr);
    eclipse = std::chrono::system_clock::now() - timer;
    NLOG(debug) << "FilterGround eclipse: " << eclipse.count() * 1000;

    return std::move(ransac_result);
}

bool RANSAC::PassThrough(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_ptr, const pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud_ptr)
{
    output_cloud_ptr->header.frame_id = input_cloud_ptr->header.frame_id;
    output_cloud_ptr->points.reserve(input_cloud_ptr->points.size());

    const auto& param = st_parameters_->ransac_param;
    bool z_filter_active = param.d_z_filter_m >= 0;

    for (const auto &p : input_cloud_ptr->points)
    {
        bool in_roi = (p.x > param.d_x_min_th_m && p.x < param.d_x_max_th_m) &&
                      (p.y > param.d_y_min_th_m && p.y < param.d_y_max_th_m) &&
                      (p.z > param.d_z_min_th_m && p.z < param.d_z_max_th_m);

        bool in_filter_zone = (p.x <=  param.d_x_filter_m && p.x >= -param.d_x_filter_m &&
                               p.y <=  param.d_y_filter_m && p.y >= -param.d_y_filter_m);

        if (in_roi && !(in_filter_zone && ((z_filter_active && p.z >= param.d_z_filter_m) ||
                                           (!z_filter_active && p.z <= -param.d_z_filter_m))))
        {
            output_cloud_ptr->points.push_back(p);
        }
    }

    return true;
}

bool RANSAC::VoxelGridFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_ptr, const pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud_ptr)
{
    output_cloud_ptr->header.frame_id = input_cloud_ptr->header.frame_id;
    output_cloud_ptr->points.reserve(input_cloud_ptr->points.size());

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(input_cloud_ptr);
    voxel_grid.setLeafSize(st_parameters_->ransac_param.d_voxel_leaf_size_m, st_parameters_->ransac_param.d_voxel_leaf_size_m, st_parameters_->ransac_param.d_voxel_leaf_size_m);
    voxel_grid.setDownsampleAllData(true);
    voxel_grid.filter(*output_cloud_ptr);

    return true;
}

std::vector<pPointCloudXYZ> RANSAC::FilterGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_ptr)
{
    // Create point clouds for top and bottom sections
    pPointCloudXYZ top_cloud_ptr(new PointCloudXYZ);
    pPointCloudXYZ bottom_cloud_ptr(new PointCloudXYZ);

    // Reserve memory for efficiency
    size_t cloud_size = input_cloud_ptr->points.size();
    top_cloud_ptr->points.reserve(cloud_size);
    bottom_cloud_ptr->points.reserve(cloud_size);

    // Filter points based on z value
    for (const auto &p : input_cloud_ptr->points)
    {
        if (p.z > st_parameters_->ransac_param.d_z_min_th_m && p.z < st_parameters_->ransac_param.d_ground_limit_m)
            bottom_cloud_ptr->points.push_back(p);
        else
            top_cloud_ptr->points.push_back(p);
    }

    // Shrink to fit to reduce memory usage
    top_cloud_ptr->points.shrink_to_fit();
    bottom_cloud_ptr->points.shrink_to_fit();

    // Set headers
    top_cloud_ptr->header = input_cloud_ptr->header;
    bottom_cloud_ptr->header = input_cloud_ptr->header;

    if (st_parameters_->ransac_param.b_use) {
        pcl::ModelCoefficients::Ptr coefficients_ptr(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices);

        pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // pPointCloudXYZ input_cloud_ptr(new PointCloudXYZ(*bottom_cloud_ptr));
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(st_parameters_->ransac_param.d_distance_m);
        seg.setMaxIterations(st_parameters_->ransac_param.u_iteration);
        seg.setInputCloud(bottom_cloud_ptr);
        seg.segment(*inliers_ptr, *coefficients_ptr);

        // // Normal estimation
        // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr(new pcl::PointCloud<pcl::Normal>());
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr normals_tree(new pcl::search::KdTree<pcl::PointXYZ>);
        // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n3d;
        // n3d.setKSearch(st_parameters_->ransac_param.n_k_search);
        // n3d.setSearchMethod(normals_tree);
        // n3d.setInputCloud(bottom_cloud_ptr);
        // n3d.compute(*cloud_normals_ptr);

        // // RANSAC segmentation
        // pcl::ModelCoefficients::Ptr coefficients_ptr(new pcl::ModelCoefficients);
        // pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices);
        // pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        // seg.setDistanceThreshold(st_parameters_->ransac_param.d_distance_m);
        // seg.setMaxIterations(st_parameters_->ransac_param.u_iteration);
        // seg.setNormalDistanceWeight(st_parameters_->ransac_param.d_distance_weight);
        // seg.setOptimizeCoefficients(true);
        // seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        // seg.setMethodType(pcl::SAC_RANSAC);
        // seg.setProbability(st_parameters_->ransac_param.d_prob);
        // seg.setInputCloud(bottom_cloud_ptr);
        // seg.setInputNormals(cloud_normals_ptr);
        // seg.segment(*inliers_ptr, *coefficients_ptr);

        // Extract inliers and outliers
        pPointCloudXYZ no_ground_cloud_ptr(new PointCloudXYZ);
        pPointCloudXYZ ground_cloud_ptr(new PointCloudXYZ);

        // Set headers
        no_ground_cloud_ptr->header = input_cloud_ptr->header;
        ground_cloud_ptr->header = input_cloud_ptr->header;

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(bottom_cloud_ptr);
        extract.setIndices(inliers_ptr);
        extract.setNegative(false);
        extract.filter(*ground_cloud_ptr);

        extract.setNegative(true);
        extract.filter(*no_ground_cloud_ptr);
        *no_ground_cloud_ptr += *top_cloud_ptr;
        std::vector<pPointCloudXYZ> vec_pointcloud(2);
        vec_pointcloud[0] = ground_cloud_ptr;
        vec_pointcloud[1] = no_ground_cloud_ptr;

        return std::move(vec_pointcloud);
    }
    else {
        std::vector<pPointCloudXYZ> vec_pointcloud(2);
        vec_pointcloud[0] = bottom_cloud_ptr;
        vec_pointcloud[1] = top_cloud_ptr;

        return std::move(vec_pointcloud);
    }
}

} // namespace RANSAC