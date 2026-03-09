#pragma once
#include "3d_localizer/matcher/icp3d.h"
#include "3d_perception/segmentation/segmentation.h"
// #include "common/3d/kdtree_wrapper_3d.h"
#include "common/3d/point_cloud_utils.h"
#include "common/3d/preprocessor.h"
#include "common/pch.h"
//#include "common/pose3d.h"
//#include "common/3d/pose3d.h"
#include "common/3d/scan3d.h"
#include "common/time_checker.h"

#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/search/kdtree.h>

namespace ANSWER {
namespace PERCEPTION {
class WingbodyDetector {
private:
    /* data */
    Segmentation segmentation_;
    void UpdateParameters();
    Preprocessor preprocessor_;
    std::unique_ptr<MATCHER::icp3d> icp3d_;

    bool ground_removal_;
    bool roi_filter_;
    bool use_radius_search_;
    bool use_normals_;
    bool use_sor_filter_;
    bool view_point_arrange_;
    float min_range_;
    float max_range_;
    float voxel_leaf_size_;
    float ground_removal_distance_threshold_;
    float horizontal_angle_range_;
    float min_height_;
    float max_height_;
    float search_radius_;
    float sor_mean_k_;
    float sor_std_dev_mul_thresh_;

    float normal_angle_threshold_;
    float cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    int ground_removal_max_iterations_;

    bool use_voxel_filter_;
    bool b_detecting_;
    float rmse_threshold_;
    float confidence_threshold_;

    Pose3D odom_delta_;
    Pose3D robot_to_pallet_center_;
    Pose3D delta_transform_;

    Poco::JSON::Array::Ptr tmpl_lists_;
    Poco::JSON::Array::Ptr lack_positions_;

    std::mutex do_detecting_mtx_;
    std::mutex delta_mtx_;

    std::vector<Scan3D> model_scans_;
    std::vector<double> model_widths_;
    std::vector<double> model_heights_;

    std::thread wingbody_icp_thread_;
    std::unique_ptr<KDTree3d> kdtree_3d_;

public:
    WingbodyDetector(/* args */);
    ~WingbodyDetector();

    void ProcessPointCloud(const Scan3D &pcd);
    void BuildKDTree(Scan3D &inliers, const PointCloud3DAdaptor &adaptor);
    bool LoadTemplateFile(
        const std::string &model_path, const std::string &file_name,
        Scan3D &scan);
    void DetectWingbodyThread(
        const Scan3D &frame, const Pose3D &guess = Pose3D(),
        const Pose3D &odom_delta = Pose3D(),
        const Pose3D &cur_robot_pose = Pose3D(), const int time_stamp = 0);

    void DetectWingbody(
        const Scan3D &frame, const Pose3D &guess = Pose3D(),
        const Pose3D &cur_robot_pose = Pose3D(), const int timestamp = 0);
    Scan3D TransformTemplate(
        const int &pallet_type, const Eigen::Matrix4d &transformation) const;

    Scan3D TransformTemplate(
        const int &pallet_type, const Eigen::Matrix3d &rotation,
        const Eigen::Vector3d &translation) const;

    const Pose3D GetICPResultPos()
    {
        std::lock_guard<std::mutex> lock(delta_mtx_);
        return robot_to_pallet_center_;
    }
    void SetICPResultPos(const Pose3D delta)
    {
        std::lock_guard<std::mutex> lock(delta_mtx_);
        robot_to_pallet_center_ = delta;
    }

    const Pose3D GetDeltaTransform()
    {
        std::lock_guard<std::mutex> lock(delta_mtx_);
        return delta_transform_;
    }
    void SetDeltaTransform(const Pose3D delta)
    {
        std::lock_guard<std::mutex> lock(delta_mtx_);
        delta_transform_ = delta;
    }
    void SetGlobalConfidence(const double &confidence)
    {
        icp3d_->SetGlobalConfidence(confidence);
    }
};
}  // namespace PERCEPTION
}  // namespace ANSWER