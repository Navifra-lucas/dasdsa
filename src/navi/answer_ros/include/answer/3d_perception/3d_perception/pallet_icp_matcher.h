#pragma once
#include "3d_localizer/matcher/icp3d.h"
#include "3d_perception/segmentation/segmentation.h"
#include "common/3d/point_cloud_utils.h"
#include "common/3d/preprocessor.h"
#include "common/callbacks.h"
#include "common/pch.h"
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
class PalletICPMatcher {
public:
    PalletICPMatcher();
    ~PalletICPMatcher();
    void DetectPallets(
        const Scan3D &frame, const int &pallet_type = 0,
        const Pose3D &guess = Pose3D(), const Pose3D &cur_robot_pose = Pose3D(),
        const long int &time_stamp = 0);

    void DetectPalletsThread(
        const Scan3D &frame, const int &pallet_type = 0,
        const Pose3D &guess = Pose3D(), const Pose3D &odom_delta = Pose3D(),
        const Pose3D &cur_robot_pose = Pose3D(),
        const long int &time_stamp = 0);

    bool DetectPalletsByPillars(
        const Scan3D &frame, const int &pallet_type,
        const Pose3D &guess, const Pose3D &cur_robot_pose,
        const long int &time_stamp);

    void SetGlobalConfidence(const double &confidence)
    {
        icp3d_->SetGlobalConfidence(confidence);
    }

    // Load PCD/PLY file and convert to vector<Eigen::Vector3d>
    bool LoadTemplateFile(
        const std::string &model_path, const std::string &file_name,
        Scan3D &scan, bool center_template = true);

    // Transform template point cloud with initial guess (rotation +
    // translation)
    Scan3D TransformTemplate(
        const int &pallet_type, const Eigen::Matrix3d &rotation,
        const Eigen::Vector3d &translation) const;

    // Transform template point cloud with SE3 transformation matrix
    Scan3D TransformTemplate(
        const int &pallet_type, const Eigen::Matrix4d &transformation) const;

    // ICP alignment: align template to target point cloud
    // Returns transformation matrix and fitness score
    struct ICPResult {
        Eigen::Matrix4d transformation;
        double fitness_score;
        int num_iterations;
        bool converged;
    };

    ICPResult AlignICP(
        const int &pallet_type, const Scan3D &target_cloud,
        const Eigen::Matrix4d &initial_guess = Eigen::Matrix4d::Identity(),
        int max_iterations = 50, double transformation_epsilon = 1e-6,
        double max_correspondence_distance = 0.5);

    // GICP alignment using PCL's Generalized ICP
    ICPResult AlignGICP(
        const int &pallet_type, const Scan3D &target_cloud,
        const Eigen::Matrix4d &initial_guess = Eigen::Matrix4d::Identity(),
        int max_iterations = 50, double transformation_epsilon = 1e-6,
        double max_correspondence_distance = 0.5);

    void DetectThread();

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
    void SetCorrespondenceDistance(const float &max_correspondence_distance)
    {
        max_correspondence_distance_ = max_correspondence_distance;
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

private:
    // Helper function: Find nearest neighbors for correspondence
    void FindCorrespondences(
        const Scan3D &source, const Scan3D &target, KDTree3d *target_kdtree,
        int K, double max_distance, std::vector<int> &source_indices,
        std::vector<int> &target_indices);

    // Helper function: Compute transformation using point-to-plane ICP
    Eigen::Matrix4d ComputeTransformation(
        const Scan3D &source, const Scan3D &target,
        const std::vector<int> &source_indices,
        const std::vector<int> &target_indices);
    Preprocessor preprocessor_;
    Segmentation segmentation_;

    Scan3D pallet_template_;
    Pose3D robot_to_pallet_center_;
    Pose3D odom_delta_;
    std::unique_ptr<MATCHER::icp3d> icp3d_;

    bool ground_removal_;
    bool roi_filter_;
    bool use_radius_search_;
    bool use_normals_;
    bool use_voxel_filter_;
    bool use_range_filter_;
    bool b_detecting_;
    bool use_pcl_icp_;
    float min_range_;
    float max_range_;
    float voxel_leaf_size_;
    float ground_removal_distance_threshold_;
    float horizontal_angle_range_;
    float vertical_distance_range_;
    float search_radius_;

    int max_iteration_;
    float transformation_epsilon_;
    float max_correspondence_distance_;

    float normal_angle_threshold_;
    float cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    int ground_removal_max_iterations_;
    float rmse_threshold_;
    float confidence_threshold_;

    bool use_template_range_filter_;
    bool use_sor_filter_;
    bool view_point_arrange_;
    int sor_mean_k_;
    float sor_std_dev_mul_thresh_;

    Scan3D frame_;
    int pallet_type_;
    Pose3D guess_;
    Pose3D delta_transform_;

    std::string pallet_template_path_;
    Poco::JSON::Object::Ptr type1_;
    Poco::JSON::Object::Ptr type2_;
    Poco::JSON::Array::Ptr tmpl_lists_;

    std::vector<Scan3D> model_scans_;
    std::vector<double> model_widths_;
    std::vector<double> model_heights_;
    std::vector<bool> model_ground_pallet_;
    std::vector<bool> model_use_pillar_detection_;
    std::vector<bool> model_center_template_;
    std::vector<bool> model_ground_removal_;

    float pillar_cluster_tolerance_;
    int pillar_min_cluster_size_;
    int pillar_max_cluster_size_;
    float pillar_normal_z_max_;
    float pillar_spacing_tolerance_;
    float pillar_x_distance_tolerance_;
    bool pillar_skip_ground_removal_;

    std::thread pallet_icp_thread_;
    std::mutex do_detecting_mtx_;
    std::mutex delta_mtx_;

    std::unique_ptr<KDTree3d> kdtree_3d_;
    void UpdateParameters();
};
}  // namespace PERCEPTION
}  // namespace ANSWER