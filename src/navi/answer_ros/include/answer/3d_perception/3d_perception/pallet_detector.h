#pragma once
#include "3d_perception/segmentation/segmentation.h"
#include "common/3d/point_cloud_utils.h"
#include "common/3d/preprocessor.h"
#include "common/callbacks.h"
#include "common/pch.h"
#include "common/time_checker.h"

namespace ANSWER {
namespace PERCEPTION {
class PalletDetector {
public:
    PalletDetector();
    ~PalletDetector();
    void DetectPallets(const Scan3D &frame, const bool is_unloading = false);

private:
    Preprocessor preprocessor_;
    Segmentation segmentation_;

    bool ground_removal_;
    bool roi_filter_;
    bool use_radius_search_;
    bool use_normals_;
    bool use_voxel_filter_;
    bool use_range_filter_;
    bool use_sor_filter_;
    float min_range_;
    float max_range_;
    float min_height_;
    float max_width_;
    float voxel_leaf_size_;
    float ground_removal_distance_threshold_;
    float horizontal_angle_range_;
    float max_height_;
    float search_radius_;
    double pallet_unloading_height_min_;
    double pallet_unloading_height_max_;
    double sor_std_dev_mul_thresh_;

    float normal_angle_threshold_;
    float cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    int ground_removal_max_iterations_;
    int sor_mean_k_;
    std::unique_ptr<KDTree3d> kdtree_3d_;
    void UpdateParameters();
};
}  // namespace PERCEPTION
}  // namespace ANSWER