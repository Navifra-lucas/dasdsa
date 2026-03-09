/**
 * @class parameter box
 * @brief various parameters for slam
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */
#ifndef SLAM_PARAM_H
#define SLAM_PARAM_H

#include "common/answer_utils.h"
#include "glog/logging.h"
#include "opencv2/opencv.hpp"

#include <filesystem>

using namespace std;
namespace ANSWER {
namespace SLAM2D {
class slam_parameter_container {
public:
    slam_parameter_container() {}
    ~slam_parameter_container() {}

    // virtual void LoadParam() = 0;
    void CheckParam()
    {
        // check values
        CHECK_GT(f_localization_correction_limit, 0);
        CHECK_GT(f_scan_accumulation_condition_dist, 0);
        CHECK_GT(f_scan_accumulation_condition_rot, 0);
        CHECK_GT(f_scan_accumulation_condition_dist_localization, 0);
        CHECK_GT(f_scan_accumulation_condition_rot_localization, 0);
        CHECK_GT(n_scan_accumulation_max_num, 0);
        CHECK_GT(n_scan_accumulation_max_num_localization, 0);
        CHECK_GT(n_scan_accumulation_step_num_localization, 0);
        CHECK_GT(f_hit_probability, 0);
        CHECK_GT(f_miss_probability, 0);
        CHECK_GT(f_high_hit_probability, 0);
        CHECK_GT(f_high_miss_probability, 0);
        CHECK_GT(f_ceres_scan_matching_occupied_space_weight, 0);
        CHECK_GT(f_ceres_scan_matching_translation_weight, 0);
        CHECK_GT(f_ceres_scan_matching_rotation_weight, 0);
        CHECK_GT(f_min_match_ratio, 0);
        CHECK_GT(f_icp_success_rmse, 0);
        CHECK_GT(f_localization_correction_weight, 0);
        CHECK_GT(f_local_constraint_translation_weight, 0);
        CHECK_GT(f_local_constraint_rotation_weight, 0);
        CHECK_GT(f_global_constraint_translation_weight, 0);
        CHECK_GT(f_global_constraint_rotation_weight, 0);
        CHECK_GT(f_huber_loss_delta, 0);
        CHECK_GT(f_node_search_index_diff_min, 0);
        CHECK_GT(f_node_search_dist_diff_max, 0);
        CHECK_GT(n_loop_candidate_num, 0);
        CHECK_GT(n_fixed_frame_num, 0);
    }

public:
    bool b_save_icp_point_cloud;
    bool b_dense_point_cloud_map;
    bool b_save_posegraph_info;
    int n_fixed_frame_num;
    int n_scan_accumulation_max_num;
    int n_scan_accumulation_max_num_localization;
    int n_scan_accumulation_step_num_localization;
    int n_loop_candidate_num;
    int n_sliding_window_node_num;
    int n_confidence_offset_ratio;
    float f_scan_accumulation_condition_dist;
    float f_scan_accumulation_condition_rot;
    float f_scan_accumulation_condition_dist_localization;
    float f_scan_accumulation_condition_rot_localization;
    float f_localization_correction_limit;
    float f_localization_correction_limit_deg;
    float f_hit_probability;
    float f_miss_probability;
    float f_high_hit_probability;
    float f_high_miss_probability;
    float f_ceres_scan_matching_occupied_space_weight;
    float f_ceres_scan_matching_translation_weight;
    float f_ceres_scan_matching_rotation_weight;
    float f_node_search_index_diff_min;
    float f_node_search_dist_diff_max;
    float f_min_match_ratio;
    float f_icp_success_rmse;
    float f_localization_correction_weight;
    float f_local_constraint_translation_weight;
    float f_local_constraint_rotation_weight;
    float f_global_constraint_translation_weight;
    float f_global_constraint_rotation_weight;
    float f_huber_loss_delta;
    std::string map_path;
    std::string map_name;
    int n_scan_data_usage_interval;
    float f_out_map_resolution;
    float f_out_occ_thresh;
    float f_out_free_thresh;
};

}  // namespace SLAM2D
}  // namespace ANSWER
#endif