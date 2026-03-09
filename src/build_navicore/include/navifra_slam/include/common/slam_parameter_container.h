/**
 * @class parameter box
 * @brief various parameters for slam
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */
#ifndef SLAM_PARAM_H
#define SLAM_PARAM_H
#include "common/slam_utils.h"
#include "glog/logging.h"
#include "opencv2/opencv.hpp"
#include "util/logger.hpp"

#include <boost/filesystem.hpp>

#include <filesystem>

using namespace std;
namespace NaviFra {
namespace SLAM2D {
struct slam_parameter_container {
public:
  slam_parameter_container()
      : b_save_icp_point_cloud(false), b_dense_point_cloud_map(false),
        b_save_posegraph_info(false), n_fixed_frame_num(0),
        n_scan_accumulation_max_num(0),
        n_scan_accumulation_max_num_localizaiton(0),
        n_scan_accumulation_step_num_localizaiton(0), n_loop_candidate_num(0),
        n_sliding_window_node_num(0), n_confidence_offset_ratio(0),
        f_scan_accumulation_condition_dist(0.f),
        f_scan_accumulation_condition_rot(0.f),
        f_scan_accumulation_condition_dist_localization(0.f),
        f_scan_accumulation_condition_rot_localization(0.f),
        f_localization_correction_limit(0.f),
        f_localization_correction_limit_deg(0.f), f_hit_probability(0.f),
        f_miss_probability(0.f), f_high_hit_probability(0.f),
        f_high_miss_probability(0.f),
        f_ceres_scan_matching_occupied_space_weight(0.f),
        f_ceres_scan_matching_translation_weight(0.f),
        f_ceres_scan_matching_rotation_weight(0.f),
        f_node_search_index_diff_min(0.f), f_node_search_dist_diff_max(0.f),
        f_min_match_ratio(0.f), f_icp_success_rmse(0.f),
        f_localization_correction_weight(0.f),
        f_local_constraint_translation_weight(0.f),
        f_local_constraint_rotation_weight(0.f),
        f_global_constraint_translation_weight(0.f),
        f_global_constraint_rotation_weight(0.f), f_huber_loss_delta(0.f) {
    std::string path;
    for (auto path_candidate : slam_utils::GetPackagePath()) {
      NLOG(info) << path_candidate;
      if (boost::filesystem::exists(path_candidate) == true) {
        path = path_candidate + "slam.yaml";
        NLOG(info) << "parameter file " << path;
        break;
      }
    }
    CHECK_NE(path.empty(), true)
        << "package path not defined or parameter files not exist";

    // NLOG(info) << "parameter path : " << path;
    // NLOG(info) << "std parameter path : " << std::filesystem::current_path();
    cv::FileStorage fs(path, cv::FileStorage::READ);
    CHECK_NE(fs.isOpened(), false);

    fs["localization_correction_limit"] >> f_localization_correction_limit;
    fs["localization_correction_limit_deg"] >>
        f_localization_correction_limit_deg;
    fs["scan_accumulation_condition_dist"] >>
        f_scan_accumulation_condition_dist;
    fs["scan_accumulation_condition_rot"] >> f_scan_accumulation_condition_rot;
    fs["scan_accumulation_condition_dist_localization"] >>
        f_scan_accumulation_condition_dist_localization;
    fs["scan_accumulation_condition_rot_localization"] >>
        f_scan_accumulation_condition_rot_localization;
    fs["scan_accumulation_max_num"] >> n_scan_accumulation_max_num;
    fs["scan_accumulation_max_num_localization"] >>
        n_scan_accumulation_max_num_localizaiton;
    fs["scan_accumulation_step_num_localization"] >>
        n_scan_accumulation_step_num_localizaiton;
    fs["hit_probability"] >> f_hit_probability;
    fs["miss_probability"] >> f_miss_probability;
    fs["high_hit_probability"] >> f_high_hit_probability;
    fs["high_miss_probability"] >> f_high_miss_probability;

    fs["ceres_scan_matching_occupied_space_weight"] >>
        f_ceres_scan_matching_occupied_space_weight;
    fs["ceres_scan_matching_translation_weight"] >>
        f_ceres_scan_matching_translation_weight;
    fs["ceres_scan_matching_rotation_weight"] >>
        f_ceres_scan_matching_rotation_weight;

    fs["min_match_ratio"] >> f_min_match_ratio;
    fs["icp_success_rmse"] >> f_icp_success_rmse;
    fs["localization_correction_weight"] >> f_localization_correction_weight;
    fs["local_constraint_translation_weight"] >>
        f_local_constraint_translation_weight;
    fs["local_constraint_rotation_weight"] >>
        f_local_constraint_rotation_weight;
    fs["global_constraint_translation_weight"] >>
        f_global_constraint_translation_weight;
    fs["global_constraint_rotation_weight"] >>
        f_global_constraint_rotation_weight;
    fs["huber_loss_delta"] >> f_huber_loss_delta;

    fs["node_search_index_diff_min"] >> f_node_search_index_diff_min;
    fs["node_search_dist_diff_max"] >> f_node_search_dist_diff_max;

    fs["loop_candidate_num"] >> n_loop_candidate_num;
    fs["sliding_window_node_num"] >> n_sliding_window_node_num;
    fs["confidence_offset_ratio"] >> n_confidence_offset_ratio;
    fs["fixed_frame_num"] >> n_fixed_frame_num;
    fs["dense_point_cloud_map"] >> b_dense_point_cloud_map;
    fs["save_posegraph_info"] >> b_save_posegraph_info;
    fs["save_icp_point_cloud"] >> b_save_icp_point_cloud;

    // check values
    CHECK_GT(f_localization_correction_limit, 0);
    CHECK_GT(f_scan_accumulation_condition_dist, 0);
    CHECK_GT(f_scan_accumulation_condition_rot, 0);
    CHECK_GT(f_scan_accumulation_condition_dist_localization, 0);
    CHECK_GT(f_scan_accumulation_condition_rot_localization, 0);
    CHECK_GT(n_scan_accumulation_max_num, 0);
    CHECK_GT(n_scan_accumulation_max_num_localizaiton, 0);
    CHECK_GT(n_scan_accumulation_step_num_localizaiton, 0);
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
    // CHECK_GT(n_sliding_window_node_num, 0);
    CHECK_GT(n_fixed_frame_num, 0);
  }
  ~slam_parameter_container() {}
  bool b_save_icp_point_cloud;
  bool b_dense_point_cloud_map;
  bool b_save_posegraph_info;
  int n_fixed_frame_num;
  int n_scan_accumulation_max_num;
  int n_scan_accumulation_max_num_localizaiton;
  int n_scan_accumulation_step_num_localizaiton;
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
};

} // namespace SLAM2D
} // namespace NaviFra
#endif