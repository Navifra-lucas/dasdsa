#pragma once
#include "glog/logging.h"

#include <limits>

struct localizer_parameter_container {
public:
    localizer_parameter_container()
        : b_use_localization(false)
        , b_use_reflector(false)
        , b_use_weighted_matching(false)
        , b_use_sampling_init_matching(false)
        , b_use_lidar_odometry(false)
        , n_knn_search_num(-1)
        , n_max_localize_iteration(-1)
        , n_max_find_init_iteration(-1)
        , n_localize_error_count_alram(-1)
        , f_init_search_max_dist(-1.f)
        , f_matching_dist_max(-1.f)
        , f_matching_dist_min(-1.f)
        , f_matching_dist_coarse(-1.f)
        , f_matching_ratio_thres(-1.f)
        , f_localizer_not_use_ratio(-1.f)
        , f_external_pos_expiration_dist_m(-1.f)
        , f_external_pos_expiration_deg(-1.f)
        , f_external_pos_timeout_sec(5.0f)
        , f_diff_error_x_m(std::numeric_limits<float>::max())
        , f_diff_error_y_m(std::numeric_limits<float>::max())
        , f_diff_error_deg(std::numeric_limits<float>::max())
        , f_estimation_cond_dist_m(0.f)
        , f_estimation_cond_deg(0.f)
        , f_estimation_cond_steer_deg(0.f)
        , f_large_correction_ignore_x(0.1)
        , f_large_correction_ignore_y(0.1)
        , f_large_correction_ignore_t(3)
        , f_inout_cos_angle_threshold(-1.f)
        , s_kernel_name("huber")
        , s_matcher_solver("svd")
    {
    }
    ~localizer_parameter_container() {}

    bool b_use_localization;
    bool b_use_reflector;
    bool b_use_weighted_matching;
    bool b_use_sampling_init_matching;
    bool b_use_lidar_odometry;
    int n_knn_search_num;
    int n_max_localize_iteration;
    int n_max_find_init_iteration;
    int n_localize_error_count_alram;
    float f_init_search_max_dist;
    float f_matching_dist_max;
    float f_matching_dist_min;
    float f_matching_dist_coarse;
    float f_matching_ratio_thres;
    float f_localizer_not_use_ratio;
    float f_external_pos_expiration_dist_m;
    float f_external_pos_expiration_deg;
    float f_external_pos_timeout_sec;
    float f_diff_error_x_m;
    float f_diff_error_y_m;
    float f_diff_error_deg;
    float f_estimation_cond_dist_m;
    float f_estimation_cond_deg;
    float f_estimation_cond_steer_deg;
    float f_large_correction_ignore_x;
    float f_large_correction_ignore_y;
    float f_large_correction_ignore_t;
    float f_inout_cos_angle_threshold;
    std::string s_kernel_name;
    std::string s_matcher_solver;

    bool CheckParam()
    {
        CHECK_GT(n_max_localize_iteration, 0);
        CHECK_GT(n_max_find_init_iteration, 0);
        CHECK_GT(n_knn_search_num, 0);
        CHECK_GT(n_localize_error_count_alram, 0);
        CHECK_GT(f_init_search_max_dist, 0);
        CHECK_GT(f_matching_dist_max, 0);
        CHECK_GT(f_matching_dist_min, 0);
        CHECK_GT(f_matching_dist_coarse, 0);
        CHECK_GT(f_matching_ratio_thres, 0);
        CHECK_GE(f_localizer_not_use_ratio, 0);
        CHECK_GE(f_external_pos_expiration_dist_m, 0);
        CHECK_GE(f_external_pos_expiration_deg, 0);
        CHECK_GE(f_estimation_cond_dist_m, 0);
        CHECK_GE(f_estimation_cond_deg, 0);
        CHECK_GE(f_estimation_cond_steer_deg, 0);
        return true;
    }
};