#ifndef LOCALIZER_PARAM_H
#define LOCALIZER_PARAM_H
namespace NaviFra {
// namespace LOCALIZER2D
struct localizer_parameter_container {
    /* data */
public:
    localizer_parameter_container()
        : b_use_localization(true)
        , b_use_new_icp(false)
        , f_estimation_cond_dist_m(0.1f)
        , f_estimation_cond_deg(10.f)
        , f_estimation_cond_steer_deg(40.f)
        , f_init_search_dist_min(0.1f)
        , f_init_search_dist_max(0.5f)
        , f_init_matching_ratio_thres(0.05f)
        , f_matching_ratio_thres(0.7f)
        , f_matching_dist_max(0.3f)
        , f_matching_dist_min(0.05f)
        , f_localizer_not_use_ratio(0.f)
        , f_max_qr_distance(0.f)
        , f_diff_error_x_m(0.5f)
        , f_diff_error_y_m(0.5f)
        , f_diff_error_deg(30.0f)
        , f_diff_error_confidence(0.f)
        , f_del_odom_condition_dist(0.01f)
        , f_rotate_vel_condition_deg(1.f)
        , f_external_pos_expiration_time_s(1.0)
        , f_external_pos_expiration_dist_m(0.1)
        , f_external_pos_expiration_deg(10)
        , n_max_localize_iteration(10)
        , n_max_init_localize_iteration(30)
        , n_localize_error_ratio_th(40)
        , n_knn_search_num(1)
        , n_localize_check_condition(5)
        , n_localize_error_count_alram(5)
        , f_estimation_cond_acc_linear_mss(0.3)
        , f_estimation_cond_acc_angular_degss(10)
    {
#ifdef WIN32
#endif
    }

    ~localizer_parameter_container() {}

public:
    // bool
    bool b_use_localization;
    bool b_use_new_icp;
    // float
    float f_estimation_cond_dist_m;
    float f_estimation_cond_deg;
    float f_estimation_cond_steer_deg;

    float f_init_search_dist_min;
    float f_init_search_dist_max;
    float f_init_matching_ratio_thres;

    float f_matching_stop_thres;
    float f_matching_ratio_thres;
    float f_matching_dist_max;
    float f_matching_dist_min;
    float f_localizer_not_use_ratio;
    float f_max_qr_distance;

    float f_diff_error_x_m = 0.2;
    float f_diff_error_y_m = 0.2;
    float f_diff_error_deg = 10.0;
    float f_diff_error_confidence = 0.0;
    float f_del_odom_condition_dist;
    float f_rotate_vel_condition_deg;

    float f_external_pos_expiration_time_s;
    float f_external_pos_expiration_dist_m;
    float f_external_pos_expiration_deg;

    float f_estimation_cond_acc_linear_mss;
    float f_estimation_cond_acc_angular_degss;

    // int
    int n_max_localize_iteration;
    int n_max_init_localize_iteration;
    int n_localize_error_ratio_th;
    int n_knn_search_num;
    int n_localize_check_condition;
    int n_localize_error_count_alram;

    float f_min_dist_compare = 15;
    float f_max_dist_compare = 40;
    float f_ratio_compare = 3;
};

}  // namespace NaviFra
#endif