#pragma once

#include "2d_localizer/common/localizer_parameter_container.h"
#include "2d_localizer/matcher/icp2d.h"
#include "2d_localizer/matcher/reflector_matcher.h"
#include "2d_localizer/matcher/reflector_position_estimator.h"
#include "2d_slam/globalmapper/mapper.h"
#include "2d_slam/localmapper/localmap/grid2d.h"
#include "2d_slam/localmapper/scanmatcher/ceres_scan_matcher.h"
#include "common/callbacks.h"
#include "common/icp_result.h"
#include "common/intensity_filter.h"
#include "common/pch.h"
#include "common/pose2d.h"
#include "common/scan2d.h"
#include "common/voxel_filter.h"
#include "logger/logger.h"

#include <iostream>
#include <memory>
#include <thread>
#include <vector>

using namespace std;

namespace ANSWER {

enum LOCALIZER_STATUS
{
    INIT_POSE = 0,
    LOCALIZATION,
};

struct SamplingMatcher {
    int sample_num;
    float sample_max_dist;
    float sample_max_angle;
    bool b_use_uniform_sampling;
    float sample_dist_resolution;
    float sample_angle_resolution;
};

class Localizer {
private:
    /* data */

    Pose2D o_prev_robot_pos_;  // 이전 로봇 pos (ICP결과)
    Pose2D o_robot_pos_;  // 현재 로봇 pos (지난 ICP결과 + Odometry 쌓일때마다
                          // 갱신)
    Pose2D o_vel_pos_;  // 현재 로봇 속도 (ICP 실행 전 각속도 검사)

    Pose2D o_prev_odom_;  // 이전 로봇 Odom
    Pose2D
        o_current_odom_;  // 현재 로봇 Odom (오돔 차이 구해서 o_robot_pos_ 갱신)
    Pose2D o_delta_odom_;  // 전프레임과 Odom 차이 (ICP 실행 전 위치차이 검사)

    PointCloud2D
        vec_current_data_;  // scan_cloud data from lidar_merger (laser frame)

    PointCloud2D reflectors_;
    PointCloud2D reflector_map_;

    ReflectorMatcher reflector_matcher_;
    IntensityFilter intensity_filter_;
    std::vector<float> intensities_;

    // std::vector<std::unique_ptr<ReflectorPositionEstimator>>
    //     reflector_estimators_;

    std::unique_ptr<ReflectorPositionEstimator> reflector_estimators_;

    std::thread localization_thread_;
    std::thread robot_pose_pub_thread_;
    LocalizeResult st_localize_result_;

    // shared_ptr<PointToPlaneICP> localization_matcher_;
    // shared_ptr<PointToPlaneICP> initial_pose_matcher_;
    shared_ptr<localizer_parameter_container> localizer_param_;

    std::mutex mtx_lock_robot_pose_;
    std::mutex mtx_localizer_run_check_;
    std::mutex mtx_localizer_error_code_;
    std::mutex mtx_set_lidar_data_;
    // for lidar odometry
    unique_ptr<SLAM2D::CeresScanMatcher> scan_matcher_;
    shared_ptr<SLAM2D::Localmap2D> current_localmap_;
    Pose2D map_to_robot_;
    Pose2D last_map_to_robot_;
    Pose2D requested_init_pose_;
    Pose2D corrected_transform_;
    bool b_set_localmap_ = false;
    bool b_initialized_ = false;
    bool b_set_map_ = false;
    bool b_pub_pose_ = false;
    bool b_find_init_pose_;
    bool b_init_odom_;
    bool b_register_reflector_;
    bool reflector_based_localization_;
    float f_last_confidence_ = 0.f;
    float f_reflector_distance_m_register_tolerance_;
    float f_reflector_visualize_distance_threshold_;
    int n_reflector_detected_count_condition_;
    int n_reflector_time_expiration_;
    bool b_use_reflector_based_localization_as_init_guess_;
    bool b_use_reflector_;
    bool b_use_external_pos_;
    SamplingMatcher sampling_matcher_;

    std::chrono::system_clock::time_point
        odom_time_;  // When Odom data is entered
    std::chrono::system_clock::time_point
        scan_time_;  // When scan_cloud data is entered

    // ----------------------- Parameters  --------------------------------

    bool b_localization_run_ = false;  // 현재 Localize thread가 실행중인지

    float f_acc_dist_m_ = 0.f;  // 오도메트리에 의해 추정된 로봇의 이동거리
                                // (Between ICP 수행단위간) why acc?
    float f_acc_deg_ = 0.f;  // 오도메트리에 의해 추정된 로봇의 회전량 Unit :
                             // Deg (Between ICP 수행단위간)
    float f_acc_steer_deg_ = 0.f;

    float f_external_acc_dist_m_ =
        0.f;  // 외부에서 받은 로봇의 이동거리 (Between ICP 수행단위간)
    float f_external_acc_deg_ =
        0.f;  // 외부에서 받은 로봇의 회전량 Unit : Deg (Between ICP 수행단위간)

    float f_initial_condition_dist_ = 0.f;  // For QR localization
    float f_initial_condition_deg_ = 0.f;  // For QR localization
    float f_initial_condition_steer_deg_ = 0.f;  // For QR localization

    int n_error_count_ = 0;  // ICP large gap error count

    // ----------------------- Not used  --------------------------------
    // bool b_init_odom_ = false;  // Not used
    int n_localize_result_error_code_ = 0;  // Not used
    float f_rotate_vel_condition_deg_ = 1.0f;  // Not used
    float f_del_odom_condition_dist_ = 0.01f;  // Not used

    // std::chrono::system_clock::time_point navi_time_;
    // std::chrono::system_clock::time_point icp_time_;
    // std::chrono::system_clock::time_point init_pose_time_;
    // std::chrono::system_clock::time_point slam_time_;

    // ----------------------- Main process --------------------------------

    void LocalizeThread();
    bool CheckLocalizeCondition();
    void ResetLocalizationCondition();

    std::mutex mtx_lock_external_;
    std::mutex mtx_lock_findpose_;
    std::mutex mtx_lock_error_code_;
    std::mutex mtx_lock_register_reflector_;
    std::chrono::system_clock::time_point external_pose_time_;
    std::chrono::system_clock::time_point reflector_register_time_;
    TimeChecker tc_;

    std::shared_ptr<icp2d> o_icp_;
    void OnLiDAROdometry(const PointCloud2D &vec_current_data);
    void OffLiDAROdometry();

private:
    // ----------------------- Main methods --------------------------------

    void Initialize();
    void FindInitPose(const Scan2D &vec_current_data);
    bool DoLocalize(const Scan2D &vec_current_data);

    // ----------------------- Get Set -----------------------------------

    void SetVel(const Pose2D &o_vel_pose);  // From Motor driver
    void SetOdometry(const Pose2D &o_odom_pose);  // From Motor driver
    void SetSteerDeg(const float &f_deg);  // From Motor driver

    bool UpdateParameters();

    void MatchBasedSampling(
        const bool &is_random_sampling, const int &sampling_num,
        const float &sample_max_dist, const float &sample_max_angle,
        const float &sample_dist_resolution,
        const float &sample_angle_resolution);

    shared_ptr<localizer_parameter_container> GetParameter()
    {
        return localizer_param_;
    }
    // ----------------------- Not used -----------------------------------

    void AddOdometry(const Pose2D &prev_to_current);
    void LocalizeOnce();

    void StartLocalization();
    void TerminateLocalization();
    void PublishRobotPose();

    bool SetJSONMap(const Poco::JSON::Object::Ptr map);  // From Map server
    bool SetMap(const PointCloud2D &vec_icp_db);  // From Map server
    void SetInitPose(const Pose2D &initpose);
    void TriggerFindPose(const bool &b_trigger);
    void FindPose(const Pose2D &initpose, const bool &b_trigger);
    void ReadAndUpdateReflectorMap();
    void RegisterReflectorTrigger();
    void SwitchReflectorMode();
    void RegisterReflector(const Eigen::VectorXd &reflectors);
    void GenerateReflectorMapFile(
        const std::string &reflector_info,
        std::unique_ptr<ReflectorPositionEstimator> &reflector_estimators,
        PointCloud2D &reflector_map);
    // void
    Poco::JSON::Object::Ptr GenerateAnswerReport(
        const Pose2D &map_to_old, const Pose2D &icp_correction,
        const std::tuple<float, float, float, float> &section_matching_ratio,
        const float &elasped_time, const LocalizeResult &result);
    bool CheckStatus(const ANSWER::STATUS &status);
    bool CheckSetMap();
    bool CheckInitialized();
    bool CheckExternalPose();

public:
    Localizer();
    ~Localizer();

    const LocalizeResult GetLocalizeResult();
    void ResetExternalPosTime();
    float GetExternalPosTime();
    bool GetExternalPosDistDeg();
    void SetExternalPose(
        const Pose2D &o_external_pose, const std::string &source);

    const int GetLocalizeErrorCode()
    {
        std::lock_guard<std::mutex> lock(mtx_localizer_error_code_);
        return n_localize_result_error_code_;
    }
    void ErrorClear()
    {
        AnswerStatus::GetInstance().SetAlarm(ALARM::NONE);
        {
            std::lock_guard<std::mutex> lock(mtx_localizer_error_code_);
            n_localize_result_error_code_ = 0;
        }
    }
    void SetAlarm(const ALARM &alarm)
    {
        AnswerStatus::GetInstance().SetAlarm(alarm);
        {
            std::lock_guard<std::mutex> lock(mtx_localizer_error_code_);
            n_localize_result_error_code_ = static_cast<int>(alarm);
        }
    }
    void InitErrorCount();
    const Pose2D GetRobotPose();
    void SetRobotPose(const Pose2D &o_robot_pos);

    const LocalizeResult SamplingBasedFindPose(
        const Pose2D map_to_robot, const PointCloud2D robot_to_scan, icp2d *icp,
        localizer_parameter_container *param, const bool &is_random_sampling,
        const int &sample_num, const float &sample_max_dist,
        const float &sample_max_angle, const float &sample_dist_resolution,
        const float &sample_angle_resolution);
};
}  // namespace ANSWER