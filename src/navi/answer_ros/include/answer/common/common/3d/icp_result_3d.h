#pragma once
#include "common/answer_utils.h"
#include "common/pch.h"

#include <chrono>
#include <numeric>
#include <sstream>
#include <string>

namespace ANSWER {
struct IMatchingResult3D {
private:
    bool b_valid;
    float f_match_ratio;
    float f_rmse;
    Pose3D correction;
    float computation_milliseconds = std::numeric_limits<float>::max();
    std::chrono::system_clock::time_point time_stamp;

public:
    IMatchingResult3D()
        : b_valid(false)
        , f_match_ratio(0.f)
        , f_rmse(0.f)

    {
        time_stamp = std::chrono::system_clock::now();
        correction = Pose3D();
    }
    ~IMatchingResult3D() {}

    //
    void SetValid(const bool valid) { b_valid = valid; }
    const bool GetValid() const { return b_valid; }
    //
    void SetMatchRatio(const float match_ratio) { f_match_ratio = match_ratio; }
    const float GetMatchRatio() const { return f_match_ratio; }
    //
    void SetRMSE(const float rmse) { f_rmse = rmse; }
    const float GetRMSE() const { return f_rmse; }
    //
    void SetTimeStamp(const std::chrono::system_clock::time_point time)
    {
        time_stamp = time;
    }
    const std::chrono::system_clock::time_point GetTimeStamp() const
    {
        return time_stamp;
    }
    //
    void SetCorrection(const Pose3D &correction)
    {
        this->correction = correction;
    }
    Pose3D GetCorrection() const { return correction; }
    //
    void SetComputationTime(const float time)
    {
        computation_milliseconds = time;
    }
    const float GetComputationTime() const { return computation_milliseconds; }

    // virtual std::string PrintResult() = 0;
};
namespace SLAM3D {
struct ICPResult3D : public IMatchingResult3D {
public:
    ICPResult3D()
        : target_index(0)
        , source_index(0)
        , n_iteration(0)
    {
    }
    ICPResult3D(const bool valid, const float match_ratio, const float rmse)
        : target_index(0)
        , source_index(0)
        , n_iteration(0)
    {
        SetValid(valid);
        SetMatchRatio(match_ratio);
        SetRMSE(rmse);
    }

private:
    int target_index;
    int source_index;
    int n_iteration;

public:
    void SetTargetIndex(const int index) { target_index = index; }
    const int GetTargetIndex() const { return target_index; }
    void SetSourceIndex(const int index) { source_index = index; }
    const int GetSourceIndex() const { return source_index; }

    void SetIteration(const int iteration) { n_iteration = iteration; }
    const int GetIteration() const { return n_iteration; }
    // std::string PrintResult()
    // {
    //     std::stringstream ss;
    //     ss << "[" << target_index << " vs " << source_index << "]"
    //        << " ICP Result: corrected " << GetCorrection().GetPose().x() << "
    //        "
    //        << GetCorrection().GetPose().y() << " "
    //        << GetCorrection().GetPose().z() * 180. / M_PI << " confidence "
    //        << GetMatchRatio() << " rmse " << GetRMSE() << " time "
    //        << GetComputationTime() << " valid " << GetValid();
    //     if (GetValid()) {
    //         return answer_utils::ColorizeString(ss.str(),
    //         answer_utils::GREEN);
    //     }
    //     else {
    //         return answer_utils::ColorizeString(ss.str(), answer_utils::RED);
    //     }
    // }
};
}  // namespace SLAM3D
namespace LOCALIZATION3D {
struct LocalizeResult : public IMatchingResult3D {
public:
    LocalizeResult()
    {
        f_search_dist = 0.f;
        n_iteration = 0;
        localizer_mode_ = "lidar";
    }
    ~LocalizeResult() {}

    // std::string PrintResult()
    // {
    //     std::stringstream ss;
    //     ss << "Localize Result: corrected " << GetCorrection().GetPose().x()
    //        << " " << GetCorrection().GetPose().y() << " "
    //        << GetCorrection().GetPose().z() * 180. / M_PI << " confidence "
    //        << GetMatchRatio() << " rmse " << GetRMSE() << " time "
    //        << GetComputationTime() << " valid " << GetValid();
    //     return ss.str();
    // }

private:
    float f_search_dist;
    int n_iteration;
    PointCloud2D current_data;
    std::string localizer_mode_;
    float debug_matching_ratio = 0.f;
    std::string solver_type_;

public:
    void SetSearchDistance(const float search_dist)
    {
        f_search_dist = search_dist;
    }
    const float GetSearchDistance() const { return f_search_dist; }
    void SetIteration(const int iteration) { n_iteration = iteration; }
    const int GetIteration() const { return n_iteration; }
    void SetCurrentData(const PointCloud2D &data) { current_data = data; }
    const PointCloud2D GetCurrentData() const { return current_data; }
    void SetLocalizerMode(const std::string &mode) { localizer_mode_ = mode; }
    const std::string GetLocalizerMode() const { return localizer_mode_; }
    void SetDebugMatchingRatio(const float ratio)
    {
        debug_matching_ratio = ratio;
    }
    float GetDebugMatchingRatio() const { return debug_matching_ratio; }
    void SetSolverType(const std::string &type) { solver_type_ = type; }
    const std::string GetSolverType() const { return solver_type_; }
};

}  // namespace LOCALIZATION3D
}  // namespace ANSWER