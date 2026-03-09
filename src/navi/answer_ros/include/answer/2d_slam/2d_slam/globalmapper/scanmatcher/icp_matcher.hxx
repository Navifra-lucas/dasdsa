/**
 * @class icp matcher
 * @brief  match two point cloud using libpointmatcher(icp) for loop closing.
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */
#ifndef POINT_MATCHER_H
#define POINT_MATCHER_H
#include "2d_slam/common/icp_result.h"
#include "2d_slam/common/slam_parameter_container.h"
#include "common/pose2d.h"
#include "glog/logging.h"

#include <chrono>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
using namespace std;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

namespace ANSWER {
namespace SLAM2D {
class ICPMatcher {
private:
    /* data */

    PM::ICP icp_mapping_;
    PM::ICP icp_templete_matching_;
    PM::ICPSequence icp_localization_;
    PM::ICPSequence icp_localization_recovery_;
    // unordered_map<string, float> *slam_param_;
    const slam_parameter_container* slam_param_;
    string s_path_;
    DP reference_;
    bool b_reference_set_ = false;
    bool b_is_localization_ = false;
    bool b_is_initial_pose_ = false;
    bool b_recovery_ = false;
    float f_initial_matching_distance_ = 0.0f;
    std::shared_ptr<PM::DataPointsFilter> maxDensitySubsample_;
    std::shared_ptr<PM::DataPointsFilter> densityFilter_;
    std::shared_ptr<PM::DataPointsFilter> any_filter1;
    std::shared_ptr<PM::DataPointsFilter> any_filter2;
    std::vector<std::shared_ptr<PM::DataPointsFilter>> vec_data_filters_;

public:
    ICPMatcher(/* args */);
    ~ICPMatcher();

    DP GenerateData(const PointCloud2D& reference);
    const ICPResult DoICP(const Pose2D& initial_guess, const PointCloud2D& reference, const PointCloud2D& dynamic);

    const ICPResult DoTempleteMatching(const Pose2D& initial_guess, const PointCloud2D& reference, const PointCloud2D& dynamic);

    const ICPResult DoLocalization(const PointCloud2D& dynamic, const bool& b_initial_pose);
    const ICPResult ValidateICPResult(PM::ICP* icp, const DP& ref, const DP& data_out, const float& match_raito);

    void SetReferenceData(const PointCloud2D& reference);
    const DP& GetReferenceData() { return reference_; }

    void SetParam(const slam_parameter_container* param, const bool localization = false);

    bool AlignSequence(const PointCloud2D& dynamic);
    void SaveMap()
    {
        if (reference_.getNbPoints() == 0)
            return;
        else {
            if (vec_data_filters_.size() != 0) {
                for (auto anyfilter : vec_data_filters_) {
                    // NLOG(info) << "apply filter..";
                    reference_ = anyfilter->filter(reference_);
                }
            }
            static int count;
            reference_.save(s_path_ + to_string(count++) + "map.vtk");
        }
    }
};

}  // namespace SLAM2D
}  // namespace ANSWER

#endif