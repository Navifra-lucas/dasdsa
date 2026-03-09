#pragma once
#include "common/callbacks.h"
#include "common/pch.h"
#include "common/pose2d.h"
namespace ANSWER {
class ImuPoseEstimator2D {
public:
    ImuPoseEstimator2D();
    ~ImuPoseEstimator2D();

    void Initialize(const std::string &config_path);
    void SetInitialPose(const Pose2D &pose);
    void Predict(const Scan2D &scan, const double &dt);
    void Update(const Pose2D &pose, const Eigen::MatrixXd &covariance);
    const Pose2D GetPose();
}
}  // namespace ANSWER