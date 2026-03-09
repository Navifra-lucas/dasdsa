
/*
 * @brief Localizer3D class
 * @details This class is used to localize the robot in 3D space.
 * @note can be loosely coupled with wheel odometry and imu odometry
 * @author donghak.lee@navifra.com
 * @date 2023-10-01
 */
#pragma once
#include "3d_localizer/common/localizer3d_parameter_container.h"
//#include "common/3d/pose3d.h"
#include "common/3d/preprocessor.h"
#include "common/3d/scan3d.h"
#include "common/answer_utils.h"
#include "common/callbacks.h"
#include "common/ceres_util/normalize_angle.h"
#include "common/configurator.h"
#include "common/pch.h"
#include "common/pose2d.h"

namespace ANSWER {
class Localizer3D {
private:
    /* data */
    void DoLocalize(const Scan3D &frame);
    bool LoadParam();

    Localizer3DParameterContainer config_;
    Preprocessor preprocessor_;
    std::mutex mtx_lock_robot_pose_;
    std::thread robot_pose_pub_thread_;

    Pose3D robot_pose_;
    Pose3D odometry_;
    Pose3D prev_odometry_;

    bool initialized_;
    bool set_map_;
    bool init_odom_;
    bool pub_pose_;

    float acc_dist_m_;
    float acc_rot_deg_;

    // const Eigen::Isometry3d ConvertSE3fToIsometry3d(const Pose3D &pose)
    // {
    //     Eigen::Matrix3d rotation = pose.so3().matrix().cast<double>();
    //     Eigen::Vector3d translation = pose.translation().cast<double>();
    //     Eigen::Isometry3d isometry;
    //     isometry.translation() = translation;
    //     isometry.linear() = rotation;
    //     return isometry;
    // }

public:
    Localizer3D();
    ~Localizer3D();

    void StartLocalization();
    void TerminateLocalization();
    bool Initialize();
    void SetOdometry2D(const Pose2D &odom);
    void SetOdometry3D(const Pose3D &odom);
    bool SetMap(const PointCloud3D &map);
    const Pose3D GetRobotPose();
    void PublishRobotPose();
};
}  // namespace ANSWER