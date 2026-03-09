
/**
 * @class ceres_scan_matcher
 * @brief  Align scans with an existing map using Ceres solver.
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */

#ifndef CERES_SCAN_MATCHER_2D_H
#define CERES_SCAN_MATCHER_2D_H

#include "ceres/ceres.h"
#include "common/pose2d.h"
#include "common/scan2d.h"
#include "localmapper/localmap/grid2d.h"
#include "localmapper/localmap/localmap2d.h"

#include "Eigen/Core"
#include <memory>
#include <vector>

namespace NaviFra {
namespace SLAM2D {
class CeresScanMatcher {
public:
    CeresScanMatcher(
        const float ceres_scan_matching_occupied_space_weight_,
        const float ceres_scan_matching_translation_weight_,
        const float ceres_scan_matching_rotation_weight_);
    ~CeresScanMatcher();

    // Aligns 'point_cloud' within the 'grid' given an
    // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
    // 'summary'.
    void Match(
        const Eigen::Vector2d& target_translation,
        const Pose2D& initial_pose_estimate,
        const Scan2D& point_cloud,
        const Localmap2D* localmap,
        Pose2D* pose_estimate) const;

private:
    ceres::Solver::Options ceres_solver_options_;

    const float ceres_scan_matching_occupied_space_weight;
    const float ceres_scan_matching_translation_weight;
    const float ceres_scan_matching_rotation_weight;
};

}  // namespace SLAM2D
}  // namespace NaviFra
#endif  // CERES_SCAN_MATCHER_2D_H_
