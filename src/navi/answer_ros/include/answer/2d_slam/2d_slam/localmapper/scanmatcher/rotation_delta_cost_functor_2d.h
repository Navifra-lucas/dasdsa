/**
* @class rotation delta cost functor
* @brief Computes the cost of rotating 'pose' to 'target_angle'. Cost increases with
         the solution's distance from 'target_angle'.
* @author logan (donghak lee)
* contact : donghak.lee@navifra.com
*/
#ifndef ROTATION_DELTA_COST_FUNCTOR_2D_H
#define ROTATION_DELTA_COST_FUNCTOR_2D_H

#include "ceres/ceres.h"

#include "Eigen/Core"

namespace ANSWER {
namespace SLAM2D {

class RotationDeltaCostFunctor2D {
public:
    static ceres::CostFunction* CreateAutoDiffCostFunction(const double scaling_factor, const double target_angle)
    {
        return new ceres::AutoDiffCostFunction<RotationDeltaCostFunctor2D, 1 /* residuals */, 3 /* pose variables */>(
            new RotationDeltaCostFunctor2D(scaling_factor, target_angle));
    }

    template <typename T>
    bool operator()(const T* const pose, T* residual) const
    {
        residual[0] = scaling_factor_ * (pose[2] - angle_);
        return true;
    }

private:
    explicit RotationDeltaCostFunctor2D(const double scaling_factor, const double target_angle)
        : scaling_factor_(scaling_factor)
        , angle_(target_angle)
    {
    }

    RotationDeltaCostFunctor2D(const RotationDeltaCostFunctor2D&) = delete;
    RotationDeltaCostFunctor2D& operator=(const RotationDeltaCostFunctor2D&) = delete;

    const double scaling_factor_;
    const double angle_;
};
}  // namespace SLAM2D
}  // namespace ANSWER

#endif  // ROTATION_DELTA_COST_FUNCTOR_2D_H
