#pragma once
#include "ceres/autodiff_cost_function.h"
#include "ceres/ceres.h"
#include "ceres/cost_function.h"
#include "common/pch.h"
#include "common/pose2d.h"
#include "common/scan2d.h"
#include "logger/logger.h"

using Point2Df = std::pair<double, float>;
using Point2DfSet = std::vector<Point2Df>;
using SelectedIndex = std::vector<size_t>;
// Define the cost function.

namespace ANSWER {
struct CircleCostFunctor {
    CircleCostFunctor(double x, double y, double radius)
        : x_(x)
        , y_(y)
        , radius_(radius)
    {
    }

    template <typename T>
    bool operator()(const T *const center, T *residual) const
    {
        // center[0] and center[1] are the parameters to be optimized (circle
        // center).
        T dx = T(x_) - center[0];
        T dy = T(y_) - center[1];
        // Compute the distance from the point to the center.
        T distance = (dx * dx + dy * dy);
        // Residual is the difference between the actual distance and the fixed
        // radius.
        residual[0] = distance - T(radius_) * T(radius_);
        return true;
    }

private:
    const double x_, y_;
    const double radius_;
};

class IntensityFilter {
private:
    Eigen::VectorXd EstimateCenterOfReflector(
        const std::vector<Point2DfSet> &candidates);

    std::vector<std::vector<size_t>> ClusterPoints(
        const Scan2D &msg, SelectedIndex &selected);
    SelectedIndex ExtractMeaningfulRawDataIndex(const Scan2D &msg);
    bool NormalizeIntensity(Scan2D &msg);
    const std::vector<Point2DfSet> ConvertClusterDataToPointCloud(
        const Scan2D &msg, const std::vector<SelectedIndex> &selectedset);

    float f_min_distance_cond_;
    float f_max_distance_cond_;
    float f_min_intensity_cond_;
    float f_reflector_diameter_;
    int n_intensity_local_max_neighboor_num_;

public:
    IntensityFilter(/* args */);
    ~IntensityFilter();

    bool FilterIntensities(
        const Scan2D &msg, Eigen::VectorXd &res,
        const float &f_reflector_distance_m_register_tolerance);
    void SetParameters(
        const float &min_distance_cond, const float &max_distance_cond,
        const float &min_intensity_cond, const float &reflector_diameter,
        const int &intensity_local_max_neighboor_num)
    {
        f_min_distance_cond_ = min_distance_cond;
        f_max_distance_cond_ = max_distance_cond;
        f_min_intensity_cond_ = min_intensity_cond;
        f_reflector_diameter_ = reflector_diameter;
        n_intensity_local_max_neighboor_num_ =
            intensity_local_max_neighboor_num;
        LOG_INFO(
            "min dist {} , max dist {} , min intensity {} , reflector diameter {} intensity local max neighboor num {}",
            f_min_distance_cond_, f_max_distance_cond_, f_min_intensity_cond_,
            f_reflector_diameter_, n_intensity_local_max_neighboor_num_);
    }
};

}  // namespace ANSWER