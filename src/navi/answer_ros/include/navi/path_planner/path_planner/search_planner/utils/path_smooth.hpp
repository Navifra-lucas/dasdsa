/*
 * @file	: path_smooth.hpp
 * @date	: Feb 7, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: util for global planner
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef GLOBAL_PLANNER_PATH_SMOOTH_HPP_
#define GLOBAL_PLANNER_PATH_SMOOTH_HPP_

#include <vector>
#include <utility>

#include "utils/pose.hpp"
#include "utils/grid_map/map_info.hpp"
#include "utils/grid_map/grid_struct.hpp"
#include "utils/grid_map/grid_map_calculator.hpp"

namespace NVFR {

class PathSmoother
{
public:
  struct TrajInfo_t {
    double d_target_spd;
    double d_max_curv;
  };

  struct SmoothInfo_t {
    //
  };

  PathSmoother();
  virtual ~PathSmoother() = default;

  bool MakeTrajectory(Path&, const TrajInfo_t&) const;

  bool MakeSmoothPath(Path&, const SmoothInfo_t&) const;

private:

};

class CubicSpline {
public:
    void fit(const std::vector<double>& x, const std::vector<double>& y);
    double interpolate(double x_query) const;

private:
    std::vector<double> x_, a_, b_, c_, d_;
    int findInterval(double x_query) const;
};

class BSpline {
  using PointType = std::pair<double,double>;
public:
  BSpline(const Path& path);
  Path smoother(int degree) const;

private:
  std::vector<PointType> vec_point_;
  double calcN(int i, int k, double u, const std::vector<double>& knots) const;
  PointType bspline(double u, int degree, const std::vector<PointType>& vec_point, const std::vector<double>& knots) const;
};

} // namespace NVFR

#endif
