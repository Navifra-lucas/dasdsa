
/*
 * @file	: robot_config.hpp
 * @date	: Feb. 24, 2025
 * @author	: "HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: robot configuration
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_ROBOT_CONFIG_HPP_
#define NAVIFRA_ROBOT_CONFIG_HPP_

#include <memory>
#include <vector>

#include "utils/motion_planner_type_list.hpp"
#include "utils/pose.hpp"

namespace NVFR {

class RobotConfig
{
public:
  using Ptr = std::shared_ptr<RobotConfig>;
  using ConstPtr = std::shared_ptr<const RobotConfig>;

  RobotConfig() {};
  ~RobotConfig() {};

  MPTL::ROBOT_SHAPE GetShape() const { return e_robot_shape_; };
  std::vector<Pos> GetConfigPoints() const { return vec_config_points_; };
  std::vector<Pos> GetBoxSpinPoints() const { return vec_box_spin_points_; };
  double GetRadius() const { return d_radius_m_; };
  /*
    [0] : d_max_x_m
    [1] : d_min_x_m
    [2] : d_max_y_m
    [3] : d_min_y_m
  */
  std::vector<double> GetBoxConfig() const { return std::vector<double>({d_max_x_m_, d_min_x_m_, d_max_y_m_, d_min_y_m_}); };

  void SetCircleRobot(double d_radius_m);

  void SetBoxRobot(double d_max_x_m, double d_min_x_m, double d_max_y_m, double d_min_y_m, double d_padding_m);

private:
  MPTL::ROBOT_SHAPE e_robot_shape_ = MPTL::ROBOT_SHAPE::BOX;
  std::vector<Pos> vec_config_points_;
  std::vector<Pos> vec_box_spin_points_;
  double d_radius_m_ = 1.0;

  double d_max_x_m_ = 1.0;
  double d_min_x_m_ = -1.0;
  double d_max_y_m_ = 1.0;
  double d_min_y_m_ = -1.0;

  std::vector<Pos> SetConfigPoints(double d_max_x_m, double d_min_x_m, double d_max_y_m, double d_min_y_m, double d_padding_m);
  std::vector<Pos> SetBoxSpinPoints(double d_radius_m, double d_padding_m);

};

} // namespace NVFR

#endif
