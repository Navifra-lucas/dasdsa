
/*
 * @file	: pointcloud.hpp
 * @date	: Nov. 12, 2024
 * @author	: "Sanghyeon, Lee(Reech)" (rechard@navifra.com)"
 * @brief	: 점군 정보를 저장
 * @remark	:
 * @warning	:
 * 	Copyright(C) 2024 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef NAVIFRA_POINTCLOUD_HPP_
#define NAVIFRA_POINTCLOUD_HPP_

#include <iostream>

namespace NaviFra {
class PointCloud // 가장 기초가 되는 pos자료클래스
{
public:
  /// @brief Number of points.
  size_t size() const { return points.size(); }

  /// @brief Check if the point cloud is empty.
  bool empty() const { return points.empty(); }

  /// @brief Get i-th point.
  NaviFra::SimplePos &point(size_t i) { return points[i]; }

  /// @brief Get i-th point.
  const NaviFra::SimplePos &point(size_t i) const { return points[i]; }

  /// @brief cloud 획득한 시간
  std::chrono::steady_clock::time_point time;

  /// @brief pointcloud
  std::vector<NaviFra::SimplePos> points; ///< Point coordinates (x, y, z, 1)
};
} // namespace NaviFra

#endif
