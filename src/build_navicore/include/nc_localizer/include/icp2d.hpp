#pragma once

#include "icp/localizer_parameter_container.hpp"
#include "icp/point_to_plane_icp.hpp"
#include "kdtree_wrapper.hpp"
#include "nanoflann.hpp"
#include "simplepos/simplepos.hpp"
#include "time_checker/time_checker.h"
#include "util/ansi_color.h"
#include "util/logger.hpp"

#include <libnabo/nabo/nabo.h>
// #include <opencv2/opencv.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/tbb.h>

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <memory>
#include <random>
#include <tuple>
#include <unordered_map>
using ITERATION_LIMIT = int;
using KDTREE_K = int;
using MATCHING_DIST = float;

namespace NaviFra {

class icp2d {
private:
  /* --------------------------------- data ----------------------------------
   */

  /**
   * @param nabo_kdtree_       libnabo 기반 트리. 로컬라이저에선 이제
   * 안씀...잘가..
   * @param kdtree_            nanoflann 기반 트리.
   * @param map_cloud_         지도 (포인트 데이터) -> 멤버변수에서
   * 제외해도문제없는가
   * @param last_confidence_   마지막 컨피던스
   *
   * @param model_             지도 (포인트 데이터) - Eigen
   * @param vec_normals_       지도 노말벡터
   */
  Nabo::NNSearchF *nabo_kdtree_;
  std::unique_ptr<KDTree2f> kdtree_;
  PointCloudf map_cloud_;
  float last_confidence_;

  Eigen::MatrixXf model_;
  localizer_parameter_container *param_container_;
  localizer_parameter_container *init_pose_param_container_;
  std::vector<SimplePos> vec_normals_;

  /* --------------------------------- Methods
   * ---------------------------------- */
  std::vector<SimplePos> ComputeNormals(const std::vector<SimplePos> &points,
                                        const int &step = 1);
  std::tuple<ITERATION_LIMIT, KDTREE_K, MATCHING_DIST>
  GetInitialAdaptiveParameters(const NaviFra::SimplePos &o_predict_robot_pos,
                               const bool is_tracking = true);
  std::pair<KDTREE_K, MATCHING_DIST>
  UpdateAdaptiveParameters(const int &iteration,
                           localizer_parameter_container *param_container);
  void InspectConfidence(
      const float &global_robot_angle,
      const std::vector<float> &vec_matched_angle,
      std::tuple<float, float, float, float> &section_matching_ratio);

public:
  icp2d(/* args */);

  ~icp2d();

  LOCALIZE_RESULT
  AlignGaussNewton(const std::vector<SimplePos> &vec_current_data,
                   const NaviFra::SimplePos &o_predict_robot_pos,
                   const NaviFra::SimplePos &pose_diff,
                   const int &n_localizer_status,
                   localizer_parameter_container *param_container,
                   const bool is_tracking = true);

  LOCALIZE_RESULT AlignGaussPoint2Plane(
      const std::vector<SimplePos> &vec_current_data,
      const NaviFra::SimplePos &o_predict_robot_pos,
      const NaviFra::SimplePos &pose_diff, const int &n_localizer_status,
      localizer_parameter_container *param_container,
      std::tuple<float, float, float, float> &section_matching_ratio,
      const bool is_tracking = true);
  bool BuildKDTree(const vector<SimplePos> &vec_db_pts);
  bool SetParameters(localizer_parameter_container *param_container,
                     const bool b_initial_pose);
  std::tuple<nanoflann::KNNResultSet<float>, std::vector<size_t>,
             std::vector<float>>
  FindCorrespondence(const KDTree2f *kdtree, const Eigen::Vector2f &query,
                     const int &K);
  float ComputePoint2PlaneError(const Eigen::Vector2f &target,
                                const Eigen::Vector2f &query,
                                const SimplePos &normal);
  Eigen::Vector3f
  ComputePoint2PlaneJacobian(const SimplePos &lidar_ptr,
                             const NaviFra::SimplePos &estimated,
                             const SimplePos &normal);
  float CalculateConfidence(const int &scan_size,
                            const std::vector<bool> &vec_effective,
                            const int &K);
  bool CheckConvergence(const Eigen::Vector3f &delta,
                        const int &current_iteration,
                        const int &minimum_iteration,
                        const float &current_computation,
                        const float &time_limit = 1000);
  const std::vector<SimplePos> &GetNormalVectors() { return vec_normals_; }
};

} // namespace NaviFra