
#ifndef POINT_TO_PLANE_ICP_HPP_
#define POINT_TO_PLANE_ICP_HPP_

#include "icp/localizer_parameter_container.hpp"
#include "simplepos/simplepos.hpp"
#include "time_checker/time_checker.h"
#include "util/ansi_color.h"

#include "util/logger.hpp"

#include <libnabo/nabo/nabo.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/tbb.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <memory>
#include <random>
#include <unordered_map>

#define CAL_NORMAL_VECTOR
#if 1
#define ICP_PARALLEL
#pragma message("ICP parallel programming")
#else
#pragma message("ICP sequential programming")
#endif

namespace NaviFra {
enum LOCALIZER_STATUS {
  INIT_POSE = 0,
  TRACKING = 1,
  CALIBRATION = 2,
  MERGEDMAP = 3,

};

struct LOCALIZE_RESULT {
  std::chrono::steady_clock::time_point time_stamp =
      std::chrono::steady_clock::now();
  float f_confidence = 100.f;
  int n_precision = 0;
  float f_search_dist = 0.f;
  float f_rmse = 0.f;
  NaviFra::SimplePos o_corrected_pose;
};
struct BasicPoint_t {
  double x, y;
  BasicPoint_t(double x_ = 0., double y_ = 0.) : x(x_), y(y_) {}
};

struct ICPPoint_t {
  bool b_matched_flag = false;
  float f_mathced_distance_m = 0.f;
  float f_new_pt_x_m = 0.f;
  float f_new_pt_y_m = 0.f;
  float f_new_pt_z_m = 0.f;
  float f_new_pt_normal_vec_x = 0.f;
  float f_new_pt_normal_vec_y = 0.f;
  float f_db_pt_x_m = 0.f;
  float f_db_pt_y_m = 0.f;
  float f_db_pt_z_m = 0.f;
  float f_db_pt_normal_vec_x = 0.f;
  float f_db_pt_normal_vec_y = 0.f;
  int n_matched_index = 0;
};

class PointToPlaneICP {
public:
  PointToPlaneICP();
  virtual ~PointToPlaneICP();

  void SetParameter(int n_max_icp_execution_cnt, float f_icp_conv_cond_m,
                    float f_matching_dist_max = 2.0f,
                    float f_matching_ratio_thres = 0.4f,
                    float f_matching_dist_thres = 0.05,
                    float f_icp_matching_ignore_ratio = 0.3f,
                    int n_knn_search_num = 1,
                    int n_localize_check_condition = 1,
                    float f_localizer_not_use_ratio = 0.f);
  void CaclNewpointForICP(const NaviFra::SimplePos &o_robot_pos,
                          const std::vector<SimplePos> &vec_original_pts,
                          std::vector<SimplePos> &vec_transform_new_data);

  double CalcRMSE(const std::vector<SimplePos> &vec_new_data,
                  const std::vector<ICPPoint_t> &vec_matching_pts);

  float GetMatchingRatio() const;
  int GetPrecision() const;
  float GetSearchDist() const;
  float GetRMSE() const;
  const vector<SimplePos> GetDBMap() { return vec_db_pts_; }

  std::vector<ICPPoint_t>
  FindMatchingPointwithVector(const std::vector<SimplePos> &vec_db_pts,
                              const vector<SimplePos> &vec_new_data,
                              const int n_result, int &n_matching_data,
                              bool &b_far_search_flag,
                              const SimplePos &o_calc_robot_pos);

  std::vector<ICPPoint_t>
  FindMatchingPointNAccData(const std::vector<SimplePos> &vec_db_pts,
                            const vector<SimplePos> &vec_new_data,
                            const int n_result, int &n_matching_data,
                            bool &b_far_search_flag);

  NaviFra::SimplePos
  CalcRobotPosByICPInv(const NaviFra::SimplePos &o_robot_pos,
                       const vector<ICPPoint_t> &vec_matching_pts);
  NaviFra::SimplePos CalcICPpos(const std::vector<SimplePos> &vec_db_pts,
                                std::vector<SimplePos> &vec_new_data,
                                NaviFra::SimplePos &o_predict_robot_pos,
                                int n_result);
  LOCALIZE_RESULT DoICP(const std::vector<SimplePos> &vec_current_data,
                        const NaviFra::SimplePos &o_predict_robot_pos,
                        const int &n_localizer_status);

  Eigen::MatrixXd ComputeNormal(const std::vector<BasicPoint_t> &vecPoint,
                                double dNx, double dNy);
  void CalDBNormalVector(const vector<SimplePos> &vec_db_pts, int nRangeCnt);
  void DrawingMatchedInfo(const vector<ICPPoint_t> &vec_matching_pts,
                          int nLarge);
  void SetDBMap(const vector<SimplePos> &vec_db_pts);
  bool CheckRMSE(const float f_rmse, const int n_localizer_status);
  bool SetParameters(const localizer_parameter_container *param_container,
                     const bool b_initial_pose = false);
  void DrawingData2(const vector<SimplePos> &vec_pts1,
                    const vector<SimplePos> &vec_pts2, int nLarge);

  int n_max_icp_execution_cnt_;
  float f_matching_stop_thres;

  float f_matching_ratio_ = 0.0f;
  float f_matching_ratio_max_ = 0.0f;

  float f_matching_ratio_thres_ = 0.4f;
  float f_matching_dist_min_ = 0.05f;
  float f_matching_dist_max_ = 0.05f;
  float f_matching_dist_ = 0.f;
  float f_localizer_not_use_ratio_ = 0.f;
  int n_precision_ = 0;
  float f_rmse_ = 0.f;
  int n_knn_search_num_ = 1;
  int n_localize_check_condition_ = 1;

  float f_min_dist_compare_ = 15;
  float f_max_dist_compare_ = 40;
  float f_ratio_compare_ = 3;

  bool b_init_icp_db_flag_ = false;
  pcl::KdTreeFLANN<pcl::PointXY> o_kdtree_;
  std::vector<BasicPoint_t> vecNormalPoint_;

  Nabo::NNSearchF *nabo_kdtree_;
  Eigen::MatrixXf model_;
  vector<SimplePos> vec_db_pts_;
  vector<SimplePos> vec_db_temp_pts_;
  std::unique_ptr<tbb::task_arena> arena_;
  std::unique_ptr<tbb::global_control> tbb_global_control_;
  TimeChecker time_recoder_;
};
} // namespace NaviFra

#endif
