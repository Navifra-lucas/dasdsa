#ifndef LOCAL_MAP_HPP_
#define LOCAL_MAP_HPP_

#include "lookup_table.h"
#include "simplepos/simplepos.hpp"

#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

using namespace std;
using namespace NaviFra;

namespace NaviFra {
/**
 * Struct to handle parameters about localmap
 * All value must be initialized
 */
struct localmap_param_box {
  float size_x = 20.0;
  float size_y = 20.0;
  float resolution = 0.1;
  float occ_padding_size = 0.8;
  float update_condition_dist = 0.;
  float update_condition_deg = 0.;
  float update_occupancy_probability = 0.85;
  float update_free_probability = 0.45;
  int update_frequency = 50;
  int update_queue_size = 2;
  int n_localmap_size_x_pixel = 0;
  int n_localmap_size_y_pixel = 0;
};

struct globalmap_param_box {
  int i_map_height;
  int i_map_width; // 1px = 1cm = 0.01m
  int f_offset_x;
  int f_offset_y;
  float f_globalmap_resolution = 0.01;
};

struct localmap {
  NaviFra::SimplePos map_to_localmap;
  // vector<int8_t> data;
  vector<int8_t> data;
};

class LocalMap {
private:
  bool b_is_globalmap_updated_ = false;
  bool b_is_initial_localmap_generated_ = false;

  float f_update_dist_ = 0.;
  float f_update_deg_ = 0.;

  int n_localmap_size_x_pixel_;
  int n_localmap_size_y_pixel_;

  string str_save_path_;
  localmap_param_box o_localmap_param_;
  globalmap_param_box o_globalmap_param_;

  // NaviFra::SimplePos map_to_robot_;
  NaviFra::SimplePos localmap_to_robot_;
  NaviFra::SimplePos prior_map_to_robot_pose_;

  std::vector<int8_t> vec_n8_globalmap_;
  // localmap localmap_;

public:
  // Map global_map_;
  LocalMap();
  virtual ~LocalMap();

  void SetLocalParam(const localmap_param_box &localmap_param);
  void SetGlobalParam(const globalmap_param_box &globalmap_param);

  void InitialLocalMap(const NaviFra::SimplePos &map_to_robot);

  void GetInflateGlobalmap(const std::vector<int8_t> &vec_n8_globalmap,
                           std::vector<int8_t> &vec_n8_padded_global_map);
  bool BuildLocalMap(const vector<NaviFra::SimplePos> &vec_data,
                     NaviFra::SimplePos map_to_robot,
                     vector<int8_t> &inflated_localmap);
  void SetGlobalmap(const std::vector<int8_t> &vec_n8_globalmap);
  void GetLocalParam(localmap_param_box &localmap_param);
  void SaveMap(int size_x, int size_y, const vector<int8_t> &prob_map,
               const string function_name);
  void SaveMap2(int size_x, int size_y, const vector<int8_t> &prob_map,
                const string function_name);
  void InflateMap(const int size_x, const int size_y,
                  const vector<int8_t> &input_map, vector<int8_t> &output_map);
  void UpdateGridCellUisngLiDAR(localmap &target_localmap,
                                const vector<NaviFra::SimplePos> &vec_data);

  localmap localmap_;

private:
  void MakeInitialLocalMap(const NaviFra::SimplePos &map_to_robot,
                           localmap &target_localmap);

  // void InflateMap(const int size_x, const int size_y, const vector<int8_t>&
  // input_map, vector<int8_t>& output_map);

  void CopyPriorLocalMapToCurrentLocalMap(localmap &source_localmap,
                                          localmap &target_localmap);
  // void UpdateGridCellUisngLiDAR(localmap& target_localmap, const
  // vector<NaviFra::SimplePos>& vec_data);
  void UpdateLocalMap(const NaviFra::SimplePos &map_to_robot,
                      const vector<NaviFra::SimplePos> &vec_data,
                      vector<int8_t> &inflated_localmap);

  vector<pair<int, int>> BresenhamLine(const NaviFra::SimplePos &start,
                                       const NaviFra::SimplePos &end);

  bool CheckMovement(const NaviFra::SimplePos &delta_pose);

  inline bool CheckBoundary(const int x, const int y, const int lower_x,
                            const int lower_y, const int upper_x,
                            const int upper_y) {
    if (x < lower_x || y < lower_y || x >= upper_x || y >= upper_y) {
      return false;
    }
    return true;
  }
};
} // namespace NaviFra
#endif