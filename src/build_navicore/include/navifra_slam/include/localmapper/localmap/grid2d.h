/**
 * @class grid 2d
 * @brief  probability grid map of each localmap.
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */
#ifndef GRID_2D_H
#define GRID_2D_H

#include "common/pose2d.h"
#include "common/scan2d.h"
#include "localmapper/localmap/probability_values.h"
#include "map_limits.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "time_checker/time_checker.h"

#include <functional>
#include <future>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>
using namespace std;
using namespace cv;
namespace NaviFra {
namespace SLAM2D {
class Grid2D {
public:
  Grid2D() {}

  Grid2D(const MapLimits &limits, const float map_resolution);
  ~Grid2D();
  // Returns the limits of this Grid2D.
  const MapLimits &limits() const { return limits_; }

  // Finishes the update sequence.
  void FinishUpdate();

  void ProcessGrid(const int start_x, const int end_x, const int start_y,
                   const int end_y, const Eigen::Array2i &offset,
                   const MapLimits &limits_, const float f_map_resolution_,
                   const float meter_to_grid);
  void PerformComputationMultithread();

  // Returns the correspondence cost of the cell with 'cell_index'.
  float GetCorrespondenceCost(const Eigen::Array2i &cell_index) const {
    if (!limits().Contains(cell_index)) {
      // NLOG(info)<<"here??";
      return kMaxCorrespondenceCost;
    }
    auto val = correspondence_cost_cells()[ToFlatIndex(cell_index)];
    // NLOG(info) << "val : "
    //           <<
    //           ProbabilityValues::GetInstance()->ValueToCorrespondenceCost(val);
    return ProbabilityValues::GetInstance()->ValueToCorrespondenceCost(val);
  }

  // Returns the minimum possible correspondence cost.
  float GetMinCorrespondenceCost() const { return kMinCorrespondenceCost; }

  // Returns the maximum possible correspondence cost.
  float GetMaxCorrespondenceCost() const { return kMaxCorrespondenceCost; }

  // Returns true if the probability at the specified index is known.
  bool IsKnown(const Eigen::Array2i &cell_index) const {
    return limits_.Contains(cell_index) &&
           correspondence_cost_cells_[ToFlatIndex(cell_index)] !=
               kUnknownCorrespondenceValue;
  }

  // Fills in 'offset' and 'limits' to define a subregion of that contains all
  // known cells.
  void ComputeCroppedLimits(Eigen::Array2i *const offset,
                            CellLimits *const limits) const;

  // Grows the map as necessary to include 'point'. This changes the meaning of
  // these coordinates going forward. This method must be called immediately
  // after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
  void GrowLimits(const Point2D &point);
  bool ApplyLookupTable(const Eigen::Array2i &cell_index,
                        const std::vector<uint16_t> &table);

  float GetProbability(const Eigen::Array2i &cell_index) const {
    if (!limits().Contains(cell_index))
      return kMinProbability;
    return ProbabilityValues::GetInstance()->CorrespondenceCostToProbability(
        ProbabilityValues::GetInstance()->ValueToCorrespondenceCost(
            correspondence_cost_cells()[ToFlatIndex(cell_index)]));
  }

  void SetProbability(const Eigen::Array2i &cell_index,
                      const float probability);
  // std::unique_ptr<Grid2D> ComputeCroppedGrid() const override;
  void DrawProbMap();
  void ExtractProbMap();
  void ExtractProbMapUsingMultiThread();
  void ExtractGlobalProbMap();
  const std::vector<pair<Point2D, float>> &GetProbMap() const {
    return prob_map_;
  }
  std::vector<float> &GetProbXm() {
    // std::lock_guard<std::mutex> lock(get_prob_x_);
    return prob_map_x_;
  }
  std::vector<float> &GetProbYm() {
    // std::lock_guard<std::mutex> lock(get_prob_y_);
    return prob_map_y_;
  }

  std::vector<float> &GetProbP() {
    // std::lock_guard<std::mutex> lock(get_prob_p_);
    return prob_map_p_;
  }
  PointCloud2D &GetPointCloud() {
    std::lock_guard<std::mutex> lock(mutex_get_point_cloud_);
    return selected_point_cloud_;
  }
  PointCloud2D &GetOriginalPointCloud() {
    std::lock_guard<std::mutex> lock(mutex_get_point_cloud_);
    return original_point_cloud_;
  }

  void resize(const int size_x, const int size_y) {
    limits_.mutable_cell_limits().num_x_cells = size_x;
    limits_.mutable_cell_limits().num_y_cells = size_y;
    // known_cells_box_.extend(Eigen::AlignedBox2i(size_x,size_y));
    mutable_correspondence_cost_cells()->resize(size_x * size_y);
  }
  void ExploreMap(const Eigen::Array2i &offset, const int n_start_x,
                  const int n_end_x, const int n_start_y, const int n_end_y,
                  const int n_limit_x, const int n_limit_y,
                  std::vector<pair<Pose2D, float>> &prob_map,
                  std::vector<Pose2D> &selected_point_cloud,
                  std::vector<Pose2D> &original_point_cloud);
  // const int& thread_id);

protected:
  void GrowLimits(const Eigen::Vector2f &point,
                  const std::vector<std::vector<uint16_t> *> &grids,
                  const std::vector<uint16_t> &grids_unknown_cell_values);

  const std::vector<uint16_t> &correspondence_cost_cells() const {
    return correspondence_cost_cells_;
  }
  const std::vector<int> &update_indices() const { return update_indices_; }
  const Eigen::AlignedBox2i &known_cells_box() const {
    return known_cells_box_;
  }

  std::vector<uint16_t> *mutable_correspondence_cost_cells() {
    return &correspondence_cost_cells_;
  }

  std::vector<int> *mutable_update_indices() { return &update_indices_; }
  Eigen::AlignedBox2i *mutable_known_cells_box() { return &known_cells_box_; }

  // Converts a 'cell_index' into an index into 'cells_'.
  int ToFlatIndex(const Eigen::Array2i &cell_index) const {
    // NLOG(info) << limits_.cell_limits().num_x_cells<<"
    // "<<limits_.cell_limits().num_y_cells; NLOG(info) << cell_index.x()<<"
    // "<<cell_index.y();
    CHECK(limits_.Contains(cell_index)) << cell_index;
    return limits_.cell_limits().num_x_cells * cell_index.y() + cell_index.x();
  }

private:
  MapLimits limits_;
  std::vector<uint16_t> correspondence_cost_cells_;
  std::vector<int> update_indices_;
  std::vector<pair<Point2D, float>> prob_map_;
  std::vector<float> prob_map_x_;
  std::vector<float> prob_map_y_;
  std::vector<float> prob_map_p_;
  PointCloud2D selected_point_cloud_;
  PointCloud2D original_point_cloud_;
  // mutex data_insert_mutex_;
  // mutex get_prob_x_;
  // mutex get_prob_y_;
  // mutex get_prob_p_;

  // int n_pointcloud_num_ = 0;
  // int n_localmap_point_num_ = 0;

  // Bounding box of known cells to efficiently compute cropping limits.
  Eigen::AlignedBox2i known_cells_box_;
  // const std::vector<float> *value_to_correspondence_cost_table_;

  const uint16_t kUnknownProbabilityValue = 0;
  const uint16_t kUnknownCorrespondenceValue = kUnknownProbabilityValue;
  const uint16_t kUpdateMarker = 1u << 15;
  const float kMinProbability = 0.1f;
  const float kMaxProbability = 1.f - kMinProbability;
  const float kMinCorrespondenceCost = 1.f - kMaxProbability;
  const float kMaxCorrespondenceCost = 1.f - kMinProbability;
  const float f_map_resolution_ = 0.05;

  std::mutex mutex_get_point_cloud_;
  std::mutex mutex_get_original_point_cloud_;
};

} // namespace SLAM2D
} // namespace NaviFra
#endif