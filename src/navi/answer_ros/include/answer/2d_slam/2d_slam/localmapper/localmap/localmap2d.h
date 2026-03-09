/**
 * @class localmap 2d
 * @brief Generate lidar odometry and point cloud for icp using this localmap.
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */
#ifndef LOCALMAP_2D_H
#define LOCALMAP_2D_H
#include "2d_slam/localmapper/localmap/grid2d.h"
#include "common/kdtree_wrapper.h"
#include "common/pose2d.h"
#include "common/scan2d.h"
#include "map_limits.h"

#include <memory>
#include <mutex>
#include <vector>

#include "Eigen/Core"

// #include "value_conversion_tables.h"

namespace ANSWER {
namespace SLAM2D {
class Localmap2D {
public:
    Localmap2D() {}
    Localmap2D(
        const Pose2D &origin, const float map_resolution,
        const std::vector<uint16_t> *hit_table,
        const std::vector<uint16_t> *miss_table);
    ~Localmap2D();
    // Localmap2D(const Localmap2D&) = delete;
    // Localmap2D& operator=(const Localmap2D&) = delete;

    // Inserts 'range_data' into the Localmap collection.
    void InsertRangeData(
        const Scan2D &range_data, const bool b_high_resolution = false);

    // Number of RangeData inserted.
    int num_range_data() const { return num_range_data_; }

    void SetProbTable(
        const std::vector<uint16_t> *hit_table,
        const std::vector<uint16_t> *miss_table);

    Pose2D GetOrigin() const { return origin_; }
    void SetOrigin(const Pose2D &new_origin) { origin_ = new_origin; }
    void DrawLocalmap();
    void LocalmapUpdateFinish();
    void GlobalmapUpdateFinish();
    Grid2D *GetProbabilityGrid() const { return probability_grid_.get(); }
    const int &GetIndex() const { return index_; }
    void SetIndex(const int index) { index_ = index; }

#if USE_NANO
    const KDTreeWrapper::KDTree2D *GetKDTree() const { return kdtree_.get(); }
#else
    KD_TREE<KDTreeWrapper::PointType, KDTreeWrapper::DataSource> *GetKDTree()
        const
    {
        return kdtree_wrapper_->GetKdTree();
    }
#endif

    PointCloud2D &GetPointCloud()
    {
        // return point_cloud_;
        return point_cloud_wrapper_.GetPointCloud();
    }
    PointCloud2D &GetOriginalPointCloud() { return original_point_cloud_; }
    bool insertion_finished() const { return insertion_finished_; }
    void IncreaseMergeCount()
    {
        std::lock_guard<std::mutex> lock(mutex_get_merge_count_);
        merge_count_++;
    }
    const int &GetMergeCount()
    {
        std::lock_guard<std::mutex> lock(mutex_get_merge_count_);
        return merge_count_;
    }
    bool IsMergeDone()
    {
        bool res = false;
        {
            std::lock_guard<std::mutex> lock(mutex_get_merge_count_);
            res = merge_count_ >= 3;
        }
        if (res) {
            point_cloud_wrapper_.ClearVoxelMap();
            return res;
        }
        else
            return res;
    }

    void ExtractPointCloud()
    {
        if (probability_grid_ != nullptr) {
            probability_grid_->ExtractProbMap();
            // probability_grid_->DrawProbMap();
            // point_cloud_ = probability_grid_->GetPointCloud();
            original_point_cloud_ = probability_grid_->GetPointCloud();
            LocalmapUpdateFinish();
            // probability_grid_.reset();
        }
    }
    void ReleaseProbMap()
    {
        if (probability_grid_ != nullptr) {
            prob_map_ = std::move(probability_grid_->GetProbMap());
            probability_grid_.reset();
        }
    }
    std::vector<std::pair<Point2D, float>> GetProbMap() { return prob_map_; }
    void ExtractPointCloudUsingMultiThread()
    {
        probability_grid_->ExtractProbMapUsingMultiThread();
    }
    // const float& GetDistance() { return distance_; }
    // void SetDistance(const float distance) { distance_ = distance; }
    void resize(const int size_x, const int size_y)
    {
        probability_grid_->resize(size_x, size_y);
    }
    void set_num_range_data(const int num_range_data)
    {
        num_range_data_ = num_range_data;
    }
#if USE_NANO
    void AddPointToKDTree(const size_t start, const size_t end)
    {
        kdtree_->addPoints(start, end);
    }
#else
    void AddPointToIKDTree(const size_t start, const size_t end)
    {
        kdtree_wrapper_->GetKdTree()->Add_Points_With_Range(start, end);
    }
#endif
    inline void AddPointCloudToKDTree(PointCloud2D &point_cloud)
    {
        int before = size_of_point_cloud_;
        // NLOG(info) << "before size of point cloud : " << before;
        point_cloud_wrapper_.AddPointCloud(point_cloud);

#if USE_NANO
        AddPointToKDTree(
            size_of_point_cloud_,
            point_cloud_wrapper_.GetPointCloud().size() - 1);
#else
        AddPointToIKDTree(
            size_of_point_cloud_,
            point_cloud_wrapper_.GetPointCloud().size() - 1);
#endif
        size_of_point_cloud_ = point_cloud_wrapper_.GetPointCloud().size();
        // NLOG(info) << "before : " << before << " after : " <<
        // point_cloud_wrapper_.GetPointCloud().size()
        // << " input : " << point_cloud.size() << " new : " <<
        // point_cloud_wrapper_.GetPointCloud().size() - before;
        ComputeNormals();
    }
    inline void BuildKDTree(PointCloud2D &target)
    {
        size_of_point_cloud_ = target.size();

#if USE_NANO
        LOG_INFO("Build nanoflann");
        kdtree_ = KDTreeWrapper::CreateKDTree2D(point_cloud_wrapper_);
        point_cloud_wrapper_.SetPointCloud(target);

        AddPointToKDTree(0, size_of_point_cloud_ - 1);
#else
        LOG_INFO("Build IKD Tree");
        kdtree_wrapper_ = KDTreeWrapper::KDTreeFactory::create("ikd");
        point_cloud_wrapper_.AddPointCloud(target);

        kdtree_wrapper_->CreateKDTree(point_cloud_wrapper_);
#endif
        ComputeNormals();
    }
    void ComputeNormals();
    const PointCloud2D &GetNormals() const { return normals_; }
    const Point2D FindNormalMap(size_t hash_key) const
    {
        Point2D res;
        auto it = normal_map_.find(hash_key);
        if (it != normal_map_.end()) {
            return it->second;
        }
        else {
            return res;
        }
    }

private:
    void set_insertion_finished(bool insertion_finished)
    {
        insertion_finished_ = insertion_finished;
    }

    void GrowAsNeeded(const Scan2D &range_data);
    void CastRays(const Scan2D &range_data);

    bool isEqual(const Eigen::Array2i &lhs, const Eigen::Array2i &rhs);
    std::vector<Eigen::Array2i> RayToPixelMask(
        const Eigen::Array2i &scaled_begin, const Eigen::Array2i &scaled_end,
        int subpixel_scale);

private:
    const int kSubpixelScale = 1000;
    MapLimits limits_;
    Pose2D origin_;
    int num_range_data_ = 0;
    bool insertion_finished_ = false;
    const std::vector<uint16_t> *hit_table_;
    const std::vector<uint16_t> *miss_table_;
    std::unique_ptr<Grid2D> probability_grid_;
    PointCloud2D original_point_cloud_;
    PointCloud2D normals_;
    int size_of_point_cloud_ = 0;
    KDTreeWrapper::PointCloud2DWrapper point_cloud_wrapper_;
    const float f_map_resolution_ = 0.05f;
    int index_ = 0;
    int merge_count_ = 0;
    // float distance_ = 0.f;
    bool b_high_resolution_ = false;
    std::mutex mutex_get_merge_count_;

#if USE_NANO
    std::unique_ptr<KDTreeWrapper::KDTree2D> kdtree_;
#else
    std::unique_ptr<KDTreeWrapper::KDTree> kdtree_wrapper_;
    std::mutex mutex_normal_map_;
#endif
    tsl::robin_map<size_t, Point2D> normal_map_;
    std::vector<std::pair<Point2D, float>> prob_map_;

};  // namespace SLAM2D
}  // namespace SLAM2D
}  // namespace ANSWER

#endif