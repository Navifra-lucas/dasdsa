#pragma once

#include "common/scan2d.h"
#include "common/time_checker.h"
#include "ikd_tree/ikd_Tree.h"
#include "nanoflann/nanoflann.h"
#include "tsl/robin_map.h"

namespace ANSWER {
namespace SLAM2D {
namespace KDTreeWrapper {
struct PointCloud2DWrapper {
    PointCloud2DWrapper() {}
    ~PointCloud2DWrapper() {}
    PointCloud2DWrapper(PointCloud2D &point_cloud)
        : point_cloud_(point_cloud)
    {
    }
    PointCloud2D point_cloud_;
    tsl::robin_map<size_t, Point2D> voxel_map_;
    inline void SetPointCloud(PointCloud2D &point_cloud)
    {
        for (auto ptr : point_cloud) {
            float precision_x = std::roundf(ptr.x() * 100) * 0.01;
            float precision_y = std::roundf(ptr.y() * 100) * 0.01;
            size_t x = std::hash<float>{}(precision_x);
            size_t y = std::hash<float>{}(precision_y);
            size_t combined = x ^ (y << 1);

            if (voxel_map_.find(combined) == voxel_map_.end()) {
                voxel_map_[combined] = ptr;
                point_cloud_.emplace_back(ptr);
            }
        }
    }
    inline void AddPointCloud(PointCloud2D &point_cloud)
    {
        SetPointCloud(point_cloud);
    }
    inline void ClearVoxelMap() { voxel_map_.clear(); }
    inline PointCloud2D &GetPointCloud() { return point_cloud_; }
    inline const auto &GetPointCloud() const { return point_cloud_; }
    inline std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>
    convertPointCloud2DToPCL() const
    {
        std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>
            pcl_cloud;
        pcl_cloud.reserve(point_cloud_.size());
        for (const auto &pt : point_cloud_) {
            pcl::PointXYZ p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = 0.0f;
            pcl_cloud.push_back(p);
        }
        return pcl_cloud;
    }

    inline size_t kdtree_get_point_count() const { return point_cloud_.size(); }
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return point_cloud_[idx].x();
        else if (dim == 1)
            return point_cloud_[idx].y();
        else
            return 0;
    }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /*bb*/) const
    {
        return false;
    }
};  // namespace KDTreeWrapper

typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloud2DWrapper>,
    PointCloud2DWrapper, 2>
    KDTree2D;

inline std::unique_ptr<KDTree2D> CreateKDTree2D(
    const PointCloud2DWrapper &point_cloud)
{
    return std::make_unique<KDTree2D>(
        2, point_cloud,
        nanoflann::KDTreeSingleIndexAdaptorParams(
            10, nanoflann::KDTreeSingleIndexAdaptorFlags::None, 0));
}

// using PointType = pcl::PointXYZ;
using PointType = Eigen::Vector2f;
using DataSource = PointCloud2DWrapper;
using PointVector = KD_TREE<PointType, DataSource>::PointVector;

class KDTree {
public:
    virtual void CreateKDTree(const PointCloud2DWrapper &point_cloud) = 0;
    virtual inline KD_TREE<PointType, DataSource> *GetKdTree() = 0;
    virtual ~KDTree() {}
};

class IKDWrapper : public KDTree {
public:
    IKDWrapper() {}
    void CreateKDTree(const PointCloud2DWrapper &point_cloud)
    {
        kdtree_ = std::make_shared<KD_TREE<PointType, DataSource>>(point_cloud);
        TimeChecker tc;
        kdtree_->Build(point_cloud.GetPointCloud());
    }
    inline KD_TREE<PointType, DataSource> *GetKdTree() { return kdtree_.get(); }

private:
    std::shared_ptr<KD_TREE<PointType, PointCloud2DWrapper>> kdtree_;
    PointCloud2DWrapper point_cloud_;
};

class KDTreeFactory {
public:
    static std::unique_ptr<KDTree> create(const std::string &type)
    {
        if (type == "ikd") {
            return std::make_unique<IKDWrapper>();
        }
        return nullptr;
    }
};

}  // namespace KDTreeWrapper
}  // namespace SLAM2D
}  // namespace ANSWER