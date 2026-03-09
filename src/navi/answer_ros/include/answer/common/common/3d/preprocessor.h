#pragma once
#include "common/3d/point_cloud_utils.h"
//#include "common/3d/pose3d.h"
#include "common/3d/scan3d.h"
#include "common/pch.h"
#include "logger/logger.h"
#include "tsl/robin_map.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using Voxel = Eigen::Vector3i;
struct VoxelHash {
    size_t operator()(const Voxel &voxel) const
    {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        return ((1 << 20) - 1) &
            (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }
};
namespace ANSWER {
class Preprocessor {
private:
    /* data */
public:
    Preprocessor(/* args */);
    ~Preprocessor();

    Scan3D FilterByRange(
        const Scan3D &frame, const double min_range, const double max_range);

    Scan3D FilterByROI(
        const Scan3D &frame, const double horizontal_angle_range,
        const double max_height,
        const double min_height = std::numeric_limits<double>::lowest(),
        const double max_width = std::numeric_limits<double>::max());

    Scan3D FilterByVoxel(const Scan3D &frame, double voxel_size);
    Scan3D RemoveGround(
        const Scan3D &frame, const double &distance_threshold,
        const int &iterations);
    Scan3D RemoveStatisticalOutliers(
        const Scan3D &frame, KDTree3d *kdtree, const int K,
        const double std_dev_mul_thresh);
};

}  // namespace ANSWER