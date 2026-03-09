/**
 * @class voxel filter
 * @brief filter out lidar points using voxel
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */
#ifndef VOXEL_FILTER_H
#define VOXEL_FILTER_H

#include "common/pch.h"
#include "common/scan2d.h"

#include <bitset>

using namespace std;
namespace ANSWER {
namespace SLAM2D {
class VoxelFilter {
private:
    /* data */
public:
    VoxelFilter(/* args */);
    ~VoxelFilter();

    Scan2D FilterOut(const PointCloud2D& point_cloud, const float resolution);

    Scan2D RandomizedVoxelFilter(const PointCloud2D& point_cloud, const float resolution);

    uint64_t GetVoxelCellIndex(const Point2D& point, const float resolution);

    std::vector<bool> RandomizedVoxelFilterIndices(const PointCloud2D& point_cloud, const float resolution);
};

}  // namespace SLAM2D
}  // namespace ANSWER

#endif  // VOXEL_FILTER_H
