#pragma once
#include "common/pch.h"
#include "common/scan3d.h"
#include "logger/logger.h"
#include "tsl/robin_map.h"

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

    PointCloud3D FilterByRange(
        const PointCloud3D &frame, const float min_range,
        const float max_range);

    PointCloud3D FilterByVoxel(const PointCloud3D &frame, float voxel_size);
};

}  // namespace ANSWER