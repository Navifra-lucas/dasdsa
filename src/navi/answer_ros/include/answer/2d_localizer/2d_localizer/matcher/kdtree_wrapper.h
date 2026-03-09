#pragma once

#include "nanoflann/nanoflann.h"

#include <vector>

/**
 * @brief K최근접이웃을 위한 PointCloud 구조체
 * @warning inline 함수 이름 바꾸면 작동안함
 */
struct PointCloudf {
    struct Point {
        float x, y;
    };
    std::vector<Point> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        float res = std::numeric_limits<float>::max();
        if (dim == 0)
            return pts[idx].x;
        else if (dim == 1)
            return pts[idx].y;

        return res;
    }

    // Optional bounding-box computation: return false to default to a standard
    // bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned
    //   in "bb" so it can be avoided to redo it again. Look at bb.size() to
    //   find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    const bool kdtree_get_bbox(BBOX & /* bb */) const
    {
        return false;
    }
};
// 단일인덱스 기반 K최근접이웃 검색
typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloudf>, PointCloudf, 2 /* dim */
    >
    KDTree2f;
