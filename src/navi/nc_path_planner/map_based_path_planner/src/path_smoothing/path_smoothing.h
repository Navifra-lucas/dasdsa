#ifndef C_PATH_SMOOTHING_H
#define C_PATH_SMOOTHING_H

#include "map/map.hpp"
#include "pos/pos.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <queue>
#include <set>
#include <vector>

using namespace std;
namespace NaviFra {
struct MapInfo2 {
    int n_x_pixel, n_y_pixel, n_map_size;
    float f_resolution_m;
    std::shared_ptr<std::vector<int8_t>> vec_map;
};

class PathSmoothing {
private:
    float f_pos_dist_ = 0.1;
    int WAY_POINT_INTERVAL = 5;
    int BIG_WAY_POINT_INTERVAL = 200;
    int WAY_POINT_VALUE = 5;

    vector<NaviFra::Pos> m_WayPointList;
    vector<NaviFra::Pos> m_InflectionWayPointList;
    std::shared_ptr<MapInfo2> ptr_map_;

private:
    vector<NaviFra::Pos> doCubicSplineInterpolation(double timeSlice, vector<NaviFra::Pos> stlWayPoint);
    bool Obstacleflag(float f_midPointX, float f_midPointY);

public:
    void SetMap(std::shared_ptr<Map> o_map);
    void SetParam(float f_pos_dist, int WAY_POINT_INTERVAL_cnt);
    vector<NaviFra::Pos> smoothingPath(vector<NaviFra::Pos> originalPathList);
    vector<NaviFra::Pos> smoothingCubicSplinePath(vector<NaviFra::Pos> originalPathList);
    vector<NaviFra::Pos> generateWaypointFromPath(vector<NaviFra::Pos> originalPathList, int ninterval);
    vector<NaviFra::Pos> doLeastSquare(vector<NaviFra::Pos> waypointlist);
    vector<NaviFra::Pos> getWayPointList();
    vector<NaviFra::Pos> getInflectionWayPointList();

    PathSmoothing();
    virtual ~PathSmoothing();
};
};  // namespace NaviFra
#endif
