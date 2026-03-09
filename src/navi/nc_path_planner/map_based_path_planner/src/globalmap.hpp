#ifndef GLOBALMAP_HPP_
#define GLOBALMAP_HPP_

#include "nav_msgs/OccupancyGrid.h"

#include <stdint.h>

#include <vector>

// #include "map/localmap/mapinfo.hpp"
#include "core/map/map.hpp"
#include "mapinfo.hpp"
#include "pos/pos.hpp"

using namespace std;
namespace NaviFra {

class GlobalMap {
public:
    struct Padding {
        Padding(int x, int y, int c)
            : n_px(x)
            , n_py(y)
            , cost(c){};
        int n_px, n_py;
        int8_t cost;
    };

    void SetMapInfo(float f_resolution_m, float f_padding_size_m);
    void SetGlobalMap(const nav_msgs::OccupancyGrid& map_data);
    const MapInfo_t& GetGlobalMapInfo() const { return st_global_map_info_; };
    const std::vector<int8_t>& GetGlobalMap() const { return vec_global_map_; };
    // void GetLocalMap(std::vector<int8_t>& vec_local_map, const MapInfo_t& st_local_map_info_);
    void GetMapInfo(MapInfo_t& st_new_global_map_info);
    void GetGlobalMap(std::vector<int8_t>& vec_global_map);  // , Map& o_global_map);
    void GetLocalMap(Map& o_local_map, const NaviFra::Pos o_robot_pos);
    void UpdateGridCellUisngLiDAR(Map& o_local_map, const vector<NaviFra::SimplePos>& vec_data, const NaviFra::Pos& o_map_to_localmap);
    void InflateMap(const int size_x, const int size_y, Map& o_local_map, const float f_padding_size);

private:
    MapInfo_t st_global_map_info_;
    std::vector<int8_t> vec_global_map_;
    std::vector<int8_t> vec_local_map_;

    std::vector<Padding> vec_padding_lt_;

    int8_t DownSampling(const nav_msgs::OccupancyGrid& map_data, int n_begin_x_px, int n_begin_y_px, int n_end_x_px, int n_end_y_px);
};

}  // namespace NaviFra

#endif