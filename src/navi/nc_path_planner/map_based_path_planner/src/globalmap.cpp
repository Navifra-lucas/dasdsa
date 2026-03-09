#include <unistd.h>

// #include "map/localmap/globalmap.hpp"
#include "globalmap.hpp"

using namespace NaviFra;

void GlobalMap::SetMapInfo(float f_resolution_m, float f_padding_size_m)  // 로컬맵 RESOLUTION
{
    st_global_map_info_.f_resolution_m = f_resolution_m;

    // set padding lookup table
    vec_padding_lt_.clear();
    int n_padding_size = (int)std::ceil(f_padding_size_m / f_resolution_m);
    int n_padding_size_sq = n_padding_size * n_padding_size;
    for (int i = -n_padding_size; i < n_padding_size + 1; ++i) {
        for (int j = -n_padding_size; j < n_padding_size + 1; ++j) {
            if (i == 0 && j == 0)
                continue;
            int n_dist_sq = i * i + j * j;
            if (n_padding_size_sq >= n_dist_sq) {
                // double d_r = 1.0 - n_dist_sq / n_padding_size_sq;
                // double d_cost = 100.0 * d_r * d_r;
                double d_r = static_cast<double>(n_dist_sq) / static_cast<double>(n_padding_size_sq);
                double d_cost = 100.0 * (1.0 - d_r) * (1.0 - d_r);
                int8_t cost = static_cast<int8_t>(d_cost);
                if (cost <= 0)
                    cost = 1;
                else if (cost >= 100)
                    cost = 99;
                vec_padding_lt_.emplace_back(Padding(i, j, cost));
            }
        }
    }
}

void GlobalMap::SetGlobalMap(const nav_msgs::OccupancyGrid& map_data)  // /map 토픽에서 받은 글로벌맵
{
    // exception
    assert(st_global_map_info_.f_resolution_m >= map_data.info.resolution);

    // set global map info
    st_global_map_info_.f_origin_x_m = map_data.info.origin.position.x;
    st_global_map_info_.f_origin_y_m = map_data.info.origin.position.y;

    // calculate crop size
    if (st_global_map_info_.f_resolution_m == map_data.info.resolution) {
        // same resolution
        // set map info
        st_global_map_info_.n_size_x = map_data.info.width;
        st_global_map_info_.n_size_y = map_data.info.height;
        // set map data
        vec_global_map_.resize(st_global_map_info_.n_size_x * st_global_map_info_.n_size_y);
        vec_global_map_ = map_data.data;
        // for (int idx = 0; idx < vec_global_map_.size(); ++idx)
        //   // vec_global_map_[idx] = (int8_t)map_data.data[idx];
        //   vec_global_map_[idx] = map_data.data[idx];
    }
    else {
        // different resolution
        // calculate global map size & resize global map
        float f_resolution_ratio = st_global_map_info_.f_resolution_m / map_data.info.resolution;
        st_global_map_info_.n_size_x = (int)(float(map_data.info.width) / f_resolution_ratio);
        st_global_map_info_.n_size_y = (int)(float(map_data.info.height) / f_resolution_ratio);

        // set map data
        vec_global_map_.resize(st_global_map_info_.n_size_x * st_global_map_info_.n_size_y);
        float f_data_x_px, f_data_y_px;
        int n_begin_x_px, n_begin_y_px, n_end_x_px, n_end_y_px;
        for (int idx = 0; idx < vec_global_map_.size(); ++idx) {
            // calculate crop index
            f_data_x_px = ((float)(idx % st_global_map_info_.n_size_x) + 0.5f) * f_resolution_ratio;
            f_data_y_px = ((float)(idx / st_global_map_info_.n_size_x) + 0.5f) * f_resolution_ratio;
            n_begin_x_px = (int)(f_data_x_px - f_resolution_ratio / 2);
            n_begin_y_px = (int)(f_data_y_px - f_resolution_ratio / 2);
            n_end_x_px = (int)(f_data_x_px + f_resolution_ratio / 2);
            n_end_y_px = (int)(f_data_y_px + f_resolution_ratio / 2);
            vec_global_map_[idx] = DownSampling(map_data, n_begin_x_px, n_begin_y_px, n_end_x_px, n_end_y_px);
        }
        // print
        // int n_begin_x_px = (int)((-st_global_map_info_.f_resolution_m/2) / map_data.info.resolution + 0.5 + 5);
        // int n_begin_y_px = (int)((-st_global_map_info_.f_resolution_m/2) / map_data.info.resolution + 0.5 + 5);
        // int n_end_x_px = (int)((st_global_map_info_.f_resolution_m/2) / map_data.info.resolution + 0.5 + 5);
        // int n_end_y_px = (int)((st_global_map_info_.f_resolution_m/2) / map_data.info.resolution + 0.5 + 5);
        // printf("[SetGlobalMap] n_begin_x_px, n_begin_y_px, n_end_x_px, n_end_y_px : %d, %d, %d, %d\n", n_begin_x_px, n_begin_y_px,
        // n_end_x_px, n_end_y_px); end print
    }
    st_global_map_info_.f_size_x_m = st_global_map_info_.n_size_x * st_global_map_info_.f_resolution_m;
    st_global_map_info_.f_size_y_m = st_global_map_info_.n_size_y * st_global_map_info_.f_resolution_m;
    // st_global_map_info_.f_origin_x_m = - ((float)(st_global_map_info_.n_size_x) / 2) * st_global_map_info_.f_resolution_m;
    // st_global_map_info_.f_origin_y_m = - ((float)(st_global_map_info_.n_size_y) / 2) * st_global_map_info_.f_resolution_m;
}

void GlobalMap::GetMapInfo(MapInfo_t& st_new_global_map_info)
{
    st_new_global_map_info = st_global_map_info_;
}

int8_t GlobalMap::DownSampling(const nav_msgs::OccupancyGrid& map_data, int n_begin_x_px, int n_begin_y_px, int n_end_x_px, int n_end_y_px)
{
    n_begin_x_px = std::max(n_begin_x_px, static_cast<int>(0));
    n_begin_y_px = std::max(n_begin_y_px, static_cast<int>(0));
    n_end_x_px = std::min(n_end_x_px, static_cast<int>(map_data.info.width));
    n_end_y_px = std::min(n_end_y_px, static_cast<int>(map_data.info.height));
    int n_sampling_size = (n_end_x_px - n_begin_x_px) * (n_end_y_px - n_begin_y_px);
    int cnt = 0;
    for (int n_y_px = n_begin_y_px; n_y_px < n_end_y_px; ++n_y_px)
        for (int n_x_px = n_begin_x_px; n_x_px < n_end_x_px; ++n_x_px)
            if (map_data.data[n_y_px * map_data.info.width + n_x_px] != 0) {
                ++cnt;
                // break;
            }
    float f_occ_ratio;
    f_occ_ratio = static_cast<float>(cnt) / static_cast<float>(n_sampling_size);
    if (f_occ_ratio > 0.03) {
        return 100;
    }
    return 0;
    // return (cnt < 3) ? 0 : 100;
    // return (cnt == 0) ? 0 : 100;
}

void GlobalMap::GetLocalMap(Map& o_local_map, const NaviFra::Pos o_robot_pos)  // const geometry_msgs::Pose& o_robot_pos)
{
    // find idx containing robot pos in global map
    // int n_global_robot_pos_x_px = (int)( (o_robot_pos.position.x - st_global_map_info_.f_origin_x_m) / st_global_map_info_.f_resolution_m
    // + 0.5f ); int n_global_robot_pos_y_px = (int)( (o_robot_pos.position.y - st_global_map_info_.f_origin_y_m) /
    // st_global_map_info_.f_resolution_m + 0.5f );
    int n_global_robot_pos_x_px =
        (int)((o_robot_pos.GetXm() - st_global_map_info_.f_origin_x_m) / st_global_map_info_.f_resolution_m + 0.5f);
    int n_global_robot_pos_y_px =
        (int)((o_robot_pos.GetYm() - st_global_map_info_.f_origin_y_m) / st_global_map_info_.f_resolution_m + 0.5f);
    // exception
    assert(n_global_robot_pos_x_px >= 0);
    assert(n_global_robot_pos_y_px >= 0);
    assert(n_global_robot_pos_x_px < st_global_map_info_.n_size_x);
    assert(n_global_robot_pos_y_px < st_global_map_info_.n_size_y);
    // set local map info
    int n_l2g_x_px = n_global_robot_pos_x_px - o_local_map.GetXpx() / 2;
    int n_l2g_y_px = n_global_robot_pos_y_px - o_local_map.GetYpx() / 2;
    o_local_map.SetMapOrigin(
        n_l2g_x_px * st_global_map_info_.f_resolution_m + st_global_map_info_.f_origin_x_m,
        n_l2g_y_px * st_global_map_info_.f_resolution_m + st_global_map_info_.f_origin_y_m);
    // crop global map for local map
    // based on global
    int n_begin_x_px = std::max(n_l2g_x_px, 0);
    int n_begin_y_px = std::max(n_l2g_y_px, 0);
    int n_end_x_px = std::min(n_l2g_x_px + o_local_map.GetXpx(), st_global_map_info_.n_size_x);
    int n_end_y_px = std::min(n_l2g_y_px + o_local_map.GetYpx(), st_global_map_info_.n_size_y);
    // initialize
    int n_local_size_x = o_local_map.GetXpx();
    std::vector<int8_t>* vec_local_map_ptr = o_local_map.GetMapPointer();
    std::fill(vec_local_map_ptr->begin(), vec_local_map_ptr->end(), 100);
    // << std::endl;
    for (int n_y_px = n_begin_y_px; n_y_px < n_end_y_px; ++n_y_px) {
        std::copy(
            vec_global_map_.begin() + n_y_px * st_global_map_info_.n_size_x + n_begin_x_px,
            vec_global_map_.begin() + n_y_px * st_global_map_info_.n_size_x + n_end_x_px,
            vec_local_map_ptr->begin() + (n_y_px - n_l2g_y_px) * n_local_size_x + (n_begin_x_px - n_l2g_x_px));
        // for (int n_x_px = n_begin_x_px; n_x_px < n_end_x_px; ++n_x_px)
        //   vec_local_map_ptr->at( (n_y_px - n_l2g_y_px) * n_local_size_x + (n_x_px - n_l2g_x_px) ) = vec_global_map_[n_y_px *
        //   st_global_map_info_.n_size_x + n_x_px];
    }
}

void GlobalMap::GetGlobalMap(std::vector<int8_t>& vec_global_map)  //, Map& o_global_map);
{
    vec_global_map = vec_global_map_;
    // o_global_map.
}

void GlobalMap::UpdateGridCellUisngLiDAR(
    Map& o_local_map, const vector<NaviFra::SimplePos>& vec_data, const NaviFra::Pos& o_map_to_localmap)
{
    NaviFra::Pos localmap_to_robot(o_local_map.GetXpx() / 2, o_local_map.GetYpx() / 2, 0);
    // float f_occupancy_probability = 0.85;
    vec_local_map_ = o_local_map.GetMap();

    for (int i = 0; i < vec_data.size(); i++) {
        NaviFra::SimplePos robot_to_point = vec_data[i];
        auto start = NaviFra::SimplePos(0, 0, 0);
        auto end = NaviFra::SimplePos(
            robot_to_point.GetXm() / st_global_map_info_.f_resolution_m, robot_to_point.GetYm() / st_global_map_info_.f_resolution_m, 0);
        auto localmap_to_pixel = localmap_to_robot * end;
        auto rotated_end = NaviFra::SimplePos(0, 0, o_map_to_localmap.GetDeg()) * end;

        int n_idx_x = rotated_end.GetXm() + o_local_map.GetXpx() / 2 + 0.5;
        int n_idx_y = rotated_end.GetYm() + o_local_map.GetYpx() / 2 + 0.5;

        if (n_idx_x < 0 || n_idx_y < 0 || n_idx_x >= o_local_map.GetXpx() || n_idx_y >= o_local_map.GetYpx()) {
            continue;
        }
        int n_idx = n_idx_x + n_idx_y * o_local_map.GetXpx();
        vec_local_map_[n_idx] = 100;
    }
    o_local_map.SetMap(vec_local_map_);
}

void GlobalMap::InflateMap(const int size_x, const int size_y, Map& o_local_map, const float f_padding_size)
{
    std::vector<int8_t>* vec_localmap_ptr = o_local_map.GetMapPointer();
    int n_padding_size = (int)(f_padding_size / st_global_map_info_.f_resolution_m + 0.5f);

    for (int map_idx = 0; map_idx < o_local_map.GetXpx() * o_local_map.GetYpx(); ++map_idx) {
        int map_y = map_idx / size_x;
        int map_x = map_idx - map_y * size_x;

        // ignore no occupied
        if (vec_localmap_ptr->at(map_idx) != 100)
            continue;

        // ignore surronding occupied
        if ((map_x == 0 || vec_localmap_ptr->at(map_idx - 1) == 100) && (map_x == size_x - 1 || vec_localmap_ptr->at(map_idx + 1) == 100) &&
            (map_y == 0 || vec_localmap_ptr->at(map_idx - size_x) == 100) &&
            (map_y == size_y - 1 || vec_localmap_ptr->at(map_idx + size_x) == 100))
            continue;

        for (std::vector<Padding>::const_iterator it = vec_padding_lt_.begin(); it != vec_padding_lt_.end(); ++it) {
            int idx_x = map_x + it->n_px;
            int idx_y = map_y + it->n_py;
            int padding_idx = idx_x + idx_y * size_x;

            // Check if the index is within bounds
            if (idx_x < 0 || idx_x >= size_x || idx_y < 0 || idx_y >= size_y)
                continue;

            if (vec_localmap_ptr->at(padding_idx) == 100)
                continue;

            if (vec_localmap_ptr->at(padding_idx) < it->cost)
                vec_localmap_ptr->at(padding_idx) = it->cost;
        }
    }
}
