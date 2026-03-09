#ifndef NAVIFRA_ASTART_HPP_
#define NAVIFRA_ASTART_HPP_

#include "map/map.hpp"
#include "pos/pos.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <queue>
#include <set>
#include <vector>

namespace NaviFra {
struct MapInfo {
    int n_x_pixel, n_y_pixel, n_map_size;
    float f_x_origin_m, f_y_origin_m;
    float f_resolution_m;
    std::shared_ptr<std::vector<int8_t>> vec_map;
};

struct PixelXY {
    int x, y;
    bool operator==(const PixelXY& st_pixel);
};

class Node {
public:
    float G, H;
    PixelXY st_pixel_;
    int n_closest_id;
    std::shared_ptr<Node> st_parent_node_;

    Node(PixelXY st_pixel, std::shared_ptr<Node> st_parent = nullptr)
    {
        st_parent_node_ = st_parent;
        st_pixel_ = st_pixel;
        G = H = 0;
        n_closest_id = -1;
    };
    ~Node(){};

    float GetScore() { return G + H; };
};

using uint = unsigned int;
class AStar {
public:
    const float COST_WEIGHT = 0.2f;

    AStar();
    void SetMap(std::shared_ptr<NaviFra::Map> o_map);
    void SetBasePath(std::vector<NaviFra::Pos>* vec_path_ptr);
    void Set16Direction(bool);
    std::vector<NaviFra::Pos> FindPath(int n_start_x_pixel, int n_start_y_pixel, int n_target_x_pixel, int n_target_y_pixel);
    bool DetectCollision(PixelXY st_pixel);
    bool DetectCollision(PixelXY st_pixel, PixelXY st_start, PixelXY st_goal);

private:
    std::vector<PixelXY> o_direction_;
    std::shared_ptr<MapInfo> ptr_map_;
    std::vector<NaviFra::Pos>* vec_base_path_ptr_ = nullptr;

    uint un_directions_;
    std::vector<float> vec_direction_cost_;
    std::vector<std::shared_ptr<Node>> vec_visit_;
    static PixelXY GetDelta(PixelXY st_source, PixelXY st_target);
    static uint Manhattan(PixelXY st_source, PixelXY st_target);
    static uint Euclidean(PixelXY st_source, PixelXY st_target);
    static uint Octagonal(PixelXY st_source, PixelXY st_target);

    int findClosestWaypointIdx(const PixelXY& st_cur_pos, int n_near_id = -1);
};

struct compare {
    bool operator()(std::shared_ptr<Node> a, std::shared_ptr<Node> b) { return a->GetScore() > b->GetScore(); }
};
}  // namespace NaviFra

#endif
