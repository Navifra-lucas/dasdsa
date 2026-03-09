#ifndef MAP_BASED_PATH_PLANNER_HPP_
#define MAP_BASED_PATH_PLANNER_HPP_
#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"
#include "astar/astar.hpp"
#include "core/map/map.hpp"
#include "debug/debug_visualizer.hpp"
#include "node/navi_node.hpp"
#include "path_smoothing/path_smoothing.h"
#include "pos/pos.hpp"
// #include "map/sensor_map.hpp"
#include "msg_information/parameter_information/param_msg.hpp"

#include <opencv2/opencv.hpp>

namespace NaviFra {
class MapPlanner {
private:
    typedef float Weight;
    // NaviFra::Dijkstra o_dijkstra_algorithm_;
    NaviFra::AStar o_astar_algorithm_;
    Parameters_t st_param_;
    PathSmoothing o_path_smoothing_;

private:
    std::vector<NaviFra::NaviNode> vec_topology_node_;
    std::vector<NaviFra::NaviNode> vec_only_node_path_;

public:
    MapPlanner();
    virtual ~MapPlanner();

    std::vector<NaviFra::Pos> PlanPath(const NaviFra::Pos& o_robot_pos, const std::string& str_node_id);
    std::vector<NaviFra::Pos> PlanPath(const NaviFra::Pos& o_robot_pos, const NaviFra::Pos& o_goal_pos);
    std::vector<NaviFra::Pos> PlanPath(const vector<std::string>& vec_node_id);
    std::vector<NaviFra::Pos> PlanPath(const NaviFra::Pos& o_goal_pos, const float& f_head_rad);
    // std::vector<Pos> PlanPath(const NaviFra::Pos &o_robot_pos, const NaviFra::Pos &o_goal_pos, const float f_global_xm, const float
    // f_global_ym, const float f_global_res);
    // std::vector<Pos> PlanPath(const NaviFra::Pos &o_robot_pos, const NaviFra::NaviNode &o_goal_node);

    std::vector<NaviNode> GetNaviNodeVec() { return vec_only_node_path_; };
    std::vector<NaviFra::NaviNode> FindNeighborNodes(const NaviFra::Pos& o_robot_pos, const int& n_find_num = 999999);

    void SetMap(const std::vector<NaviNode>& vec_topological_map);
    void SetMap(const NaviFra::Map& o_navi_map);
    void SetBasePath(std::vector<NaviFra::Pos>* vec_path_ptr);
    void SetParameter(const Parameters_t& param) { st_param_ = param; };

    NaviFra::Pos GetLocalGoal(const NaviFra::Pos& o_robot_pos, const vector<Pos> vec_global_path);

    bool GetOnPath(const Pos& o_robot_pos);

    NaviFra::Pos GetNodePoseByName(const string& node_name);
    NaviFra::Pos GetNodePoseByID(const string& node_id)
    {
        NaviFra::Pos result;
        return result;
    };
    std::string GetNodeNameByNodeID(const string& node_id) { return ""; };
    bool ExistObstaclePath(const NaviFra::Pos& o_robot_pos, const vector<Pos> vec_local_path, int* n_collision_idx = nullptr);
    void GetNearGoalIndex(int& n_goal_px, int& n_goal_py);
    void GetFarGoalIndex(int& n_goal_px, int& n_goal_py);

    bool ExistObstacleOnLocalGoal(const NaviFra::Pos& o_local_goal);

    // void SaveMap2(int size_x,int size_y, std::make_shared<std::vector<char>> prob_map, const string function_name, int x1, int y1);
    void SaveMap2(int size_x, int size_y, const string function_name, int x1, int y1);

    float f_ori_x_;
    float f_ori_y_;
    float f_res_;

private:
    std::shared_ptr<NaviFra::Map> o_navi_map_;
    // float f_ori_x_;
    // float f_ori_y_;
    bool b_path_smoothing_ = true;
    std::mutex mtx_set_map_;
};
};  // namespace NaviFra

#endif
