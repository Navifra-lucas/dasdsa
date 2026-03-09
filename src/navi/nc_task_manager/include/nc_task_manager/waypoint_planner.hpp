
#ifndef NAVIFRA_WAYPOINT_PLANNER_HPP_
#define NAVIFRA_WAYPOINT_PLANNER_HPP_
#include "node/navi_node.hpp"
#include "dijkstra/dijkstra.hpp"
#include "pos/pos.hpp"
#include "ros/ros.h"

#include <core_msgs/JsonList.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <math.h>

#include <cfloat>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

namespace NaviFra {

class WaypointPlanner {
private:
    typedef float Weight;
    float f_start_thr_dist_m_ = 0.25;
    std::vector<Pos> vec_path_;
    NaviFra::Dijkstra o_dijkstra_algorithm_;
    std::vector<NaviFra::NaviNode> vec_all_node_;
    std::vector<NaviFra::NaviEdge> vec_edge_data_;
    ros::Subscriber param_update_sub_;
    ros::NodeHandle nh_;

public:
    WaypointPlanner();
    virtual ~WaypointPlanner();


    void InitParameter();
    void RecvParamUpdate(const std_msgs::String::ConstPtr& msg);

    /**
     * @brief Set the Map object
     *
     * @param vec_topological_map
     */
    void SetMap(const std::vector<NaviNode>& vec_topological_map);
    NaviFra::Pos::DriveInfo_t GetDriveInfo(const NaviNode& node_from, const NaviNode& node_to);

    /**
     * @brief PlanPath using node id (nate)
     *
     * @param o_robot_pos
     * @param str_node_id
     * @return std::vector<Pos>
     */
    std::vector<NaviFra::NaviNode> PlanPath(const Pos& o_robot_pos, const std::string& str_node_id);
    std::vector<NaviFra::NaviNode> PlanPath(const vector<std::string>& vec_node_id);

    /**
     * @brief check all node && return match node list
     *
     * @param vec_node_id
     * @param vec_path_node_raw
     * @return bool
     */
    bool GetMatchingNodes(const vector<std::string>& vec_node_id, vector<NaviFra::NaviNode>& vec_path_node_raw);

    /**
     * @brief check all node && return match node
     *
     * @param str_node_id
     * @param o_node
     * @return bool
     */
    bool GetMatchingNode(const std::string& str_node_id, NaviFra::NaviNode& o_node);

    /**
     * @brief remove duplicate nodes
     *
     * @param vec_path_node_raw
     * @return bool
     */
    bool RemoveDuplicateNodes(const vector<NaviFra::NaviNode>& vec_path_node_raw, vector<NaviFra::NaviNode>& vec_path_node);

    /**
     * @brief Nodes not connected to the next will be linked via the shortest path, From node list
     *
     * @param vec_path_node
     * @param vec_path_node2
     * @return bool
     */
    bool ConnectNodePath(const vector<NaviFra::NaviNode>& vec_path_node, vector<NaviFra::NaviNode>& vec_path_node2);

    /**
     * @brief Nodes not connected to the next will be linked via the shortest path, From goal node
     *
     * @param o_start_node
     * @param o_goal_node
     * @param vec_path_node
     * @return bool
     */
    bool ConnectNodePath(
        const NaviFra::NaviNode& o_start_node, const NaviFra::NaviNode& o_goal_node, const NaviFra::NaviNode& o_edge_node,
        vector<NaviFra::NaviNode>& vec_path_node);

    /**
     * @brief Find Start Pos Algorithm && 연결된 노드 리스트 저장
     *
     * @param o_robot_pos
     * @param o_goal_node
     * @param vec_path_node
     * @return bool
     */
    bool GetNodePath(const Pos& o_robot_pos, const NaviFra::NaviNode& o_goal_node, vector<NaviFra::NaviNode>& vec_path_node);

    /**
     * @brief Get the On Path object check (nate)
     *
     * @param o_robot_pos
     * @return true
     * @return false
     */
    bool GetOnPath(const Pos& o_robot_pos);

    float CalcPosDistance_(const NaviFra::Pos& o_pos1, const NaviFra::Pos& o_pos2);
    int sign(float val);
    float CalcAngleDomainDeg_(float f_angle_deg);
    Pos TransformRotationRad_(const Pos& o_pos, float f_rad);
    Pos TransformRotationDeg_(const Pos& o_pos, float f_deg);
    float CalcDistanceFromDotToLine_(const Pos& o_line_s, const Pos& o_line_e, const Pos& o_target_pos);
    Pos CalcPosDotOnLine_(const Pos& o_line_s, const Pos& o_line_e, const Pos& o_target_pos);
    void SetMap(const core_msgs::JsonList& msg);

    NaviFra::Pos GetNodePoseByName(const string& node_name);
    NaviFra::Pos GetNodePoseByID(const string& node_id);
    std::string GetNodeNameByNodeID(const string& node_id);
    NaviFra::NaviNode GetNode(const string& node_id);

    /**
     * @brief MakeGraph using topological map node & link data (nate)
     *
     * @param o_navi_node
     */
    void MakeGraph(const NaviFra::NaviNode& o_navi_node);
    float CalcAngleDifferenceRad_(const NaviFra::Pos& o_pos, const NaviFra::Pos& o_pos2, const NaviFra::Pos& o_pos3);

    /**
     * @brief Find close nodes (nate)
     *
     * @param o_robot_pos
     * @param n_find_num
     * @return std::vector<NaviFra::NaviNode>
     */
    std::vector<NaviFra::NaviNode> FindNeighboringNodeVec(const NaviFra::Pos& o_robot_pos, int n_find_num = 999999);

private:
    /**
     * @brief Find close links (nate)
     *
     * @param o_robot_pos
     * @param n_find_num
     * @return std::vector<NaviFra::NaviEdge>
     */
    std::vector<NaviFra::NaviEdge> FindNeighboringEdgeVec(const NaviFra::Pos& o_robot_pos, int n_find_num = 999999);
};
};  // namespace NaviFra

#endif
