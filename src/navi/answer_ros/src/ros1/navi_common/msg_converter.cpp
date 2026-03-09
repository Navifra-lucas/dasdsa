#include "ros1/navi_common/msg_converter.hpp"

#include "logger/logger.h"

#include "utils/common_math.hpp"

namespace MsgConverter {

template <typename I, typename O>
O Convert(const I&)
{
    return O();
}

// Msg to Data
template <>
Pose Convert<geometry_msgs::Pose2D, Pose>(
    const geometry_msgs::Pose2D& msg)
{
    return Pose(msg.x, msg.y, msg.theta * CM::Deg2Rad);
}

template <>
Arr3d Convert<geometry_msgs::Pose2D, Arr3d>(
    const geometry_msgs::Pose2D& msg)
{
    return {msg.x, msg.y, msg.theta * CM::Deg2Rad};
}

template <>
Pose Convert<geometry_msgs::Pose, Pose>(
    const geometry_msgs::Pose& msg)
{
    Pose o_pose(msg.position.x, msg.position.y, msg.position.z, CM::Quaternion2Yaw(
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w));
    return o_pose;
}

template <>
Arr3d Convert<geometry_msgs::Pose, Arr3d>(
    const geometry_msgs::Pose& msg)
{
    return {msg.position.x, msg.position.y, CM::Quaternion2Yaw(
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)};
}

template <>
DriveInfo Convert<answer_msgs::DriveInfo, DriveInfo>(
    const answer_msgs::DriveInfo& msg)
{
    DriveInfo o_drive_info;
    if (MPTL::IsValid<MPTL::MOVEDIR>(msg.n_move_dir)) {
        o_drive_info.SetMoveDir(static_cast<MPTL::MOVEDIR>(msg.n_move_dir));
    }
    if (MPTL::IsValid<MPTL::DRIVE>(msg.n_drive_type)) {
        o_drive_info.SetDriveType(
            static_cast<MPTL::DRIVE>(msg.n_drive_type));
    }
    if (MPTL::IsValid<MPTL::START>(msg.n_start_type)) {
        o_drive_info.SetStartType(static_cast<MPTL::START>(msg.n_start_type));
    }
    if (MPTL::IsValid<MPTL::GOAL>(msg.n_goal_type)) {
        o_drive_info.SetGoalType(static_cast<MPTL::GOAL>(msg.n_goal_type));
    }
    return o_drive_info;
}

template <>
NodeInfo Convert<answer_msgs::NodeInfo, NodeInfo>(
    const answer_msgs::NodeInfo& msg)
{
    NodeInfo o_node_info(
        msg.s_id, msg.s_name,
        Convert<geometry_msgs::Pose2D, Pose>(msg.o_pose));
    return o_node_info;
}

template <>
AlignInfo Convert<answer_msgs::MissionAlign, AlignInfo>(
    const answer_msgs::MissionAlign& msg)
{
    AlignInfo o_align_info;

    MPTL::ALIGNDIR e_align_dir = MPTL::ALIGNDIR::AUTO;
    if (MPTL::IsValid<MPTL::ALIGNDIR>(msg.n_align_direction)) {
        e_align_dir = static_cast<MPTL::ALIGNDIR>(msg.n_align_direction);
    }
    else {
        LOG_ERROR("Wrong align direction ({})", msg.n_align_direction);
    }
    o_align_info.SetAlignDirection(e_align_dir);
    o_align_info.SetAlignTargetRad(msg.d_align_target_deg * CM::Deg2Rad);
    return o_align_info;
}

template <>
MissionEdge Convert<answer_msgs::EdgeInfo, MissionEdge>(
    const answer_msgs::EdgeInfo& msg)
{
    MissionEdge o_mission_edge;

    MPTL::EDGE_PATH e_path_type = MPTL::EDGE_PATH::LINE;
    if (MPTL::IsValid<MPTL::EDGE_PATH>(msg.n_path_type)) {
        e_path_type = static_cast<MPTL::EDGE_PATH>(msg.n_path_type);
    }
    else {
        LOG_ERROR("Wrong path type ({})", msg.n_path_type);
    }
    o_mission_edge.SetPathType(e_path_type);

    o_mission_edge.SetDriveInfo(
        Convert<answer_msgs::DriveInfo, DriveInfo>(msg.o_drive_info));
    o_mission_edge.SetFromNode(
        Convert<answer_msgs::NodeInfo, NodeInfo>(msg.o_from_node));
    o_mission_edge.SetToNode(
        Convert<answer_msgs::NodeInfo, NodeInfo>(msg.o_to_node));
    o_mission_edge.SetFromNodeControl(
        Convert<geometry_msgs::Pose2D, Pose>(msg.o_from_node_control));
    o_mission_edge.SetToNodeControl(
        Convert<geometry_msgs::Pose2D, Pose>(msg.o_to_node_control));
    o_mission_edge.SetTimeControl(msg.d_time_control);
    o_mission_edge.SetTargetSpd(msg.d_target_speed);
    o_mission_edge.SetRobotYaw(msg.d_robot_yaw);
    return o_mission_edge;
}

template <>
MissionBundle_t Convert<answer_msgs::MissionAlign, MissionBundle_t>(
    const answer_msgs::MissionAlign& msg)
{
    MissionBundle_t st_mission_bundle;
    st_mission_bundle.uuid = msg.uuid;
    st_mission_bundle.e_mission_type = MPTL::MISSION::ALIGN;

    st_mission_bundle.o_align_info =
        Convert<answer_msgs::MissionAlign, AlignInfo>(msg);
    return st_mission_bundle;
}

template <>
MissionBundle_t Convert<answer_msgs::MissionNodePath, MissionBundle_t>(
    const answer_msgs::MissionNodePath& msg)
{
    MissionBundle_t st_mission_bundle;
    st_mission_bundle.uuid = msg.uuid;
    st_mission_bundle.e_mission_type = MPTL::MISSION::NODE_PATH;

    for (const auto& edge_info : msg.edge_array) {
        st_mission_bundle.vec_mission_edge.emplace_back(
            Convert<answer_msgs::EdgeInfo, MissionEdge>(edge_info));
    }

    st_mission_bundle.b_arrive_align = false;
    if (msg.b_arrive_align) {
        st_mission_bundle.b_arrive_align = true;
        st_mission_bundle.o_align_info =
            Convert<answer_msgs::MissionAlign, AlignInfo>(msg.o_arrive_align);
    }
    return st_mission_bundle;
}

template <>
MissionBundle_t Convert<answer_msgs::MissionExplore, MissionBundle_t>(
    const answer_msgs::MissionExplore& msg)
{
    MissionBundle_t st_mission_bundle;
    st_mission_bundle.uuid = msg.uuid;
    st_mission_bundle.e_mission_type = MPTL::MISSION::EXPLORE;

    MissionEdge o_mission_edge;
    o_mission_edge.SetTargetSpd(msg.d_target_speed);
    o_mission_edge.SetToNodePose(
        Convert<geometry_msgs::Pose2D, Pose>(
            msg.o_target_pose));
    st_mission_bundle.vec_mission_edge.emplace_back(o_mission_edge);
    return st_mission_bundle;
}

template <>
Polygon Convert<sensor_msgs::LaserScan, Polygon>(
    const sensor_msgs::LaserScan& msg)
{
    std::vector<Pos> vec_pos;
    for (size_t i = 0; i < msg.ranges.size(); ++i) {
        float angle = msg.angle_min + i * msg.angle_increment;
        float range = msg.ranges[i];
        if (range < msg.range_min || range > msg.range_max) {
            continue;
        }
        float x = range * cosf(angle);
        float y = range * sinf(angle);
        vec_pos.emplace_back(Pos(x, y));
    }
    return vec_pos;
}

template <>
Polygon Convert<std::vector<geometry_msgs::Point32>, Polygon>(
    const std::vector<geometry_msgs::Point32>& points)
{
    std::vector<Pos> vec_pos;
    for (const auto& p : points) {
        vec_pos.emplace_back(Pos(p.x, p.y, p.z));
    }
    return vec_pos;
}

template <>
pcl::PointCloud<pcl::PointXYZ>::Ptr Convert<sensor_msgs::LaserScan, pcl::PointCloud<pcl::PointXYZ>::Ptr>(
    const sensor_msgs::LaserScan& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->header.frame_id = msg.header.frame_id;
    for (size_t i = 0; i < msg.ranges.size(); ++i) {
        float angle = msg.angle_min + i * msg.angle_increment;
        float range = msg.ranges[i];
        if (range < msg.range_min || range > msg.range_max) {
            continue;
        }
        pcl::PointXYZ point;
        point.x = range * cosf(angle);
        point.y = range * sinf(angle);
        point.z = 0.0;
        cloud->points.push_back(point);
    }
    return cloud;
}

template <>
pcl::PointCloud<pcl::PointXYZ>::Ptr Convert<sensor_msgs::PointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr>(
    const sensor_msgs::PointCloud& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->header.frame_id = msg.header.frame_id;
    for (const auto& p : msg.points) {
        pcl::PointXYZ point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        cloud->points.push_back(point);
    }
    return cloud;
}

template <>
MapInfo_t Convert<nav_msgs::MapMetaData, MapInfo_t>(
    const nav_msgs::MapMetaData& msg)
{
    MapInfo_t st_map_info;
    st_map_info.n_size_x_px = msg.width;
    st_map_info.n_size_y_px = msg.height;
    st_map_info.d_resolution_m = msg.resolution;
    st_map_info.d_origin_x_m = -1.0 * msg.origin.position.x;
    st_map_info.d_origin_y_m = -1.0 * msg.origin.position.y;
    return st_map_info;
}

// Data to Msg
template <>
geometry_msgs::Pose2D
    Convert<Pose, geometry_msgs::Pose2D>(
        const Pose& o_pose)
{
    geometry_msgs::Pose2D msg;
    msg.x = o_pose.GetXm();
    msg.y = o_pose.GetYm();
    msg.theta = o_pose.GetDeg();
    return msg;
}

template <>
geometry_msgs::Pose2D
    Convert<Arr3d, geometry_msgs::Pose2D>(
        const Arr3d& arr)
{
    geometry_msgs::Pose2D msg;
    msg.x = arr[0];
    msg.y = arr[1];
    msg.theta = arr[2] * CM::Rad2Deg;
    return msg;
}

template <>
geometry_msgs::Pose
    Convert<Pose, geometry_msgs::Pose>(
        const Pose& o_pose)
{
    geometry_msgs::Pose msg;
    msg.position.x = o_pose.GetXm();
    msg.position.y = o_pose.GetYm();
    msg.position.z = o_pose.GetZm();
    auto q = CM::Yaw2Quaternion(o_pose.GetRad());
    msg.orientation.x = q[0];
    msg.orientation.y = q[1];
    msg.orientation.z = q[2];
    msg.orientation.w = q[3];
    return msg;
}

template <>
geometry_msgs::Pose
    Convert<Arr3d, geometry_msgs::Pose>(
        const Arr3d& arr)
{
    geometry_msgs::Pose msg;
    msg.position.x = arr[0];
    msg.position.y = arr[1];
    msg.position.z = 0.0;
    auto q = CM::Yaw2Quaternion(arr[2]);
    msg.orientation.x = q[0];
    msg.orientation.y = q[1];
    msg.orientation.z = q[2];
    msg.orientation.w = q[3];
    return msg;
}

template <>
answer_msgs::DriveInfo
    Convert<DriveInfo, answer_msgs::DriveInfo>(
        const DriveInfo& o_drive_info)
{
    answer_msgs::DriveInfo msg;
    switch (o_drive_info.GetMoveDir()) {
        case MPTL::MOVEDIR::FORWARD:
            msg.n_move_dir = answer_msgs::DriveInfo::FORWARD;
            break;
        case MPTL::MOVEDIR::BACKWARD:
            msg.n_move_dir = answer_msgs::DriveInfo::BACKWARD;
            break;
    }
    switch (o_drive_info.GetDriveType()) {
        case MPTL::DRIVE::BIKE:
            msg.n_drive_type = answer_msgs::DriveInfo::BIKE;
            break;
        case MPTL::DRIVE::QUAD:
            msg.n_drive_type = answer_msgs::DriveInfo::QUAD;
            break;
    }
    switch (o_drive_info.GetStartType()) {
        case MPTL::START::NORMAL_START:
            msg.n_start_type = answer_msgs::DriveInfo::NORMAL;
            break;
        case MPTL::START::SLOW_START:
            msg.n_start_type = answer_msgs::DriveInfo::SLOW;
            break;
        case MPTL::START::DOCKING_START:
            msg.n_start_type = answer_msgs::DriveInfo::SLOW;
            break;
    }
    switch (o_drive_info.GetGoalType()) {
        case MPTL::GOAL::NORMAL_GOAL:
            msg.n_goal_type = answer_msgs::DriveInfo::NORMAL;
            break;
        case MPTL::GOAL::SLOW_GOAL:
            msg.n_goal_type = answer_msgs::DriveInfo::SLOW;
            break;
        case MPTL::GOAL::DOCKING_GOAL:
            msg.n_goal_type = answer_msgs::DriveInfo::SLOW;
            break;
    }

    return msg;
}

template <>
answer_msgs::MissionAlign
    Convert<AlignInfo, answer_msgs::MissionAlign>(
        const AlignInfo& o_align_info)
{
    answer_msgs::MissionAlign msg;
    msg.n_align_direction = static_cast<int>(o_align_info.GetAlignDirection());
    msg.d_align_target_deg = o_align_info.GetTargetRad() * CM::Rad2Deg;
}

template <>
answer_msgs::NodeInfo
    Convert<NodeInfo, answer_msgs::NodeInfo>(
        const NodeInfo& o_node_info)
{
    answer_msgs::NodeInfo msg;
    msg.s_id = o_node_info.GetId();
    msg.s_name = o_node_info.GetName();
    msg.o_pose =
        Convert<Pose, geometry_msgs::Pose2D>(
            o_node_info.GetConstPose());
    return msg;
}

template <>
answer_msgs::NaviInfo
    Convert<NaviInfo, answer_msgs::NaviInfo>(
    const NaviInfo& o_navi_info)
{
    answer_msgs::NaviInfo msg;

    // mission type
    switch (o_navi_info.e_mission) {
        case MPTL::MISSION::IDLE:
            msg.n_mission_type = answer_msgs::NaviInfo::IDLE;
            break;
        case MPTL::MISSION::ALIGN:
            msg.n_mission_type = answer_msgs::NaviInfo::ALIGN;
            break;
        case MPTL::MISSION::NODE_PATH:
            msg.n_mission_type = answer_msgs::NaviInfo::NODE_PATH;
            break;
        case MPTL::MISSION::START_PATH:
            msg.n_mission_type = answer_msgs::NaviInfo::START_PATH;
            break;
        case MPTL::MISSION::SEARCH_PATH:
            msg.n_mission_type = answer_msgs::NaviInfo::SEARCH_PATH;
            break;
        case MPTL::MISSION::EXPLORE:
            msg.n_mission_type = answer_msgs::NaviInfo::EXPLORE;
            break;
    }

    // drive info
    msg.o_drive_info =
        Convert<DriveInfo, answer_msgs::DriveInfo>(
            o_navi_info.o_drive_info);

    // mission nodes info
    msg.o_mission_nodes.o_goal_node =
        Convert<NodeInfo, answer_msgs::NodeInfo>(
            o_navi_info.o_goal_node);
    msg.o_mission_nodes.o_target_node =
        Convert<NodeInfo, answer_msgs::NodeInfo>(
            o_navi_info.o_target_node);
    msg.o_mission_nodes.o_curr_node =
        Convert<NodeInfo, answer_msgs::NodeInfo>(
            o_navi_info.o_curr_node);
    msg.o_mission_nodes.o_next_node =
        Convert<NodeInfo, answer_msgs::NodeInfo>(
            o_navi_info.o_next_node);

    // mission progress
    msg.o_mission_progress.f_align_target_deg = o_navi_info.d_align_target_deg;
    msg.o_mission_progress.f_remain_deg = o_navi_info.d_remain_deg;
    msg.o_mission_progress.f_total_length_curr_to_next_m = o_navi_info.d_total_length_curr_to_next_m;
    msg.o_mission_progress.f_total_length_start_to_target_m = o_navi_info.d_total_length_start_to_target_m;
    msg.o_mission_progress.f_traveled_length_curr_to_next_m = o_navi_info.d_traveled_length_curr_to_next_m;
    msg.o_mission_progress.f_traveled_length_start_to_target_m = o_navi_info.d_traveled_length_start_to_target_m;

    // motion status
    msg.o_motion_status.b_spin_turn = o_navi_info.b_spin_turn;
    msg.o_motion_status.b_pause = o_navi_info.b_pause;
    msg.o_motion_status.b_avoid = o_navi_info.b_avoid;
    msg.o_motion_status.b_stop = o_navi_info.b_stop;
    msg.o_motion_status.b_obstacle = o_navi_info.b_obstacle;
    msg.o_motion_status.b_collision = o_navi_info.b_collision;
    msg.o_motion_status.b_away_from_path = o_navi_info.b_away_from_path;
    msg.o_motion_status.n_linear_speed_percent = o_navi_info.n_linear_speed_percent;
    msg.o_motion_status.f_target_linear_vel_on_path_m = o_navi_info.d_target_linear_vel_on_path_m;

    // motion error
    msg.o_pose_error =
        Convert<Pose, geometry_msgs::Pose2D>(
            o_navi_info.o_pose_error);

    return msg;
}

}  // namespace MsgConverter