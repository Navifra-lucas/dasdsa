/*
 * @file	: msg_converter.hpp
 * @date	: Mar 27, 2025
 * @author	:"HaeChang, Kim(henry)" (henry@navifra.com)"
 * @brief	: converter msg to util struct
 * @remark	:
 * @warning	:
 * Copyright(C) 2025 NaviFra Coperation. All Rights are Reserved.
 */

#ifndef MSG_CONVERTER_HPP_
#define MSG_CONVERTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "answer_msgs/DriveInfo.h"
#include "answer_msgs/EdgeInfo.h"
#include "answer_msgs/MissionAlign.h"
#include "answer_msgs/MissionNodePath.h"
#include "answer_msgs/MissionExplore.h"
#include "answer_msgs/MissionNodeInfo.h"
#include "answer_msgs/MissionProgress.h"
#include "answer_msgs/MotionStatus.h"
#include "answer_msgs/NaviInfo.h"
#include "answer_msgs/NodeInfo.h"

#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"

#include "utils/grid_map/grid_map.hpp"
#include "utils/mission_bundle.hpp"
#include "utils/mission_edge.hpp"
#include "utils/mission_info.hpp"
#include "utils/motion_planner_type_list.hpp"
#include "utils/navi_info.hpp"
#include "utils/pose.hpp"

using namespace NVFR;

namespace MsgConverter {

template <typename I, typename O>
O Convert(const I &);

// Msg to Data
template <>
Pose Convert<geometry_msgs::Pose2D, Pose>(
    const geometry_msgs::Pose2D&);

template <>
Arr3d Convert<geometry_msgs::Pose2D, Arr3d>(
    const geometry_msgs::Pose2D&);

template <>
Pose Convert<geometry_msgs::Pose, Pose>(
    const geometry_msgs::Pose&);

template <>
Arr3d Convert<geometry_msgs::Pose, Arr3d>(
    const geometry_msgs::Pose&);

template <>
DriveInfo Convert<answer_msgs::DriveInfo, DriveInfo>(
    const answer_msgs::DriveInfo&);

template <>
NodeInfo Convert<answer_msgs::NodeInfo, NodeInfo>(
    const answer_msgs::NodeInfo&);

template <>
AlignInfo Convert<answer_msgs::MissionAlign, AlignInfo>(
    const answer_msgs::MissionAlign&);

template <>
MissionEdge Convert<answer_msgs::EdgeInfo, MissionEdge>(
    const answer_msgs::EdgeInfo&);

template <>
MissionBundle_t Convert<answer_msgs::MissionAlign, MissionBundle_t>(
    const answer_msgs::MissionAlign&);

template <>
MissionBundle_t Convert<answer_msgs::MissionNodePath, MissionBundle_t>(
    const answer_msgs::MissionNodePath&);

template <>
MissionBundle_t Convert<answer_msgs::MissionExplore, MissionBundle_t>(
    const answer_msgs::MissionExplore&);

template <>
Polygon Convert<sensor_msgs::LaserScan, Polygon>(
    const sensor_msgs::LaserScan&);

template <>
Polygon Convert<std::vector<geometry_msgs::Point32>, Polygon>(
    const std::vector<geometry_msgs::Point32>&);

template <>
pcl::PointCloud<pcl::PointXYZ>::Ptr Convert<sensor_msgs::LaserScan, pcl::PointCloud<pcl::PointXYZ>::Ptr>(
    const sensor_msgs::LaserScan&);

template <>
pcl::PointCloud<pcl::PointXYZ>::Ptr Convert<sensor_msgs::PointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr>(
    const sensor_msgs::PointCloud&);

template <>
MapInfo_t Convert<nav_msgs::MapMetaData, MapInfo_t>(
    const nav_msgs::MapMetaData&);

// Data to Msg
template <>
geometry_msgs::Pose2D
    Convert<Pose, geometry_msgs::Pose2D>(
        const Pose&);

template <>
geometry_msgs::Pose2D
    Convert<Arr3d, geometry_msgs::Pose2D>(
        const Arr3d&);

template <>
geometry_msgs::Pose
    Convert<Pose, geometry_msgs::Pose>(
        const Pose&);

template <>
geometry_msgs::Pose
    Convert<Arr3d, geometry_msgs::Pose>(
        const Arr3d&);

template <>
answer_msgs::DriveInfo
    Convert<DriveInfo, answer_msgs::DriveInfo>(
        const DriveInfo&);

template <>
answer_msgs::MissionAlign
    Convert<AlignInfo, answer_msgs::MissionAlign>(
        const AlignInfo&);

template <>
answer_msgs::NodeInfo
    Convert<NodeInfo, answer_msgs::NodeInfo>(
        const NodeInfo&);

template <>
answer_msgs::NaviInfo
    Convert<NaviInfo, answer_msgs::NaviInfo>(
        const NaviInfo&);

}  // namespace MsgConverter

#endif
