#ifndef TOPIC_KEY_HPP_
#define TOPIC_KEY_HPP_

#include <string>

namespace NVFR {

namespace KEY {

enum class TOPICS {
    ODOM,
    LIDAR,
    IMU,
    ROBOT_POSE,
    MAP,
    PARAM_READ_ALL,
    PARAM_READ_NODE,
    LOG_LV_ALL,
    LOG_LV_NODE,
    NAVI_MISSION_ALIGN,
    NAVI_MISSION_NODE_PATH,
    NAVI_MISSION_EXPLORE,
    NAVI_CMD_SIGNAL,
    EXPR_CMD_SIGNAL,
    CMD_VEL,
    NAVI_MISSION_RESPONSE,
    NAVI_STATUS,
    NAVI_ERROR,
    NAVI_INFO,
    POSE_GRAPH_NODE,
    EXPR_MISSION_STATUS,
};

enum class SERVICES {
    GLOBAL_MAP,
    GRAPH_MAP_DIRECTORY,
    GRAPH_MAP_LOAD_MAP,
    GRAPH_MAP_LOCALIZATION,
    GRAPH_MAP_MISSION_NODE,
    GRAPH_MAP_MISSION_NODE_LIST,
};

enum class ACTIONS {
    //
};

} // KEY

namespace NAME {

namespace TOPICS {

// sub-data
inline const std::string LIDAR = "lidar";
// sub-cmd(common)
inline const std::string PARAM_READ_ALL = "answer/param/read/all";
inline const std::string PARAM_READ_NODE = "answer/param/read/";
inline const std::string LOG_LV_ALL = "answer/log_lv/all";
inline const std::string LOG_LV_NODE = "answer/log_lv/";
// sub-cmd(NAVI)
inline const std::string NAVI_MISSION_ALIGN = "answer/navigator/mission_align";
inline const std::string NAVI_MISSION_NODE_PATH = "answer/navigator/mission_node_path";
inline const std::string NAVI_MISSION_EXPLORE = "answer/navigator/mission_explore";
inline const std::string NAVI_CMD_SIGNAL = "answer/navigator/cmd_signal";
// sub-cmd(EXPR)
inline const std::string EXPR_CMD_SIGNAL = "answer/explorer/cmd_signal";
// pub-data
inline const std::string CMD_VEL = "cmd_vel";
// pub-status(NAVI)
inline const std::string NAVI_MISSION_RESPONSE = "answer/navigator/mission_response";
inline const std::string NAVI_STATUS = "answer/navigator/status";
inline const std::string NAVI_ERROR = "answer/navigator/error";
inline const std::string NAVI_INFO = "answer/navigator/info";
// pub-status(EXPR)
inline const std::string EXPR_MISSION_STATUS = "answer/explorer/status";

} // namespace TOPICS

namespace SERVICES {

inline const std::string GRAPH_MAP_DIRECTORY = "answer/graph_map/directory";
inline const std::string GRAPH_MAP_LOAD_MAP = "answer/graph_map/load_map";
inline const std::string GRAPH_MAP_LOCALIZATION = "answer/graph_map/localization";
inline const std::string GRAPH_MAP_MISSION_NODE = "answer/graph_map/mission/node";
inline const std::string GRAPH_MAP_MISSION_NODE_LIST = "answer/graph_map/mission/node_list";

} // namespace SERVICES

namespace ACTIONS {

//

} // namespace ACTIONS

namespace TFS {

inline const std::string BASE_LINK = "base_link";

} // namespace TFS

namespace MSG_TYPES {

inline const std::string GEO_POSE = "geometry_msgs/pose";
inline const std::string GEO_POSE_STMP = "geometry_msgs/pose_stamped";
inline const std::string GEO_POSE_WITH_COV_STMP = "geometry_msgs/pose_with_covariance_stamped";
inline const std::string GEO_TWIST = "geometry_msgs/twist";
inline const std::string GEO_TWIST_STMP = "geometry_msgs/twist_stamped";
inline const std::string NAV_ODOMETRY = "nav_msgs/odometry";
inline const std::string SEN_LASERSCAN = "sensor_msgs/laser_scan";
inline const std::string SEN_PCD = "sensor_msgs/point_cloud";

} // namespace MSG_TYPES

} // namespace NAMES

} // namespace NVFR

#endif
