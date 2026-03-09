#pragma once

#include <string>
namespace ANSWER {
namespace KEY {
namespace SUBSCRIPTION {
enum class SENSOR
{
    ODOM,
    FRONT_LIDAR,
    REAR_LIDAR,
    POINT_CLOUD_2D,
    POINT_CLOUD_3D,
    POINT_CLOUD_3D_2,
    DEPTH_CAMERA_1,
    FORK_PLC_INFO,
    IMU,
    IMAGE_1,
    IMAGE_2,
    IMAGE_3,
    IMAGE_4,
    ROBOT_POSE
};
enum class COMMAND
{
    INIT_POSE,
    CORRECTION_TRIGGER,
    EXTERNAL_POSE,
    CLEAR_ALARM,
    MAP_SAVE,
    REGISTER_REFLECTOR,
    LOG_LEVEL,
    PERCEPTION_CMD,
    WINGBODY_CMD
};
enum class REQUEST
{
    REQUSET_GLOBAL_MAP,
};
}  // namespace SUBSCRIPTION
namespace PUBLICATION {
enum class SLAM
{
    MAP,
    POSE,
    POSE_GRAPH,
    POSE_GRAPH_NODE,
    MAPPING_PROGRESS,
    REPORT,
    STATUS,
    DETECTED_REFLECTORS,
    SET_REFLECTORS,
};

enum class SENSOR
{
    ODOM,
    FRONT_LIDAR,
    REAR_LIDAR,
    LEFT_LIDAR,
    RIGHT_LIDAR,
    MERGED_LIDAR,
    POINT_CLOUD_2D,
    POINT_CLOUD_3D,
    IMU,
    IMAGE_1,
    IMAGE_2,
    IMAGE_3,
    IMAGE_4,
};
enum class PERCEPTION
{
    WINGBODY_DETECTION,
    PALLET_POSE,

};
}  // namespace PUBLICATION

namespace SERVICES {
enum class MAP
{
    GLOBAL_MAP
};
enum class CONTROL
{
    ANSWER_CONTROL,
    START_SCAN,
    STOP_SCAN,
    SCAN_QUALITY,
    LOG_LEVEL,
    RECORD_DATA,
    UPDATE_PARAM,
    SAMPLING_MATCH,
};
}  // namespace SERVICES
namespace EXE_ARGUMENTS {
inline const std::string ROBOT_NAME = "robot_name";
inline const std::string OFFSET_PATH = "offset_path";
inline const std::string NOT_DEFINED = "not_defined";
// DOF
inline const std::string DOF = "dof";
inline const std::string ANSWER_2D = "2d";
inline const std::string ANSWER_3D = "3d";
// MODE
inline const std::string MODE = "mode";
inline const std::string SLAM = "slam";
inline const std::string LOCALIZATION = "localization";
}  // namespace EXE_ARGUMENTS
}  // namespace KEY

namespace TOPICS {
inline const std::string POINT_CLOUD_2D = "scan_cloud";
inline const std::string ODOM = "odom";
inline const std::string INIT_POSE = "initialpose";
inline const std::string ROBOT_POSE = "localization/robot_pos";
inline const std::string MAP = "map";
inline const std::string REPORT = "answer/report";
inline const std::string EXTERNAL_POSE = "external/robot_pos";
inline const std::string CLEAR_ALARM = "answer/clear_alarm";
inline const std::string CORRECTION_TRIGGER = "correction_init_pos";
inline const std::string ANSWER_STATUS = "answer/status";
inline const std::string POSE_GRAPH = "answer/pose_graph";
inline const std::string POSE_GRAPH_NODE = "answer/pose_graph_node";
inline const std::string MAPPING_PROGRESS = "answer/mapping_progress";
inline const std::string MAP_SAVE = "answer/map_save";
inline const std::string REGISTER_REFLECTOR = "answer/register_reflector";
inline const std::string REQUSET_GLOBAL_MAP = "answer/request_global_map";
inline const std::string DETECTED_REFLECTORS = "answer/detected_reflectors";
inline const std::string SET_REFLECTORS = "answer/set_reflectors";
inline const std::string LOG_LEVEL = "answer/log_level";
inline const std::string PALLET_POSE = "perception/pose";
inline const std::string WINGBODY_DETECTION = "wingbody/pose";
}  // namespace TOPICS
namespace SERVICES {
inline const std::string CONTROL_ANSWER = "answer/control";
inline const std::string SCAN_QUALITY_SERVICE = "scan_quality_service";
inline const std::string GLOBAL_MAP = "global_map";
inline const std::string RECORD_DATA = "answer/record_data";
inline const std::string UPDATE_PARAM = "answer/update_param";
inline const std::string LOG_LEVEL = "answer/log_level";
inline const std::string SAMPLING_MATCH = "answer/sampling_match";
}  // namespace SERVICES
}  // namespace ANSWER