# MissionType
int32 IDLE        = 0
int32 ALIGN       = 1
int32 NODE_PATH   = 2
int32 START_PATH  = 3
int32 SEARCH_PATH = 4
int32 EXPLORE     = 5

# mission type
int32     n_mission_type

# mission nodes info
MissionNodeInfo o_mission_nodes

# drive info
DriveInfo o_drive_info

# mission progress
MissionProgress o_mission_progress

# motion status
MotionStatus o_motion_status

# motion error
geometry_msgs/Pose2D o_pose_error
