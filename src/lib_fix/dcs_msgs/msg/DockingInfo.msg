# Mission type constants
int32 NONE     = 0
int32 PALLET   = 1
int32 WINGBODY = 2
int32 GOBACK   = 3

# Mission type field
int32 n_mission_type

# Pose information
geometry_msgs/Pose o_fNode
geometry_msgs/Pose o_local_pose
geometry_msgs/Pose o_global_pose