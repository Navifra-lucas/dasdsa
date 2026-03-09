int32 LINE   = 0
int32 ARC    = 1
int32 BEZIER = 2
int32 POLY   = 3

DriveInfo o_drive_info

NodeInfo o_from_node
NodeInfo o_to_node

int32 n_path_type
geometry_msgs/Pose2D o_from_node_control
geometry_msgs/Pose2D o_to_node_control
float64 d_time_control
float64 d_target_speed
float64 d_robot_yaw