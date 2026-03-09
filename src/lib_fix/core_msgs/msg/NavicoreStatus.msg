Header header

int64 LOADING=99
int64 READY=100
int64 WAITING=101
int64 RUNNING=102
int64 COMPLETED=103
int64 SO_FAR_FROM_PATH=104
int64 ABORTED=105
int64 ARRIVED=106
int64 TERMINATED=107
int64 AVOIDANCE_RUNNING=108
int64 TASKING=109

int64 PAUSED_BY_USER=110
int64 PAUSED_BY_STATUS=111
int64 PAUSED_BY_OBS=112

int64     n_status 
string    s_status 
int64     n_ex_status
string    s_next_node
string    s_next_node_id
string    s_current_node
string    s_current_node_id
string    s_goal_node
string    s_goal_node_id

float32   f_path_progress
float32   f_collision_remained_sec
float32   f_max_linear_vel_of_path

float32   f_goal_pos_x_m
float32   f_goal_pos_y_m
float32   f_goal_pos_deg
float32   f_robot_pos_x_m
float32   f_robot_pos_y_m
float32   f_robot_pos_deg
float32   f_error_pos_x_m
float32   f_error_pos_y_m
float32   f_error_pos_deg
float32   f_robot_target_linear_vel_x
float32   f_robot_target_linear_vel_y
float32   f_robot_target_angular_vel_w

float32   f_robot_current_linear_vel_x
float32   f_robot_current_linear_vel_y
float32   f_robot_current_angular_vel_w

int64     n_on_path
int64     n_spin_turn
int64     n_avoid_state
int64     n_map_rate
int64     n_confidence
bool      b_obstacle_stop
bool      b_avoid_status
float32   f_path_error_dist_m
float32   f_speed_percent
float32   f_speed_limit_ms
float32   f_linear_speed_max_ms
int64     n_drive_type
int64     n_diagonal_direction

float32 f_mileage_total
float32 f_mileage_local
float32 f_robot_battery

float32 f_max_linear_velocity
float32 f_min_linear_velocity
float32 f_linear_acceleration
float32 f_linear_dcceleration
float32 f_max_angular_velocity
float32 f_min_angular_velocity
float32 f_angular_acceleration
float32 f_max_curvature
string[] s_warning