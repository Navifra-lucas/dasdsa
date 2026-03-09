int64 NONE=0
int64 LINE=1
int64 ARC=2

int64 FRONT=1000
int64 REAR=1001
int64 SIDE=1002
int64 FRONTDIAGONAL=1003
int64 REARDIAGONAL=1004
int64 FRONTDOCK=1005
int64 REARDOCK=1006

string LINE_CD = D40101
string ARC_CD = D40102

string id
string name
string from_link
string to_link
int64 type


bool    from_driving_active
int64   from_driving_direction
float32 from_driving_linear_speed
float32 from_driving_diagonal_curve
bool    from_driving_start_smooth
bool    from_driving_stop_smooth
bool    from_driving_avoidance_right
bool    from_driving_avoidance_left
int64    from_driving_avoidance_step
bool    from_driving_collision_enable
float32 from_driving_collision_margin_front_m
float32 from_driving_collision_margin_rear_m
float32 from_driving_collision_margin_left_m
float32 from_driving_collision_margin_right_m

bool    to_driving_active
int64   to_driving_direction
float32 to_driving_linear_speed
float32 to_driving_diagonal_curve
bool    to_driving_start_smooth
bool    to_driving_stop_smooth
bool    to_driving_avoidance_right
bool    to_driving_avoidance_left
int64    to_driving_avoidance_step
bool    to_driving_collision_enable
float32 to_driving_collision_margin_front_m
float32 to_driving_collision_margin_rear_m
float32 to_driving_collision_margin_left_m
float32 to_driving_collision_margin_right_m

float32 entry_angle_deg
float32 exit_angle_deg

float32 arc_center_x
float32 arc_center_y

float32 arc_start_position_x
float32 arc_start_position_y

float32 arc_end_position_x
float32 arc_end_position_y

float32 line_1_start_position_x
float32 line_1_start_position_y
float32 line_1_end_position_x
float32 line_1_end_position_y

float32 line_2_start_position_x
float32 line_2_start_position_y
float32 line_2_end_position_x
float32 line_2_end_position_y

bool b_arc_is_ccw
float32 radius
