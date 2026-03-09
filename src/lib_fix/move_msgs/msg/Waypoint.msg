string s_id                 # 노드ID
string s_name               # 노드이름 
# 1000 : front , 1001 : front diagoanl , 2000 : rear , 2001: rear diagonal 
int32 n_drive_type           # 노드 주행 타입
float32 f_speed_ms          # 다음 노드까지 주행 속도
float32 f_x_m               # 노드 좌표 x
float32 f_y_m               # 노드 좌표 y
float32 f_angle_deg         # 노드 각도 deg

float32 f_curve_radius      # 세점에서 커브시 커브 반지름 (f_curve_radius, f_curvature 택1)
float32 f_curvature         # 두점에서 커브시 커브 곡률 (f_curve_radius, f_curvature 택1)

# 0 : 회피 off, 1: 차선기반 회피(양쪽), 2: 차선기반 회피(왼쪽), 3: 차선기반 회피(오른쪽), 4: 맵기반 회피
int32 n_avoid_type           # 회피 타입
float32 f_avoid_lanewidth   # 회피 폭
float32 f_avoid_speed_ms    # 회피 속도

float32 f_diagonal_heading_bias             # 사행으로 갈때 강제로 heading bias 주는 옵션

bool b_start_quick                          # 출발시 빠르게 출발하는 옵션
bool b_stop_quick                           # 도착시 빠르게 정지하는 옵션

bool b_diagonal_align_skip                  # 사행 출발시 align 하지말고 바로 출발하는 옵션

bool b_smooth_path                          # 경로 smooth 적용

bool b_lccs

float32[] list_f_move_obstacle_margin       # 도착지와 로봇사이 장애물 감지영역 임의 생성옵션 [front, rear, left, right] m단위