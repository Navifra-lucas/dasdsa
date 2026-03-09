# NavigationCommand.msg
string uuid                                 # uuid
Waypoint[] list_waypoints                   # 이동 경로를 이루는 좌표 리스트
LidarObstacle[] list_lidar_obs              # lidar 장애물 감지 영역 설정
CameraObstacle[] list_camera_obs            # camera 장애물 감지 영역 설정
Dock o_dock                                 # 도킹시 사용하는 옵션

bool b_start_createpath                     # 시작시 경로 생성 옵션
bool b_start_pause                          # 시작시 명령은 받았으나 pause상태 대기 옵션

float32[] list_f_target_obstacle_margin     # 도착지에 장애물 감지영역 임의 생성옵션 [front, rear, left, right] m단위

bool b_arrive_align                         # 도착지에 도착 후 align 할지 말지 여부 default : false
float32 f_arrive_boundary_dist              # 도착지에 도착 후 거리상 크기에 따라서 알람
float32 f_arrive_boundary_deg               # 도착지에 도착 후 각도상 크기에 따라서 알람

bool b_loaded                               # 적재 상태인지 구분명령