#!/bin/bash

# 색상 정의
GREEN='\033[0;32m'
CYAN='\033[0;36m'
GRAY='\033[1;30m'
NC='\033[0m' # No Color

echo "=== 토픽 순차 발행 시나리오를 시작합니다 ==="

# ------------------------------------------
# 1. 시작 단계 입력 받기
# ------------------------------------------
echo -n "몇 번째 단계부터 시작하시겠습니까? (기본값: 1): "
read INPUT_STEP

# 입력이 없으면 1로 설정, 있으면 그 숫자로 설정
if [ -z "$INPUT_STEP" ]; then
    START_STEP=1
else
    START_STEP=$INPUT_STEP
fi

echo -e "${GREEN}>> ${START_STEP}단계부터 실행합니다.${NC}\n"

# ------------------------------------------
# 2. 실행 로직을 담당하는 함수 정의
# ------------------------------------------
run_step() {
    local current_step=$1   # 단계 번호
    local msg=$2            # 메시지
    local cmd=$3            # 실행할 명령어

    # 시작 단계보다 현재 단계가 작으면 건너뜀 (Skip)
    if [ "$current_step" -lt "$START_STEP" ]; then
        echo -e "${GRAY}[SKIP] Step ${current_step} 건너뜀${NC}"
        return
    fi

    # 정상 실행 로직
    echo -e "${CYAN}--------------------------------------------------${NC}"
    echo -e "다음 단계: ${GREEN}$msg${NC}"
    read -p "실행하려면 [Enter] 키를 누르세요..."
    
    # 명령어 실행
    eval "$cmd"
    echo "✅ ${current_step}단계 완료"
}

# ==========================================
# 시나리오 시작
# ==========================================

# 1단계
CMD_1="rostopic pub -1 /nc_task_manager/fork_docking core_msgs/ForkLift '{s_current_node_id: '', s_target_node_id: '', f_current_x: 7.88, f_current_y: 3.950,
  f_current_deg: 0.0, f_target_x: 4.380, f_target_y: 3.950, f_target_deg: 0.0, n_rack_level: 0,
  n_target_level: 0, n_target_height: 0, n_drive_type: 1, n_rack_type: 2, n_pallet_type: 0}'"
run_step 1 "Step 1: 2단 로딩 - 윙바디" "$CMD_1"

# 2단계
CMD_2="rostopic pub -1 /nc_task_manager/fork_docking core_msgs/ForkLift '{s_current_node_id: '', s_target_node_id: '', f_current_x: 9.593, f_current_y: -6.912,
  f_current_deg: 180.0, f_target_x: 13.0, f_target_y: -6.912, f_target_deg: 180.0, n_rack_level: 0,
  n_target_level: 0, n_target_height: 0, n_drive_type: 5, n_rack_type: 1, n_pallet_type: 0}'"
run_step 2 "Step 2: 2단 언로딩 - 랙" "$CMD_2"

# 3단계
CMD_3="rostopic pub -1 /nc_task_manager/fork_docking core_msgs/ForkLift '{s_current_node_id: '', s_target_node_id: '', f_current_x: 9.593, f_current_y: -6.912,
  f_current_deg: 180.0, f_target_x: 13.0, f_target_y: -6.912, f_target_deg: 180.0, n_rack_level: 0,
  n_target_level: 0, n_target_height: 0, n_drive_type: 4, n_rack_type: 3, n_pallet_type: 0}'"
run_step 3 "Step 3: 1단 로딩 - 랙" "$CMD_3"

# 5단계: 목표 지점 발행 (MoveBaseGoal)
CMD_4="rostopic pub -1 /nc_task_manager/fork_docking core_msgs/ForkLift '{s_current_node_id: '', s_target_node_id: '', f_current_x: 9.593, f_current_y: -6.912,
  f_current_deg: 180.0, f_target_x: 13.0, f_target_y: -6.912, f_target_deg: 180.0, n_rack_level: 0,
  n_target_level: 0, n_target_height: 0, n_drive_type: 5, n_rack_type: 3, n_pallet_type: 0}'"
run_step 4 "Step 4: 2단 언로딩 - 랙" "$CMD_4"

# 5단계: 목표 지점 발행 (MoveBaseGoal)
CMD_5="rostopic pub -1 /nc_task_manager/fork_docking core_msgs/ForkLift '{s_current_node_id: '', s_target_node_id: '', f_current_x: 9.593, f_current_y: -6.912,
  f_current_deg: 180.0, f_target_x: 13.0, f_target_y: -6.912, f_target_deg: 180.0, n_rack_level: 0,
  n_target_level: 0, n_target_height: 0, n_drive_type: 1, n_rack_type: 1, n_pallet_type: 0}'"
run_step 5 "Step 5: 1단 로딩 - 랙" "$CMD_5"

CMD_6="rostopic pub -1 /nc_task_manager/fork_docking core_msgs/ForkLift '{s_current_node_id: '', s_target_node_id: '', f_current_x: 7.88, f_current_y: -10.980,
  f_current_deg: 0.0, f_target_x: 4.380, f_target_y: -10.980, f_target_deg: 0.0, n_rack_level: 0,
  n_target_level: 0, n_target_height: 0, n_drive_type: 3, n_rack_type: 2, n_pallet_type: 0}'"
run_step 6 "Step 6: 1단 언로딩 - 윙바디" "$CMD_6"

# ------------------------------------------
# 7단계: Live Path 발행 (Custom Waypoints)
# ------------------------------------------
# Waypoint 데이터 정의 (원하는 값으로 수정)
# 예시: 3개의 Waypoint
WP_DRIVE_TYPES=(0 0 0)
WP_SPEEDS=(0.5 0.8 0.5)
WP_XS=(10.0 12.0 14.0)
WP_YS=(5.0 6.0 5.0)

# Waypoint 리스트 YAML 문자열 생성
WAYPOINTS_YAML=""
CNT=${#WP_XS[@]}

for ((i=0; i<CNT; i++)); do
    WAYPOINTS_YAML+="
- s_id: '$i'
  s_name: '$i'
  n_drive_type: ${WP_DRIVE_TYPES[$i]}
  f_speed_ms: ${WP_SPEEDS[$i]}
  f_x_m: ${WP_XS[$i]}
  f_y_m: ${WP_YS[$i]}
  f_angle_deg: 0.0
  f_curve_radius: 0.0
  f_curvature: 0.0
  n_avoid_type: 0
  f_avoid_lanewidth: 0.0
  f_avoid_speed_ms: 0.0
  f_diagonal_heading_bias: 0.0
  b_start_quick: false
  b_stop_quick: false
  b_diagonal_align_skip: false
  b_smooth_path: false
  b_lccs: false
  list_f_move_obstacle_margin: [0]"
done

# 전체 메시지 구성
# 주의: YAML 들여쓰기에 주의해야 합니다.
CMD_7="rostopic pub -1 /navifra/live_path move_msgs/CoreCommand \"uuid: ''
list_waypoints:${WAYPOINTS_YAML}
list_lidar_obs:
- list_f_obstacle_margin: [0]
  b_obstacle_side_check: false
  side_check_margin_reduction_left: 0.0
  side_check_margin_reduction_right: 0.0
list_camera_obs:
- list_f_obstacle_margin: [0]
  f_camera_roi_y_m: 0.0
  f_camera_roi_x_m: 0.0
  f_camera_roi_z_m: 0.0
  f_camera_roi2_y_m: 0.0
  f_camera_roi2_x_m: 0.0
  f_camera_roi2_z_m: 0.0
  f_camera_docking_roi_y_m: 0.0
  f_camera_docking_roi_z_m: 0.0
  f_plt_type_x: 0.0
  f_plt_type_y: 0.0
o_dock: {n_docking_mode: 0, f_docking_check_dist: 0.0, f_global_offset_x: 0.0, f_global_offset_y: 0.0,
  f_global_offset_deg: 0.0}
b_start_createpath: false
b_start_pause: false
list_f_target_obstacle_margin: [0]
b_arrive_align: false
f_arrive_boundary_dist: 0.0
f_arrive_boundary_deg: 0.0
b_loaded: false\""

run_step 7 "Step 7: Live Path (Dynamic)" "$CMD_7"

echo -e "${GREEN}=== 모든 시나리오가 종료되었습니다 ===${NC}"