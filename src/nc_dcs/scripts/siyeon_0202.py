#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from core_msgs.msg import ForkLift
from move_msgs.msg import CoreCommand, Waypoint, Dock, LidarObstacle, CameraObstacle
from geometry_msgs.msg import PoseWithCovarianceStamped

# 색상 정의
GREEN = '\033[0;32m'
CYAN = '\033[0;36m'
GRAY = '\033[1;30m'
NC = '\033[0m' # No Color

# 전역 변수
robot_pos = None

def robot_pos_callback(msg):
    global robot_pos
    robot_pos = msg

def run_step(step_num, start_step, msg, pub, ros_msg):
    """
    단계별 실행 함수
    :param step_num: 현재 단계 번호
    :param start_step: 시작 단계 번호
    :param msg: 출력할 메시지
    :param pub: ROS Publisher 객체
    :param ros_msg: 발행할 ROS 메시지 객체
    """
    # 시작 단계보다 현재 단계가 작으면 건너뜀 (Skip)
    if step_num < start_step:
        print(f"{GRAY}[SKIP] Step {step_num} 건너뜀{NC}")
        return

    # 정상 실행 로직
    print(f"{CYAN}--------------------------------------------------{NC}")
    print(f"다음 단계: {GREEN}{msg}{NC}")
    try:
        user_input = input("실행하려면 [Enter] 키를 누르세요... (중단하려면 'stop' 입력)")
        if user_input.lower() == 'stop':
            print(f"\n{GRAY}사용자 요청('stop')에 의해 스크립트를 종료합니다.{NC}")
            sys.exit(0)
    except KeyboardInterrupt:
        print("\n종료합니다.")
        sys.exit(0) 
    
    # 메시지 발행
    print(f"Publishing to {pub.name}...") 
    pub.publish(ros_msg)
    # 메시지가 확실히 발행되도록 잠시 대기 (latching이 아닐 경우)
    rospy.sleep(0.5) 
    print(f"✅ {step_num}단계 완료")

def get_forklift_msg(cur_x, cur_y, cur_deg, tar_x, tar_y, tar_deg, drive_type, rack_type, pallet_type=0):
    """
    ForkLift 메시지 생성 헬퍼 함수
    """
    msg = ForkLift()
    msg.s_current_node_id = ''
    msg.s_target_node_id = ''
    msg.f_current_x = float(cur_x)
    msg.f_current_y = float(cur_y)
    msg.f_current_deg = float(cur_deg)
    msg.f_target_x = float(tar_x)
    msg.f_target_y = float(tar_y)
    msg.f_target_deg = float(tar_deg)
    msg.n_rack_level = 0
    msg.n_target_level = 0
    msg.n_target_height = 0
    msg.n_drive_type = int(drive_type)
    msg.n_rack_type = int(rack_type)
    msg.n_pallet_type = int(pallet_type)
    return msg

def get_waypoint_msg(drive_type, f_speed_ms, f_x_m, f_y_m, f_curve_radius, b_stop_quick):
    """
    Waypoint 메시지 생성 헬퍼 함수
    """
    wp = Waypoint()
    wp.s_id = '' 
    wp.s_name = ''
    wp.n_drive_type = int(drive_type)
    wp.f_speed_ms = float(f_speed_ms)
    wp.f_x_m = float(f_x_m)
    wp.f_y_m = float(f_y_m)
    # 기본값 설정 (필요시 파라미터로 받도록 수정 가능)
    wp.f_angle_deg = 0.0
    wp.f_curve_radius = f_curve_radius
    wp.f_curvature = 0
    wp.n_avoid_type = 0
    wp.f_avoid_lanewidth = 0.0
    wp.f_avoid_speed_ms = 0.0
    wp.f_diagonal_heading_bias = 0.0
    wp.b_start_quick = False
    wp.b_stop_quick = b_stop_quick
    wp.b_diagonal_align_skip = False
    wp.b_smooth_path = False
    wp.b_lccs = False
    wp.list_f_move_obstacle_margin = [0.0]
    return wp

def get_live_path_msg(list_waypoints):
    """
    CoreCommand 메시지 생성 헬퍼 함수
    """
    cmd = CoreCommand()
    cmd.uuid = ''
    cmd.list_waypoints = list_waypoints # Waypoint 객체 리스트 할당
    
    # 기타 필드 초기화 (기본값)
    cmd.list_lidar_obs = [LidarObstacle(list_f_obstacle_margin=[0.0], b_obstacle_side_check=False, side_check_margin_reduction_left=0.0, side_check_margin_reduction_right=0.0)]
    cmd.list_camera_obs = [CameraObstacle(list_f_obstacle_margin=[0.0])] # 기본값으로 초기화
    cmd.o_dock = Dock() # 기본값
    
    cmd.b_start_createpath = False
    cmd.b_start_pause = False
    cmd.list_f_target_obstacle_margin = [0.0]
    cmd.b_arrive_align = False
    cmd.f_arrive_boundary_dist = 0.0
    cmd.f_arrive_boundary_deg = 0.0
    cmd.b_loaded = False
    
    return cmd

def create_path_command_with_ids(waypoints):
    """
    Waypoint 리스트에 ID를 할당하고 CoreCommand 메시지를 생성하는 헬퍼 함수
    """
    for i, wp in enumerate(waypoints):
        wp.s_id = str(i)
        wp.s_name = str(i)
    return get_live_path_msg(waypoints)

def main():
    # ROS 노드 초기화
    rospy.init_node('siyeon_scenario_execution', anonymous=True)
    
    # Publisher 생성
    pub_fork = rospy.Publisher('/nc_task_manager/fork_docking', ForkLift, queue_size=1)
    pub_live_path = rospy.Publisher('/navifra/live_path', CoreCommand, queue_size=1)

    # Subscriber 생성
    rospy.Subscriber('/localization/robot_pos', PoseWithCovarianceStamped, robot_pos_callback)

    print("=== 토픽 순차 발행 시나리오를 시작합니다 (Python + ROS) ===")

    # 1. 시작 단계 입력 받기
    try:
        user_input = input("몇 번째 단계부터 시작하시겠습니까? (기본값: 1): ")
        if not user_input:
            start_step = 1
        else:
            start_step = int(user_input)
    except ValueError:
        start_step = 1

    print(f"{GREEN}>> {start_step}단계부터 실행합니다.{NC}\n")

    # 명령어 리스트 정의: (step_num, description, message_object, publisher)
    commands = []

    # # 1단계
    # path_start_1 = [
    #     get_waypoint_msg(1000, 1.5, 7.63, -9.593, 0.0, True),
    #     get_waypoint_msg(2000, 1.5, 7.63, 8.05, 0.0, True)
    # ]
    # commands.append((1, "Step 1: 1번 위치 이동", create_path_command_with_ids(path_start_1), pub_live_path))
    
    # # 2단계
    # commands.append((2, "Step 2: 1번 위치 1단 로딩 - 윙바디", 
    #                  get_forklift_msg(7.63, 8.05, 0.0, 4.4, 8.05, 0.0, 1, 2), 
    #                  pub_fork))

    # # 3단계
    # path_1_start = [
    #     get_waypoint_msg(1000, 1.5, 7.63, 8.05, 0.0, True),
    #     get_waypoint_msg(2000, 1.5, 7.63, -9.380, 0.0, True),
    #     get_waypoint_msg(2000, 1.5, 9.593, -9.380, 180.0, True),
    # ]
    # commands.append((3, "Step 3: 렉 4번 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path))

    # # 4단계
    # commands.append((4, "Step 4: 렉 4번 위치 1단 언로딩 - 윙바디", 
    #                  get_forklift_msg(9.593, -9.380, 180.0, 12.98, -9.380, 180.0, -1, 1), 
    #                  pub_fork))
    # # 5단계
    # path_1_start = [
    #     get_waypoint_msg(1000, 1.5, 9.593, -9.380, 0.0, True),
    #     get_waypoint_msg(1000, 1.5, 7.63, -9.38, 0.0, True),
    #     get_waypoint_msg(1000, 1.5, 7.63, 5.85, 0.0, True),
    # ]
    # commands.append((5, "Step 5: 2번 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path))

    # # 6단계
    # commands.append((6, "Step 6: 2번 위치 1단 로딩 - 윙바디", 
    #                  get_forklift_msg(7.63, 5.85, 0.0, 4.4, 5.85, 0.0, 1, 2), 
    #                  pub_fork))
    # # 7단계
    # path_1_start = [
    #     get_waypoint_msg(1000, 1.5, 7.63, 5.85, 0.0, True),
    #     get_waypoint_msg(2000, 1.5, 7.63, -6.880, 0.0, True),
    #     get_waypoint_msg(2000, 1.5, 9.593, -6.880, 0.0, True)
    # ]
    # commands.append((7, "Step 7: 렉방 3번 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path))
 
    # # 8단계
    # commands.append((8, "Step 8: 렉방 3번 위치 1단 언로딩 - 윙바디", 
    #                  get_forklift_msg(9.593, -6.880, 180.0, 12.96, -6.880, 180.0, -1, 1), 
    #                  pub_fork))

    # # 9단계
    # commands.append((9, "Step 9: 렉방 3번 위치 1단 로딩 - 윙바디", 
    #                  get_forklift_msg(9.593, -6.880, 180.0, 12.96, -6.880, 180.0, 1, 1), 
    #                  pub_fork))
    
    # # 10단계
    # path_1_start = [
    #     get_waypoint_msg(1000, 1.5, 9.593, -6.880, 0.0, True),
    #     get_waypoint_msg(1000, 1.5, 9.0, -6.880, 0.0, True),
    #     get_waypoint_msg(1000, 1.5, 9.0, 6.1, 0.0, True)
    # ]
    # commands.append((10, "Step 10: 3번 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path))
    
    # # 11단계
    # commands.append((11, "Step 11: 3번 위치 1단 언로딩 - 윙바디", 
    #                  get_forklift_msg(9.0, 6.1, 0.0, 4.4, 6.1, 0.0, 3, 2), 
    #                  pub_fork))      

    # # 12단계
    # path_1_start = [
    #     get_waypoint_msg(1000, 1.5, 9.0, 6.1, 0.0, True),
    #     get_waypoint_msg(2000, 1.5, 9.0, -9.380, 0.0, True),
    #     get_waypoint_msg(2000, 1.5, 9.593, -9.380, 180.0, True),
    # ]
    # commands.append((12, "Step 12: 렉 4번 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path))
    
    # # 13단계
    # commands.append((13, "Step 13: 렉방 4번 위치 1단 로딩 - 윙바디", 
    #                  get_forklift_msg(9.593, -9.380, 180.0, 12.98, -9.380, 180.0, 1, 1), 
    #                  pub_fork))
    # # 14단계
    # path_1_start = [
    #     get_waypoint_msg(1000, 1.5, 9.593, -9.38, 0.0, True),
    #     get_waypoint_msg(1000, 1.5, 9.0, -9.38, 0.0, True),
    #     get_waypoint_msg(1000, 1.5, 9.0, 8.3, 0.0, True)
    # ]
    # commands.append((14, "Step 14: 1번 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path))
    
    # # 15단계
    # commands.append((15, "Step 15: 1번 위치 1단 언로딩 - 윙바디", 
    #                  get_forklift_msg(9.0, 8.3, 0.0, 4.4, 8.3, 0.0, 3, 2), 
    #                  pub_fork))    

    # # 16단계
    # path_start_1 = [
    #     get_waypoint_msg(1000, 1.5, 9.0, 8.3, 0.0, True),
    #     get_waypoint_msg(1000, 1.5, 7.63, -9.593, 0.0, True)
    # ]
    # commands.append((16, "Step 16: Start 위치 이동", create_path_command_with_ids(path_start_1), pub_live_path))
    

    # 1단계
    path_start_1 = [
        get_waypoint_msg(1000, 1.5, 9.0, -9.593, 0.0, True),
        get_waypoint_msg(2000, 1.5, 9.0, 8.3, 0.0, True)
    ]
    commands.append((1, "Step 1: 1번 위치 이동", create_path_command_with_ids(path_start_1), pub_live_path))
    
    # 2단계
    commands.append((2, "Step 2: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 101, 2), 
                     pub_fork))

    # 3단계
    commands.append((3, "Step 3: 1번 위치 1단 로딩 - 윙바디", 
                     get_forklift_msg(9.0, 8.3, 0.0, 4.4, 8.3, 0.0, 1, 2), 
                     pub_fork))

    # 4단계
    path_1_start = [
        get_waypoint_msg(1000, 1.5, 9.0, 8.3, 0.0, True),
        get_waypoint_msg(2000, 1.5, 9.0, -9.380, 0.0, True),
        get_waypoint_msg(2000, 1.5, 9.593, -9.380, 180.0, True),
    ]
    commands.append((4, "Step 4: 렉 4번 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path))
        
    # 5단계
    commands.append((5, "Step 5: 무브중 tilt 동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50, 2), 
                     pub_fork))
    # 6단계
    commands.append((6, "Step 6: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99, 1), 
                     pub_fork))
    
    # 7단계
    commands.append((7, "Step 7: 렉 4번 위치 1단 언로딩 - 랙방", 
                     get_forklift_msg(9.593, -9.380, 180.0, 12.98, -9.380, 180.0, -1, 1), 
                     pub_fork))
    # 8단계
    path_1_start = [
        get_waypoint_msg(1000, 1.5, 9.593, -9.380, 0.0, True),
        get_waypoint_msg(1000, 1.5, 9.0, -9.38, 0.0, True),
        get_waypoint_msg(1000, 1.5, 9.0, 6.1, 0.0, True),
    ]
    commands.append((8, "Step 8: 2번 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path))
        
    # 9단계
    commands.append((9, "Step 9: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 101, 2), 
                     pub_fork))
        
    # 10단계
    commands.append((10, "Step 10: 2번 위치 1단 로딩 - 윙바디", 
                     get_forklift_msg(9.0, 6.1, 0.0, 4.4, 6.1, 0.0, 1, 2), 
                     pub_fork))
    # 11단계
    path_1_start = [
        get_waypoint_msg(1000, 1.5, 9.0, 6.1, 0.0, True),
        get_waypoint_msg(2000, 1.5, 9.0, -6.880, 0.0, True),
        get_waypoint_msg(2000, 1.5, 9.593, -6.880, 0.0, True)
    ]
    commands.append((11, "Step 11: 렉방 3번 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path))
    
    # 12단계
    commands.append((12, "Step 12: 무브중 tilt 동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50, 2), 
                     pub_fork))
    
    # 13단계
    commands.append((13, "Step 13: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99, 1), 
                     pub_fork))

    # 14단계
    commands.append((14, "Step 14: 렉방 3번 위치 1단 언로딩 - 랙방", 
                     get_forklift_msg(9.593, -6.880, 180.0, 12.96, -6.880, 180.0, -1, 1), 
                     pub_fork))

    # 15단계
    commands.append((15, "Step 15: 렉방 3번 위치 1단 로딩 - 랙방", 
                     get_forklift_msg(9.593, -6.880, 180.0, 12.96, -6.880, 180.0, 1, 1), 
                     pub_fork))

    # 16단계
    path_1_start = [
        get_waypoint_msg(1000, 1.5, 9.593, -6.880, 0.0, True),
        get_waypoint_msg(1000, 1.5, 9.0, -6.880, 0.0, True),
        get_waypoint_msg(1000, 1.5, 9.0, -8.780, 0.0, True)
    ]
    commands.append((16, "Step 16: 6번 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path))
    
    # 17단계
    commands.append((17, "Step 17: 무브중 tilt 동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50, 2), 
                     pub_fork))
    
    # 18단계
    commands.append((18, "Step 18: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 103, 2), 
                     pub_fork))
    
    # 19단계
    commands.append((19, "Step 19: 6번 위치 1단 언로딩 - 윙바디 2", 
                     get_forklift_msg(9.0, -8.780, 0.0, 4.35, -8.780, 0.0, 3, 2), 
                     pub_fork))      

   # 20단계
    path_2_start = [
        get_waypoint_msg(1000, 1.5, 9.0, -8.780, 0.0, True),
        get_waypoint_msg(2000, 1.5, 9.0, -9.380, 0.0, True),
        get_waypoint_msg(2000, 1.5, 9.593, -9.380, 0.0, True)
    ]
    commands.append((20, "Step 20: 렉 4번 위치 이동", create_path_command_with_ids(path_2_start), pub_live_path))
    
    # 21단계
    commands.append((21, "Step 21: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 101, 1), 
                     pub_fork))
    
    # 22단계
    commands.append((22, "Step 22: 렉방 4번 위치 1단 로딩 - 랙방", 
                     get_forklift_msg(9.593, -9.380, 180.0, 12.982, -9.380, 180.0, 1, 1), 
                     pub_fork))
    
    # 23단계
    path_2_start = [
        get_waypoint_msg(1000, 1.5, 9.593, -9.380, 0.0, True),
        get_waypoint_msg(1000, 1.5, 9.0, -9.380, 0.0, True),
        get_waypoint_msg(1000, 1.5, 9.0, -6.580, 0.0, True)
    ]
    commands.append((23, "Step 23: 8번 위치 이동", create_path_command_with_ids(path_2_start), pub_live_path))
    
    # 24단계
    commands.append((24, "Step 24: 무브중 tilt 동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50, 2), 
                     pub_fork))
    
    # 25단계
    commands.append((25, "Step 25: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 103, 2), 
                     pub_fork))
    
    # 26단계
    commands.append((26, "Step 26: 8번 위치 1단 언로딩 - 윙바디 2", 
                     get_forklift_msg(9.0, -6.580, 0.0, 4.35, -6.580, 0.0, 3, 2), 
                     pub_fork))    

    # 27단계
    path_2_start = [
        get_waypoint_msg(1000, 1.5, 9.0, -6.580, 0.0, True),
        get_waypoint_msg(2000, 1.5, 9.0, -1.870, 0.0, True),
        get_waypoint_msg(2000, 1.5, 9.593, -1.870, 0.0, True)
    ]
    commands.append((27, "Step 27: 렉 1번 위치 이동", create_path_command_with_ids(path_2_start), pub_live_path))
    
    # 28단계
    commands.append((28, "Step 28: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 101, 1), 
                     pub_fork))
    
    # 29단계
    commands.append((29, "Step 29: 렉방 4번 위치 1단 로딩 - 랙방", 
                     get_forklift_msg(9.593, -1.870, 180.0, 12.970, -1.870, 180.0, 1, 1), 
                     pub_fork))
    

    path_1_start = [
        get_waypoint_msg(1000, 1.5, 9.593, -1.870, 0.0, True),
        get_waypoint_msg(2000, 1.5, 9.0, -1.870, 0.0, True),
        get_waypoint_msg(2000, 1.5, 9.0, -16.860, 0.0, True)
    ]
    commands.append((30, "Step 30: 비전방 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path))
    
    # 31단계
    commands.append((31, "Step 31: 무브중 tilt 동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50, 2), 
                     pub_fork))
    
    # 32단계
    commands.append((32, "Step 32: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99, 1), 
                     pub_fork))

    # 33단계
    commands.append((33, "Step 33: 비전방 1단 언로딩", 
                     get_forklift_msg(9.0, -16.860, 180.0, 12.918, -16.860, 180.0, -1, 1), 
                     pub_fork))
    
    # 34단계
    path_2_start = [
        get_waypoint_msg(1000, 1.5, 9.0, -16.860, 0.0, True),
        get_waypoint_msg(2000, 1.5, 9.0, -4.365, 0.0, True),
        get_waypoint_msg(2000, 1.5, 9.593, -4.365, 0.0, True)
    ]
    commands.append((34, "Step 34: 렉 2번 위치 이동", create_path_command_with_ids(path_2_start), pub_live_path))
    
    # 35단계
    commands.append((35, "Step 35: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 101, 1), 
                     pub_fork))
    
    # 36단계
    commands.append((36, "Step 36: 렉방 2번 위치 1단 로딩 - 랙방", 
                     get_forklift_msg(9.593, -4.365, 180.0, 12.970, -4.365, 180.0, 1, 1), 
                     pub_fork))
    

    path_1_start = [
        get_waypoint_msg(1000, 1.5, 9.593, -4.365, 0.0, True),
        get_waypoint_msg(2000, 1.5, 9.0, -4.365, 0.0, True),
        get_waypoint_msg(2000, 1.5, 9.0, -16.860, 0.0, True)
    ]
    commands.append((37, "Step 37: 렉방 3번 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path))
    
    # 38단계
    commands.append((38, "Step 38: 무브중 tilt 동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50, 2), 
                     pub_fork))
    
    # 39단계
    commands.append((39, "Step 39: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99, 3), 
                     pub_fork))

    # 40단계
    commands.append((40, "Step 40: 비전방 2단 언로딩", 
                     get_forklift_msg(9.0, -16.860, 180.0, 12.918, -16.860, 180.0, -1, 3), 
                     pub_fork))


    
    # 순차 실행
    try:
        for step_num, msg, ros_msg, pub in commands:
            run_step(step_num, start_step, msg, pub, ros_msg)
    except KeyboardInterrupt:
        print(f"\n{GRAY}사용자 요청에 의해 스크립트를 종료합니다.{NC}")
        sys.exit(0)

    print(f"{GREEN}=== 모든 시나리오가 종료되었습니다 ==={NC}")

if __name__ == "__main__":
    main()
