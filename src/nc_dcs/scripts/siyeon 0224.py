#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from core_msgs.msg import ForkLift, CheonilReadRegister, NaviAlarm
from move_msgs.msg import CoreCommand, Waypoint, Dock, LidarObstacle, CameraObstacle
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, Int32

# 색상 정의
GREEN = '\033[0;32m'
CYAN = '\033[0;36m'
GRAY = '\033[1;30m'
NC = '\033[0m' # No Color

# 전역 변수
robot_pos = None
lift_up_down_done = None
fork_lift_reached = None
goal_arrived = None
goal_arrived_check_start = False
scenario_control_1 = None
scenario_control_2 = None

def robot_pos_callback(msg):
    global robot_pos
    robot_pos = msg

def cheonil_read_register_callback(msg):
    global lift_up_down_done
    if msg.fork_up_down_complete:
        lift_up_down_done = True

def fork_lift_reached_callback(msg):
    global fork_lift_reached
    if msg.data:
        fork_lift_reached = True

def navifraAlarmCallback(msg):
    global goal_arrived, goal_arrived_check_start
    if goal_arrived_check_start:
        if msg.alarm == NaviAlarm.GOAL_ARRIVED:
            goal_arrived = True

def scenario_control_callback(msg):
    global scenario_control_1, scenario_control_2
    if msg.data == 1:
        scenario_control_1 = True
    elif msg.data == 2:
        scenario_control_2 = True

def run_step(step_num, start_step, msg, pub, ros_msg, wait_lift_at_start=False, check_goal_arrived=False, wait_fork_lift_done=False, post_delay=0.0):
    """
    단계별 실행 함수
    :param step_num: 현재 단계 번호
    :param start_step: 시작 단계 번호
    :param msg: 출력할 메시지
    :param pub: ROS Publisher 객체
    :param ros_msg: 발행할 ROS 메시지 객체
    :param wait_lift_at_start: 시작 전 리프트 상하 이동 완료 대기 여부 (Optional)
    :param check_goal_arrived: 목표 도착 여부 확인 플래그 (Optional, 기본값 True)
    """
    global goal_arrived, fork_lift_reached, lift_up_down_done, goal_arrived_check_start


    # 정상 실행 로직
    print(f"{CYAN}--------------------------------------------------{NC}")
    print(f"다음 단계: {GREEN}{msg}{NC}")
    
    # 시작 전 리프트 완료 대기
    if wait_lift_at_start:
        print(f"[{step_num}단계 시작 전] 리프트 상하 이동 완료 대기 중...")
        while not rospy.is_shutdown():
            if lift_up_down_done:
                print(f"리프트 상하 이동 완료 확인됨. 단계 시작.")
                break
            rospy.sleep(0.1)
    
    if check_goal_arrived:
        print(f"[{step_num}단계 시작 전] 목표 도착 여부 확인 중...")
        while not rospy.is_shutdown():
            if goal_arrived:
                goal_arrived_check_start = False
                print(f"목표 도착 확인됨. 단계 시작.")
                break
            rospy.sleep(0.1)
    
    if wait_fork_lift_done:
        print(f"[{step_num}단계 시작 전] Fork Lift 도착 여부 확인 중...")
        while not rospy.is_shutdown():
            if fork_lift_reached:
                print(f"Fork Lift 도착 확인됨. 단계 시작.")
                break
            rospy.sleep(0.1)

    # 플래그 초기화 (다음 단계를 위해 리셋, 단 wait_lift_at_start 체크 이후에 해야 함)
    if wait_lift_at_start: 
        lift_up_down_done = False 
    
    if check_goal_arrived: 
        goal_arrived = False 
    
    if wait_fork_lift_done: 
        fork_lift_reached = False 
    
    # 메시지 발행
    print(f"Publishing to {pub.name}...") 
    pub.publish(ros_msg)
    
    if pub.name == '/navifra/live_path':
        goal_arrived_check_start = True

    # 후처리 대기 (post_delay)
    if post_delay > 0.0:
        print(f"단계 완료 후 {post_delay}초 대기...")
        rospy.sleep(post_delay)

    print(f"✅ {step_num}단계 완료")


def run_rest(step_num, msg, duration_sec):
    print(f"{CYAN}--------------------------------------------------{NC}")
    print(f"다음 단계: {GRAY}[대기] {msg}{NC}")
    print(f"{duration_sec}초 대기 중...")
    rospy.sleep(float(duration_sec))
    print(f"✅ {step_num}단계 (대기) 완료")


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
    global goal_arrived, goal_arrived_check_start
    rospy.init_node('siyeon_scenario_execution', anonymous=True)
    
    # Publisher 생성
    pub_fork = rospy.Publisher('/nc_task_manager/fork_docking', ForkLift, queue_size=1)
    pub_live_path = rospy.Publisher('/navifra/live_path', CoreCommand, queue_size=1)
    pub_scenario_complete = rospy.Publisher('/wia_agent/scenario_complete', Int32, queue_size=1)

    # Subscriber 생성
    rospy.Subscriber('/localization/robot_pos', PoseWithCovarianceStamped, robot_pos_callback)
    rospy.Subscriber('/fork_lift_reached', Bool, fork_lift_reached_callback)
    rospy.Subscriber('/cheonil/read_register', CheonilReadRegister, cheonil_read_register_callback)
    rospy.Subscriber('/navifra/alarm', NaviAlarm, navifraAlarmCallback)
    rospy.Subscriber('/wia_agent/scenario_control', Int32, scenario_control_callback)

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
    # 쉬기 전용: (step_num, "설명", 대기_초) 3개만 넣으면 run_rest() 호출 (post_delay와 독립)
    # 예: commands.append((99, "Step 99: 10초 대기", 10.0))
    commands = []

    # 1단계
    path_start_1 = [
        get_waypoint_msg(1000, 1.4, 9.0, -9.593, 0.0, True),
        get_waypoint_msg(2000, 1.4, 9.0, 8.3, 0.0, True)
    ]
    commands.append((1, "Step 1: 1번 위치 이동 ( 윙바디 1번 )", create_path_command_with_ids(path_start_1), pub_live_path))
    
    # 2단계
    commands.append((2, "Step 2: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 101, 2), 
                     pub_fork, False, False, False))

    # 3단계
    commands.append((3, "Step 3: 1번 위치 1단 로딩 - 윙바디 1번", 
                     get_forklift_msg(9.0, 8.3, 0.0, 4.4, 8.3, 0.0, 1, 2), 
                     pub_fork, True, True, True))

    # 4단계
    path_1_start = [
        get_waypoint_msg(1000, 1.4, 9.0, 8.3, 0.0, True),
        get_waypoint_msg(2000, 1.4, 9.0, -9.385, 0.0, True),
        # get_waypoint_msg(2000, 1.4, 9.560, -9.385, 0.0, True),
    ]
    commands.append((4, "Step 4: 렉 4번 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path, True, False, True))
        
    # 5단계
    commands.append((5, "Step 5: 무브중 tilt 동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50, 2), 
                     pub_fork, True, False, False))
    # 6단계
    commands.append((6, "Step 6: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99, 1), 
                     pub_fork, False, False, True))
    
    # 7단계
    commands.append((7, "Step 7: 렉 4번 위치 1단 언로딩 - 랙방", 
                     get_forklift_msg(9.0, -9.385, 180.0, 13.045, -9.385, 180.0, -1, 1), 
                     pub_fork, True, True, True))
    # 8단계
    path_1_start = [
        # get_waypoint_msg(1000, 1.4, 9.560, -9.385, 0.0, True),
        get_waypoint_msg(1000, 1.4, 9.0, -9.385, 0.0, True),
        get_waypoint_msg(1000, 1.4, 9.0, 6.1, 0.0, True),
    ]
    commands.append((8, "Step 8: 2번 위치 이동 ( 윙바디 2번 )", create_path_command_with_ids(path_1_start), pub_live_path, True, False, True))
        
    # 9단계
    commands.append((9, "Step 9: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 101, 2), 
                     pub_fork, True, False, False))
        
    # 10단계
    commands.append((10, "Step 10: 2번 위치 1단 로딩 - 윙바디 2번", 
                     get_forklift_msg(9.0, 6.1, 0.0, 4.42, 6.1, 0.0, 1, 2), 
                     pub_fork, True, True, True))
    
    # 11단계
    path_1_start = [
        get_waypoint_msg(1000, 1.4, 9.0, 6.1, 0.0, True),
        get_waypoint_msg(2000, 1.4, 9.0, -6.895, 0.0, True),
        # get_waypoint_msg(2000, 1.4, 9.540, -6.895, 0.0, True)
    ]
    commands.append((11, "Step 11: 렉방 3번 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path, True, False, True))
    
    # 12단계
    commands.append((12, "Step 12: 무브중 tilt 동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50, 2), 
                     pub_fork, True, False, False))
    
    # 13단계
    commands.append((13, "Step 13: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99, 1), 
                     pub_fork, False, False, True))

    # 14단계
    commands.append((14, "Step 14: 렉방 3번 위치 1단 언로딩 - 랙방", 
                     get_forklift_msg(9.0, -6.895, 180.0, 13.025, -6.895, 180.0, -1, 1), 
                     pub_fork, True, True, True))

    path_2_start = [
        # get_waypoint_msg(1000, 1.4, 9.593, -6.895, 0.0, True),
        get_waypoint_msg(1000, 1.4, 9.0, -6.895, 0.0, True),
        get_waypoint_msg(2000, 1.4, 9.0, -9.593, 0.0, True),
    ]

    # 15단계
    commands.append((15, "Step 15: 시작 대기 위치 이동", create_path_command_with_ids(path_2_start), pub_live_path, True, False, True))


    # 시나리오 1번 끝

    # 16단계
    path_2_start = [
        # get_waypoint_msg(1000, 1.4, 9.593, -6.895, 0.0, True),
        get_waypoint_msg(1000, 1.4, 9.0, -9.593, 0.0, True),
        get_waypoint_msg(2000, 1.4, 9.0, -4.37, 0.0, True),
    ]

    commands.append((16, "Step 16: 렉방 2번 위치 이동", create_path_command_with_ids(path_2_start), pub_live_path, True, False, False))

    # 17단계
    commands.append((17, "Step 17: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 101, 1), 
                     pub_fork, False, False, False))

    # 18단계
    commands.append((18, "Step 18: 렉방 1번 위치 1단 로딩 - 랙방", 
                     get_forklift_msg(9.0, -4.37, 180.0, 13.025, -4.37, 180.0, 1, 1), 
                     pub_fork, True, True, True))

    # 19단계
    path_1_start = [
        # get_waypoint_msg(1000, 1.4, 9.593, -4.37, 0.0, True),
        get_waypoint_msg(1000, 1.4, 9.0, -4.37, 0.0, True),
        get_waypoint_msg(1000, 1.0, 9.0, -10.98, 0.0, True)
    ]
    commands.append((19, "Step 19: 윙바디 상차 3번 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path, True, False, True))
    
    # 20단계
    commands.append((20, "Step 20: 무브중 tilt 동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50, 2), 
                     pub_fork, True, False, False))
    
    # 21단계
    commands.append((21, "Step 21: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 103, 2), 
                     pub_fork, False, False, True))
    
    # 22단계
    commands.append((22, "Step 22: 1단 언로딩 - 윙바디 상차 _ 3번", 
                     get_forklift_msg(9.0, -10.98, 0.0, 4.35, -10.98, 0.0, 3, 2), 
                     pub_fork, True, True, True))      

    # 23단계
    path_2_start = [
        get_waypoint_msg(1000, 1.4, 9.0, -10.980, 0.0, True),
        get_waypoint_msg(2000, 1.4, 9.0, -1.86, 0.0, True),
        # get_waypoint_msg(2000, 1.4, 9.593, -1.86, 0.0, True)
    ]
    commands.append((23, "Step 23: 렉 1번 위치 이동", create_path_command_with_ids(path_2_start), pub_live_path, True, False, True))
    
    # 24단계
    commands.append((24, "Step 24: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 101, 1), 
                     pub_fork, True, False, False))
    
    # 25단계
    commands.append((25, "Step 25: 렉방 1번 위치 1단 로딩 - 랙방", 
                     get_forklift_msg(9.0, -1.86, 180.0, 13.025, -1.86, 180.0, 1, 1), 
                     pub_fork, True, True, True))
    
    # 26단계
    path_1_start = [
        # get_waypoint_msg(1000, 1.4, 9.593, -1.86, 0.0, True),
        get_waypoint_msg(1000, 1.4, 9.0, -1.86, 0.0, True),
        get_waypoint_msg(1000, 1.4, 9.0, -13.18, 0.0, True)
    ]
    commands.append((26, "Step 26: 윙바디 상차 _ 4번 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path, True, False, True))
    
    # 27단계
    commands.append((27, "Step 27: 무브중 tilt 동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50, 2), 
                     pub_fork, True, False, False))
    
    # 28단계
    commands.append((28, "Step 28: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 103, 2), 
                     pub_fork, False, False, True))
    
    # 29단계
    commands.append((29, "Step 29 : 1단 언로딩 - 윙바디 상차 _ 4번", 
                     get_forklift_msg(9.0, -13.18, 0.0, 4.35, -13.18, 0.0, 3, 2), 
                     pub_fork, True, True, True))    


    # 30단계
    path_1_start = [
        get_waypoint_msg(1000, 1.4, 9.0, -13.18, 0.0, True),
        get_waypoint_msg(2000, 1.4, 9.0, -9.385, 0.0, True),
        # get_waypoint_msg(2000, 0.5, 9.560, -9.385, 0.0, True)
    ]
    commands.append((30, "Step 30: 시작 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path, True, False, True))

    # 31단계: 엔터 대기
    commands.append((31, "WAIT_FOR_ENTER", None))  # 사용자 엔터 대기

    # 32단계
    commands.append((32, "Step 32: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 101, 1), 
                     pub_fork, True, False, False))
    
    # 33단계
    commands.append((33, "Step 33: 렉방 4번 위치 1단 로딩 - 랙방", 
                     get_forklift_msg(9.0, -9.385, 180.0, 13.05, -9.385, 180.0, 1, 1), 
                     pub_fork, True, False, True))


    # 34단계
    path_1_start = [
        # get_waypoint_msg(1000, 0.5, 9.560, -9.385, 0.0, True),
        get_waypoint_msg(1000, 1.4, 9.0, -9.385, 0.0, True),
        get_waypoint_msg(2000, 0.5, 9.0, -16.910, 0.0, True),
        # get_waypoint_msg(2000, 0.5, 9.487, -16.890, 0.0, True)

    ]
    commands.append((34, "Step 34: 비전방 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path, True, False, True))
    
    # 35단계
    commands.append((35, "Step 35: 무브중 tilt 동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50, 2), 
                     pub_fork, True, False, False))
    
    # 36단계
    commands.append((36, "Step 36: 무브중 lift동작",
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99, 1), 
                     pub_fork, False, False, True))

    # 37단계
    commands.append((37, "Step 37: 비전방 1단 언로딩",
                     get_forklift_msg(9.0, -16.910, 180.0, 12.972, -16.890, 180.0, -1, 1), 
                     pub_fork, True, True, True))


    # 38단계
    path_2_start = [
        # get_waypoint_msg(1000, 1.4, 9.487, -16.890, 0.0, True),
        get_waypoint_msg(1000, 1.4, 9.0, -16.910, 0.0, True),
        get_waypoint_msg(2000, 0.5, 9.0, -6.895, 0.0, True),
        # get_waypoint_msg(2000, 0.5, 9.540, -6.895, 0.0, True),
    ]
    commands.append((38, "Step 38: 렉 3번 위치 이동", create_path_command_with_ids(path_2_start), pub_live_path, True, False, True))
    
    # 39단계
    commands.append((39, "Step 39: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 101, 1), 
                     pub_fork, True, False, False))
    
    # 40단계
    commands.append((40, "Step 40: 렉방 3번 위치 1단 로딩 - 랙방",
                     get_forklift_msg(9.0, -6.895, 180.0, 13.030, -6.895, 180.0, 1, 1), 
                     pub_fork, True, True, True))


    # 41단계
    path_1_start = [
        # get_waypoint_msg(1000, 1.4, 9.540, -6.895, 0.0, True),
        get_waypoint_msg(1000, 1.4, 9.0, -6.895, 0.0, True),
        get_waypoint_msg(2000, 0.5, 9.0, -16.890, 0.0, True),
        # get_waypoint_msg(2000, 0.5, 9.487, -16.890, 0.0, True)
    ]
    commands.append((41, "Step 41: 비전방 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path, True, False, True))
    
    # 42단계
    commands.append((42, "Step 42: 무브중 tilt 동작",
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50, 2), 
                     pub_fork, True, False, False))
    
    # 43단계
    commands.append((43, "Step 43: 무브중 lift동작",
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99, 3), 
                     pub_fork, False, False, True))

    # 44단계
    commands.append((44, "Step 44: 비전방 2단 언로딩",
                     get_forklift_msg(9.0, -16.890, 180.0, 12.977, -16.890, 180.0, -1, 3), 
                     pub_fork, True, True, True))

    # 45단계: 엔터 대기
    commands.append((45, "WAIT_FOR_ENTER", None))  # 사용자 엔터 대기

    # 46단계
    commands.append((46, "Step 46: 비전방 2단 로딩", 
                     get_forklift_msg(9.0, -16.890, 180.0, 12.977, -16.890, 180.0, 1, 3), 
                     pub_fork, True, False, True))
    

    # 47단계
    path_1_start = [
        # get_waypoint_msg(1000, 0.5, 9.487, -16.890, 0.0, True),
        get_waypoint_msg(1000, 1.4, 9.0, -16.890, 0.0, True),
        get_waypoint_msg(2000, 1.4, 9.0, 1.7, 0.0, True)
    ]
    commands.append((47, "Step 47: 평치 안착 위치 이동", create_path_command_with_ids(path_1_start), pub_live_path, True, False, True))
    
    # 48단계
    commands.append((48, "Step 48: 무브중 tilt 동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50, 2), 
                     pub_fork, True, False, False))
    
    # 49단계
    commands.append((49, "Step 49: 무브중 lift동작", 
                     get_forklift_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99, 0), 
                     pub_fork, False, False, True))
    
    # 50단계
    commands.append((50, "Step 50: 평치 1단 언로딩", 
                     get_forklift_msg(9.0, 1.7, 180.0, 13.0, 1.7, 180.0, -1, 0), 
                     pub_fork, True, True, True))

    
    # 순차 실행
    try:
        for cmd in commands:
            # 쉬기 전용 단계: (step_num, "설명", 대기_초) 3개 요소
            if len(cmd) == 3:
                step_num, msg, rest_sec = cmd
                # 엔터 대기 특수 커맨드
                if msg == "WAIT_FOR_ENTER":
                    if step_num == 31:
                        # Step 30의 이동 완료 확인
                        print(f"[Step {step_num}] Step 30 이동 완료 대기 중...")
                        while not rospy.is_shutdown():
                            if goal_arrived:
                                goal_arrived_check_start = False
                                goal_arrived = False
                                print(f"Step 30 이동 완료 확인됨.")
                                break
                            rospy.sleep(0.1)
                        pub_scenario_complete.publish(4)
                        print(f"{GREEN}[ScenarioComplete] 시나리오 완료 트리거(4) 전송됨{NC}")
                    input(f"{CYAN}[PAUSE] 엔터키를 누르면 다음 단계로 진행합니다...{NC}")
                    continue
                if step_num < start_step:
                    print(f"{GRAY}[SKIP] Step {step_num} (대기) 건너뜀{NC}")
                    continue
                run_rest(step_num, msg, rest_sec)
                continue

            # 튜플 언패킹 (가변 길이 지원)
            wait_lift_at_start = False # 기본값
            check_goal_arrived = False  # 기본값
            wait_fork_lift_done = False # 기본값
            post_delay = 0.0 # 기본값

            if len(cmd) == 8:
                step_num, msg, ros_msg, pub, wait_lift_at_start, check_goal_arrived, wait_fork_lift_done, post_delay = cmd
            elif len(cmd) == 7:
                step_num, msg, ros_msg, pub, wait_lift_at_start, check_goal_arrived, wait_fork_lift_done = cmd
            elif len(cmd) == 6:
                step_num, msg, ros_msg, pub, wait_lift_at_start, check_goal_arrived = cmd
            elif len(cmd) == 5:
                step_num, msg, ros_msg, pub, wait_lift_at_start = cmd
            else:
                step_num, msg, ros_msg, pub = cmd

            # 시작 단계보다 현재 단계가 작으면 건너뜀 (Skip)
            if step_num < start_step:
                print(f"{GRAY}[SKIP] Step {step_num} 건너뜀{NC}")
                continue

            if step_num == 1:
                while not rospy.is_shutdown():
                    if scenario_control_1:
                        break
                    rospy.sleep(0.1)
            elif step_num == 9:
                pub_scenario_complete.publish(1)
            elif step_num == 16:    # 시나리오 1번 종료
                # Step 15의 이동 완료 확인
                print(f"[Step {step_num}] Step 15 이동 완료 대기 중...")
                while not rospy.is_shutdown():
                    if goal_arrived:
                        goal_arrived_check_start = False
                        goal_arrived = False
                        print(f"Step 15 이동 완료 확인됨.")
                        break
                    rospy.sleep(0.1)
                print(f"{GREEN}[ScenarioComplete] 시나리오 완료 트리거(2) 전송됨{NC}")
                pub_scenario_complete.publish(2)
                while not rospy.is_shutdown():
                    if scenario_control_2:
                        break
                    rospy.sleep(0.1)
            elif step_num == 24:    # 시나리오 2번 종료
                pub_scenario_complete.publish(3)

            run_step(step_num, start_step, msg, pub, ros_msg, 
                     wait_lift_at_start=wait_lift_at_start, 
                     check_goal_arrived=check_goal_arrived,
                     wait_fork_lift_done=wait_fork_lift_done,
                     post_delay=post_delay)
    except KeyboardInterrupt:
        print(f"\n{GRAY}사용자 요청에 의해 스크립트를 종료합니다.{NC}")
        sys.exit(0)

    # 모든 시나리오가 종료됨 (시나리오 단계 수가 바뀌어도 마지막 명령 실행 후 한 번만 전송)
    print(f"{GREEN}[ScenarioComplete] 모든 시나리오가 종료되었습니다.{NC}")
    pub_scenario_complete.publish(100)
    print(f"{GREEN}=== 모든 시나리오가 종료되었습니다 ==={NC}")

if __name__ == "__main__":
    main()
