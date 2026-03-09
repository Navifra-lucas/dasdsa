#!/usr/bin/env python3

import redis
import json
import rospy
from core_msgs.msg import VehicleList, Vehicle
from std_msgs.msg import Header
from nb_http_communicator import NbHttpCommunicator
import os
import signal
import threading
import math

# Redis 구독 및 ROS 토픽 이름 설정
REDIS_TOPIC = 'robot.status.info:*'
ROS_TOPIC = '/v2v_info'

# NbHttpCommunicator 설정
backend_host = os.getenv("BACKEND_HOST", "localhost")
backend_port = os.getenv("BACKEND_PORT", "5000")
redis_host = os.getenv("REDIS_HOST", "localhost")
redis_port = int(os.getenv("REDIS_PORT", "6379"))

# NbHttpCommunicator 인스턴스 생성
communicator = NbHttpCommunicator(
    base_url=f"http://{backend_host}:{backend_port}",
    redis_host=redis_host,
    redis_port=redis_port
)

# Redis 연결 및 종료 제어
stop_event = threading.Event()

def calculate_robot_size(robot_collision):
    # 앞, 뒤, 왼쪽, 오른쪽을 계산하기 위한 초기값 설정
    front = max(point[0] for point in robot_collision)
    rear = min(point[0] for point in robot_collision)
    left = max(point[1] for point in robot_collision)
    right = min(point[1] for point in robot_collision)

    # 앞, 뒤, 왼쪽, 오른쪽 크기 계산
    size_front = front
    size_rear = abs(rear)
    size_left = abs(left)
    size_right = abs(right)

    return size_front, size_rear, size_left, size_right

def redis_callback(message):
    try:
        # JSON 형식으로 로드
        data = json.loads(message['data'])

        # 환경 변수에서 robot_ip 가져오기
        robot_ip = os.getenv("ROBOT_IP", "")
        message_id = data.get('id')
        if not robot_ip or not message_id:
            rospy.logerr("robot_ip or message_id not found.")
            return

        # NbHttpCommunicator를 통해 find_amr로 IP를 필터링
        found_robot_id = communicator.find_amr(robot_ip)
        if found_robot_id == message_id:
            rospy.loginfo(f"Skipping vehicle_list_msg creation for robot with ID: {message_id}")
            return

        # Vehicle 메시지 생성
        vehicle_msg = Vehicle()
        vehicle_msg.id = message_id
        vehicle_msg.x_m = data['robot_pose']['position']['x']
        vehicle_msg.y_m = data['robot_pose']['position']['y']
        vehicle_msg.angle_deg = math.degrees(math.atan2(2.0 * (data['robot_pose']['orientation']['w'] * data['robot_pose']['orientation']['z'] + data['robot_pose']['orientation']['x'] * data['robot_pose']['orientation']['y']), 1.0 - 2.0 * (data['robot_pose']['orientation']['y'] * data['robot_pose']['orientation']['y'] + data['robot_pose']['orientation']['z'] * data['robot_pose']['orientation']['z'])))

        vehicle_msg.f_linear_speed_x_ms = data['robot_linear_velocity'][0]
        vehicle_msg.f_linear_speed_y_ms = data['robot_linear_velocity'][1]
        vehicle_msg.f_angular_speed_z_degs = data['robot_angular_velocity']

        # robot_collision 정보를 이용해 로봇 크기 계산
        size_front, size_rear, size_left, size_right = calculate_robot_size(data['robot_collision'])
        vehicle_msg.f_robot_size_front_m = size_front
        vehicle_msg.f_robot_size_rear_m = -size_rear
        vehicle_msg.f_robot_size_left_m = size_left
        vehicle_msg.f_robot_size_right_m = -size_right

        # VehicleList 메시지 생성
        vehicle_list_msg = VehicleList()
        vehicle_list_msg.header = Header()
        vehicle_list_msg.header.stamp = rospy.Time.now()
        vehicle_list_msg.data = [vehicle_msg]

        # ROS 메시지로 변환된 데이터를 게시
        ros_pub.publish(vehicle_list_msg)

        # rospy.loginfo(f"Published VehicleList: {vehicle_list_msg}")
    except Exception as e:
        rospy.logerr(f"Failed to process message: {e}")

def redis_listener():
    # Redis 클라이언트 설정
    redis_client = redis.StrictRedis(host=redis_host, port=redis_port, db=0, password="navifra1@3$")
    pubsub = redis_client.pubsub()
    pubsub.psubscribe(REDIS_TOPIC)

    rospy.loginfo(f"Subscribed to Redis pattern: {REDIS_TOPIC}")

    try:
        for message in pubsub.listen():
            if stop_event.is_set():
                rospy.loginfo("Stopping Redis listener...")
                break
            if message['type'] == 'pmessage':
                redis_callback(message)
    except Exception as e:
        rospy.logerr(f"Redis listener error: {e}")
    finally:
        pubsub.close()
        redis_client.close()
        rospy.loginfo("Redis connection closed.")

def signal_handler(sig, frame):
    rospy.loginfo("Interrupt received, shutting down...")
    stop_event.set()
    rospy.signal_shutdown("Shutdown signal received.")

if __name__ == '__main__':
    # SIGINT (Ctrl+C) 핸들러 설정
    signal.signal(signal.SIGINT, signal_handler)

    # ROS 노드 초기화
    rospy.init_node('redis_to_vehicle_list_converter', anonymous=True)

    # ROS 퍼블리셔 설정
    ros_pub = rospy.Publisher(ROS_TOPIC, VehicleList, queue_size=10)
    
    # 서버 연결 및 인증
    try:
        if communicator.connect_server(backend_host, backend_port):
            if communicator.authenticate("admin", "admin"):
                rospy.loginfo("NbHttpCommunicator is authenticated.")
                # Redis 구독 및 메시지 변환 시작
                redis_listener()
            else:
                rospy.logerr("Authentication failed.")
        else:
            rospy.logerr("Failed to connect to the server.")
    except Exception as e:
        rospy.logerr(f"Error during operation: {e}")
    finally:
        rospy.loginfo("Shutting down...")
