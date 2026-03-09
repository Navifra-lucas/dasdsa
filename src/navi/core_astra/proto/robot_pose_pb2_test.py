import zmq
import robot_pose_pb2
import robot_status_pb2

def handle_agent_state(msg_str):
    print("[AgentState]", msg_str)

def handle_robot_pose(data):
    prefix_len = len("robot_pose ")
    protobuf_bytes = data[prefix_len:]
    pose = robot_pose_pb2.RobotPose()
    pose.ParseFromString(protobuf_bytes)
    print(f"[RobotPose] position=({pose.position.x:.2f}, {pose.position.y:.2f}) "
          f"orientation=({pose.orientation.x:.2f}, {pose.orientation.y:.2f}, "
          f"{pose.orientation.z:.2f}, {pose.orientation.w:.2f})")

def handle_robot_status(data):
    prefix_len = len("robot_status ")
    protobuf_bytes = data[prefix_len:]
    status = robot_status_pb2.RobotStatus()
    status.ParseFromString(protobuf_bytes)

    print("[RobotStatus]")
    print(f"  id={status.id}, status={status.robot_status}, alarm={status.robot_alarm}")
    print(f"  pose=({status.robot_pose_x:.2f}, {status.robot_pose_y:.2f}, {status.robot_pose_deg:.1f} deg)")
    print(f"  battery={status.battery:.1f}%")
    print(f"  now_node={status.now_node}, next_node={status.next_node}, goal_node={status.goal_node}")
    print(f"  task={status.now_task}, is_load={status.is_load}")
    print(f"  velocity=({status.robot_linear_velocity_x:.2f}, "
          f"{status.robot_linear_velocity_y:.2f}, {status.robot_linear_velocity_z:.2f}) "
          f"angular={status.robot_angular_velocity:.2f}")
    print(f"  error_dist={status.error_dist:.3f}")
    print(f"  errors={list(status.errortable)}")
    print(f"  robot_type={status.robottype}")
    print("-" * 40)

def main():
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.connect("tcp://localhost:5557")

    # 전체 구독 (모든 메시지 수신)
    subscriber.setsockopt_string(zmq.SUBSCRIBE, "")

    while True:
        data = subscriber.recv()
        # 토픽 prefix 판별
        if data.startswith(b"agent_state"):
            handle_agent_state(data.decode('utf-8', errors='ignore'))
        elif data.startswith(b"robot_pose"):
            handle_robot_pose(data)
        elif data.startswith(b"robot_status"):
            handle_robot_status(data)
        else:
            print("[Unknown]", data)

if __name__ == "__main__":
    main()
