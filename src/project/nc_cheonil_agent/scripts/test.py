import redis
import json
import time
import rospy
from std_msgs.msg import Bool
from core_msgs.msg import NaviAlarm

class RobotCommander:
    def __init__(self):
        # ROS 설정
        rospy.init_node('test', anonymous=True)
        rospy.Subscriber('/navifra/alarm', NaviAlarm, self.navifraAlarmCallback, queue_size=1)
        rospy.Subscriber('/Navifra/Cheonil_status', Bool, self.cheonilCallback, queue_size=1)

        # Redis 설정
        self.redis_host = "172.30.1.100"
        self.redis_port = 6379
        self.robot_channel = "robot.request:0ff0cbb5-6f54-47a4-b782-913a50004995"#해당 로봇에 맞는 id로 변경

        self.base_command = {
            "uuid": "2f3d332c-d030-4cd7-987f-1de6405e09bb",
            "from": "backend"
        }

        self.run = False
        self.index = 0
        self.cycle_count = 0  # 반복 횟수 카운트
        self.max_cycles = 10  # 10번 반복

        try:
            self.redis_client = redis.Redis(
                host=self.redis_host,
                port=self.redis_port,
                decode_responses=True
            )
            self.redis_client.ping()
            print("Connected to Redis successfully")
        except redis.ConnectionError:
            print("Failed to connect to Redis")
            self.redis_client = None

        self.execute_command_sequence()

    def navifraAlarmCallback(self, msg):
        print("navifraAlarmCallback", msg.alarm)
        if msg.alarm == 1002:
            self.run = False
            self.index += 1
            time.sleep(2)
            self.execute_command_sequence()

    def cheonilCallback(self, msg):
        print("cheonilCallback", msg.data)
        if msg.data !=True and self.is_cheonil_command():
            print("cheonilCallback: Cheonil 동작 완료")
            self.run = False
            self.index += 1
            time.sleep(2)
            self.execute_command_sequence()
    
    def is_move_command(self):
        """현재 index가 move 관련인지 확인"""
        return self.index in [0, 3, 6]

    def is_cheonil_command(self):
        """현재 index가 cheonil 관련인지 확인"""
        return self.index in [1, 2, 4, 5]

    def send_robot_command(self, command_data):
        """Redis를 통해 로봇 명령 전송"""
        if self.redis_client:
            self.redis_client.publish(self.robot_channel, json.dumps(command_data))
            time.sleep(2)

    def execute_command_sequence(self):
        """명령 실행"""
        if self.cycle_count >= self.max_cycles:
            print("명령 실행 완료 (10번 반복)")
            return  # 10번 반복 후 종료

        if not self.run:
            # if self.index == 0:
            #     print(f"Cycle {self.cycle_count + 1}: Moving to node 11")
            #     move_command = {**self.base_command, "action": "move",
            #                     "data": {"node_id": "48ab66ba-5099-440e-8784-451c23d71bbe"}}
            #     self.send_robot_command(move_command)

            if self.index == 0:
                move_command = {**self.base_command, "action": "move",
                                "data": {"node_id": "18683a2f-15d0-47df-b75e-fb19d53b0bea"}}
                self.send_robot_command(move_command)
                print("Moving to node 6")

            elif self.index == 1:
                time.sleep(3)
                load_command = {**self.base_command, "action": "cheonil_lift",
                                "data": {"type": "load", "level": 2}}
                self.send_robot_command(load_command)
                print("Performing load operation")

            elif self.index == 2:
                time.sleep(3)
                unload_command = {**self.base_command, "action": "cheonil_lift",
                                  "data": {"type": "unload", "level": 2}}
                self.send_robot_command(unload_command)
                print("Performing unload operation")

            elif self.index == 3:
                time.sleep(3)
                move_command = {**self.base_command, "action": "move",
                                "data": {"node_id": "83cc7c08-1316-4c96-8c4e-3df6609bcf41"}}
                self.send_robot_command(move_command)
                print("Moving to node 5")

            elif self.index == 4:
                time.sleep(3)
                load_command = {**self.base_command, "action": "cheonil_lift",
                                "data": {"type": "load", "level": 1}}
                self.send_robot_command(load_command)
                print("Performing load operation")

            elif self.index == 5:
                time.sleep(3)
                unload_command = {**self.base_command, "action": "cheonil_lift",
                                  "data": {"type": "unload", "level": 1}}
                self.send_robot_command(unload_command)
                print("Performing unload operation")

            elif self.index == 6:
                time.sleep(3)
                move_command = {**self.base_command, "action": "move",
                                "data": {"node_id": "2fad2316-2db6-4412-b0cd-9546a3c215dc"}}
                self.send_robot_command(move_command)
                print("Moving to node 8")

                self.index = -1
                self.cycle_count += 1
                print(f"Cycle {self.cycle_count} 완료")

                if self.cycle_count >= self.max_cycles:
                    print("done")
                    return

                self.execute_command_sequence()


if __name__ == "__main__":
    commander = RobotCommander()
    rospy.spin()
