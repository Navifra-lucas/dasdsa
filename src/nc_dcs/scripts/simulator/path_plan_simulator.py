#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion
from move_msgs.msg import CoreCommand, Waypoint
from dcs_msgs.msg import DockingInfo
from std_msgs.msg import String, Bool, Header
from std_srvs.srv import Trigger, TriggerResponse

import math

class PathPlanSimulator:
    """경로계획 및 도킹 시뮬레이터"""
    
    def __init__(self):
        rospy.init_node('path_plan_simulator', anonymous=False)
        
        # State
        self.current_pose = [0.0, 0.0, 0.0]  # x, y, theta
        self.target_pose = None
        self.update_target_pose = None
        self.is_docking_ready = False
        self.is_docking_start = False
        self.is_goingback = False
        self.docking_progress = 0.0
        self.docking_duration = 20.0
        self.last_publish_time = rospy.Time(0)
        self.publish_interval = 0.5   # 1초에 2번

        
        # Subscribers
        self.cmd_sub = rospy.Subscriber('/path_plan/cmd', DockingInfo, self.cmd_callback)
        self.goback_sub = rospy.Subscriber('/path_plan/goback', Bool, self.goback_callback)
        self.path_pos_sub = rospy.Subscriber('/path_plan/pose', PoseStamped, self.path_pos_callback)
        
        # Publishers
        self.status_pub = rospy.Publisher('/path_plan/status', String, queue_size=10)
        self.docking_path_pub = rospy.Publisher('/path_plan/docking_path', PoseArray, queue_size=10)
        
        self.path_stop_serv = rospy.Service('/path_plan/done', Trigger, self.stop_callback)

        # Timer
        self.update_rate = rospy.Rate(20)  # 20Hz
        
        rospy.loginfo("[PathSim] Path Plan Simulator initialized")
        rospy.loginfo("[PathSim] Listening to: /path_plan/cmd, /path_plan/goback")
        rospy.loginfo("[PathSim] Publishing to: /path_plan/status")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw

    def stop_callback(self, req):
        self.is_docking_ready = False
        self.is_docking_start = False
        rospy.loginfo("[PathSim] Received STOP service")
        return TriggerResponse(success=True, message="Stopped docking")


    def path_pos_callback(self, msg):

        if not self.is_docking_ready:
            rospy.logdebug("[PathSim] Received path position (ignored, not docking)")
            return
        
        if not self.is_docking_start:
            self.is_docking_start = True
            self.docking_progress= 0.0
            self.start_time = rospy.Time.now()
            rospy.loginfo("Docking start trigger sub")

        self.update_target_pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            self.quaternion_to_yaw(msg.pose.orientation)
        ]
        rospy.loginfo("[PathSim] Received path position: x=%.2f, y=%.2f, theta=%.2f", 
                     self.update_target_pose[0], self.update_target_pose[1], self.update_target_pose[2])


    def cmd_callback(self, msg):
        """도킹 명령 수신"""
        self.target_pose = [
            msg.o_global_pose.position.x,
            msg.o_global_pose.position.y,
            self.quaternion_to_yaw(msg.o_global_pose.orientation)
        ]
        
        if not self.is_docking_ready:
            self.is_docking_ready = True
            self.is_docking_start = False
            self.docking_progress = 0.0
            rospy.loginfo("[PathSim] Docking started to target: x=%.2f, y=%.2f, theta=%.2f", 
                         self.target_pose[0], self.target_pose[1], self.target_pose[2])
    
    def goback_callback(self, msg):
        """후진 명령 수신"""
        if msg.data:
            self.is_goingback = True
            self.docking_progress = 0.0
            rospy.loginfo("[PathSim] GoBack started")
    
    def quaternion_to_yaw(self, q):
        """쿼터니언을 yaw 각도로 변환"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def update_state(self):
        """도킹/후진 진행 상태 업데이트"""

        # --- 도킹 진행 중이 아니라면 무시 ---
        if not self.is_docking_start or self.start_time is None:
            return

        # 경과시간
        now = rospy.Time.now()
        elapsed = (now - self.start_time).to_sec()

        # progress 계산 (0 → 1)
        self.docking_progress = min(1.0, elapsed / self.docking_duration)

        # current_pose 보간
        if self.target_pose is not None:
            sx, sy, srad = self.current_pose
            gx, gy, grad = self.target_pose

            alpha = self.docking_progress

            self.current_pose[0] = (1 - alpha) * sx + alpha * gx
            self.current_pose[1] = (1 - alpha) * sy + alpha * gy
            self.current_pose[2] = (1 - alpha) * srad + alpha * grad

        # 도킹 완료
        if self.docking_progress >= 1.0:
            self.current_pose = self.target_pose[:]   # 최종 위치 고정
            self.is_docking_start = False             # END 상태
            self.is_docking_ready = False             # READY 해제
            self.publish_status("Docking Done")
            rospy.loginfo("[PathSim] Docking completed at x=%.2f, y=%.2f",
                        self.current_pose[0], self.current_pose[1])

    
    def publish_status(self, status):
        """상태 메시지 퍼블리시"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        rospy.loginfo("[PathSim] Published status: %s", status)
    
    def publish_docking_path(self):
        if not self.is_docking_start or self.target_pose is None:
            return


        now = rospy.Time.now()
        if (now - self.last_publish_time).to_sec() < self.publish_interval:
            return  # ★ 0.5초 안 됐으면 패킷 안 보냄

        self.last_publish_time = now 
        
        # --------------------------
        # START NODE: current pose
        # --------------------------
        sx = self.current_pose[0]
        sy = self.current_pose[1]
        srad = self.current_pose[2]
        sdeg = math.degrees(srad)

        # --------------------------
        # GOAL NODE: target pose
        # --------------------------
        gx = self.target_pose[0]
        gy = self.target_pose[1]
        grad = self.target_pose[2]
        gdeg = math.degrees(grad)

        # Create PoseArray message
        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"

        # Convert start pose to quaternion
        start_pose = Pose()
        start_pose.position.x = sx
        start_pose.position.y = sy
        start_pose.position.z = 0.0
        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, srad)
        start_pose.orientation.x = qx
        start_pose.orientation.y = qy
        start_pose.orientation.z = qz
        start_pose.orientation.w = qw
        pose_array.poses.append(start_pose)

        # Convert goal pose to quaternion
        goal_pose = Pose()
        goal_pose.position.x = gx
        goal_pose.position.y = gy
        goal_pose.position.z = 0.0
        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, grad)
        goal_pose.orientation.x = qx
        goal_pose.orientation.y = qy
        goal_pose.orientation.z = qz
        goal_pose.orientation.w = qw
        pose_array.poses.append(goal_pose)

        # publish
        self.docking_path_pub.publish(pose_array)

        rospy.loginfo("[PathSim] PoseArray published: start(%.2f,%.2f,%.1f°) → goal(%.2f,%.2f,%.1f°)",
                    sx, sy, sdeg, gx, gy, gdeg)
        
    def run(self):
        """메인 루프"""
        rospy.loginfo("[PathSim] Starting main loop...")
        
        while not rospy.is_shutdown():
            self.update_state()
            if self.is_docking_start and self.update_target_pose is not None:
                self.publish_docking_path()
            self.update_rate.sleep()


if __name__ == '__main__':
    try:
        simulator = PathPlanSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass
