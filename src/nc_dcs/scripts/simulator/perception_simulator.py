#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import random
import math

class PerceptionSimulator:
    """인식 시뮬레이터 - 팔레트/랙 위치 인식"""
    
    def __init__(self):
        rospy.init_node('perception_simulator', anonymous=False)
        
        # State
        self.perception_active = False
        
        # Fake target positions (can be randomized)
        self.pallet_positions = [
            (4.0, 0.0, 0.0),    # x, y, theta
            (4.0, 0.0, 0.0),
            (4.0, 0.0, 0.0)
        ]
        self.current_target_index = 0
        
        # Subscribers
        self.cmd_sub = rospy.Subscriber('/perception/cmd', String, self.cmd_callback)
        
        # Publishers
        self.pose_pub = rospy.Publisher('/perception/pose', PoseStamped, queue_size=10)
        
        # Timer
        self.update_rate = rospy.Rate(20)  # 20Hz
        
        rospy.loginfo("[PercSim] Perception Simulator initialized")
        rospy.loginfo("[PercSim] Listening to: /perception/cmd")
        rospy.loginfo("[PercSim] Publishing to: /perception/pose")
    
    def cmd_callback(self, msg):
        """인식 명령 수신"""
        try:
            import json
            data = json.loads(msg.data)
            if isinstance(data, dict):
                cmd = data.get("cmd", "").lower()
            else:
                cmd = str(data).lower()
        except ValueError:
            cmd = msg.data.lower()
            
        rospy.loginfo("[PercSim] Received command: %s (raw: %s)", cmd, msg.data)
        
        if "start" in cmd or "enable" in cmd or "on" in cmd:
            self.perception_active = True
            rospy.loginfo("[PercSim] Perception activated")
            
            # Publish target pose after short delay (simulate processing time)
            rospy.Timer(rospy.Duration(0.5), self.publish_target_pose, oneshot=True)
        
        elif "stop" in cmd or "disable" in cmd or "off" in cmd:
            self.perception_active = False
            rospy.loginfo("[PercSim] Perception deactivated")
        
    
    def publish_target_pose(self, event=None):
        """타겟 위치 퍼블리시 (시뮬레이션)"""
        if not self.perception_active:
            return
        
        # Get target position
        pos = self.pallet_positions[self.current_target_index % len(self.pallet_positions)]
        
        # Add small random noise
        # noise_x = random.uniform(-0.05, 0.05)
        # noise_y = random.uniform(-0.05, 0.05)
        # noise_theta = random.uniform(-0.05, 0.05)
        
        # Create PoseStamped message
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        theta = pos[2]
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(theta / 2.0)  # Simplified 
        msg.pose.orientation.w = math.cos(theta / 2.0)
        
        # Publish
        self.pose_pub.publish(msg)
        # rospy.loginfo("[PercSim] Published %s pose: x=%.2f, y=%.2f, theta=%.2f", 
        #              self.target_type, msg.pose.position.x, msg.pose.position.y, theta)
        
        # Move to next target for variety
        self.current_target_index += 1
    
    def run(self):
        """메인 루프"""
        rospy.loginfo("[PercSim] Starting main loop...")
        rospy.loginfo("[PercSim] Send commands to /perception/cmd to start")
        rospy.loginfo("[PercSim]   - 'start' or 'enable' to activate")
        rospy.loginfo("[PercSim]   - 'stop' or 'disable' to deactivate")
        
        while not rospy.is_shutdown():
            # Continuously publish if active (simulating continuous tracking)
            if self.perception_active:
                self.publish_target_pose()
            
            self.update_rate.sleep()


if __name__ == '__main__':
    try:
        simulator = PerceptionSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass
