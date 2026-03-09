#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from core_msgs.msg import WiaForkInfo, CheonilReadRegister

class ForkSimulator:
    """포크리프트 제어 시뮬레이터"""
    
    def __init__(self):
        rospy.init_node('fork_simulator', anonymous=False)
        
        # Current state
        self.fork_height = 0  # mm
        self.fork_width = 0   # 0: narrow, 1: wide
        self.fork_tilt = 0.0  # 0: down, 1: up
        
        # Target state
        self.target_height = 0
        self.target_width = 0
        self.target_tilt = 0.0
        
        # Movement simulation parameters
        self.height_speed = 20  # mm/tick
        self.moving = False
        
        # Subscribers
        self.fork_cmd_sub = rospy.Subscriber('/forkinfo', WiaForkInfo, self.fork_cmd_callback)
        
        # Publishers
        self.status_pub = rospy.Publisher('/cheonil/read_register', CheonilReadRegister, queue_size=10)
        
        # Timer for status update
        self.update_rate = rospy.Rate(20)  # 20Hz
        
        rospy.loginfo("[ForkSim] Fork Simulator initialized")
        rospy.loginfo("[ForkSim] Listening to: /forkinfo")
        rospy.loginfo("[ForkSim] Publishing to: /cheonil/read_register")
    
    def fork_cmd_callback(self, msg):
        """포크리프트 명령 수신"""
        rospy.loginfo("[ForkSim] Received command: height=%d, width=%d, tilt=%.2f", 
                      msg.n_fork_height, msg.n_fork_wide, msg.f_fork_tilt)
        
        # Update targets if valid
        if msg.n_fork_height >= 0:
            self.target_height = msg.n_fork_height
            self.moving = True
            rospy.loginfo("[ForkSim] Target height set to: %d mm", self.target_height)
        
        elif msg.n_fork_wide >= 0:
            self.target_width = msg.n_fork_wide + 1
            rospy.loginfo("[ForkSim] Target width set to: %d", self.target_width)
        
        elif msg.f_fork_tilt >= 0:
            self.target_tilt = msg.f_fork_tilt + 1
            rospy.loginfo("[ForkSim] Target tilt set to: %.2f", self.target_tilt)
    
    def update_state(self):
        """포크 상태 업데이트 (시뮬레이션)"""
        # Simulate height movement
        if self.fork_height != self.target_height:
            diff = self.target_height - self.fork_height
            if abs(diff) <= self.height_speed:
                self.fork_height = self.target_height
                rospy.loginfo("[ForkSim] Height reached target: %d mm", self.fork_height)
            else:
                step = self.height_speed if diff > 0 else -self.height_speed
                self.fork_height += step
        
        # Instant update for width and tilt (simplified)
        if self.fork_width != self.target_width:
            self.fork_width = self.target_width
            rospy.loginfo("[ForkSim] Width changed to: %d", self.fork_width)
        
        if abs(self.fork_tilt - self.target_tilt) > 0.01:
            self.fork_tilt = self.target_tilt
            rospy.loginfo("[ForkSim] Tilt changed to: %.2f", self.fork_tilt)
    
    def publish_status(self):
        """PLC 상태 퍼블리시"""
        msg = CheonilReadRegister()
        
        # Fill status message
        msg.fork_up_down_position = self.fork_height
        msg.fork_up_down_complete = 1 if self.fork_height == self.target_height else 0
        msg.fork_width = self.fork_width
        msg.tilting_up_down = int(self.fork_tilt)
        msg.battery = 95  # Fake battery level
        msg.auto_on = 1
        
        self.status_pub.publish(msg)
    
    def run(self):
        """메인 루프"""
        rospy.loginfo("[ForkSim] Starting main loop...")
        
        while not rospy.is_shutdown():
            self.update_state()
            self.publish_status()
            self.update_rate.sleep()


if __name__ == '__main__':
    try:
        simulator = ForkSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass
