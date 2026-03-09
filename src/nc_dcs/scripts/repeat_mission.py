#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from core_msgs.msg import ForkLift
from std_msgs.msg import Bool

class MissionRepeater:
    def __init__(self):
        rospy.init_node('mission_repeater', anonymous=False)
        
        self.repetitions = 0
        self.current_mission = None
        self.mission_captured = False
        self.is_publishing = False
        
        # Publishers
        self.fork_pub = rospy.Publisher('/nc_task_manager/fork_docking', ForkLift, queue_size=10)
        
        # Subscribers
        self.fork_sub = rospy.Subscriber('/nc_task_manager/fork_docking', ForkLift, self.fork_callback)
        self.completion_sub = rospy.Subscriber('/fork_lift_reached', Bool, self.completion_callback)
        
        rospy.loginfo("Mission Repeater Node Initialized")

    def get_repetitions(self):
        while not rospy.is_shutdown():
            try:
                val = input("Enter number of repetitions: ")
                self.repetitions = int(val)
                rospy.loginfo(f"Set to repeat {self.repetitions} times.")
                rospy.loginfo("Waiting for the first mission to be published to /nc_task_manager/fork_docking...")
                break
            except ValueError:
                print("Invalid input. Please enter an integer.")

    def fork_callback(self, msg):
        if not self.mission_captured:
            self.current_mission = msg
            self.mission_captured = True
            rospy.loginfo("Mission captured! Will repeat this mission upon completion.")
            rospy.loginfo(f"Mission details - Drive Type: {msg.n_drive_type}, Target Height: {msg.n_target_height}")

    def completion_callback(self, msg):
        if not self.mission_captured:
            return

        if msg.data: # If completion is True
            if self.repetitions > 0:
                rospy.loginfo(f"Completion signal received. Repetitions left: {self.repetitions}")
                
                rospy.sleep(1.0)
                
                rospy.loginfo("Republishing mission...")
                self.is_publishing = True
                self.fork_pub.publish(self.current_mission)
                self.is_publishing = False
                
                self.repetitions -= 1
            elif self.repetitions == 0:
                rospy.loginfo("Completion signal received. All repetitions finished.")

    def run(self):
        self.get_repetitions()
        rospy.spin()

if __name__ == '__main__':
    try:
        repeater = MissionRepeater()
        repeater.run()
    except rospy.ROSInterruptException:
        pass
