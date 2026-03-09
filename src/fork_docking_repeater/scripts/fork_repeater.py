#!/usr/bin/env python

import rospy
import time
from core_msgs.msg import ForkLift
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PoseWithCovarianceStamped


class ForkRepeater:
    def __init__(self):
        rospy.init_node('fork_repeater_node')
        
        self.pub_fork = rospy.Publisher('/nc_task_manager/fork_docking', ForkLift, queue_size=1)
        self.pub_trigger = rospy.Publisher('/wia_agent/wingbody_trigger', Int32, queue_size=1)
        
        self.reached = False
        self.sub = rospy.Subscriber('/fork_lift_reached', Bool, self.reached_cb)
        
        self.robot_pos = None
        self.robot_pos_sub = rospy.Subscriber('/localization/robot_pos', PoseWithCovarianceStamped, self.robot_pos_cb)

        # Define coordinates for each index (0-4)
        # Format: {index: {'cx': val, 'cy': val, 'cdeg': val, 'tx': val, 'ty': val, 'tdeg': val}}
        # Default values are currently set to the example provided for all indices. 
        # You can update specific values here.
        coords = {
            0: {'cx': 9.0, 'cy': 9.3, 'cdeg': 0.0, 'tx': 4.380, 'ty': 9.3, 'tdeg': 0.0},
            1: {'cx': 9.0, 'cy': 6.1, 'cdeg': 0.0, 'tx': 4.380, 'ty': 6.1, 'tdeg': 0.0},
            2: {'cx': 9.0, 'cy': 3.9, 'cdeg': 0.0, 'tx': 4.380, 'ty': 3.9, 'tdeg': 0.0},
            3: {'cx': 9.0, 'cy': 1.7, 'cdeg': 0.0, 'tx': 4.380, 'ty': 1.7, 'tdeg': 0.0},
            4: {'cx': 9.0, 'cy': -13.18, 'cdeg': 0.0, 'tx': 4.350, 'ty': -13.18, 'tdeg': 0.0}
        }
        # coords = {
        #     0: {'cx': 7.88, 'cy': 1.70, 'cdeg': 0.0, 'tx': 4.380, 'ty': 1.70, 'tdeg': 0.0},
        #     1: {'cx': 7.63, 'cy': 1.70, 'cdeg': 0.0, 'tx': 4.380, 'ty': 1.70, 'tdeg': 0.0},
        #     2: {'cx': 8.13, 'cy': 1.70, 'cdeg': 0.0, 'tx': 4.380, 'ty': 1.70, 'tdeg': 0.0},
        #     3: {'cx': 7.88, 'cy': 1.45, 'cdeg': 0.0, 'tx': 4.380, 'ty': 1.45, 'tdeg': 0.0},
        #     4: {'cx': 7.88, 'cy': 1.95, 'cdeg': 0.0, 'tx': 4.380, 'ty': 1.95, 'tdeg': 0.0}
        # }

        self.positions = {}
        self.positions2 = {}

        for i in range(5):
            pos = ForkLift()
            pos.s_current_node_id = ''
            pos.s_target_node_id = ''
            
            c = coords[i]
            pos.f_current_x = c['cx']
            pos.f_current_y = c['cy']
            pos.f_current_deg = c['cdeg']
            pos.f_target_x = c['tx']
            pos.f_target_y = c['ty']
            pos.f_target_deg = c['tdeg']
            
            pos.n_rack_level = 0
            pos.n_target_level = 0
            pos.n_target_height = 0
            pos.n_drive_type = 1
            pos.n_rack_type = 2
            pos.n_pallet_type = 0
            
            self.positions[i] = pos
        
        for i in range(5):
            pos2 = ForkLift()
            pos2.s_current_node_id = ''
            pos2.s_target_node_id = ''
            
            c = coords[i]
            pos2.f_current_x = c['cx']
            pos2.f_current_y = c['cy']
            pos2.f_current_deg = c['cdeg']
            pos2.f_target_x = c['tx']
            pos2.f_target_y = c['ty']
            pos2.f_target_deg = c['tdeg']
            
            pos2.n_rack_level = 0
            pos2.n_target_level = 0
            pos2.n_target_height = 0
            pos2.n_drive_type = 3
            pos2.n_rack_type = 2
            pos2.n_pallet_type = 0
            
            self.positions2[i] = pos2

    def reached_cb(self, msg):
        if msg.data:
            self.reached = True
            rospy.loginfo("Reach signal received.")

    def robot_pos_cb(self, msg):
        self.robot_pos = msg

    def wait_for_reach(self):
        self.reached = False
        rospy.loginfo("Waiting for /fork_lift_reached...")
        while not self.reached and not rospy.is_shutdown():
            rospy.sleep(0.1)
        self.reached = False 

    def run(self):
        try:
            full_input = input("Enter Target Index (0-4) and Repetitions (e.g., '1/5'): ")
            parts = full_input.split("/")
            if len(parts) != 2:
                rospy.logerr("Please enter two numbers separated by space.")
                return
            
            target_index = int(parts[0])
            repetitions = int(parts[1])

            if target_index not in self.positions:
                rospy.logerr("Invalid index. Available: 0-4")
                return

        except ValueError:
            rospy.logerr("Invalid input. Please enter integers.")
            return

        cmd_fork = self.positions[target_index]
        cmd_fork2 = self.positions2[target_index]
        
        cmd_trigger = Int32()
        cmd_trigger.data = target_index

        for i in range(repetitions):
            if rospy.is_shutdown(): break

            # Step 1: ForkLift Command
            rospy.loginfo("[{}/{}] Publishing ForkLift Loading Command for Index {}".format(i+1, repetitions, target_index))
            self.pub_fork.publish(cmd_fork)
            self.wait_for_reach()
            
            time.sleep(5)
            if rospy.is_shutdown(): break

            # Step 2: Wingbody Trigger
            rospy.loginfo("[{}/{}] Publishing ForkLift Unloading Command for Index {}".format(i+1, repetitions, target_index))
            self.pub_fork.publish(cmd_fork2)
            self.wait_for_reach()
            time.sleep(5)

        rospy.loginfo("Completed {} repetitions for Index {}.".format(repetitions, target_index))

if __name__ == '__main__':
    try:
        node = ForkRepeater()
        node.run()
    except rospy.ROSInterruptException:
        pass
