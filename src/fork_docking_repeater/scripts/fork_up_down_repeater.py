#!/usr/bin/env python

import rospy
import time
from core_msgs.msg import WiaForkInfo, CheonilReadRegister
from std_msgs.msg import Bool, Int32

class ForkRepeater:
    def __init__(self):
        rospy.init_node('fork_up_down_repeater_node')
        
        self.pub_fork = rospy.Publisher('/forkinfo', WiaForkInfo, queue_size=1)
        
        self.reached = False
        self.sub = rospy.Subscriber('/cheonil/read_register', CheonilReadRegister, self.reached_cb)
        
        rospy.on_shutdown(self.cleanup)

    def cleanup(self):
        rospy.loginfo("Shutting down... Sending stop command.")
        cmd_fork = WiaForkInfo()
        cmd_fork.n_fork_height = -1
        cmd_fork.n_fork_wide = -1
        cmd_fork.f_fork_tilt = -1
        cmd_fork.b_cancel = True
        self.pub_fork.publish(cmd_fork)
        # Give some time for the message to be published
        rospy.sleep(0.5)

    def reached_cb(self, msg):
        if msg.fork_up_down_complete:
            self.reached = True
            rospy.loginfo("Reach signal received.")

    def wait_for_reach(self):
        self.reached = False
        rospy.loginfo("Waiting for /fork_lift_reached...")
        while not self.reached and not rospy.is_shutdown():
            rospy.sleep(0.1)
        self.reached = False 

    def run(self):
        i = 0
        time.sleep(5)
        while not rospy.is_shutdown():
            i += 1

            # Step 1: ForkLift Command
            rospy.loginfo("[{}] Publishing ForkLift UP Command".format(i))
            cmd_fork = WiaForkInfo()
            cmd_fork.n_fork_height = 1600
            cmd_fork.n_fork_wide = -1
            cmd_fork.f_fork_tilt = -1
            cmd_fork.b_cancel = False
            self.pub_fork.publish(cmd_fork)
            self.wait_for_reach()
            
            time.sleep(1)
            if rospy.is_shutdown(): break

            # Step 2: Wingbody Trigger
            rospy.loginfo("[{}] Publishing ForkLift DOWN Command".format(i))
            cmd_fork.n_fork_height = 300
            cmd_fork.n_fork_wide = -1
            cmd_fork.f_fork_tilt = -1
            cmd_fork.b_cancel = False
            self.pub_fork.publish(cmd_fork)
            self.wait_for_reach()
            time.sleep(1)

if __name__ == '__main__':
    try:
        node = ForkRepeater()
        node.run()
    except rospy.ROSInterruptException:
        pass
