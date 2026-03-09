#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import random

rospy.init_node("odom_random_pub")
pub = rospy.Publisher("/odom", Odometry, queue_size=10)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom"
    msg.pose.pose.position.x = random.uniform(-10, 10)
    msg.pose.pose.position.y = random.uniform(-10, 10)
    msg.pose.pose.orientation.w = 1.0  # 단순화
    pub.publish(msg)
    rate.sleep()
