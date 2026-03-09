#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
nc_intensity_detector의 검출 결과를 모니터링하는 스크립트
"""

import rospy
from geometry_msgs.msg import PoseStamped
import math

class DetectionMonitor:
    def __init__(self):
        rospy.init_node('detection_monitor', anonymous=True)
        
        self.pose_sub = rospy.Subscriber('/nc_intensity_detector/pose', 
                                        PoseStamped, 
                                        self.pose_callback)
        
        self.detection_count = 0
        self.last_detection_time = None
        
        rospy.loginfo("Detection Monitor started. Waiting for detections...")
        
    def pose_callback(self, msg):
        """포즈 메시지 수신 시 호출"""
        self.detection_count += 1
        self.last_detection_time = rospy.Time.now()
        
        # Quaternion to yaw
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        # 간단한 yaw 계산 (z축 회전만 고려)
        yaw = 2 * math.atan2(qz, qw)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Detection #%d", self.detection_count)
        rospy.loginfo("  Position: x=%.3f m, y=%.3f m, z=%.3f m", 
                     msg.pose.position.x,
                     msg.pose.position.y,
                     msg.pose.position.z)
        rospy.loginfo("  Orientation: yaw=%.2f deg", math.degrees(yaw))
        rospy.loginfo("  Frame: %s", msg.header.frame_id)
        rospy.loginfo("  Timestamp: %s", msg.header.stamp)
        rospy.loginfo("=" * 60)
        
    def run(self):
        """모니터링 실행"""
        rate = rospy.Rate(1)  # 1 Hz
        
        while not rospy.is_shutdown():
            if self.last_detection_time is not None:
                elapsed = (rospy.Time.now() - self.last_detection_time).to_sec()
                if elapsed > 5.0:
                    rospy.logwarn("No detection for %.1f seconds", elapsed)
            rate.sleep()

if __name__ == '__main__':
    try:
        monitor = DetectionMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass

