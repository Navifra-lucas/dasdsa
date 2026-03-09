#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
nc_dcs 테스트용 통합 런처
- 3개의 시뮬레이터를 한번에 실행
"""

import rospy
from core_msgs.msg import ForkLift

def send_test_mission():
    """테스트 미션 발송"""
    rospy.sleep(3.0)  # Wait for nodes to initialize
    
    pub = rospy.Publisher('/nc_task_manager/fork_docking', ForkLift, queue_size=10)
    rospy.sleep(1.0)
    
    # Create test mission
    msg = ForkLift()
    msg.s_current_node_id = "NODE_001"
    msg.s_target_node_id = "NODE_002"
    msg.f_current_x = 1.0
    msg.f_current_y = 1.0
    msg.f_current_deg = 0.0
    msg.f_target_x = 3.0
    msg.f_target_y = 2.0
    msg.f_target_deg = 0.0
    msg.n_rack_level = 1
    msg.n_target_level = 2
    msg.n_target_height = 1500  # 1.5m
    msg.n_drive_type = 1
    msg.n_rack_type = 0
    
    rospy.loginfo("[TestLauncher] Sending test mission: drive_type=%d, height=%d", 
                  msg.n_drive_type, msg.n_target_height)
    pub.publish(msg)
    rospy.loginfo("[TestLauncher] Mission sent!")


if __name__ == '__main__':
    rospy.init_node('test_launcher', anonymous=False)
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("nc_dcs Test Environment")
    rospy.loginfo("=" * 60)
    rospy.loginfo("Simulators should be running:")
    rospy.loginfo("  - fork_simulator.py")
    rospy.loginfo("  - path_plan_simulator.py")
    rospy.loginfo("  - perception_simulator.py")
    rospy.loginfo("=" * 60)
    
    # Send test mission
    send_test_mission()
    
    rospy.spin()
