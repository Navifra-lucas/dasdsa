#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import threading
import time
import os
import datetime
import math

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from core_msgs.msg import NavicoreStatus
from core_msgs.msg import BatteryInfo
from task_msgs.msg import Charging
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class BatterySim :
    def __init__(self):
        self.current_soc = 80
        self.speed = 0.0
        self.status = "idle" # charging, idle
        rospy.init_node('BatterySim', anonymous=True)
        self.listener()
        self.talker()

        self.f_battery_charge_full_time_hr = rospy.get_param('battery_sim/f_battery_charge_full_time_hr', 1.0)
        self.f_battery_discharge_full_time_hr = rospy.get_param('battery_sim/f_battery_discharge_full_time_hr', 1.0)
        self.current_time = datetime.datetime.now()
        t = threading.Thread(target=self.doState, args=())
        t.setDaemon(True)
        t.start()

            
    def doState(self):
        while not rospy.is_shutdown():
            b_msg = BatteryInfo()
            if(self.status == "charging"):
                b_msg.b_charging_state = True
                if self.current_soc < 100:
                    self.current_soc += 1/(self.f_battery_charge_full_time_hr*360)
            elif(self.status == "idle"):
                b_msg.b_charging_state = False
                if self.current_soc > 1:
                    if( self.speed > 0.1):
                        self.current_soc -= 1/(self.f_battery_discharge_full_time_hr*360)*self.speed*10
                    else:
                        self.current_soc -= 1/(self.f_battery_discharge_full_time_hr*360)
            b_msg.un16_battery_soc = int(self.current_soc)
            self.battery_pub.publish(b_msg)
            time.sleep(0.1)
        
    def listener(self):
        rospy.Subscriber('/odom', Odometry, self.OdomCallback, queue_size=1)
        rospy.Subscriber('/nc_task_manager/charging', Charging, self.ChargeCallback, queue_size=5)
        rospy.Subscriber('/navifra/param_update', String, self.updateCallback, queue_size=5)

    def OdomCallback(self,msg):        
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
    
    def updateCallback(self,msg):        
        self.f_battery_charge_full_time_hr = rospy.get_param('battery_sim/f_battery_charge_full_time_hr', 1.0)
        self.f_battery_discharge_full_time_hr = rospy.get_param('battery_sim/f_battery_discharge_full_time_hr', 1.0)
        print("BatterySim: Updated parameters",self.f_battery_charge_full_time_hr," ",self.f_battery_discharge_full_time_hr)
        
    def ChargeCallback(self,msg):        
        if msg.mode == "charging":
            self.status = "charging"
        if msg.mode == "uncharging":
            self.status = "idle"

    def talker(self):
        self.battery_pub             = rospy.Publisher('/battery_info', BatteryInfo, queue_size=1)

if __name__ == '__main__':
    stateMachine = BatterySim()
    rospy.spin()

                        

    