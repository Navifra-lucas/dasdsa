#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from PyQt5.QtCore import QObject, pyqtSignal
import time
from datetime import datetime
from obstacle import Obs
import rospy
from std_msgs.msg import String

class Worker(QObject):
    finished = pyqtSignal()

    def __init__(self):
        super().__init__()
        # init node
        rospy.init_node('ros_gui_node', anonymous=True)
        sleep_dt = rospy.get_param('~sleep_rate', 0.1)

        # ros publishers
        self.vir_obs_pub = rospy.Publisher('/virtual_obs', String, queue_size=1)

        # ros msgs
        self.vir_obs_msgs = String()

        # data
        self.vir_obs = Obs()
        self.sleep_dt = sleep_dt
        self.running = False
        self.thread_running = False

    def run(self):
        while not rospy.is_shutdown():
            if self.running:
                self.moveVirObs()
                self.pubVirObs()
            time.sleep(self.sleep_dt)
        self.finished.emit()

    def resume(self):
        if self.thread_running == True:
            self.running = True

    def pause(self):
        self.running = False
    
    def stop(self):
        rospy.signal_shutdown("QT GUI EXITS")

    def InitThreadStatus(self, thread):
        self.thread_running = True
        thread.start()

    def setLinearVelocity(self, vx, vy=0):
        self.vir_obs.setLinearVelocity(vx, vy)

    def setAngularVelocity(self, w=0):
        self.vir_obs.setAngularVelocity(w)

    def setPosition(self, x=None, y=None, head=None):
        self.vir_obs.setPosition(x, y, head)

    def setObsSize(self, size):
        self.vir_obs.setObsSize(size)

    def getRunningStatus(self):
        return self.running
    
    def pubVirObs(self):
        self.vir_obs_msgs.data = str(self.vir_obs.getPosiion_().x) + " / " + str(self.vir_obs.getPosiion_().y) + " / " +str(self.vir_obs.getObsSize_())
        print(datetime.now().strftime('%Y-%m-%d %H:%M:%S')+"\t"+self.vir_obs_msgs.data)
        self.vir_obs_pub.publish(self.vir_obs_msgs)

    def moveVirObs(self):
        self.vir_obs.Move(self.sleep_dt)
