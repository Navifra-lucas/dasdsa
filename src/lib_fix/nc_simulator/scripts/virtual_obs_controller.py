#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, os
from PyQt5.QtWidgets import *
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QThread
from qt_worker import Worker

dir_path = os.path.dirname(os.path.realpath(__file__))
ui_path_select = dir_path.split("/")
ui_path_select.pop()
ui_path = os.path.join(*ui_path_select)
ui_path = "/"+ui_path+"/resources/main_window.ui"
# print(ui_path)
form_class = uic.loadUiType(ui_path)[0]

class MyROSQtApp(QMainWindow, form_class):
    def __init__(self, app):
        super().__init__()
        self.rthread = QThread()
        self.worker = Worker()

        self.worker.moveToThread(self.rthread)
        self.rthread.started.connect(self.worker.run)
        self.worker.finished.connect(self.worker.deleteLater)
        self.rthread.finished.connect(self.rthread.deleteLater)
        app.aboutToQuit.connect(self.worker.stop)
        app.aboutToQuit.connect(self.rthread.quit)

        self.worker.InitThreadStatus(self.rthread)

        self.setupUi(self)
        self.v_text.setText("linear velocity = 0[m/s]")
        self.w_text.setText("angular velocity = 0[deg/s]")

        self.v_slider.setValue(0)
        self.v_slider.valueChanged.connect(self.linear_vel_change)
        
        self.w_slider.setValue(0)
        self.w_slider.valueChanged.connect(self.angular_vel_change)

        self.pushButton1.clicked.connect(self.update_obs)
        self.pushButton2.clicked.connect(self.updateVirObsStatus)

        # virtual obstacle velocity profile
        self.setWindowTitle('ROS Control GUI')
        self.show()

    def closeEvent(self, event):
        message = QtWidgets.QMessageBox.question(self, "Question", "Are you sure you want to quit?")
        if message == QtWidgets.QMessageBox.Yes:
            event.accept()
        else:
            event.ignore()
    
    def linear_vel_change(self,value):
        lin_v = value / 100
        self.worker.setLinearVelocity(lin_v) 
        self.v_text.setText("linear velocity = " + str(lin_v) + "[m/s]")

    def angular_vel_change(self,value):
        ang_w = value / 10
        self.worker.setAngularVelocity(ang_w)
        self.w_text.setText("angular velocity = " + str(ang_w) + "[deg/s]")
    
    def update_obs(self):
        if not self.worker.getRunningStatus():
            t_x = self.lineEdit01.text()
            t_y = self.lineEdit02.text()
            t_size = self.lineEdit03.text()
            self.worker.setPosition(x=float(t_x), y=float(t_y))
            self.worker.setObsSize(float(t_size))

    def updateVirObsStatus(self):
        state = self.pushButton2.isChecked()
        if state:
            self.worker.resume()
        else:
            self.worker.pause()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyROSQtApp(app)
    sys.exit(app.exec_())   # run GUI until system exits
