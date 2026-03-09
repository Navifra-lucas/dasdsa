#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from math import pi, cos, sin

class Position():
    def __init__(self):
        '''
        x [m]
        y [m]
        head [rad]
        '''
        self.x=0
        self.y=0
        self.head=0
    
    def __add__(self, other):
        return self.x+other.x, self.y+other.y, self.head+other.head

class Velocity():
    def __init__(self):
        self.linear = LinearVelocity()
        self.angular = AngularVelocity()

class LinearVelocity():
    def __init__(self):
        self.vx=0
        self.vy=0

class AngularVelocity():
    def __init__(self):
        self.w=0

class Obs:
    def __init__(self):
        self.position = Position()
        self.velocity = Velocity()
        self.size=0

    def setPosition(self, x=None, y=None, head=None):
        '''
        x -> global position X[m]\n
        y -> global position Y[m]\n
        head -> global heading phsi[rad]\n
        '''
        if not x == None:
            self.position.x=x
            # print("x: " + str(x))
        if not y == None:
            self.position.y=y
            # print("y: " + str(y))
        if not head == None:
            self.position.head=head
            # print("head: " + str(head))

    def setLinearVelocity(self, vx, vy=0):
        '''
        vx -> longitudinal velocity [m/s]\n
        vy -> lateral velocity [m/s]\n
        '''
        self.velocity.linear.vx=vx
        self.velocity.linear.vy=vy

    def setAngularVelocity(self, w=0):
        '''
        w -> angular velocity [deg/s]\n
        saved as [rad/s] for calculation
        '''
        self.velocity.angular.w=w*pi/180

    def setObsSize(self, size=0):
        '''
        size [m]
        '''
        self.size=size
    
    def getObsSize_(self):
        return self.size
    
    def getPosiion_(self):
        return self.position
    
    def Move(self, dt):
        '''
        dt update rate[sec]
        '''
        self.position.head = self.position.head + dt*self.velocity.angular.w
        self.position.x = self.position.x + dt*self.velocity.linear.vx*cos(self.position.head)
        self.position.y = self.position.y + dt*self.velocity.linear.vx*sin(self.position.head)