#!/usr/bin/env python

from __future__ import print_function
import roslib
import rospy
import time

from threading import Thread
from geometry_msgs.msg import Twist

class BebopController():
    def __init__(self, publisher):
        self.publisher = publisher
        self.moving = True
        self.controller = None

    def move_by_velocity(self, velocity):
        vel_msg = Twist()
        vel_msg.linear.x = velocity[0]
        vel_msg.linear.y = velocity[1]
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        self.moving = False
        if self.controller:
            self.controller.join()
        self.controller = Thread(target=self.__publish, args=[vel_msg])
        self.controller.start()
    
    def __publish(self, msg):
        self.moving = True
        while self.moving:
            self.publisher.publish(msg)