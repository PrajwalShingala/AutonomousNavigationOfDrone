#!/usr/bin/env python
import math
import numpy as np
import time

import message_filters
import rospy

from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64

collision_dist = 0.0

i = 0

pub = rospy.Publisher('/detect_collide', Float64, queue_size = 1)

def callback(data):

	global collision_dist
	global i 

	drone_pos = np.array([data.pose[1].position.x, data.pose[1].position.y])
	tag_pos = np.array([data.pose[4].position.x, data.pose[4].position.y])
	
	collision_dist = math.sqrt((drone_pos[0] - tag_pos[0])**2 + (drone_pos[1] - tag_pos[1])**2)

	if collision_dist <= 3.0:

		collision_flag = 1.0
	else:
		collision_flag = 0.0
	
	if data.twist[4].linear.x != 0:
		
		print(collision_flag)
		
		if i%1 == 0:
		
			pub.publish(collision_flag)

	else:
		print('Tag not started moving yet')

	i += 1

def detect_collision():

	rospy.init_node('detect_collision', anonymous=True)

	rospy.Subscriber('/gazebo/model_states', ModelStates, callback)

	rospy.spin()

if __name__ == '__main__':

	detect_collision()
