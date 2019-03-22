#!/usr/bin/env python

import math
import numpy as np 
import time

import rospy
import message_filters

from nav_msgs.msg import Odometry
from apriltags_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
#from geometry_msgs.msg import PoseArray

from velObs_mul_cone import getOptimal_velocity
print 'Importing matlab engine!'
import matlab.engine
eng = matlab.engine.start_matlab()
print 'Done Importing matlab engine!'



id_dict = {}
x_tag_previous = []
y_tag_previous = []
z_tag_previous = []
time_stamp_previous = []
timeOfLastPublish = []
#flag = True
size_prev = 0
initialised = [] 
x_tag_new = []
y_tag_new = []
z_tag_new = []
time_stamp_new = []
vel_x = []
vel_y = []
vel_z = []
#index = []
pub = rospy.Publisher('/safe_velocities', Twist, queue_size = 10)

filehandle = open("log_path.txt", "w")

def publ(x, y, z, w):
	vel_msg = Twist()
	vel_msg.linear.x = x 
	vel_msg.linear.y = y 
	vel_msg.linear.z = z 
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = w
	pub.publish(vel_msg)

def conversion(x, y, z, drone_odom):
	position_array = np.array([[x, y, z]])

	quaternion = (drone_odom.pose.pose.orientation.x, drone_odom.pose.pose.orientation.y, drone_odom.pose.pose.orientation.z, drone_odom.pose.pose.orientation.w)

	euler = euler_from_quaternion(quaternion)

	alpha = euler[2] #- angle_true_north

	position_drone = np.array([[drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z]])

	transformation_matrix = np.array([[math.cos(alpha), -1* math.sin(alpha), 0], [math.sin(alpha), math.cos(alpha), 0], [0, 0, 1]])

	new_position = np.matmul(transformation_matrix, np.transpose(position_array)) + np.transpose(position_drone) 

	#print new_position
	
	return new_position[0], new_position[1], new_position[2]


def initialize(val,drone_odom):
	global x_tag_previous
	global y_tag_previous
	global z_tag_previous
	global timeOfLastPublish
	global time_stamp_previous
	global initialised
		#global index
	#b_x = val.pose.pose.position.z
	#b_y = -val.pose.pose.position.x
	#b_z = -val.pose.pose.position.y 
	#val.pose.pose.position.x, val.pose.pose.position.y, val.pose.pose.position.z =  b_x, b_y, b_z
	x_tag_previous_temp = val.pose.pose.position.x
	y_tag_previous_temp = val.pose.pose.position.y
	z_tag_previous_temp = val.pose.pose.position.z
	x_tag_previous_temp, y_tag_previous_temp, z_tag_previous_temp = conversion(x_tag_previous_temp,y_tag_previous_temp,z_tag_previous_temp, drone_odom)
	x_tag_previous.append(x_tag_previous_temp)
	y_tag_previous.append(y_tag_previous_temp)
	z_tag_previous.append(z_tag_previous_temp)
	t = val.pose.header.stamp.secs + val.pose.header.stamp.nsecs/math.pow(10,9)
	time_stamp_previous.append(t)
	timeOfLastPublish.append(t)
	initialised.append(True)


def callback(tag_odom, drone_odom):
	global id_dict
	global x_tag_previous
	global y_tag_previous
	global z_tag_previous
	global time_stamp_previous
	global timeOfLastPublish
	#global flag
	global initialised
	global size_prev
	global x_tag_new
	global y_tag_new
	global z_tag_new
	global time_stamp_new
	global vel_x
	global vel_y
	global vel_z

	#tag_index = []
	#tag_id = []
	#size_new = len(tag_odom.detections)
	
	if tag_odom.detections:
		#Iterating through all tags
		flag = len(tag_odom.detections)
		for tag_val in tag_odom.detections:
			#b_x = tag_val.pose.pose.position.z
			#b_y = -tag_val.pose.pose.position.x
			#b_z = -tag_val.pose.pose.position.y 
			#tag_val.pose.pose.position.x, tag_val.pose.pose.position.y, tag_val.pose.pose.position.z =  b_x, b_y, b_z
			
			#Seeing tag for the first time
			if not tag_val.id in id_dict:
				#Adding values to dictionary and tag_previous array
				initialize(tag_val, drone_odom)
				#print len(x_tag_previous)

				#Saving index of current detected tag.
				id_dict[tag_val.id] = len(x_tag_previous) - 1

				#Initializing tag_new variables with same size as tag_previous.

				x_tag_new = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
				y_tag_new = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
				z_tag_new = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
				vel_x = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
				vel_y = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
				vel_z = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
				time_stamp_new = [0] * len(time_stamp_previous) #np.zeros_like(time_stamp_previous)
				#x_tag_new, y_tag_new,z_tag_new,vel_x,vel_y,vel_z = [0] * len(x_tag_previous) #
				#time_stamp_new = [0] * len(time_stamp_previous)
				#print x_tag_new
				continue
			else:
				index = id_dict[tag_val.id]
				#print id_dict

				#If tag detected in the past re-appears.
				if not initialised[index]:
					
					#print "not initialised", index
					x_tag_previous[index] = tag_val.pose.pose.position.x
					y_tag_previous[index] = tag_val.pose.pose.position.y
					z_tag_previous[index] = tag_val.pose.pose.position.z
					x_tag_previous[index], y_tag_previous[index], z_tag_previous[index] = conversion(x_tag_previous[index], y_tag_previous[index], z_tag_previous[index],drone_odom)
					#time_stamp_previous[index] = rospy.get_time()
					time_stamp_previous[index] = tag_val.pose.header.stamp.secs + (tag_val.pose.header.stamp.nsecs/math.pow(10,9))
					timeOfLastPublish[index] = time_stamp_previous[index]
					x_tag_new = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
					y_tag_new = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
					z_tag_new = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
					vel_x = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
					vel_y = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
					vel_z = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
					time_stamp_new = [0] * len(time_stamp_previous)  #np.zeros_like(time_stamp_previous)
					#x_tag_new, y_tag_new,z_tag_new,vel_x,vel_y,vel_z = [0] * len(x_tag_previous)
					#time_stamp_new = [0] * len(time_stamp_previous)

					initialised[index] = True
				else:
					#index = id_dict[tag_val.id]
					#print "initialised", index
					#print x_tag_new
					x_tag_new[index] = tag_val.pose.pose.position.x
					y_tag_new[index] = tag_val.pose.pose.position.y
					z_tag_new[index] = tag_val.pose.pose.position.z 
					#time_stamp_new[index] = rospy.get_time()
					time_stamp_new[index] = tag_val.pose.header.stamp.secs + (tag_val.pose.header.stamp.nsecs/math.pow(10,9))
					#print x_tag_new[index]
					#print "x_tag_new[index]", x_tag_new[index]
					x_tag_new[index], y_tag_new[index], z_tag_new[index] = conversion(x_tag_new[index], y_tag_new[index], z_tag_new[index],drone_odom)
					dist_trav_x = x_tag_new[index] - x_tag_previous[index]
					dist_trav_y = y_tag_new[index] - y_tag_previous[index]
					dist_trav_z = z_tag_new[index] - z_tag_previous[index]
					time_elapsed = time_stamp_new[index] - time_stamp_previous[index]
					if time_stamp_new[index] - timeOfLastPublish[index] > 2:
						initialised[index] = False
						print "Reset velocity ka timer"
					vel_x[index] = dist_trav_x/time_elapsed
					if abs(vel_x[index]) < 0.05:
						vel_x[index] = 0.0
					vel_y[index] = dist_trav_y/time_elapsed
					if abs(vel_y[index]) < 0.05:
						vel_y[index] = 0.0 

					vel_z[index] = dist_trav_z/time_elapsed
					if abs(vel_z[index]) < 0.05:
						vel_z[index] = 0.0 

					vel_x = np.array(vel_x)
					vel_y = np.array(vel_y)
					vel_z = np.array(vel_z)


					#print x_tag_new[index]
					print tag_val.id
					print vel_x[index], vel_y[index], vel_z[index]
					if initialised[index]:
						timeOfLastPublish[index] = tag_val.pose.header.stamp.secs + (tag_val.pose.header.stamp.nsecs/math.pow(10,9))
						'''flag = checkCollision(np.array([x_tag_new,y_tag_new]), (drone_odom.pose.pose.position.x,drone_odom.pose.pose.position.y),
								   (drone_odom.twist.twist.linear.x, drone_odom.twist.twist.linear.y, drone_odom.twist.twist.angular.z),
								   np.array([vel_x, vel_y]),
								   0.01,
								   filehandle, drone_odom
								   )
						print flag'''
						#print index
						#print index == len(tag_odom.detections) - 1
						print 'YOOOOOOOOOOO: ', tag_odom.detections.index(tag_val)
						if tag_odom.detections.index(tag_val) == flag - 1:
							#print drone_odom.twist.twist.angular.z
							print 'found tag'
							print np.array([vel_x, vel_y]).T
							print 'obs_pos'
							print np.array([x_tag_new, y_tag_new]).T
							
							print vel_x
							print vel_y 

							new_vel = getOptimal_velocity(np.array([drone_odom.pose.pose.position.x,drone_odom.pose.pose.position.y]),
															 np.array([drone_odom.twist.twist.linear.x, drone_odom.twist.twist.linear.y]),
															  np.array([x_tag_new,y_tag_new]).T, np.array([vel_x, vel_y]).T, eng)

							# if checkCollision(np.array([x_tag_new,y_tag_new]), (drone_odom.pose.pose.position.x,drone_odom.pose.pose.position.y),
							# 	   (drone_odom.twist.twist.linear.x, drone_odom.twist.twist.linear.y, drone_odom.twist.twist.angular.z),
							# 	   np.array([vel_x, vel_y]),
							# 	   0.1,
							# 	   filehandle, drone_odom, True
							# 	   ):
							# 	i = 1.0
							# 	while(i <= 2.0):
							# 		if not checkCollision((x_tag_new,y_tag_new), (drone_odom.pose.pose.position.x,drone_odom.pose.pose.position.y),
							# 			   (drone_odom.twist.twist.linear.x, drone_odom.twist.twist.linear.y, drone_odom.twist.twist.angular.z + i),
							# 			   (vel_x, vel_y),
							# 			   0.1,
							# 			   filehandle, drone_odom, False
							# 			   ):
							# 			publ(drone_odom.twist.twist.linear.x, drone_odom.twist.twist.linear.y, drone_odom.twist.twist.linear.z, drone_odom.twist.twist.angular.z + i)
							# 			v = math.sqrt(drone_odom.twist.twist.linear.x**2 + drone_odom.twist.twist.linear.y**2)
							# 			time.sleep((math.pi * v)/(2* (drone_odom.twist.twist.angular.z + i)))
							# 			print "Printing something!!!!!!!!!"
							# 			break

							# 		if not checkCollision((x_tag_new,y_tag_new), (drone_odom.pose.pose.position.x,drone_odom.pose.pose.position.y),
							# 			   (drone_odom.twist.twist.linear.x, drone_odom.twist.twist.linear.y, drone_odom.twist.twist.angular.z - i),
							# 			   (vel_x, vel_y),
							# 			   0.1,
							# 			   filehandle, drone_odom, False
							# 			   ):
							# 			publ(drone_odom.twist.twist.linear.x, drone_odom.twist.twist.linear.y, drone_odom.twist.twist.linear.z, drone_odom.twist.twist.angular.z - i)
							# 			# time.sleep(2.0)
							# 			v = math.sqrt(drone_odom.twist.twist.linear.x**2 + drone_odom.twist.twist.linear.y**2)
							# 			time.sleep((math.pi * v)/(2* (drone_odom.twist.twist.angular.z + i)))
							# 			print "Printing something!!!!!!!!!"
							# 			break
							# 		i = i + 0.1

		# if not (True in initialised):
		# 	#if flag:
		# 	for val in tag_odom.detections:
		# 		tag_index.append(tag_odom.detections.index(val))
		# 		tag_id.append(val.id)
		# 		x_tag_previous_temp = val.pose.pose.position.x
		# 		y_tag_previous_temp = val.pose.pose.position.y
		# 		z_tag_previous_temp = val.pose.pose.position.z
		# 		x_tag_previous_temp, y_tag_previous_temp, z_tag_previous_temp = conversion(x_tag_previous_temp,y_tag_previous_temp,z_tag_previous_temp)
		# 		x_tag_previous.append(x_tag_previous_temp)
		# 		y_tag_previous.append(y_tag_previous_temp)
		# 		z_tag_previous.append(z_tag_previous_temp)
		# 		time_stamp_previous.append(rospy.get_time())
		# 		timeOfLastPublish.append(rospy.get_time())
		# 		initialised.append(True)
		# 	zipped = zip(tag_id, tag_index)
		# 	id_dict_topic = dict(zipped)
		# 	id_dict_internal = id_dict_topic.copy()
		# 	#for val in id_dict.itervalues():
		# 	#	index.append(val)
		# 	size_prev = size_new
		# 	tag_index = []
		# 	tag_id = []
		# 	#flag = False
		# if size_new > size_prev and (True in initialised):
		# 	for val in tag_odom.detections:
		# 		tag_index.append(tag_odom.detections.index(val))
		# 		tag_id.append(val.id)
				
		# 	zipped = zip(tag_id, tag_index)
		# 	id_dict = dict(zipped)
		# 	size_prev = size_new
			

def velocity_mul_obs():
	rospy.init_node('velocity_mul_obs', anonymous = True)

	tag_odom = message_filters.Subscriber('/tag_detections', AprilTagDetectionArray, queue_size = 1)

	drone_odom = message_filters.Subscriber('/bebop2/odometry_sensor1/odometry', Odometry, queue_size = 1)

	ts = message_filters.ApproximateTimeSynchronizer([tag_odom, drone_odom], 10, 0.1, allow_headerless=True)

	ts.registerCallback(callback)

	rospy.spin()

if __name__ == '__main__':
	#try :
	velocity_mul_obs()
	# except rospy.ROSInterruptException:
	# 	filehandle.close()
	# 	pass