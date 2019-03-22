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
x_tag_previous = np.array([])
y_tag_previous = np.array([])
z_tag_previous = np.array([])
time_stamp_previous = np.array([])
timeOfLastPublish = np.array([])
time_last_action = np.array([])
#flag = True
size_prev = 0
initialised = np.array([])
x_tag_new = np.array([])
y_tag_new = np.array([])
z_tag_new = np.array([])
time_stamp_new = np.array([])
vel_x = np.array([])
vel_y = np.array([])
vel_z = np.array([])

curr_vel = None
new_vel = None
last_time_compute = time.time()
wait_flag = True

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

def publ_cone(vx_old, vy_old, vx_new, vy_new, vel_flag):
	vel_msg = Twist()
	vel_msg.linear.x = vx_old 
	vel_msg.linear.y = vy_old
	vel_msg.linear.z = 0 
	vel_msg.angular.x = vx_new
	vel_msg.angular.y = vy_new
	vel_msg.angular.z = vel_flag
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
	global time_last_action
		#global index
	#b_x = val.pose.pose.position.z
	#b_y = -val.pose.pose.position.x
	#b_z = -val.pose.pose.position.y 
	#val.pose.pose.position.x, val.pose.pose.position.y, val.pose.pose.position.z =  b_x, b_y, b_z
	x_tag_previous_temp = val.pose.pose.position.x
	y_tag_previous_temp = val.pose.pose.position.y
	z_tag_previous_temp = val.pose.pose.position.z
	x_tag_previous_temp, y_tag_previous_temp, z_tag_previous_temp = conversion(x_tag_previous_temp,y_tag_previous_temp,z_tag_previous_temp, drone_odom)
	
	# print x_tag_previous.shape
	# print x_tag_previous_temp.shape

	x_tag_previous = np.append(x_tag_previous, x_tag_previous_temp, axis=0)
	y_tag_previous = np.append(y_tag_previous, y_tag_previous_temp, axis=0)
	z_tag_previous = np.append(z_tag_previous, z_tag_previous_temp, axis=0)
	t = np.array(val.pose.header.stamp.secs + val.pose.header.stamp.nsecs/math.pow(10,9)).reshape(1,)
	
	# print time_stamp_previous.shape
	# print t
	
	time_stamp_previous = np.append(time_stamp_previous, t, axis=0)
	timeOfLastPublish = np.append(timeOfLastPublish, t, axis=0)
	time_last_action = np.append(time_last_action, t-5, axis=0)
	# print "time last action: ", time_last_action
	initialised = np.append(initialised, np.array(True).reshape(1,), axis=0)


def callback(tag_odom, drone_odom):

	global wait_flag
	global curr_vel
	global id_dict
	global new_vel
	global last_time_compute
	global x_tag_previous
	global y_tag_previous
	global z_tag_previous
	global time_stamp_previous
	global timeOfLastPublish
	global time_last_action

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

	# if wait_flag:
	# 	time.sleep(3)
	# 	wait_flag = False


	#tag_index = []
	#tag_id = []
	#size_new = len(tag_odom.detections)
	
	if tag_odom.detections:
		#Iterating through all tags
		flag = len(tag_odom.detections)
		#print "The length of the odom detections is:", flag

		for tag_val in tag_odom.detections:
			#b_x = tag_val.pose.pose.position.z
			#b_y = -tag_val.pose.pose.position.x
			#b_z = -tag_val.pose.pose.position.y 
			#tag_val.pose.pose.position.x, tag_val.pose.pose.position.y, tag_val.pose.pose.position.z =  b_x, b_y, b_z
			
			#Seeing tag for the first time
			if not tag_val.id in id_dict:
				#Adding values to dictionary and tag_previous array
				initialize(tag_val, drone_odom)
				#print tag_val.pose.pose.position.x, "x tag previous not global"
				#print len(x_tag_previous)

				#Saving index of current detected tag.
				id_dict[tag_val.id] = len(x_tag_previous) - 1

				#Initializing tag_new variables with same size as tag_previous.

				# print "new tag seen!"

				length = len(x_tag_previous)

				x_tag_new = np.zeros((length,1)) #x_tag_new = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
				y_tag_new = np.zeros((length,1)) #y_tag_new = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
				z_tag_new = np.zeros((length,1)) #z_tag_new = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
				vel_x = np.zeros((length,1)) #vel_x = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
				vel_y = np.zeros((length,1)) #vel_y = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
				vel_z = np.zeros((length,1)) #vel_z = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
				time_stamp_new = np.zeros((length,1)) #time_stamp_new = [0] * len(time_stamp_previous) #np.zeros_like(time_stamp_previous)
				#x_tag_new, y_tag_new,z_tag_new,vel_x,vel_y,vel_z = [0] * len(x_tag_previous) #
				#time_stamp_new = [0] * len(time_stamp_previous)
				#print x_tag_new
				continue
			else:

				index = id_dict[tag_val.id]
				#print index
				# print "index given!"
				#If tag detected in the past re-appears.
				if not initialised[index]:
					
					#print "not initialised", index
					x_tag_previous[index] = tag_val.pose.pose.position.x
					y_tag_previous[index] = tag_val.pose.pose.position.y
					z_tag_previous[index] = tag_val.pose.pose.position.z

					#print tag_val.pose.pose.position.x, "x tag previous not global" 

					x_tag_previous[index], y_tag_previous[index], z_tag_previous[index] = conversion(x_tag_previous[index], y_tag_previous[index], z_tag_previous[index],drone_odom)
					#time_stamp_previous[index] = rospy.get_time()
					time_stamp_previous[index] = tag_val.pose.header.stamp.secs + (tag_val.pose.header.stamp.nsecs/math.pow(10,9))
					timeOfLastPublish[index] = time_stamp_previous[index]
					time_last_action[index] = time_stamp_previous[index] - 5
					#print "time last action: ", time_last_action[index]



					# print "tag re appeared!"

					# length = len(x_tag_previous)

					# x_tag_new = np.zeros((length,0)) #x_tag_new = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
					# y_tag_new = np.zeros((length,0)) #y_tag_new = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
					# z_tag_new = np.zeros((length,0)) #z_tag_new = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
					# vel_x = np.zeros((length,0)) #vel_x = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
					# vel_y = np.zeros((length,0)) #vel_y = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
					# vel_z = np.zeros((length,0)) #vel_z = [0] * len(x_tag_previous) #np.zeros_like(x_tag_previous)
					# time_stamp_new = np.zeros((length,0)) #time_stamp_new = [0] * len(time_stamp_previous) #np.zeros_like(time_stamp_previous)
					
					#x_tag_new, y_tag_new,z_tag_new,vel_x,vel_y,vel_z = [0] * len(x_tag_previous)
					#time_stamp_new = [0] * len(time_stamp_previous)

					initialised[index] = True
					# time.sleep(1)
				else:
					#index = id_dict[tag_val.id]
					#print "initialised", index
					#print x_tag_new

					# print "now initialised completely! "
					# print x_tag_new.shape, "x_tag_new ka shape"
					# print "here is the x_tag_new", x_tag_new
					x_tag_new[index,0] = tag_val.pose.pose.position.x
					y_tag_new[index,0] = tag_val.pose.pose.position.y
					z_tag_new[index,0] = tag_val.pose.pose.position.z 

					#print x_tag_new[index,0], " - - -- - distance tag from drone"
					#time_stamp_new[index] = rospy.get_time()
					time_stamp_new[index,0] = tag_val.pose.header.stamp.secs + (tag_val.pose.header.stamp.nsecs/math.pow(10,9))
					#print x_tag_new[index]
					#print "x_tag_new[index]", x_tag_new[index]
					x_tag_new[index,0], y_tag_new[index,0], z_tag_new[index,0] = conversion(x_tag_new[index,0], y_tag_new[index,0], z_tag_new[index,0],drone_odom)
					
					dist_trav_x = x_tag_new[index,0] - x_tag_previous[index]
					dist_trav_y = y_tag_new[index,0] - y_tag_previous[index]
					dist_trav_z = z_tag_new[index,0] - z_tag_previous[index]

					# print "x tag new pos", x_tag_new[index,0], y_tag_new[index,0], z_tag_new[index,0]
					#print "x tag prev pos", x_tag_previous[index], y_tag_previous[index], z_tag_previous[index]

					time_elapsed = time_stamp_new[index,0] - time_stamp_previous[index]
					if time_stamp_new[index,0] - timeOfLastPublish[index] > 1000:
						initialised[index] = False
						print "Reset velocity ka timer"
					vel_x[index,0] = dist_trav_x/time_elapsed
					if abs(vel_x[index,0]) < 0.05:
						vel_x[index,0] = 0.0
					vel_y[index,0] = dist_trav_y/time_elapsed
					if abs(vel_y[index,0]) < 0.05:
						vel_y[index,0] = 0.0 

					vel_z[index,0] = dist_trav_z/time_elapsed
					if abs(vel_z[index,0]) < 0.05:
						vel_z[index,0] = 0.0 

					vel_x = np.array(vel_x)
					vel_y = np.array(vel_y)
					vel_z = np.array(vel_z)

					# print drone_odom.twist.twist.linear.x, drone_odom.twist.twist.linear.y

					#print vel_x, "- - - - velocity in x "

					time_since_last_action = time_stamp_new[index,0] - time_last_action[index]

					#print(time_since_last_action)

					if initialised[index] and time_since_last_action > 2.0:
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
						#print 'YOOOOOOOOOOO: ', tag_odom.detections.index(tag_val)
						if tag_odom.detections.index(tag_val) == flag - 1:

							if time.time() - last_time_compute < 5.0: 	
								curr_vel = new_vel
							else:
								print 'five sec over!'
								curr_vel = None

							if curr_vel is None:
								print "Resetting current velocity from drone odom!"
								curr_vel = [drone_odom.twist.twist.linear.x, drone_odom.twist.twist.linear.y]
								print "Printing current velocity: ", curr_vel

							# curr_vel = [drone_odom.twist.twist.linear.x, drone_odom.twist.twist.linear.y]

						# print vel_x, vel_y

							new_vel = getOptimal_velocity(np.array([drone_odom.pose.pose.position.x,drone_odom.pose.pose.position.y]),
															 np.array(curr_vel),
															  np.array([x_tag_new,y_tag_new]).T, np.array([vel_x, vel_y]).T, eng, 4.0)
							
							last_time_compute = time.time()

							print new_vel

							if abs(curr_vel[0] - new_vel[0]) < 0.05 and abs(curr_vel[1] - new_vel[1]) < 0.05:
								vel_flag = 0
							else:
								print 'difference in velocity:', 'x: ', curr_vel[0] - new_vel[0], 'y: ', curr_vel[1] - new_vel[1]
								vel_flag = -5

							# print 'Obstacle velocity is: ', np.array([vel_x, vel_y])

							# print 'Drone current velocity is: ', drone_odom.twist.twist.linear.x, drone_odom.twist.twist.linear.y
							
							# if abs(curr_vel[0] - new_vel[0]) > 0.05 or abs(curr_vel[1] - new_vel[1]) > 0.05:	
							print "publishing new vel"
							publ_cone(curr_vel[0], curr_vel[1], new_vel[0], new_vel[1], vel_flag)

							time_last_action[index] = time_stamp_new[index,0]

			

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