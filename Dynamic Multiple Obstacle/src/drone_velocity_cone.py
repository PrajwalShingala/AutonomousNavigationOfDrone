#!/usr/bin/env python
from __future__ import print_function

#import python libraries
import math

#import ros libraries
import roslib
import rospy

#import ros messages and functions
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry


class Drone(object):
	"""docstring for [object Object]."""
	def __init__(self,
				 forward_velocity,
				 angular_velocity,
				 trajectory_handle,
				 path_handle,
				 velocity_handle
				 ):
		self.forward_velocity = forward_velocity
		self.angular_velocity = angular_velocity
		self.trajectory_handle = trajectory_handle
		self.path_handle = path_handle
		self.velocity_handle = velocity_handle
		self.x = 0
		self.y = 0
		self.yaw = 0
		self.velocity = None
		self.done = False
		self.kill_thread = False

	def updateOdometry(self, odom):

		self.x = odom.pose.pose.position.x
		self.y = odom.pose.pose.position.y
		quaternion = (
					  odom.pose.pose.orientation.x,
					  odom.pose.pose.orientation.y,
					  odom.pose.pose.orientation.z,
					  odom.pose.pose.orientation.w
					 )
		euler = euler_from_quaternion(quaternion)
		self.yaw = euler[2]
		self.velocity = odom.twist.twist

	#generate ros trajectory message from data
	def generateTrajectoryMessage(self, px, py, pyaw):

		path = Path()

		temp_poses = []

		path.header.stamp = rospy.Time.now() 
		path.header.frame_id = "world"

		for i in range(len(px)):

			pose = PoseStamped()
			pose.header.stamp = rospy.Time.now()
			pose.header.frame_id = "world"
			pose.pose.position.x = px[i]
			pose.pose.position.y = py[i]
			pose.pose.position.z = 1
			quaternion = quaternion_from_euler(0, 0, pyaw[i])
			pose.pose.orientation.x = quaternion[0]
			pose.pose.orientation.y = quaternion[1]
			pose.pose.orientation.z = quaternion[2]
			pose.pose.orientation.w = quaternion[3]

			path.poses.append(pose)

		return path
	
	def dubinsMoveDrone(self, mode, pathlength, px, py, pyaw):

		path = self.generateTrajectoryMessage(px, py, pyaw)

		self.path_handle.publish(path)

		vel_msg = Twist()
		vel_msg.linear.x = self.forward_velocity
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0

		time = [1,1,1]
		for i in range(3):
			time[i] = pathlength[i]/self.forward_velocity

		i = 0
		for modes in mode:
			if modes == 'R':
				direction = -1
			elif modes == 'S': 
				direction = 0
			else:
				direction = 1

			vel_msg.angular.z = self.angular_velocity* direction
			
			start_time = rospy.Time.now().to_sec()

			while( rospy.Time.now().to_sec() - start_time < time[i] and not self.kill_thread and time[i] > 0.05):
				self.velocity_handle.publish(vel_msg)
			i = i + 1

		self.done = True


